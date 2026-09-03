#ifndef SONIC_G1_PHYSICAL_RUNTIME_SAFETY_HPP
#define SONIC_G1_PHYSICAL_RUNTIME_SAFETY_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <type_traits>
#include <utility>

#include "mode5_contract_generated.hpp"

namespace sonic::physical_runtime_safety {

inline constexpr bool MatchesMode5LowStateIdentity(
    std::uint8_t mode_machine, std::uint8_t mode_pr) noexcept {
  return mode_machine == mode5_contract::kModeMachine &&
         mode_pr == mode5_contract::kModePr;
}

/// Whether a measured joint position has left the G1 hard range.
///
/// The commanded-target clamp in CreatePolicyCommand() bounds what the
/// controller asks for; it says nothing about where the robot actually is.  A
/// joint driven past its hard limit by an external push, by gravity, or by a
/// gear slip leaves no commanded-side evidence at all, so this predicate is
/// the only check on measured travel.
///
/// A NaN measurement returns false: a non-finite state is not "outside the
/// range", it is unusable, and it has its own fault reason.  Callers must run
/// their non-finite check FIRST.  That ordering dependency is regression
/// tested in unit_tests/test_physical_runtime_safety.cpp.
inline constexpr bool MeasuredPositionOutsideHardRange(
    double measured_q, double lower_limit, double upper_limit) noexcept {
  return measured_q < lower_limit || measured_q > upper_limit;
}

/// Bound one control step to `max_delta_rad` around the previously executed
/// wire target.
///
/// `max_delta_rad <= 0`, or a non-finite bound, means the limiter is disabled
/// and the desired target passes through unchanged: the control is opt-in.
///
/// `previous_executed_q` is empty only before INIT has published its first
/// command.  The CONTROL path never sees that -- INIT always publishes a ramp
/// target first -- but with no reference there is nothing to bound against, so
/// the desired target passes through.
///
/// This is a step bound, NOT a range bound.  A bounded step taken from a
/// previous target that already sits near a hard limit can still land outside
/// it, so callers must clamp the result to the hard joint range AFTER
/// limiting, never before.
inline double LimitCommandStep(double desired_q,
                               std::optional<double> previous_executed_q,
                               double max_delta_rad) noexcept {
  if (!(max_delta_rad > 0.0) || !std::isfinite(max_delta_rad) ||
      !previous_executed_q.has_value()) {
    return desired_q;
  }
  const double previous = *previous_executed_q;
  if (!std::isfinite(previous) || !std::isfinite(desired_q)) {
    return desired_q;
  }
  return std::clamp(desired_q, previous - max_delta_rad,
                    previous + max_delta_rad);
}

/// Whether an encoder observation-gather failure may try another mode.
///
/// A physical runtime, or any runtime with an explicit expected stream mode,
/// must preserve that declared observation contract and fail closed.  The
/// unbound simulation path retains upstream's diagnostic fallback for ordinary
/// robot/teleoperation modes.  SMPL mode 2 remains fail closed even there:
/// its robot-joint compatibility channel is not an equivalent reference.
inline constexpr bool EncoderModeFallbackAllowed(
    bool simulation_only, int expected_stream_mode,
    int intended_encoder_mode) noexcept {
  return simulation_only && expected_stream_mode < 0 &&
         intended_encoder_mode != 2;
}

/// Serializes publication of CRC-accepted LowState identity with the final
/// pre-takeover confirmation that arms identity monitoring.
///
/// An observation that wins the mutex before arming is published first, so
/// ArmIf() must validate it. An observation that wins after arming sees the
/// armed state and reports divergence. There is no interleaving in which a
/// callback can decide "not armed" and publish a mismatch after confirmation.
class Mode5IdentityGate {
 public:
  template <typename Publish, typename OnMismatch>
  bool PublishAndCheck(std::uint8_t mode_machine, std::uint8_t mode_pr,
                       Publish&& publish, OnMismatch&& on_mismatch) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool mismatch =
        armed_ && !MatchesMode5LowStateIdentity(mode_machine, mode_pr);
    if (mismatch) {
      // Complete the one-way damping latch before releasing the same mutex
      // that serializes publication with takeover arming. The arming thread
      // can never return through a concurrently published mismatch whose
      // safety action is still pending.
      std::forward<OnMismatch>(on_mismatch)();
    }
    // Before arming, publication lets ArmIf() validate the latest sample.
    // After arming, a mismatch is made visible only after damping is latched.
    std::forward<Publish>(publish)();
    return mismatch;
  }

  template <typename CurrentSampleIsValid>
  bool ArmIf(CurrentSampleIsValid&& current_sample_is_valid) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::forward<CurrentSampleIsValid>(current_sample_is_valid)()) {
      return false;
    }
    armed_ = true;
    return true;
  }

  bool IsArmed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return armed_;
  }

 private:
  mutable std::mutex mutex_;
  bool armed_ = false;
};

/// Read a mutable active-command source only while active control is still
/// permitted. A wire-side damping latch must not wait behind a producer that
/// is stalled while holding the active-command buffer's lock.
template <typename ReadActiveCommand>
auto ReadActiveCommandUnlessDamping(bool damping_required,
                                    ReadActiveCommand&& read_active_command) {
  using Snapshot =
      std::decay_t<decltype(std::forward<ReadActiveCommand>(
          read_active_command)())>;
  if (damping_required) {
    return std::optional<Snapshot>{};
  }
  return std::optional<Snapshot>{
      std::forward<ReadActiveCommand>(read_active_command)()};
}

/// Fetch a potentially blocking active-command snapshot before entering the
/// mutex that serializes the final damping decision and wire publication.
///
/// commit_locked() must re-evaluate every one-way damping source while
/// commit_mutex is held. A damping latch may win while read_active_command()
/// is blocked; in that case commit_locked() must discard the prefetched active
/// snapshot. Keeping the lock acquisition in this helper makes that ordering
/// directly regression-testable.
template <typename ReadActiveCommand, typename CommitLocked>
auto PrefetchActiveCommandThenCommitLocked(
    bool damping_required_before_prefetch, std::mutex& commit_mutex,
    ReadActiveCommand&& read_active_command, CommitLocked&& commit_locked) {
  auto command_snapshot = ReadActiveCommandUnlessDamping(
      damping_required_before_prefetch,
      std::forward<ReadActiveCommand>(read_active_command));
  std::lock_guard<std::mutex> lock(commit_mutex);
  return std::forward<CommitLocked>(commit_locked)(command_snapshot);
}

enum class AcceptedWindowState {
  CURRENT,
  STARTUP_GRACE,
  MISSING,
  STALE,
};

inline AcceptedWindowState EvaluateAcceptedWindow(
    bool streaming_active,
    std::optional<std::chrono::steady_clock::time_point> accepted,
    std::optional<std::chrono::steady_clock::time_point> stream_enabled,
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::duration maximum_age,
    std::chrono::steady_clock::duration startup_grace) noexcept {
  if (!streaming_active) {
    return AcceptedWindowState::MISSING;
  }
  if (accepted.has_value()) {
    if (*accepted > now || now - *accepted > maximum_age) {
      return AcceptedWindowState::STALE;
    }
    return AcceptedWindowState::CURRENT;
  }
  if (stream_enabled.has_value() && *stream_enabled <= now &&
      now - *stream_enabled <= startup_grace) {
    return AcceptedWindowState::STARTUP_GRACE;
  }
  return AcceptedWindowState::MISSING;
}

inline constexpr bool ReadyForPhysicalControl(
    bool exact_episode_bound, bool streamed_motion_active,
    AcceptedWindowState window_state) noexcept {
  return exact_episode_bound && streamed_motion_active &&
         window_state == AcceptedWindowState::CURRENT;
}

inline constexpr bool HasCompletePhysicalStreamBinding(
    bool standalone_zmq, int expected_stream_mode,
    bool expected_stream_episode_bound) noexcept {
  return standalone_zmq && expected_stream_mode >= 0 &&
         expected_stream_episode_bound;
}

}  // namespace sonic::physical_runtime_safety

#endif  // SONIC_G1_PHYSICAL_RUNTIME_SAFETY_HPP
