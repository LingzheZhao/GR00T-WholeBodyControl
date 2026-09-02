#ifndef SONIC_G1_PHYSICAL_RUNTIME_SAFETY_HPP
#define SONIC_G1_PHYSICAL_RUNTIME_SAFETY_HPP

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

#include "mode5_contract_generated.hpp"

namespace sonic::physical_runtime_safety {

inline constexpr bool MatchesMode5LowStateIdentity(
    std::uint8_t mode_machine, std::uint8_t mode_pr) noexcept {
  return mode_machine == mode5_contract::kModeMachine &&
         mode_pr == mode5_contract::kModePr;
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
  template <typename Publish>
  bool PublishAndCheck(std::uint8_t mode_machine, std::uint8_t mode_pr,
                       Publish&& publish) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::forward<Publish>(publish)();
    return armed_ &&
           !MatchesMode5LowStateIdentity(mode_machine, mode_pr);
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

}  // namespace sonic::physical_runtime_safety

#endif  // SONIC_G1_PHYSICAL_RUNTIME_SAFETY_HPP
