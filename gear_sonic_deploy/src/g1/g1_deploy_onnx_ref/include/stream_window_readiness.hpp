#ifndef SONIC_G1_STREAM_WINDOW_READINESS_HPP
#define SONIC_G1_STREAM_WINDOW_READINESS_HPP

#include <chrono>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <utility>

namespace sonic::stream_window_readiness {

/// Publishes accepted-window freshness only after that same window has become
/// the active controller input.
///
/// Decode and activation are deliberately separate in the ZMQ input path. A
/// bare accepted timestamp therefore permits this unsafe observation:
///
///   accepted == current, current_motion == previous motion
///
/// CommitAfterActivation() closes that interval. It serializes activation and
/// readiness publication, while a monotonically changing generation prevents
/// a decoded result from an earlier streaming epoch from being activated or
/// advertised after a reset.
class CommitGate {
 public:
  using Clock = std::chrono::steady_clock;
  using Generation = std::uint64_t;

  /// Begin a new streaming epoch and invalidate every previous commit.
  Generation BeginGeneration() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation_ == std::numeric_limits<Generation>::max()) {
      // Zero is reserved for "no generation". Wrap is unreachable in a real
      // process, but retaining the invariant makes the comparison total.
      generation_ = 1;
    } else {
      ++generation_;
    }
    accepted_at_.reset();
    return generation_;
  }

  Generation CurrentGeneration() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
  }

  /// Run activate() and publish freshness as one generation-checked commit.
  ///
  /// A stale decoded result returns false without invoking activate(). Once a
  /// reader can observe CurrentAcceptedTime(), activate() has completed for
  /// the exact same generation.
  template <typename Activate>
  bool CommitAfterActivation(Generation generation, Activate&& activate) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation == 0 || generation != generation_) {
      return false;
    }
    std::forward<Activate>(activate)();
    accepted_at_ = Clock::now();
    return true;
  }

  std::optional<Clock::time_point> CurrentAcceptedTime() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return accepted_at_;
  }

 private:
  mutable std::mutex mutex_;
  Generation generation_ = 0;
  std::optional<Clock::time_point> accepted_at_{};
};

}  // namespace sonic::stream_window_readiness

#endif  // SONIC_G1_STREAM_WINDOW_READINESS_HPP
