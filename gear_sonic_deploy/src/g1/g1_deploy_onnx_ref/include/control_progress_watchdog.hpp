#ifndef SONIC_G1_DEPLOY_CONTROL_PROGRESS_WATCHDOG_HPP_
#define SONIC_G1_DEPLOY_CONTROL_PROGRESS_WATCHDOG_HPP_

#include <atomic>
#include <chrono>

namespace sonic::control {

// The policy producer is scheduled at 50 Hz.  Allow 25 complete control
// periods (500 ms) without a successful tick before the independent 500 Hz
// writer must stop replaying the last position command.  This is deliberately
// much wider than the old 100 ms / five-period heartbeat.
inline constexpr int kControlRateHz = 50;
static_assert(1000 % kControlRateHz == 0);
inline constexpr std::chrono::milliseconds kControlPeriod{
    1000 / kControlRateHz};
inline constexpr int kMissedControlPeriodBudget = 25;
inline constexpr auto kControlProgressTimeout =
    kControlPeriod * kMissedControlPeriodBudget;

class ControlProgressWatchdog {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using Tick = Clock::duration::rep;

  void MarkSuccessfulTick() noexcept {
    MarkSuccessfulTick(Clock::now());
  }

  void MarkSuccessfulTick(TimePoint now) noexcept {
    last_successful_tick_.store(
        now.time_since_epoch().count(), std::memory_order_release);
    armed_.store(true, std::memory_order_release);
  }

  bool DampingRequired() noexcept {
    return DampingRequired(Clock::now());
  }

  bool DampingRequired(TimePoint now) noexcept {
    if (damping_latched_.load(std::memory_order_acquire)) {
      return true;
    }
    if (!armed_.load(std::memory_order_acquire)) {
      return false;
    }

    const Tick observed =
        last_successful_tick_.load(std::memory_order_acquire);
    if (!ProgressTimedOut(now, observed)) {
      return false;
    }

    // A producer tick may have completed while the writer evaluated the old
    // timestamp.  Re-read once before making the one-way decision.  The wide
    // period budget remains the primary scheduling margin.
    const Tick confirmed =
        last_successful_tick_.load(std::memory_order_acquire);
    if (confirmed != observed || !ProgressTimedOut(now, confirmed)) {
      return false;
    }

    damping_latched_.store(true, std::memory_order_release);
    return true;
  }

  bool IsArmed() const noexcept {
    return armed_.load(std::memory_order_acquire);
  }

  bool IsDampingLatched() const noexcept {
    return damping_latched_.load(std::memory_order_acquire);
  }

  static constexpr bool ProgressTimedOut(
      TimePoint now, Tick last_successful_tick) noexcept {
    const TimePoint last_successful{
        Clock::duration{last_successful_tick}};
    return now >= last_successful &&
           now - last_successful >= kControlProgressTimeout;
  }

 private:
  std::atomic<Tick> last_successful_tick_{0};
  std::atomic<bool> armed_{false};
  std::atomic<bool> damping_latched_{false};
};

}  // namespace sonic::control

#endif  // SONIC_G1_DEPLOY_CONTROL_PROGRESS_WATCHDOG_HPP_
