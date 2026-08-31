#ifndef SONIC_G1_DEPLOY_SHUTDOWN_SAFETY_HPP_
#define SONIC_G1_DEPLOY_SHUTDOWN_SAFETY_HPP_

#include <chrono>

namespace sonic::shutdown_safety {

using SteadyClockTick = std::chrono::steady_clock::duration::rep;

// steady_clock timestamps used by the controller are positive after boot.
// Zero is reserved so the two atomics can distinguish a boundary that has not
// yet been captured/published without relying on wall-clock state.
inline constexpr SteadyClockTick kNoShutdownBoundary = 0;

constexpr SteadyClockTick EarliestBoundary(
    SteadyClockTick current, SteadyClockTick candidate) noexcept {
  return current == kNoShutdownBoundary || candidate < current
      ? candidate
      : current;
}

// Once an intentional-shutdown boundary is published, only heartbeat aging
// caused after that boundary is suppressed.  A heartbeat whose deadline was
// already due at (including exactly at) the boundary remains a safety fault.
constexpr bool HeartbeatDeadlineRequiresFault(
    SteadyClockTick now, SteadyClockTick heartbeat_deadline,
    SteadyClockTick shutdown_boundary) noexcept {
  if (shutdown_boundary != kNoShutdownBoundary) {
    return heartbeat_deadline <= shutdown_boundary;
  }
  return heartbeat_deadline <= now;
}

// The deployment supervisor normally stops before this boundary, but the
// controller must remain safe if that supervisor and its docker-compose client
// both disappear.  Zero means takeover has not begun; once a real deadline is
// published, equality is already expired so no extra writer period is granted.
constexpr bool ActuationLeaseExpired(
    SteadyClockTick now, SteadyClockTick lease_deadline) noexcept {
  return lease_deadline != kNoShutdownBoundary && now >= lease_deadline;
}

}  // namespace sonic::shutdown_safety

#endif  // SONIC_G1_DEPLOY_SHUTDOWN_SAFETY_HPP_
