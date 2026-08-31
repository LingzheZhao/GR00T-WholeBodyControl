#ifndef DEBUG_MONOTONIC_CLOCK_HPP
#define DEBUG_MONOTONIC_CLOCK_HPP

#include <cerrno>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <time.h>

namespace sonic::telemetry {

using ClockGettimeFunction = int (*)(clockid_t, struct timespec*);

/**
 * Read Linux CLOCK_MONOTONIC as an unsigned nanosecond timestamp.
 *
 * The optional function argument exists so the failure paths can be tested
 * without changing the process clock. Production callers use clock_gettime
 * directly. A clock failure or malformed result is fatal because publishing a
 * g1_debug packet without a valid source timestamp would violate the telemetry
 * wire contract.
 */
inline std::uint64_t ReadDebugPublishedMonotonicNs(
    ClockGettimeFunction clock_gettime_fn = ::clock_gettime) {
  struct timespec timestamp {};
  if (clock_gettime_fn(CLOCK_MONOTONIC, &timestamp) != 0) {
    const int error = errno;
    throw std::system_error(
        error,
        std::generic_category(),
        "clock_gettime(CLOCK_MONOTONIC) failed for g1_debug");
  }
  if (timestamp.tv_sec < 0 || timestamp.tv_nsec < 0 ||
      timestamp.tv_nsec >= 1'000'000'000L) {
    throw std::runtime_error(
        "clock_gettime(CLOCK_MONOTONIC) returned an invalid timespec");
  }

  constexpr std::uint64_t kNanosecondsPerSecond = 1'000'000'000ULL;
  const auto seconds = static_cast<std::uint64_t>(timestamp.tv_sec);
  const auto nanoseconds = static_cast<std::uint64_t>(timestamp.tv_nsec);
  if (seconds >
      (std::numeric_limits<std::uint64_t>::max() - nanoseconds) /
          kNanosecondsPerSecond) {
    throw std::overflow_error(
        "CLOCK_MONOTONIC timestamp does not fit in uint64 nanoseconds");
  }
  return seconds * kNanosecondsPerSecond + nanoseconds;
}

}  // namespace sonic::telemetry

#endif  // DEBUG_MONOTONIC_CLOCK_HPP
