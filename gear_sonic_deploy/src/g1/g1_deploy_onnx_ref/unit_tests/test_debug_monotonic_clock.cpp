#include <gtest/gtest.h>

#include "../include/output_interface/debug_monotonic_clock.hpp"

#include <cerrno>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace {

int FixedClock(clockid_t clock_id, struct timespec* timestamp) {
  if (clock_id != CLOCK_MONOTONIC) {
    return -1;
  }
  timestamp->tv_sec = 123;
  timestamp->tv_nsec = 456'789'012;
  return 0;
}

int FailingClock(clockid_t, struct timespec*) {
  errno = EIO;
  return -1;
}

int InvalidNanosecondsClock(clockid_t, struct timespec* timestamp) {
  timestamp->tv_sec = 1;
  timestamp->tv_nsec = 1'000'000'000L;
  return 0;
}

int OverflowingClock(clockid_t, struct timespec* timestamp) {
  timestamp->tv_sec =
      static_cast<time_t>(
          std::numeric_limits<std::uint64_t>::max() / 1'000'000'000ULL + 1);
  timestamp->tv_nsec = 0;
  return 0;
}

}  // namespace

TEST(DebugMonotonicClock, UsesClockMonotonicAndConvertsToNanoseconds) {
  EXPECT_EQ(
      sonic::telemetry::ReadDebugPublishedMonotonicNs(FixedClock),
      123'456'789'012ULL);
}

TEST(DebugMonotonicClock, ProductionClockIsPositiveAndNondecreasing) {
  const auto first = sonic::telemetry::ReadDebugPublishedMonotonicNs();
  const auto second = sonic::telemetry::ReadDebugPublishedMonotonicNs();
  EXPECT_GT(first, 0U);
  EXPECT_GE(second, first);
}

TEST(DebugMonotonicClock, ClockFailureIsFatalAndPreservesErrno) {
  try {
    (void)sonic::telemetry::ReadDebugPublishedMonotonicNs(FailingClock);
    FAIL() << "expected std::system_error";
  } catch (const std::system_error& error) {
    EXPECT_EQ(error.code().value(), EIO);
  }
}

TEST(DebugMonotonicClock, InvalidTimespecIsFatal) {
  EXPECT_THROW(
      sonic::telemetry::ReadDebugPublishedMonotonicNs(
          InvalidNanosecondsClock),
      std::runtime_error);
}

TEST(DebugMonotonicClock, NanosecondOverflowIsFatal) {
  EXPECT_THROW(
      sonic::telemetry::ReadDebugPublishedMonotonicNs(OverflowingClock),
      std::overflow_error);
}
