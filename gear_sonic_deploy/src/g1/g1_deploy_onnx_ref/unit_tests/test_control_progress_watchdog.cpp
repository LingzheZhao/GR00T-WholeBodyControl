#include <gtest/gtest.h>

#include "../include/control_progress_watchdog.hpp"

#include <chrono>

namespace control = sonic::control;

TEST(ControlProgressWatchdog, TimeoutIsDerivedFromTwentyFiveControlPeriods) {
  using namespace std::chrono_literals;
  EXPECT_EQ(control::kControlPeriod, 20ms);
  EXPECT_EQ(control::kControlProgressTimeout, 500ms);
  EXPECT_EQ(
      control::kControlProgressTimeout,
      control::kControlPeriod * control::kMissedControlPeriodBudget);
}

TEST(ControlProgressWatchdog, DoesNotTripBeforeTheFirstSuccessfulTick) {
  control::ControlProgressWatchdog watchdog;
  const auto far_in_the_future =
      control::ControlProgressWatchdog::TimePoint{} + std::chrono::hours{24};

  EXPECT_FALSE(watchdog.IsArmed());
  EXPECT_FALSE(watchdog.DampingRequired(far_in_the_future));
  EXPECT_FALSE(watchdog.IsDampingLatched());
}

TEST(ControlProgressWatchdog, TripsAtTheMissedPeriodBoundary) {
  using Watchdog = control::ControlProgressWatchdog;
  const Watchdog::TimePoint start{std::chrono::seconds{1}};
  Watchdog watchdog;
  watchdog.MarkSuccessfulTick(start);

  EXPECT_TRUE(watchdog.IsArmed());
  EXPECT_FALSE(watchdog.DampingRequired(
      start + control::kControlProgressTimeout - std::chrono::nanoseconds{1}));
  EXPECT_TRUE(watchdog.DampingRequired(
      start + control::kControlProgressTimeout));
  EXPECT_TRUE(watchdog.IsDampingLatched());
}

TEST(ControlProgressWatchdog, ARefreshMovesTheProducerDeadline) {
  using Watchdog = control::ControlProgressWatchdog;
  const Watchdog::TimePoint start{std::chrono::seconds{1}};
  const auto almost_timeout =
      control::kControlProgressTimeout - std::chrono::milliseconds{1};
  Watchdog watchdog;
  watchdog.MarkSuccessfulTick(start);

  const auto refreshed = start + almost_timeout;
  EXPECT_FALSE(watchdog.DampingRequired(refreshed));
  watchdog.MarkSuccessfulTick(refreshed);
  EXPECT_FALSE(watchdog.DampingRequired(refreshed + almost_timeout));
  EXPECT_TRUE(watchdog.DampingRequired(
      refreshed + control::kControlProgressTimeout));
}

TEST(ControlProgressWatchdog, HealthyIndefiniteHoldNeverTimesOut) {
  using Watchdog = control::ControlProgressWatchdog;
  Watchdog watchdog;
  Watchdog::TimePoint now{std::chrono::seconds{1}};
  watchdog.MarkSuccessfulTick(now);

  // This represents more than half an hour on a held final pose.  Progress is
  // tied only to successful 50 Hz ticks, not to motion length or hold length.
  for (int tick = 0; tick < 100'000; ++tick) {
    now += control::kControlPeriod;
    watchdog.MarkSuccessfulTick(now);
    EXPECT_FALSE(watchdog.DampingRequired(
        now + control::kControlPeriod / 2));
  }
  EXPECT_FALSE(watchdog.IsDampingLatched());
}

TEST(ControlProgressWatchdog, SuccessfulTicksCannotClearALatchedTimeout) {
  using Watchdog = control::ControlProgressWatchdog;
  const Watchdog::TimePoint start{std::chrono::seconds{1}};
  Watchdog watchdog;
  watchdog.MarkSuccessfulTick(start);
  EXPECT_TRUE(watchdog.DampingRequired(
      start + control::kControlProgressTimeout));

  watchdog.MarkSuccessfulTick(start + std::chrono::hours{1});
  EXPECT_TRUE(watchdog.DampingRequired(
      start + std::chrono::hours{1}));
  EXPECT_TRUE(watchdog.IsDampingLatched());
}
