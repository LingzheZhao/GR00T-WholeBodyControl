#include <gtest/gtest.h>

#include "../include/dex3_hands.hpp"
#include "../include/physical_runtime_safety.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <limits>
#include <mutex>
#include <optional>

namespace safety = sonic::physical_runtime_safety;

TEST(PhysicalRuntimeSafety, LowStateIdentityIsExactlyMode5Pr0) {
  EXPECT_TRUE(safety::MatchesMode5LowStateIdentity(
      sonic::mode5_contract::kModeMachine,
      sonic::mode5_contract::kModePr));
  EXPECT_FALSE(safety::MatchesMode5LowStateIdentity(
      static_cast<std::uint8_t>(sonic::mode5_contract::kModeMachine + 1),
      sonic::mode5_contract::kModePr));
  EXPECT_FALSE(safety::MatchesMode5LowStateIdentity(
      sonic::mode5_contract::kModeMachine,
      static_cast<std::uint8_t>(sonic::mode5_contract::kModePr + 1)));
}

TEST(PhysicalRuntimeSafety, DeclaredEncoderModeNeverFallsBack) {
  for (const int intended_mode : {0, 2}) {
    EXPECT_FALSE(safety::EncoderModeFallbackAllowed(
        false, intended_mode, intended_mode));
    EXPECT_FALSE(safety::EncoderModeFallbackAllowed(
        true, intended_mode, intended_mode));
  }
  // Physical runtimes fail closed even before a missing declaration is
  // rejected by startup validation.
  EXPECT_FALSE(safety::EncoderModeFallbackAllowed(false, -1, 0));
  EXPECT_FALSE(safety::EncoderModeFallbackAllowed(false, -1, 2));
}

TEST(PhysicalRuntimeSafety, UnboundSimulationKeepsOnlyCompatibleFallbacks) {
  EXPECT_TRUE(safety::EncoderModeFallbackAllowed(true, -1, 0));
  EXPECT_TRUE(safety::EncoderModeFallbackAllowed(true, -1, 1));
  EXPECT_FALSE(safety::EncoderModeFallbackAllowed(true, -1, 2));
}

TEST(PhysicalRuntimeSafety, ObservationBeforeArmMustBeValidatedByArm) {
  safety::Mode5IdentityGate gate;
  std::atomic<std::uint8_t> current_mode{
      sonic::mode5_contract::kModeMachine};
  std::atomic<std::uint8_t> current_pr{sonic::mode5_contract::kModePr};
  std::promise<void> observation_is_publishing;
  std::promise<void> release_observation;
  std::atomic<bool> mismatch_latched{false};
  const std::shared_future<void> release =
      release_observation.get_future().share();

  auto observation = std::async(std::launch::async, [&]() {
    return gate.PublishAndCheck(
        static_cast<std::uint8_t>(sonic::mode5_contract::kModeMachine + 1),
        sonic::mode5_contract::kModePr, [&]() {
          current_mode.store(
              static_cast<std::uint8_t>(
                  sonic::mode5_contract::kModeMachine + 1),
              std::memory_order_release);
          observation_is_publishing.set_value();
          release.wait();
        },
        [&]() { mismatch_latched.store(true, std::memory_order_release); });
  });
  observation_is_publishing.get_future().wait();

  std::promise<void> arm_started;
  auto arm = std::async(std::launch::async, [&]() {
    arm_started.set_value();
    return gate.ArmIf([&]() {
      return safety::MatchesMode5LowStateIdentity(
          current_mode.load(std::memory_order_acquire),
          current_pr.load(std::memory_order_acquire));
    });
  });
  arm_started.get_future().wait();
  release_observation.set_value();

  // The mismatch arrived while unarmed, so it does not itself report a
  // post-arm divergence; critically, the serialized arm sees it and refuses.
  EXPECT_FALSE(observation.get());
  EXPECT_FALSE(mismatch_latched.load(std::memory_order_acquire));
  EXPECT_FALSE(arm.get());
  EXPECT_FALSE(gate.IsArmed());
}

TEST(PhysicalRuntimeSafety, ObservationAfterArmMustReportDivergence) {
  safety::Mode5IdentityGate gate;
  std::promise<void> arm_is_validating;
  std::promise<void> release_arm;
  const std::shared_future<void> release = release_arm.get_future().share();

  auto arm = std::async(std::launch::async, [&]() {
    return gate.ArmIf([&]() {
      arm_is_validating.set_value();
      release.wait();
      return true;
    });
  });
  arm_is_validating.get_future().wait();

  std::promise<void> observation_started;
  std::atomic<bool> mismatch_latched{false};
  std::atomic<int> event_order{0};
  std::atomic<int> latch_order{0};
  std::atomic<int> publish_order{0};
  auto observation = std::async(std::launch::async, [&]() {
    observation_started.set_value();
    return gate.PublishAndCheck(
        sonic::mode5_contract::kModeMachine,
        static_cast<std::uint8_t>(sonic::mode5_contract::kModePr + 1),
        [&]() {
          publish_order.store(event_order.fetch_add(1) + 1,
                              std::memory_order_release);
        },
        [&]() {
          mismatch_latched.store(true, std::memory_order_release);
          latch_order.store(event_order.fetch_add(1) + 1,
                            std::memory_order_release);
        });
  });
  observation_started.get_future().wait();
  release_arm.set_value();

  EXPECT_TRUE(arm.get());
  EXPECT_TRUE(observation.get());
  EXPECT_TRUE(mismatch_latched.load(std::memory_order_acquire));
  EXPECT_EQ(latch_order.load(std::memory_order_acquire), 1);
  EXPECT_EQ(publish_order.load(std::memory_order_acquire), 2);
  EXPECT_TRUE(gate.IsArmed());
}

TEST(PhysicalRuntimeSafety, AcceptedWindowHasBoundedStartupGrace) {
  using namespace std::chrono_literals;
  const auto now = std::chrono::steady_clock::time_point{10s};
  constexpr auto maximum_age = 400ms;
  constexpr auto startup_grace = 400ms;

  EXPECT_EQ(safety::EvaluateAcceptedWindow(
                true, std::nullopt, now - 399ms, now, maximum_age,
                startup_grace),
            safety::AcceptedWindowState::STARTUP_GRACE);
  EXPECT_EQ(safety::EvaluateAcceptedWindow(
                true, std::nullopt, now - 401ms, now, maximum_age,
                startup_grace),
            safety::AcceptedWindowState::MISSING);
  EXPECT_EQ(safety::EvaluateAcceptedWindow(
                true, now - 400ms, now - 5s, now, maximum_age,
                startup_grace),
            safety::AcceptedWindowState::CURRENT);
  EXPECT_EQ(safety::EvaluateAcceptedWindow(
                true, now - 401ms, now - 5s, now, maximum_age,
                startup_grace),
            safety::AcceptedWindowState::STALE);
  EXPECT_EQ(safety::EvaluateAcceptedWindow(
                true, now + 1ms, now - 5s, now, maximum_age,
                startup_grace),
            safety::AcceptedWindowState::STALE);
  EXPECT_EQ(safety::EvaluateAcceptedWindow(
                false, now, now, now, maximum_age, startup_grace),
            safety::AcceptedWindowState::MISSING);
}

TEST(PhysicalRuntimeSafety, PhysicalControlRequiresCurrentExactEpisodeMotion) {
  EXPECT_TRUE(safety::ReadyForPhysicalControl(
      true, true, safety::AcceptedWindowState::CURRENT));
  EXPECT_FALSE(safety::ReadyForPhysicalControl(
      false, true, safety::AcceptedWindowState::CURRENT));
  EXPECT_FALSE(safety::ReadyForPhysicalControl(
      true, false, safety::AcceptedWindowState::CURRENT));
  EXPECT_FALSE(safety::ReadyForPhysicalControl(
      true, true, safety::AcceptedWindowState::STARTUP_GRACE));
  EXPECT_FALSE(safety::ReadyForPhysicalControl(
      true, true, safety::AcceptedWindowState::MISSING));
  EXPECT_FALSE(safety::ReadyForPhysicalControl(
      true, true, safety::AcceptedWindowState::STALE));
}

TEST(PhysicalRuntimeSafety, DampingSelectionNeverWaitsForActiveCommandBuffer) {
  using namespace std::chrono_literals;
  std::mutex active_command_mutex;
  std::promise<void> producer_holds_buffer;
  std::promise<void> release_producer;
  const std::shared_future<void> release =
      release_producer.get_future().share();

  auto producer = std::async(std::launch::async, [&]() {
    std::lock_guard<std::mutex> lock(active_command_mutex);
    producer_holds_buffer.set_value();
    release.wait();
  });
  producer_holds_buffer.get_future().wait();

  std::atomic<bool> active_reader_called{false};
  auto selection = std::async(std::launch::async, [&]() {
    return safety::ReadActiveCommandUnlessDamping(true, [&]() {
      active_reader_called.store(true, std::memory_order_release);
      std::lock_guard<std::mutex> lock(active_command_mutex);
      return 7;
    });
  });

  const bool completed_without_buffer =
      selection.wait_for(250ms) == std::future_status::ready;
  release_producer.set_value();
  producer.get();
  ASSERT_TRUE(completed_without_buffer);
  EXPECT_FALSE(selection.get().has_value());
  EXPECT_FALSE(active_reader_called.load(std::memory_order_acquire));
}

TEST(PhysicalRuntimeSafety, ActiveSelectionReadsAvailableCommand) {
  int reads = 0;
  const auto selection = safety::ReadActiveCommandUnlessDamping(
      false, [&]() {
        ++reads;
        return 42;
      });
  ASSERT_TRUE(selection.has_value());
  EXPECT_EQ(*selection, 42);
  EXPECT_EQ(reads, 1);
}

TEST(PhysicalRuntimeSafety,
     FaultCanLatchWhileWriterWaitsForActiveCommandSnapshot) {
  using namespace std::chrono_literals;
  std::mutex publish_mutex;
  std::mutex active_command_mutex;
  std::promise<void> producer_holds_buffer;
  std::promise<void> release_producer;
  const std::shared_future<void> release =
      release_producer.get_future().share();

  auto producer = std::async(std::launch::async, [&]() {
    std::lock_guard<std::mutex> lock(active_command_mutex);
    producer_holds_buffer.set_value();
    release.wait();
  });
  producer_holds_buffer.get_future().wait();

  std::atomic<bool> force_damping{false};
  std::promise<void> writer_started_active_read;
  auto writer = std::async(std::launch::async, [&]() {
    return safety::PrefetchActiveCommandThenCommitLocked(
        false, publish_mutex,
        [&]() {
          writer_started_active_read.set_value();
          std::lock_guard<std::mutex> lock(active_command_mutex);
          return 7;
        },
        [&](const std::optional<int>& prefetched_active) {
          if (force_damping.load(std::memory_order_acquire)) {
            return std::optional<int>{};
          }
          return prefetched_active;
        });
  });
  writer_started_active_read.get_future().wait();

  auto fault = std::async(std::launch::async, [&]() {
    std::lock_guard<std::mutex> lock(publish_mutex);
    force_damping.store(true, std::memory_order_release);
  });
  const bool latch_completed_while_writer_was_blocked =
      fault.wait_for(250ms) == std::future_status::ready;

  // Always release and join before asserting, so a regressed lock order fails
  // deterministically instead of leaving blocked async destructors behind.
  release_producer.set_value();
  producer.get();
  const auto selected_command = writer.get();
  fault.get();

  ASSERT_TRUE(latch_completed_while_writer_was_blocked);
  EXPECT_FALSE(selected_command.has_value());
}

TEST(PhysicalRuntimeSafety,
     FaultCanLatchWhileWriterWaitsForHandSnapshot) {
  using namespace std::chrono_literals;
  struct ActiveSnapshots {
    int body;
    int hand;
  };

  std::mutex publish_mutex;
  std::mutex hand_buffer_mutex;
  std::promise<void> producer_holds_hand_buffer;
  std::promise<void> release_producer;
  const std::shared_future<void> release =
      release_producer.get_future().share();

  auto producer = std::async(std::launch::async, [&]() {
    std::lock_guard<std::mutex> lock(hand_buffer_mutex);
    producer_holds_hand_buffer.set_value();
    release.wait();
  });
  producer_holds_hand_buffer.get_future().wait();

  std::atomic<bool> force_damping{false};
  std::promise<void> writer_started_hand_read;
  auto writer = std::async(std::launch::async, [&]() {
    return safety::PrefetchActiveCommandThenCommitLocked(
        false, publish_mutex,
        [&]() {
          const int body_snapshot = 7;
          writer_started_hand_read.set_value();
          std::lock_guard<std::mutex> lock(hand_buffer_mutex);
          return ActiveSnapshots{body_snapshot, 9};
        },
        [&](const std::optional<ActiveSnapshots>& prefetched_active) {
          return !force_damping.load(std::memory_order_acquire) &&
                 prefetched_active.has_value();
        });
  });
  writer_started_hand_read.get_future().wait();

  auto fault = std::async(std::launch::async, [&]() {
    std::lock_guard<std::mutex> lock(publish_mutex);
    force_damping.store(true, std::memory_order_release);
  });
  const bool latch_completed_while_hand_read_was_blocked =
      fault.wait_for(250ms) == std::future_status::ready;

  // Join every actor before asserting so a lock-order regression reports a
  // test failure rather than hanging in an async future destructor.
  release_producer.set_value();
  producer.get();
  const bool selected_active_command = writer.get();
  fault.get();

  ASSERT_TRUE(latch_completed_while_hand_read_was_blocked);
  EXPECT_FALSE(selected_active_command);
}

TEST(PhysicalRuntimeSafety, Dex3ActiveSnapshotRequiresBothCommands) {
  Dex3Hands::WriteSnapshot snapshot;
  EXPECT_FALSE(snapshot.HasActiveCommands());

  snapshot.left_command = std::make_shared<Dex3Hands::HandCommand>();
  EXPECT_FALSE(snapshot.HasActiveCommands());

  snapshot.right_command = std::make_shared<Dex3Hands::HandCommand>();
  EXPECT_TRUE(snapshot.HasActiveCommands());

  snapshot.left_command.reset();
  EXPECT_FALSE(snapshot.HasActiveCommands());
}

TEST(PhysicalRuntimeSafety, PhysicalStreamBindingRequiresZmqModeAndEpisode) {
  EXPECT_TRUE(safety::HasCompletePhysicalStreamBinding(true, 0, true));
  EXPECT_TRUE(safety::HasCompletePhysicalStreamBinding(true, 2, true));
  EXPECT_FALSE(safety::HasCompletePhysicalStreamBinding(false, 0, true));
  EXPECT_FALSE(safety::HasCompletePhysicalStreamBinding(true, -1, true));
  EXPECT_FALSE(safety::HasCompletePhysicalStreamBinding(true, 0, false));
}

// ---------------------------------------------------------------------------
// Opt-in physical-safety controls: measured joint-position range and per-step
// command envelope. Both default OFF in the controller; these tests cover the
// pure decision logic, not the flag plumbing.
// ---------------------------------------------------------------------------

TEST(PhysicalRuntimeSafety, MeasuredPositionRangeIsInclusiveAtBothLimits) {
  EXPECT_FALSE(safety::MeasuredPositionOutsideHardRange(0.0, -1.0, 1.0));
  // A joint resting exactly on a limit has not exceeded it.
  EXPECT_FALSE(safety::MeasuredPositionOutsideHardRange(-1.0, -1.0, 1.0));
  EXPECT_FALSE(safety::MeasuredPositionOutsideHardRange(1.0, -1.0, 1.0));
  EXPECT_TRUE(safety::MeasuredPositionOutsideHardRange(
      std::nextafter(-1.0, -2.0), -1.0, 1.0));
  EXPECT_TRUE(safety::MeasuredPositionOutsideHardRange(
      std::nextafter(1.0, 2.0), -1.0, 1.0));
}

// The controller must run its non-finite check BEFORE this predicate: NaN is
// reported as NONFINITE_MOTOR_STATE, not as a range violation. If this
// expectation ever flips, the caller ordering in CheckSafety() is wrong.
TEST(PhysicalRuntimeSafety, MeasuredPositionRangeTreatsNaNAsNotOutOfRange) {
  EXPECT_FALSE(safety::MeasuredPositionOutsideHardRange(
      std::numeric_limits<double>::quiet_NaN(), -1.0, 1.0));
  EXPECT_TRUE(safety::MeasuredPositionOutsideHardRange(
      std::numeric_limits<double>::infinity(), -1.0, 1.0));
  EXPECT_TRUE(safety::MeasuredPositionOutsideHardRange(
      -std::numeric_limits<double>::infinity(), -1.0, 1.0));
}

TEST(PhysicalRuntimeSafety, CommandStepLimitIsDisabledByNonPositiveBound) {
  // The envelope is opt-in: an absent or meaningless bound passes the target
  // through rather than silently bounding it to something.
  EXPECT_EQ(safety::LimitCommandStep(5.0, 0.0, 0.0), 5.0);
  EXPECT_EQ(safety::LimitCommandStep(5.0, 0.0, -0.4), 5.0);
  EXPECT_EQ(safety::LimitCommandStep(
                5.0, 0.0, std::numeric_limits<double>::quiet_NaN()),
            5.0);
}

TEST(PhysicalRuntimeSafety, CommandStepLimitPassesThroughWithoutAReference) {
  // Empty only before INIT has published its first command. The CONTROL path
  // always has a previous wire target, so this is the defensive case.
  EXPECT_EQ(safety::LimitCommandStep(5.0, std::nullopt, 0.4), 5.0);
}

TEST(PhysicalRuntimeSafety, CommandStepLimitBoundsTheStepNotTheRange) {
  EXPECT_DOUBLE_EQ(safety::LimitCommandStep(1.0, 0.0, 0.4), 0.4);
  EXPECT_DOUBLE_EQ(safety::LimitCommandStep(-1.0, 0.0, 0.4), -0.4);
  // A step exactly at the bound is permitted unchanged.
  EXPECT_DOUBLE_EQ(safety::LimitCommandStep(0.4, 0.0, 0.4), 0.4);
  EXPECT_DOUBLE_EQ(safety::LimitCommandStep(-0.4, 0.0, 0.4), -0.4);
  // Anything inside the bound is untouched.
  EXPECT_DOUBLE_EQ(safety::LimitCommandStep(0.1, 0.0, 0.4), 0.1);
}

// The limiter bounds distance travelled per step, which says nothing about
// where that step lands. CreatePolicyCommand() therefore clamps to the hard
// joint range AFTER limiting; this documents why that order is required.
TEST(PhysicalRuntimeSafety, CommandStepLimitCanStillLandOutsideTheHardRange) {
  constexpr double kUpperLimit = 1.0;
  const double previous = 0.9;
  const double limited = safety::LimitCommandStep(5.0, previous, 0.4);
  EXPECT_DOUBLE_EQ(limited, 1.3);
  EXPECT_GT(limited, kUpperLimit);
  EXPECT_DOUBLE_EQ(std::clamp(limited, -kUpperLimit, kUpperLimit), kUpperLimit);
}

TEST(PhysicalRuntimeSafety, CommandStepLimitLeavesNonFiniteTargetsToTheCaller) {
  // Non-finite raw targets are rejected by CreatePolicyCommand() before the
  // limiter is consulted; the limiter must not manufacture a finite value that
  // would mask them.
  const double nan_value = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(std::isnan(safety::LimitCommandStep(nan_value, 0.0, 0.4)));
  EXPECT_DOUBLE_EQ(safety::LimitCommandStep(5.0, nan_value, 0.4), 5.0);
}
