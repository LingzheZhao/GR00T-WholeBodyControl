#include <gtest/gtest.h>

#include "../include/physical_runtime_safety.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
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

TEST(PhysicalRuntimeSafety, ObservationBeforeArmMustBeValidatedByArm) {
  safety::Mode5IdentityGate gate;
  std::atomic<std::uint8_t> current_mode{
      sonic::mode5_contract::kModeMachine};
  std::atomic<std::uint8_t> current_pr{sonic::mode5_contract::kModePr};
  std::promise<void> observation_is_publishing;
  std::promise<void> release_observation;
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
        });
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
  auto observation = std::async(std::launch::async, [&]() {
    observation_started.set_value();
    return gate.PublishAndCheck(
        sonic::mode5_contract::kModeMachine,
        static_cast<std::uint8_t>(sonic::mode5_contract::kModePr + 1),
        []() {});
  });
  observation_started.get_future().wait();
  release_arm.set_value();

  EXPECT_TRUE(arm.get());
  EXPECT_TRUE(observation.get());
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
