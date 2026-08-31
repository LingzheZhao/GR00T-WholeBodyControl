#include <gtest/gtest.h>

#include "../include/mode5_contract_generated.hpp"
#include "../include/policy_parameters.hpp"
#include "../include/robot_parameters.hpp"
#include "../include/shutdown_safety.hpp"

#include <array>
#include <bit>
#include <cstdint>

namespace contract = sonic::mode5_contract;

TEST(ShutdownSafety, DeadlineAtOrBeforeBoundaryRemainsAFault) {
  constexpr sonic::shutdown_safety::SteadyClockTick boundary = 1'000;
  EXPECT_TRUE(sonic::shutdown_safety::HeartbeatDeadlineRequiresFault(
      2'000, boundary - 1, boundary));
  EXPECT_TRUE(sonic::shutdown_safety::HeartbeatDeadlineRequiresFault(
      2'000, boundary, boundary));
  EXPECT_FALSE(sonic::shutdown_safety::HeartbeatDeadlineRequiresFault(
      2'000, boundary + 1, boundary));
}

TEST(ShutdownSafety, FreshIntentionalStopSuppressesOnlyLaterAging) {
  constexpr sonic::shutdown_safety::SteadyClockTick boundary = 1'000;
  // The deadline was fresh at shutdown.  Even after the 250 ms damping burst
  // ages past it, shutdown cleanup must not manufacture a watchdog fault.
  EXPECT_FALSE(sonic::shutdown_safety::HeartbeatDeadlineRequiresFault(
      50'000, boundary + 1, boundary));
  // Without a published shutdown boundary, equality and later aging fault.
  EXPECT_TRUE(sonic::shutdown_safety::HeartbeatDeadlineRequiresFault(
      boundary, boundary, sonic::shutdown_safety::kNoShutdownBoundary));
  EXPECT_TRUE(sonic::shutdown_safety::HeartbeatDeadlineRequiresFault(
      boundary + 1, boundary,
      sonic::shutdown_safety::kNoShutdownBoundary));
}

TEST(ShutdownSafety, ConcurrentRequestsRetainTheEarliestBoundary) {
  using sonic::shutdown_safety::EarliestBoundary;
  using sonic::shutdown_safety::kNoShutdownBoundary;
  EXPECT_EQ(EarliestBoundary(kNoShutdownBoundary, 20), 20);
  EXPECT_EQ(EarliestBoundary(20, 21), 20);
  EXPECT_EQ(EarliestBoundary(20, 19), 19);
}

TEST(ShutdownSafety, IndependentActuationLeaseIsFailClosedAtItsBoundary) {
  using sonic::shutdown_safety::ActuationLeaseExpired;
  using sonic::shutdown_safety::kNoShutdownBoundary;
  EXPECT_FALSE(ActuationLeaseExpired(50'000, kNoShutdownBoundary));
  EXPECT_FALSE(ActuationLeaseExpired(999, 1'000));
  EXPECT_TRUE(ActuationLeaseExpired(1'000, 1'000));
  EXPECT_TRUE(ActuationLeaseExpired(1'001, 1'000));
}

TEST(Mode5Contract, IdentityAndArtifactsArePinned) {
  EXPECT_EQ(contract::kProfileId, "sonic-g1-mode5-derived-v1");
  EXPECT_EQ(contract::kCanonicalSha256,
            "1ab7b04a98e5e32a15371c215291940d134e9978ad3d35c16f6a74bc964a1420");
  EXPECT_EQ(contract::kJointParameterSha256,
            "8c46e85155070a359dce3059a86a98b0c7cd8d8b1ba3079e2f4d97a13236237b");
  EXPECT_EQ(contract::kKinematicsSha256,
            "4b4272979a730962981652897f5035d02abbab9db4197dd29320266f564ffdc5");
  EXPECT_FALSE(contract::kVendorAttested);
  EXPECT_EQ(contract::kContractRevision, 3);
  EXPECT_EQ(contract::kCheckpointTrainingModeMachine, 11);
  EXPECT_EQ(contract::kPhysicalPlantModeMachine, 5);
  EXPECT_EQ(contract::kModeMachine, 5);
  EXPECT_EQ(contract::kModePr, 0);
  EXPECT_EQ(contract::kMotorMode, 1);
  EXPECT_EQ(contract::kActiveMotorCount, 29);
  EXPECT_EQ(contract::kCheckpointArtifacts.size(), 3u);
  EXPECT_EQ(contract::kCheckpointArtifacts[0].sha256,
            "c7241a123eaa36b5d64bad19540efde93cac1ad443bd4572fd12ca99898118ed");
  EXPECT_EQ(contract::kFkMjcfSha256,
            "15a330f1dbff68cdbeb8284be6162a857bdcd21a858f65a8617fb32d5115f209");
  EXPECT_EQ(contract::kSimMjcfSha256,
            "388d682c8ed36e833eca986109fdca2797c4d5f3882822e2b03a1af1a2675f07");
  EXPECT_EQ(contract::kSimSceneSha256,
            "88a24971e125eb20382e59217a18ee97eb0756c3e450e29a95a9e34ef397fbfc");
  EXPECT_EQ(contract::kSimConfigSha256,
            "15520caa4b5748fab451f7f34661646bfb9db9dbd06fd8ed240f507f629d01a6");
  EXPECT_EQ(contract::kSimConfigSize, 12448u);
  EXPECT_DOUBLE_EQ(contract::kPhysicsTimestepSeconds, 0.002);
  EXPECT_EQ(contract::kPhysicsRateHz, 500);
  EXPECT_EQ(contract::kPhysicsStepsPerLowcmdPeriod, 1);
  EXPECT_EQ(contract::kPhysicsRateHz,
            contract::kWriterRateHz * contract::kPhysicsStepsPerLowcmdPeriod);
  EXPECT_EQ(contract::kTorqueApplication,
            "external-pd-recomputed-once-per-mujoco-step");
  EXPECT_EQ(contract::kTakeoverSourceRepository,
            "unitreerobotics/unitree_sdk2");
  EXPECT_EQ(contract::kTakeoverSourceRevision,
            "9754cd153af3da471b0fe5f3aa535e426fb11db3");
  EXPECT_EQ(contract::kTakeoverSourcePath,
            "example/g1/low_level/g1_ankle_swing_example.cpp");
  EXPECT_EQ(contract::kTakeoverSourceSha256,
            "9394824751ee4f74d252546fe4f3c20145f9148becfbb9eeb53cbeef449c6b8b");
  EXPECT_DOUBLE_EQ(contract::kTakeoverDurationSeconds, 3.0);
}

TEST(Mode5Contract, TakeoverScheduleMatchesOfficialUnitreeExample) {
  constexpr std::array<float, 29> expected_kp{{
      60, 60, 60, 100, 40, 40,
      60, 60, 60, 100, 40, 40,
      60, 40, 40,
      40, 40, 40, 40, 40, 40, 40,
      40, 40, 40, 40, 40, 40, 40,
  }};
  constexpr std::array<float, 29> expected_kd{{
      1, 1, 1, 2, 1, 1,
      1, 1, 1, 2, 1, 1,
      1, 1, 1,
      1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1,
  }};
  EXPECT_EQ(contract::kTakeoverKp, expected_kp);
  EXPECT_EQ(contract::kTakeoverKd, expected_kd);
  for (const auto basis : contract::kTakeoverGainBasis) {
    EXPECT_EQ(basis, "unitree-sdk2-g1-ankle-swing-example");
  }
}

TEST(Mode5Contract, CompatibilityAliasesUseGeneratedRows) {
  EXPECT_EQ(G1_NUM_MOTOR, contract::kActiveMotorCount);
  EXPECT_EQ(isaaclab_to_mujoco, contract::kMujocoOrderInIsaaclabIndex);
  EXPECT_EQ(mujoco_to_isaaclab, contract::kIsaaclabOrderInMujocoIndex);
  EXPECT_EQ(g1_action_scale, contract::kPolicyActionScale);
  EXPECT_EQ(kps, contract::kPolicyKp);
  EXPECT_EQ(kds, contract::kPolicyKd);
  EXPECT_EQ(default_angles, contract::kDefaultAngles);
  EXPECT_EQ(G1_JOINT_POSITION_LOWER_LIMITS, contract::kPositionLower);
  EXPECT_EQ(G1_JOINT_POSITION_UPPER_LIMITS, contract::kPositionUpper);
  EXPECT_EQ(G1_JOINT_VELOCITY_LIMITS, contract::kPlantVelocityLimit);
  for (int hardware_index = 0; hardware_index < G1_NUM_MOTOR;
       ++hardware_index) {
    const int policy_index = isaaclab_to_mujoco[hardware_index];
    ASSERT_GE(policy_index, 0);
    ASSERT_LT(policy_index, G1_NUM_MOTOR);
    EXPECT_EQ(mujoco_to_isaaclab[policy_index], hardware_index);
    EXPECT_LE(G1_JOINT_POSITION_LOWER_LIMITS[hardware_index],
              default_angles[hardware_index]);
    EXPECT_GE(G1_JOINT_POSITION_UPPER_LIMITS[hardware_index],
              default_angles[hardware_index]);
  }
}

TEST(Mode5Contract, ReleasedPolicyArraysAreBitExact) {
  constexpr std::array<std::uint32_t, 29> expected_kp{{
      0x42c63265, 0x42c63265, 0x4220b78a, 0x42c63265, 0x41e4028d,
      0x41e4028d, 0x42c63265, 0x42c63265, 0x4220b78a, 0x42c63265,
      0x41e4028d, 0x41e4028d, 0x4220b78a, 0x41e4028d, 0x41e4028d,
      0x4164028d, 0x4164028d, 0x4164028d, 0x4164028d, 0x4164028d,
      0x41863a04, 0x41863a04, 0x4164028d, 0x4164028d, 0x4164028d,
      0x4164028d, 0x4164028d, 0x41863a04, 0x41863a04,
  }};
  constexpr std::array<std::uint32_t, 29> expected_kd{{
      0x40c9e1b4, 0x40c9e1b4, 0x4023b477, 0x40c9e1b4, 0x3fe83fc2,
      0x3fe83fc2, 0x40c9e1b4, 0x40c9e1b4, 0x4023b477, 0x40c9e1b4,
      0x3fe83fc2, 0x3fe83fc2, 0x4023b477, 0x3fe83fc2, 0x3fe83fc2,
      0x3f683fc2, 0x3f683fc2, 0x3f683fc2, 0x3f683fc2, 0x3f683fc2,
      0x3f88b8dc, 0x3f88b8dc, 0x3f683fc2, 0x3f683fc2, 0x3f683fc2,
      0x3f683fc2, 0x3f683fc2, 0x3f88b8dc, 0x3f88b8dc,
  }};
  constexpr std::array<std::uint64_t, 29> expected_scale{{
      0x3fd6713cca842003, 0x3fd6713cca842003, 0x3fe185802a245631,
      0x3fd6713cca842003, 0x3fdc11a695046077, 0x3fdc11a695046077,
      0x3fd6713cca842003, 0x3fd6713cca842003, 0x3fe185802a245631,
      0x3fd6713cca842003, 0x3fdc11a695046077, 0x3fdc11a695046077,
      0x3fe185802a245631, 0x3fdc11a695046077, 0x3fdc11a695046077,
      0x3fdc11a695046077, 0x3fdc11a695046077, 0x3fdc11a695046077,
      0x3fdc11a695046077, 0x3fdc11a695046077, 0x3fb3127d3196b9a0,
      0x3fb3127d3196b9a0, 0x3fdc11a695046077, 0x3fdc11a695046077,
      0x3fdc11a695046077, 0x3fdc11a695046077, 0x3fdc11a695046077,
      0x3fb3127d3196b9a0, 0x3fb3127d3196b9a0,
  }};
  constexpr std::array<std::uint64_t, 29> expected_default{{
      0xbfd3f7ced916872b, 0x0000000000000000, 0x0000000000000000,
      0x3fe56872b020c49c, 0xbfd73b645a1cac08, 0x0000000000000000,
      0xbfd3f7ced916872b, 0x0000000000000000, 0x0000000000000000,
      0x3fe56872b020c49c, 0xbfd73b645a1cac08, 0x0000000000000000,
      0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
      0x3fc999999999999a, 0x3fc999999999999a, 0x0000000000000000,
      0x3fe3333333333333, 0x0000000000000000, 0x0000000000000000,
      0x0000000000000000, 0x3fc999999999999a, 0xbfc999999999999a,
      0x0000000000000000, 0x3fe3333333333333, 0x0000000000000000,
      0x0000000000000000, 0x0000000000000000,
  }};

  for (int index = 0; index < G1_NUM_MOTOR; ++index) {
    EXPECT_EQ(std::bit_cast<std::uint32_t>(contract::kPolicyKp[index]),
              expected_kp[index]);
    EXPECT_EQ(std::bit_cast<std::uint32_t>(contract::kPolicyKd[index]),
              expected_kd[index]);
    EXPECT_EQ(std::bit_cast<std::uint64_t>(contract::kPolicyActionScale[index]),
              expected_scale[index]);
    EXPECT_EQ(std::bit_cast<std::uint64_t>(contract::kDefaultAngles[index]),
              expected_default[index]);
  }
}
