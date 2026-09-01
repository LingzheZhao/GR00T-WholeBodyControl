#include <gtest/gtest.h>

#include "../include/mode5_contract_generated.hpp"
#include "../include/policy_parameters.hpp"
#include "../include/robot_parameters.hpp"

#include <array>
#include <bit>
#include <cstdint>

namespace contract = sonic::mode5_contract;

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
