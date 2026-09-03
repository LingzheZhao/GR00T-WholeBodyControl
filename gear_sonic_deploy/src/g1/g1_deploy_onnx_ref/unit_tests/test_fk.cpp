#include <gtest/gtest.h>

#include "../include/fk.hpp"
#include "../include/motion_data_reader.hpp"
#include "../include/mode5_contract_generated.hpp"
#include "fixtures/fk_mujoco_golden.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <numeric>
#include <string>
#include <vector>

namespace {

std::filesystem::path CanonicalMjcf() {
#ifndef SONIC_WBC_ROOT
#error "SONIC_WBC_ROOT must identify the checked WholeBodyControl checkout"
#endif
    return std::filesystem::path(SONIC_WBC_ROOT) /
           std::string(sonic::mode5_contract::kFkMjcfPathFromWbcRoot);
}

}  // namespace

TEST(FK, RejectsIncompatibleSequenceBeforeComputation) {
    const auto canonical_mjcf = CanonicalMjcf();
    ASSERT_TRUE(std::filesystem::is_regular_file(canonical_mjcf))
        << "canonical contract MJCF is unavailable: " << canonical_mjcf;
    RobotFK fk(canonical_mjcf.string());
    ASSERT_EQ(fk.NumJoints(), 30);

    MotionSequence undersized;
    undersized.ReserveCapacity(1, 28, 1, 1);
    undersized.timesteps = 1;
    undersized.SetBodyPartIndexes({0});
    EXPECT_THROW(undersized.ComputeFK(fk), std::invalid_argument);
}

TEST(FK, RejectsBodyMappingWithoutRoot) {
    RobotFK fk(CanonicalMjcf().string());
    MotionSequence sequence;
    sequence.ReserveCapacity(1, 29, 1, 1);
    sequence.timesteps = 1;
    sequence.SetBodyPartIndexes({1});
    EXPECT_THROW(sequence.ComputeFK(fk), std::invalid_argument);
}

TEST(FK, RejectsBodyMappingWhenRootIsNotFirst) {
    RobotFK fk(CanonicalMjcf().string());
    MotionSequence sequence;
    sequence.ReserveCapacity(1, 29, 2, 2);
    sequence.timesteps = 1;
    sequence.SetBodyPartIndexes({1, 0});
    EXPECT_THROW(sequence.ComputeFK(fk), std::invalid_argument);
}

TEST(FK, RejectsTimestepsBeyondBackingStorage) {
    RobotFK fk(CanonicalMjcf().string());
    MotionSequence sequence;
    sequence.ReserveCapacity(1, 29, 1, 1);
    sequence.timesteps = 2;
    sequence.SetBodyPartIndexes({0});
    EXPECT_THROW(sequence.ComputeFK(fk), std::invalid_argument);
}

TEST(FK, RejectsNegativeTimesteps) {
    RobotFK fk(CanonicalMjcf().string());
    MotionSequence sequence;
    sequence.ReserveCapacity(1, 29, 1, 1);
    sequence.timesteps = -1;
    sequence.SetBodyPartIndexes({0});
    EXPECT_THROW(sequence.ComputeFK(fk), std::invalid_argument);
}

TEST(FK, MatchesIndependentMujocoGolden) {
    const auto canonical_mjcf = CanonicalMjcf();
    ASSERT_TRUE(std::filesystem::is_regular_file(canonical_mjcf))
        << "canonical contract MJCF is unavailable: " << canonical_mjcf;
    RobotFK fk(canonical_mjcf.string());
    ASSERT_EQ(fk.NumJoints(), fk_mujoco_golden::kBodyNames.size());

    std::array<MotionSequence::Point, 30> positions{};
    std::array<MotionSequence::Quaternion, 30> quaternions{};
    for (size_t frame_index = 0;
         frame_index < fk_mujoco_golden::kFrames.size(); ++frame_index) {
        const auto& frame = fk_mujoco_golden::kFrames[frame_index];
        fk.DoFK(
            positions.data(), quaternions.data(), frame.root_translation,
            frame.root_quaternion, frame.policy_joint_positions.data());

        for (size_t body = 0; body < positions.size(); ++body) {
            SCOPED_TRACE(
                "frame=" + std::to_string(frame_index) + " body=" +
                std::string(fk_mujoco_golden::kBodyNames[body]));
            for (size_t axis = 0; axis < 3; ++axis) {
                EXPECT_NEAR(
                    positions[body][axis],
                    frame.body_positions[body * 3 + axis], 2e-6);
            }

            double quaternion_dot = 0.0;
            for (size_t component = 0; component < 4; ++component) {
                quaternion_dot +=
                    quaternions[body][component] *
                    frame.body_quaternions[body * 4 + component];
            }
            const double expected_sign = quaternion_dot < 0.0 ? -1.0 : 1.0;
            for (size_t component = 0; component < 4; ++component) {
                EXPECT_NEAR(
                    quaternions[body][component],
                    expected_sign *
                        frame.body_quaternions[body * 4 + component],
                    2e-6);
            }
        }
    }
}

TEST(FK, MotionSequenceRemapMatchesIndependentMujocoGolden) {
    RobotFK fk(CanonicalMjcf().string());
    ASSERT_EQ(fk.NumJoints(), fk_mujoco_golden::kBodyNames.size());

    MotionSequence sequence;
    sequence.ReserveCapacity(1, 29, 30, 30);
    sequence.timesteps = 1;
    std::vector<int> body_indexes(30);
    std::iota(body_indexes.begin(), body_indexes.end(), 0);
    sequence.SetBodyPartIndexes(body_indexes);

    const auto& frame = fk_mujoco_golden::kFrames.front();
    sequence.BodyPositions(0)[0] = frame.root_translation;
    sequence.BodyQuaternions(0)[0] = frame.root_quaternion;
    std::copy(frame.policy_joint_positions.begin(),
              frame.policy_joint_positions.end(),
              sequence.JointPositions(0));
    ASSERT_NO_THROW(sequence.ComputeFK(fk));

    for (size_t isaac_body = 0; isaac_body < body_indexes.size(); ++isaac_body) {
        const size_t mujoco_body = isaac_body == 0
            ? 0
            : static_cast<size_t>(mujoco_to_isaaclab[isaac_body - 1]) + 1;
        SCOPED_TRACE(
            "isaac_body=" + std::to_string(isaac_body) + " mujoco_body=" +
            std::string(fk_mujoco_golden::kBodyNames[mujoco_body]));
        for (size_t axis = 0; axis < 3; ++axis) {
            EXPECT_NEAR(
                sequence.BodyPositions(0)[isaac_body][axis],
                frame.body_positions[mujoco_body * 3 + axis], 2e-6);
        }

        double quaternion_dot = 0.0;
        for (size_t component = 0; component < 4; ++component) {
            quaternion_dot +=
                sequence.BodyQuaternions(0)[isaac_body][component] *
                frame.body_quaternions[mujoco_body * 4 + component];
        }
        const double expected_sign = quaternion_dot < 0.0 ? -1.0 : 1.0;
        for (size_t component = 0; component < 4; ++component) {
            EXPECT_NEAR(
                sequence.BodyQuaternions(0)[isaac_body][component],
                expected_sign *
                    frame.body_quaternions[mujoco_body * 4 + component],
                2e-6);
        }
    }
}

TEST(FK, ComputesDocumentedUnfilteredVelocityDifferences) {
    MotionSequence sequence;
    sequence.ReserveCapacity(3, 29, 1, 1);
    sequence.timesteps = 3;
    sequence.SetBodyPartIndexes({0});

    const std::array<MotionSequence::Point, 3> positions = {{
        {0.0, 0.0, 0.0},
        {0.02, -0.04, 0.06},
        {0.08, -0.02, 0.02},
    }};
    const std::array<double, 3> yaw = {0.0, 0.02, 0.05};
    for (int frame = 0; frame < sequence.timesteps; ++frame) {
        sequence.BodyPositions(frame)[0] = positions[frame];
        sequence.BodyQuaternions(frame)[0] = {
            std::cos(yaw[frame] / 2.0), 0.0, 0.0,
            std::sin(yaw[frame] / 2.0)};
    }

    sequence.ComputeGlobalVelocities(false);
    const std::array<MotionSequence::Velocity, 3> expected_linear = {{
        {1.0, -2.0, 3.0},
        {2.0, -0.5, 0.5},
        {3.0, 1.0, -2.0},
    }};
    const std::array<double, 3> expected_angular_z = {1.0, 1.5, 1.5};
    for (int frame = 0; frame < sequence.timesteps; ++frame) {
        for (size_t axis = 0; axis < 3; ++axis) {
            EXPECT_NEAR(
                sequence.BodyLinVelocities(frame)[0][axis],
                expected_linear[frame][axis], 1e-12);
        }
        EXPECT_NEAR(sequence.BodyAngVelocities(frame)[0][0], 0.0, 1e-12);
        EXPECT_NEAR(sequence.BodyAngVelocities(frame)[0][1], 0.0, 1e-12);
        EXPECT_NEAR(
            sequence.BodyAngVelocities(frame)[0][2],
            expected_angular_z[frame], 1e-12);
    }
}

TEST(FK, FilteringPreservesAConstantAnalyticVelocity) {
    MotionSequence sequence;
    sequence.ReserveCapacity(5, 29, 1, 1);
    sequence.timesteps = 5;
    sequence.SetBodyPartIndexes({0});
    for (int frame = 0; frame < sequence.timesteps; ++frame) {
        const double yaw = 0.01 * frame;
        sequence.BodyPositions(frame)[0] = {
            0.02 * frame, -0.01 * frame, 0.03 * frame};
        sequence.BodyQuaternions(frame)[0] = {
            std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0)};
    }

    sequence.ComputeGlobalVelocities();
    for (int frame = 0; frame < sequence.timesteps; ++frame) {
        EXPECT_NEAR(sequence.BodyLinVelocities(frame)[0][0], 1.0, 1e-12);
        EXPECT_NEAR(sequence.BodyLinVelocities(frame)[0][1], -0.5, 1e-12);
        EXPECT_NEAR(sequence.BodyLinVelocities(frame)[0][2], 1.5, 1e-12);
        EXPECT_NEAR(sequence.BodyAngVelocities(frame)[0][0], 0.0, 1e-12);
        EXPECT_NEAR(sequence.BodyAngVelocities(frame)[0][1], 0.0, 1e-12);
        EXPECT_NEAR(sequence.BodyAngVelocities(frame)[0][2], 0.5, 1e-12);
    }
}
