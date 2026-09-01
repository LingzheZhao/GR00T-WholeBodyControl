#include <gtest/gtest.h>

#include "../include/motion_data_reader.hpp"
#include "../include/input_interface/streamed_motion_merger.hpp"

#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace {

StreamedMotionMerger::IncomingData MakeJointMotion(
    std::vector<int64_t> frame_indices, double first_position = 1.0
) {
  StreamedMotionMerger::IncomingData data;
  data.frame_indices = std::move(frame_indices);
  data.num_frames = static_cast<int>(data.frame_indices.size());
  data.num_joints = 1;
  data.num_quat_bodies = 1;
  data.protocol_version = 1;
  data.joint_pos.resize(data.num_frames, std::vector<double>(1));
  data.joint_vel.resize(data.num_frames, std::vector<double>(1));
  data.body_quat.resize(
      data.num_frames,
      std::vector<std::array<double, 4>>(1, {1.0, 0.0, 0.0, 0.0}));
  for (int frame = 0; frame < data.num_frames; ++frame) {
    data.joint_pos[frame][0] = first_position + frame;
    data.joint_vel[frame][0] = -(first_position + frame);
  }
  return data;
}

TEST(StreamedMotionMerger, OneFramePacketUsesUnitStride) {
  StreamedMotionMerger merger;
  const auto result = merger.MergeIncomingData(
      MakeJointMotion({std::numeric_limits<int>::max()}, 7.0), 0);

  ASSERT_NE(result.motion, nullptr);
  EXPECT_TRUE(result.did_catchup_reset);
  EXPECT_EQ(result.window_start, std::numeric_limits<int>::max());
  EXPECT_EQ(result.frame_step, 1);
  EXPECT_EQ(result.motion->timesteps, 1);
  EXPECT_DOUBLE_EQ(result.motion->JointPositions(0)[0], 7.0);
  EXPECT_DOUBLE_EQ(result.motion->JointVelocities(0)[0], -7.0);
}

TEST(StreamedMotionMerger, OrdinaryStrideExtendsAlignedWindow) {
  StreamedMotionMerger merger;
  const auto first = merger.MergeIncomingData(
      MakeJointMotion({100, 102, 104}, 1.0), 0);
  ASSERT_NE(first.motion, nullptr);
  ASSERT_EQ(first.motion->timesteps, 3);
  EXPECT_TRUE(first.did_catchup_reset);
  EXPECT_EQ(first.frame_step, 2);

  const auto second = merger.MergeIncomingData(
      MakeJointMotion({104, 106, 108}, 30.0), 0);
  ASSERT_NE(second.motion, nullptr);
  EXPECT_FALSE(second.did_catchup_reset);
  EXPECT_EQ(second.window_start, 100);
  EXPECT_EQ(second.frame_step, 2);
  ASSERT_EQ(second.motion->timesteps, 5);
  EXPECT_DOUBLE_EQ(second.motion->JointPositions(0)[0], 1.0);
  EXPECT_DOUBLE_EQ(second.motion->JointPositions(1)[0], 2.0);
  EXPECT_DOUBLE_EQ(second.motion->JointPositions(2)[0], 30.0);
  EXPECT_DOUBLE_EQ(second.motion->JointPositions(3)[0], 31.0);
  EXPECT_DOUBLE_EQ(second.motion->JointPositions(4)[0], 32.0);
}

TEST(StreamedMotionMerger, ExtremeStrideMisalignmentResetsWithoutOverflow) {
  StreamedMotionMerger merger;
  constexpr int64_t kMaximum = std::numeric_limits<int>::max();
  constexpr int64_t kExtremeStride = kMaximum - 1;

  const auto first = merger.MergeIncomingData(
      MakeJointMotion({0, kExtremeStride}, 10.0), 0);
  ASSERT_NE(first.motion, nullptr);
  EXPECT_EQ(first.window_start, 0);
  EXPECT_EQ(first.frame_step, static_cast<int>(kExtremeStride));
  ASSERT_EQ(first.motion->timesteps, 2);

  // Computing old_end + step in int would overflow here.  The new chunk is
  // individually valid and has the same extreme stride, but is offset by one
  // tick from the buffered lattice, so the safe behavior is an explicit reset.
  const auto second = merger.MergeIncomingData(
      MakeJointMotion({1, kMaximum}, 20.0), 0);
  ASSERT_NE(second.motion, nullptr);
  EXPECT_TRUE(second.did_catchup_reset);
  EXPECT_EQ(second.window_start, 1);
  EXPECT_EQ(second.frame_step, static_cast<int>(kExtremeStride));
  ASSERT_EQ(second.motion->timesteps, 2);
  EXPECT_DOUBLE_EQ(second.motion->JointPositions(0)[0], 20.0);
  EXPECT_DOUBLE_EQ(second.motion->JointPositions(1)[0], 21.0);
}

TEST(StreamedMotionMerger, RejectsMalformedFrameIndexStructure) {
  const std::vector<std::vector<int64_t>> invalid_indices = {
      {-1},
      {static_cast<int64_t>(std::numeric_limits<int>::max()) + 1},
      {5, 5},
      {0, 2, 5},
  };

  for (const auto& indices : invalid_indices) {
    StreamedMotionMerger merger;
    EXPECT_EQ(merger.MergeIncomingData(MakeJointMotion(indices), 0).motion,
              nullptr);
  }
}

}  // namespace
