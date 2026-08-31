/**
 * @file policy_parameters.hpp
 * @brief Compatibility aliases for the generated G1 deployment contract.
 *
 * ## Joint Ordering
 *
 * Two ordering conventions coexist in the codebase:
 *  - **MuJoCo / hardware order** – used by the simulator, LowState / LowCmd,
 *    and output interfaces.  Joint indices follow the URDF kinematic tree.
 *  - **IsaacLab / policy order** – used by the released RL policy and by the
 *    reference-motion CSV/ZMQ wire columns.  It interleaves left/right joints
 *    differently from MuJoCo / hardware order.
 *
 * The arrays `isaaclab_to_mujoco` and `mujoco_to_isaaclab` provide the
 * remapping between the two orderings.
 *
 * Gains, action scales, maps, signs, offsets and the default pose come only
 * from mode5_contract_generated.hpp.  In particular the released policy's
 * Mode-11-trained hip-pitch gains/scales stay bit-identical while the physical
 * plant row honestly records the Mode-5 7520_14 actuator and 88 N-m ceiling.
 */

#ifndef POLICY_PARAMETERS_HPP
#define POLICY_PARAMETERS_HPP

#include <array>
#include <vector>

#include "mode5_contract_generated.hpp"


// VR5Point index (isaaclab index) left wrist, right wrist, pelvs, left ankle, right ankle
const std::array<int, 5> vr_5point_index = {28, 29, 0, 18, 19};

// Joint mapping arrays
// VR3Point index (isaaclab index) left wrist, right wrist, torso,
const std::array<int, 3> vr_3point_index = {28, 29, 9};

// Upper body joint index (mujoco order)
const std::vector<int> upper_body_joint_mujoco_order_in_isaaclab_index = { 2, 5, 8, 11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28};
const std::vector<int> upper_body_joint_mujoco_order_in_mujoco_index = { 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28};

// upper body joint index (isaaclab order)
const std::vector<int> upper_body_joint_isaaclab_order_in_isaaclab_index = { 2, 5, 8, 11, 12, 15, 16, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28};
const std::vector<int> upper_body_joint_isaaclab_order_in_mujoco_index = { 12, 13, 14, 15, 22, 16, 23, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28};

// wrist joint index (mujoco order)
const std::vector<int> wrist_joint_mujoco_order_in_isaaclab_index = {23, 25, 27, 24, 26, 28};
const std::vector<int> wrist_joint_mujoco_order_in_mujoco_index = {19, 20, 21, 26, 27, 28};

// wrist joint index (isaaclab order)
const std::vector<int> wrist_joint_isaaclab_order_in_isaaclab_index = {23, 24, 25, 26, 27, 28};
const std::vector<int> wrist_joint_isaaclab_order_in_mujoco_index = {19, 26, 20, 27, 21, 28};

// lower body joint index (mujoco order)
const std::vector<int> lower_body_joint_mujoco_order_in_isaaclab_index = {0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18};
const std::vector<int> lower_body_joint_mujoco_order_in_mujoco_index = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

// lower body joint index (isaaclab order)
const std::vector<int> lower_body_joint_isaaclab_order_in_isaaclab_index = {0, 1, 3, 4, 6, 7, 9, 10, 13, 14, 17, 18};
const std::vector<int> lower_body_joint_isaaclab_order_in_mujoco_index = {0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11};

// Historical names retained for source compatibility.  Both mappings and all
// command parameters are generated from the reviewed JSON contract.
inline constexpr auto isaaclab_to_mujoco =
    sonic::mode5_contract::kMujocoOrderInIsaaclabIndex;
inline constexpr auto mujoco_to_isaaclab =
    sonic::mode5_contract::kIsaaclabOrderInMujocoIndex;
inline constexpr auto command_sign = sonic::mode5_contract::kCommandSign;
inline constexpr auto command_offset = sonic::mode5_contract::kCommandOffset;
inline constexpr auto g1_action_scale =
    sonic::mode5_contract::kPolicyActionScale;
inline constexpr auto kps = sonic::mode5_contract::kPolicyKp;
inline constexpr auto kds = sonic::mode5_contract::kPolicyKd;
inline constexpr auto default_angles = sonic::mode5_contract::kDefaultAngles;

#endif // POLICY_PARAMETERS_HPP
