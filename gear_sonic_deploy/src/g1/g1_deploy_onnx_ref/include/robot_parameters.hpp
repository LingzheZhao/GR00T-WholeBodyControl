/**
 * @file robot_parameters.hpp
 * @brief Hardware-level constants, data structures, and joint indices for the
 *        Unitree G1 humanoid robot.
 *
 * This header defines:
 *  - DDS topic names for the Unitree SDK (low-level command / state channels).
 *  - Motor count and per-joint MotorCommand structure.
 *  - HeadingState – a compact struct bundling the captured IMU quaternion
 *    with a user-controlled delta-heading offset.
 *  - OperatorState – high-level start / stop / play flags.
 *  - Control mode enum (series vs. parallel ankle actuation).
 *  - G1JointIndex enum mapping symbolic joint names to hardware motor indices.
 *
 * All indices in this file use the **hardware / URDF ordering** (not the
 * IsaacLab training ordering – see `policy_parameters.hpp` for the mapping).
 */

#ifndef ROBOT_PARAMETERS_HPP
#define ROBOT_PARAMETERS_HPP

#include <array>
#include <atomic>
#include <string>

#include "mode5_contract_generated.hpp"

// ---------------------------------------------------------------------------
// Unitree SDK DDS topic names
// ---------------------------------------------------------------------------
static const std::string HG_CMD_TOPIC = "rt/lowcmd";       ///< Low-level motor command topic.
static const std::string HG_IMU_TORSO = "rt/secondary_imu";///< Secondary (torso) IMU topic.
static const std::string HG_STATE_TOPIC = "rt/lowstate";    ///< Low-level motor / sensor state topic.

/// Total number of actuated joints on the G1 (29-DOF configuration).
inline constexpr int G1_NUM_MOTOR =
    sonic::mode5_contract::kActiveMotorCount;

/**
 * @brief Hard joint-position limits for the 29-DOF G1, in hardware / MuJoCo order.
 *
 * These values are generated from the deployment contract and mechanically
 * checked against the canonical Unitree rev1 MJCF at CMake configure time.
 * They are not training-time soft limits and carry no implicit margin.
 */
inline constexpr auto G1_JOINT_POSITION_LOWER_LIMITS =
    sonic::mode5_contract::kPositionLower;
inline constexpr auto G1_JOINT_POSITION_UPPER_LIMITS =
    sonic::mode5_contract::kPositionUpper;
inline constexpr auto G1_JOINT_VELOCITY_LIMITS =
    sonic::mode5_contract::kPlantVelocityLimit;
inline constexpr auto G1_JOINT_EFFORT_LIMITS =
    sonic::mode5_contract::kPlantEffortLimit;

constexpr bool G1JointPositionLimitsAreValid() {
  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    if (!(G1_JOINT_POSITION_LOWER_LIMITS[i] < G1_JOINT_POSITION_UPPER_LIMITS[i])) {
      return false;
    }
  }
  return true;
}

static_assert(G1JointPositionLimitsAreValid(), "G1 joint-position limits must be ordered");

/**
 * @brief Per-joint motor command sent to the low-level controller.
 *
 * Each field is an array of size G1_NUM_MOTOR (29), indexed by hardware joint
 * index (see G1JointIndex).
 */
struct MotorCommand {
    std::array<float, G1_NUM_MOTOR> q_target = {};   ///< Target position (rad).
    std::array<float, G1_NUM_MOTOR> dq_target = {};  ///< Target velocity (rad/s).
    std::array<float, G1_NUM_MOTOR> kp = {};          ///< Position gain (Nm/rad).
    std::array<float, G1_NUM_MOTOR> kd = {};          ///< Velocity gain (Nm·s/rad).
    std::array<float, G1_NUM_MOTOR> tau_ff = {};      ///< Feed-forward torque (Nm).
};

/**
 * @brief Bundled heading state for thread-safe access via DataBuffer.
 *
 * Captures both the initial IMU base quaternion (set when heading is
 * reinitialised) and a user-adjustable delta heading offset (adjusted via
 * keyboard Q/E or D-pad).
 */
struct HeadingState {
    std::array<double, 4> init_base_quat;  ///< Captured IMU base quaternion (w,x,y,z) at init.
    double delta_heading;                   ///< Cumulative heading offset (radians).
    
    HeadingState(const std::array<double, 4>& quat = {1.0, 0.0, 0.0, 0.0}, double delta = 0.0)
        : init_base_quat(quat), delta_heading(delta) {}
};

/**
 * @brief High-level operator signals (set by input interfaces, read by control loop).
 */
struct OperatorState {
  // These flags are written by the 100 Hz input thread and read by the
  // control thread and main thread.  Plain bools make those reads/writes a
  // C++ data race, including the emergency-stop path.  Atomics keep the
  // existing assignment/boolean syntax while making the hand-off defined.
  std::atomic<bool> stop{false};   ///< Emergency stop requested.
  std::atomic<bool> start{false};  ///< Control-system start requested.
  std::atomic<bool> play{false};   ///< Motion playback active.
  /// Resident re-stand: ramp back to the INIT pose without damping or
  /// releasing LowCmd.  Consumed by the 50 Hz control thread.
  std::atomic<bool> rest_stand{false};
};

/**
 * @brief Ankle actuation mode.
 *
 * The G1's ankle uses a coupled 2-DOF mechanism that can be controlled
 * in series (pitch/roll) or parallel (A/B motor) modes.
 */
enum class Mode {
  PR = 0, ///< Series control for Pitch / Roll joints.
  AB = 1  ///< Parallel control for A / B motors.
};

/**
 * @brief Symbolic names for G1 hardware joint indices (0–28).
 *
 * Ankle joints have dual names because they can be addressed in either
 * series (Pitch/Roll) or parallel (A/B) mode.  Joints marked "INVALID"
 * are not present on the 23-DOF or waist-locked 29-DOF variants.
 */
enum G1JointIndex {
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleB = 4,
  LeftAnkleRoll = 5,
  LeftAnkleA = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleB = 10,
  RightAnkleRoll = 11,
  RightAnkleA = 11,
  WaistYaw = 12,
  WaistRoll = 13, // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistA = 13, // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistPitch = 14, // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistB = 14, // NOTE INVALID for g1 23dof/29dof with waist locked
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20, // NOTE INVALID for g1 23dof
  LeftWristYaw = 21, // NOTE INVALID for g1 23dof
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27, // NOTE INVALID for g1 23dof
  RightWristYaw = 28 // NOTE INVALID for g1 23dof
};

#endif // ROBOT_PARAMETERS_HPP
