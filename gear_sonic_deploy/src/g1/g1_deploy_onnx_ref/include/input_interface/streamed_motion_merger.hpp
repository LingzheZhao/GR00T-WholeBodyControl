/**
 * @file streamed_motion_merger.hpp
 * @brief Reusable sliding-window merger for streamed motion data.
 *
 * StreamedMotionMerger receives chunks of motion frames (joint positions /
 * velocities, body quaternions, SMPL data) from any streaming source (ZMQ,
 * ROS2, etc.) and merges them into a single growing MotionSequence using a
 * sliding-window approach.
 *
 * ## Key Concepts
 *
 * - **Frame indices**: Each incoming chunk carries a vector of monotonically
 *   increasing integer indices that identify frames in a global timeline.
 *   The merger uses these to align new data with the existing window.
 *
 * - **Frame step**: The stride between consecutive frame indices (e.g. 2 if
 *   the sender runs at 60 Hz and the consumer at 30 Hz).  Detected
 *   automatically from the first two indices of each chunk.
 *
 * - **Sliding window**: The merger maintains a window of frames centred
 *   around the current playback position.  It keeps `HISTORY_FRAMES` past
 *   frames for smooth interpolation and appends new frames from the
 *   incoming chunk.  Old frames that fall behind the window are discarded.
 *
 * - **Catch-up reset**: If the gap between the playback position and the
 *   incoming data exceeds `MAX_GAP_FRAMES`, the merger resets the window
 *   to the start of the incoming chunk and signals the caller to reset
 *   the playback cursor to frame 0.  This prevents unbounded buffering
 *   when the network falls behind.
 *
 * - **Protocol versions**: The merger itself is version-agnostic – it
 *   merges whatever data fields are present in IncomingData.  Protocol-
 *   version validation (rejecting changes mid-session, etc.) is left to
 *   the caller (e.g. ZMQEndpointInterface).
 *
 * ## Thread Safety
 *
 * The merger is **not** thread-safe.  All calls must be serialised by the
 * caller (typically by holding the data_mutex_ in ZMQEndpointInterface).
 */

#ifndef STREAMED_MOTION_MERGER_HPP
#define STREAMED_MOTION_MERGER_HPP

#include <memory>
#include <vector>
#include <array>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <iostream>
#include <iomanip>
#include <cstring>

// Forward declaration – MotionSequence is defined in motion_data_reader.hpp.
struct MotionSequence;

/**
 * @class StreamedMotionMerger
 * @brief Merges incoming motion-frame chunks into a sliding-window
 *        MotionSequence for real-time playback.
 */
class StreamedMotionMerger {
public:
    /// Compile-time toggle for debug log output.
    static constexpr bool DEBUG_LOGGING = false;
    /// Number of already-consumed frames to retain before the playback cursor
    /// (provides look-back for interpolation / blending).
    static constexpr int HISTORY_FRAMES = 5;
    /// Maximum tolerated gap (in current-rate frames) before a catch-up reset.
    static constexpr int MAX_GAP_FRAMES = 200;
    /// Fixed MotionSequence backing capacity used by the streaming path.
    static constexpr int MAX_MOTION_FRAMES = 15000;
    
    /// Returned by MergeIncomingData() to communicate what happened.
    struct MergeResult {
        std::shared_ptr<MotionSequence> motion;  ///< Merged motion (nullptr on failure).
        int window_start = 0;                    ///< Global frame index of motion[0].
        int frame_offset_adjustment = 0;         ///< Subtract from current_frame to compensate for window shift.
        bool did_catchup_reset = false;           ///< True → caller should reset playback to frame 0.
        int frame_step = 1;                       ///< Detected stride between consecutive frame indices.
        int protocol_version = 0;                 ///< Protocol version of the incoming data (1, 2, or 3).
    };
    
    /// All the data needed for one merge operation, decoded by the caller.
    struct IncomingData {
        // -- Joint data (required in v1 & v3, optional in v2) --
        std::vector<std::vector<double>> joint_pos;  ///< [frame][joint] positions (radians).
        std::vector<std::vector<double>> joint_vel;  ///< [frame][joint] velocities (rad/s).
        
        // -- Body quaternions (required for all versions) --
        std::vector<std::vector<std::array<double, 4>>> body_quat;  ///< [frame][body][w,x,y,z].
        
        // -- SMPL data (required in v2 & v3, optional in v1) --
        std::vector<std::vector<std::array<double, 3>>> smpl_joints;  ///< [frame][joint][x,y,z].
        std::vector<std::vector<std::array<double, 3>>> smpl_pose;    ///< [frame][pose][axis-angle x,y,z].
        
        std::vector<int64_t> frame_indices;  ///< Monotonic global frame indices (required).
        
        int protocol_version = 1;    ///< Protocol version (1, 2, or 3).
        bool catch_up_enabled = true; ///< true → use MAX_GAP_FRAMES; false → allow infinite delay.
        
        // Derived dimensions (must match the vector sizes above)
        int num_frames = 0;       ///< Number of frames in this chunk.
        int num_joints = 0;       ///< Joints per frame (joint_pos / joint_vel width).
        int num_quat_bodies = 0;  ///< Number of rigid bodies per frame (body_quat width).
        int num_smpl_joints = 0;  ///< SMPL joints per frame.
        int num_smpl_poses = 0;   ///< SMPL pose parameters per frame.
    };
    
    StreamedMotionMerger() {
        Reset();
    }
    
    // Reset the merger state (clear all buffered data)
    void Reset() {
        streamed_motion_ = std::make_shared<MotionSequence>();
        streamed_motion_->name = "streamed";
        streamed_motion_->ReserveCapacity(MAX_MOTION_FRAMES, 29, 1, 1, 0, 0);
        stream_window_start_ = 0;
        stream_frame_step_ = 1;
    }
    
    // Main merging method: merge incoming data with existing buffered data
    // Returns MergeResult containing the merged motion and playback adjustments
    // 
    // Note: Protocol version validation should be done by the caller before calling this method.
    // The merger doesn't care about protocol versions - it just merges the data.
    MergeResult MergeIncomingData(const IncomingData& data, int current_playback_frame) {
        MergeResult result;
        int64_t frame_step = 1;
        
        // Validate incoming data
        if (!ValidateIncomingData(data, frame_step)) {
            std::cerr << "[StreamedMotionMerger] Invalid incoming data" << std::endl;
            return result;
        }

        // A buffered window has one frame-index lattice.  A caller that wants
        // to change stride must reset the stream first; otherwise the old
        // rows cannot be addressed using the new step without misalignment.
        if (streamed_motion_ && streamed_motion_->timesteps > 0 &&
            frame_step != stream_frame_step_) {
            std::cerr << "[StreamedMotionMerger] Frame stride changed without reset"
                      << std::endl;
            return result;
        }
        
        const int64_t incoming_frame_start = data.frame_indices.front();
        const int64_t incoming_frame_end = data.frame_indices.back();
        
        if constexpr (DEBUG_LOGGING) {
            std::cout << "[StreamedMotionMerger] Processing " << data.num_frames << " frames, "
                      << "incoming_frame_start=" << incoming_frame_start 
                      << ", frame_step=" << frame_step << std::endl;
        }
        
        // Calculate sliding window parameters
        const int64_t playback_frame_after_history = std::max<int64_t>(
            0, static_cast<int64_t>(current_playback_frame) - HISTORY_FRAMES);
        int64_t playback_offset = 0;
        int64_t global_playback_frame = 0;
        if (!CheckedMultiplyNonnegative(
                frame_step, playback_frame_after_history, playback_offset) ||
            !CheckedAdd(stream_window_start_, playback_offset,
                        global_playback_frame)) {
            std::cerr << "[StreamedMotionMerger] Playback frame arithmetic overflow"
                      << std::endl;
            return result;
        }

        int64_t new_window_start = stream_window_start_;
        int64_t merge_dst_frame = 0;
        bool did_catchup = false;
        
        if (!CalculateSlidingWindow(
            incoming_frame_start,
            incoming_frame_end,
            frame_step,
            global_playback_frame,
            data.catch_up_enabled,
            new_window_start,
            merge_dst_frame,
            did_catchup
        )) {
            std::cerr << "[StreamedMotionMerger] Sliding-window arithmetic overflow"
                      << std::endl;
            return result;
        }

        // Never let an unbounded catch_up=false gap drive the fixed-capacity
        // MotionSequence writes past their 15,000-row allocation.
        if (merge_dst_frame < 0 || data.num_frames > MAX_MOTION_FRAMES ||
            merge_dst_frame >
                static_cast<int64_t>(MAX_MOTION_FRAMES - data.num_frames)) {
            std::cerr << "[StreamedMotionMerger] Window exceeds fixed capacity; "
                         "forcing catch-up reset" << std::endl;
            new_window_start = incoming_frame_start;
            merge_dst_frame = 0;
            did_catchup = true;
        }

        int new_window_start_narrow = 0;
        int merge_dst_frame_narrow = 0;
        int frame_step_narrow = 0;
        if (!NarrowToInt(new_window_start, new_window_start_narrow) ||
            !NarrowToInt(merge_dst_frame, merge_dst_frame_narrow) ||
            !NarrowToInt(frame_step, frame_step_narrow)) {
            std::cerr << "[StreamedMotionMerger] Frame arithmetic cannot be represented"
                      << std::endl;
            return result;
        }
        
        // Create new motion sequence
        auto new_motion = CreateNewMotion(data);
        
        // Copy old data to fill gap before incoming data
        if (merge_dst_frame > 0) {
            if (!CopyOldDataToNewMotion(
                streamed_motion_,
                stream_window_start_,
                new_motion,
                new_window_start,
                incoming_frame_start,
                frame_step,
                data
            )) {
                std::cerr << "[StreamedMotionMerger] Old-window copy bounds are invalid"
                          << std::endl;
                return result;
            }
        }
        
        // Copy incoming data to new motion
        CopyIncomingDataToMotion(data, new_motion, merge_dst_frame_narrow);
        
        // Update total timesteps
        int64_t total_timesteps = 0;
        if (!CheckedAdd(merge_dst_frame, data.num_frames, total_timesteps) ||
            total_timesteps < 0 || total_timesteps > MAX_MOTION_FRAMES ||
            !NarrowToInt(total_timesteps, new_motion->timesteps)) {
            std::cerr << "[StreamedMotionMerger] Merged timestep count is invalid"
                      << std::endl;
            return result;
        }
        
        if constexpr (DEBUG_LOGGING) {
            std::cout << "[StreamedMotionMerger] Merged motion: " << new_motion->timesteps 
                      << " frames (copied: " << merge_dst_frame << " + incoming: " << data.num_frames << ")" << std::endl;
        }
        
        // Calculate frame offset adjustment BEFORE updating state
        const int64_t old_window_start = stream_window_start_;
        int64_t window_shift_ticks = 0;
        if (!CheckedSubtract(
                new_window_start, old_window_start, window_shift_ticks)) {
            std::cerr << "[StreamedMotionMerger] Window-shift arithmetic overflow"
                      << std::endl;
            return result;
        }
        const int64_t window_shift = window_shift_ticks / frame_step;
        int window_shift_narrow = 0;
        if (!NarrowToInt(window_shift, window_shift_narrow)) {
            std::cerr << "[StreamedMotionMerger] Window shift cannot be represented"
                      << std::endl;
            return result;
        }
        
        // Update state
        streamed_motion_ = new_motion;
        stream_window_start_ = new_window_start;
        stream_frame_step_ = frame_step;
        
        // Build result
        result.motion = new_motion;
        result.window_start = new_window_start_narrow;
        result.frame_offset_adjustment = did_catchup ? 0 : window_shift_narrow;
        result.did_catchup_reset = did_catchup;
        result.frame_step = frame_step_narrow;
        result.protocol_version = data.protocol_version;
        
        return result;
    }
    
private:
    std::shared_ptr<MotionSequence> streamed_motion_;
    int64_t stream_window_start_ = 0;
    int64_t stream_frame_step_ = 1;

    static bool CheckedAdd(int64_t left, int64_t right, int64_t& result) {
        if ((right > 0 && left > std::numeric_limits<int64_t>::max() - right) ||
            (right < 0 && left < std::numeric_limits<int64_t>::min() - right)) {
            return false;
        }
        result = left + right;
        return true;
    }

    static bool CheckedSubtract(int64_t left, int64_t right, int64_t& result) {
        if ((right > 0 && left < std::numeric_limits<int64_t>::min() + right) ||
            (right < 0 && left > std::numeric_limits<int64_t>::max() + right)) {
            return false;
        }
        result = left - right;
        return true;
    }

    static bool CheckedMultiplyNonnegative(
        int64_t left, int64_t right, int64_t& result
    ) {
        if (left < 0 || right < 0 ||
            (left != 0 &&
             right > std::numeric_limits<int64_t>::max() / left)) {
            return false;
        }
        result = left * right;
        return true;
    }

    static bool NarrowToInt(int64_t value, int& result) {
        if (value < std::numeric_limits<int>::min() ||
            value > std::numeric_limits<int>::max()) {
            return false;
        }
        result = static_cast<int>(value);
        return true;
    }

    bool ValidateFrameIndices(
        const std::vector<int64_t>& frame_indices, int64_t& frame_step
    ) const {
        frame_step = 1;
        for (size_t index = 0; index < frame_indices.size(); ++index) {
            const int64_t frame_index = frame_indices[index];
            if (frame_index < 0 ||
                frame_index > std::numeric_limits<int>::max()) {
                std::cerr << "[StreamedMotionMerger] Frame index outside int range"
                          << std::endl;
                return false;
            }
            if (index == 0) continue;

            int64_t delta = 0;
            if (!CheckedSubtract(
                    frame_index, frame_indices[index - 1], delta) ||
                delta <= 0) {
                std::cerr << "[StreamedMotionMerger] Frame indices must be strictly "
                             "increasing"
                          << std::endl;
                return false;
            }
            if (index == 1) {
                frame_step = delta;
            } else if (delta != frame_step) {
                std::cerr << "[StreamedMotionMerger] Frame indices must have constant "
                             "stride"
                          << std::endl;
                return false;
            }
        }

        int64_t expected_span = 0;
        int64_t expected_end = 0;
        if (!CheckedMultiplyNonnegative(
                frame_step, static_cast<int64_t>(frame_indices.size() - 1),
                expected_span) ||
            !CheckedAdd(frame_indices.front(), expected_span, expected_end) ||
            expected_end != frame_indices.back()) {
            std::cerr << "[StreamedMotionMerger] Frame-index span is inconsistent"
                      << std::endl;
            return false;
        }
        return true;
    }
    
    // Validate incoming data structure
    bool ValidateIncomingData(
        const IncomingData& data, int64_t& frame_step
    ) const {
        // Check required fields
        if (data.num_frames <= 0 || data.num_frames > MAX_MOTION_FRAMES ||
            data.body_quat.size() != static_cast<size_t>(data.num_frames) ||
            data.frame_indices.size() != static_cast<size_t>(data.num_frames)) {
            std::cerr << "[StreamedMotionMerger] Missing required fields (body_quat or frame_indices)" << std::endl;
            return false;
        }
        if (!ValidateFrameIndices(data.frame_indices, frame_step)) {
            return false;
        }
        for (const auto& quaternions : data.body_quat) {
            if (quaternions.size() !=
                static_cast<size_t>(data.num_quat_bodies)) return false;
        }
        const auto validate_rows = [frames = data.num_frames](
            const auto& rows, int width) {
            if (rows.empty()) return true;
            if (rows.size() != static_cast<size_t>(frames) || width <= 0) return false;
            return std::all_of(rows.begin(), rows.end(), [width](const auto& row) {
                return row.size() == static_cast<size_t>(width);
            });
        };
        if (!validate_rows(data.joint_pos, data.num_joints) ||
            !validate_rows(data.joint_vel, data.num_joints) ||
            !validate_rows(data.smpl_joints, data.num_smpl_joints) ||
            !validate_rows(data.smpl_pose, data.num_smpl_poses)) {
            std::cerr << "[StreamedMotionMerger] Row dimensions do not match metadata"
                      << std::endl;
            return false;
        }
        
        // Validate protocol-specific requirements
        if (data.protocol_version == 3) {
            // Version 3: requires both SMPL data AND joint data
            if (data.smpl_joints.empty() || data.smpl_pose.empty()) {
                std::cerr << "[StreamedMotionMerger] Protocol v3 missing smpl_joints or smpl_pose" << std::endl;
                return false;
            }
            if (data.joint_pos.empty() || data.joint_vel.empty()) {
                std::cerr << "[StreamedMotionMerger] Protocol v3 missing joint_pos or joint_vel" << std::endl;
                return false;
            }
        } else if (data.protocol_version == 2) {
            // Version 2: requires SMPL data (joint data optional)
            if (data.smpl_joints.empty() || data.smpl_pose.empty()) {
                std::cerr << "[StreamedMotionMerger] Protocol v2 missing smpl_joints or smpl_pose" << std::endl;
                return false;
            }
        } else if (data.protocol_version == 1) {
            // Version 1: requires joint data (SMPL data optional)
            if (data.joint_pos.empty() || data.joint_vel.empty()) {
                std::cerr << "[StreamedMotionMerger] Protocol v1 missing joint_pos or joint_vel" << std::endl;
                return false;
            }
        } else {
            std::cerr << "[StreamedMotionMerger] Unsupported protocol version: " << data.protocol_version << std::endl;
            return false;
        }
        
        return true;
    }
    
    // Calculate sliding window parameters
    bool CalculateSlidingWindow(
        int64_t incoming_frame_start,
        int64_t incoming_frame_end,
        int64_t frame_step,
        int64_t global_playback_frame,
        bool catch_up_enabled,
        int64_t& new_window_start,
        int64_t& merge_dst_frame,
        bool& did_catchup
    ) {
        // Special case: first packet
        if (!streamed_motion_ || streamed_motion_->timesteps <= 0) {
            new_window_start = incoming_frame_start;
            merge_dst_frame = 0;
            did_catchup = true;
            return true;
        }
        
        // Calculate max gap based on catch_up flag
        const int64_t max_gap_frames = catch_up_enabled
            ? static_cast<int64_t>(MAX_GAP_FRAMES + HISTORY_FRAMES)
            : std::numeric_limits<int64_t>::max();

        int64_t window_span = 0;
        int64_t stream_window_end = 0;
        if (!CheckedMultiplyNonnegative(
                frame_step, streamed_motion_->timesteps - 1, window_span) ||
            !CheckedAdd(stream_window_start_, window_span, stream_window_end)) {
            return false;
        }

        if (DEBUG_LOGGING) {
            std::cout << "[StreamedMotionMerger] incoming_frame_start: " << incoming_frame_start
                      << ", incoming_frame_end: " << incoming_frame_end
                      << ", stream_window_start_: " << stream_window_start_
                      << ", stream_window_end: " << stream_window_end
                      << ", frame_step: " << frame_step
                      << ", global_playback_frame: " << global_playback_frame
                      << ", streamed_motion_->timesteps: " << streamed_motion_->timesteps
                      << std::endl;
        }

        // Check for incoming data older than current window
        if (incoming_frame_start <= stream_window_start_) {
            if constexpr (DEBUG_LOGGING) {
                std::cout << "[StreamedMotionMerger] WARNING: incoming_frame_start (" << incoming_frame_start
                          << ") < stream_window_start_ (" << stream_window_start_ << ") - forcing catch-up" << std::endl;
            }
            new_window_start = incoming_frame_start;
            merge_dst_frame = 0;
            did_catchup = true;
            return true;
        } else if (incoming_frame_end <= stream_window_end) {
            if constexpr (DEBUG_LOGGING) {
                std::cout << "[StreamedMotionMerger] WARNING: incoming_frame_end (" << incoming_frame_end
                          << ") <= stream_window_end (" << stream_window_end << ") - forcing catch-up" << std::endl;
            }
            new_window_start = incoming_frame_start;
            merge_dst_frame = 0;
            did_catchup = true;
            return true;
        }

        int64_t incoming_offset_from_old = 0;
        if (!CheckedSubtract(
                incoming_frame_start, stream_window_start_,
                incoming_offset_from_old)) {
            return false;
        }
        if (incoming_offset_from_old % frame_step != 0) {
            // Both chunks are individually well-formed, but they do not share
            // a frame-index lattice.  Reset rather than flooring the offset
            // and attaching the incoming rows to the wrong global indices.
            new_window_start = incoming_frame_start;
            merge_dst_frame = 0;
            did_catchup = true;
            return true;
        }
        
        // Tentative window aligned to playback
        const int64_t desired_window_start = global_playback_frame;
        const int64_t tentative_window_start =
            std::min(desired_window_start, incoming_frame_start);
        int64_t delta_to_incoming = 0;
        if (!CheckedSubtract(
                incoming_frame_start, tentative_window_start,
                delta_to_incoming)) {
            return false;
        }
        if (delta_to_incoming % frame_step != 0) {
            new_window_start = incoming_frame_start;
            merge_dst_frame = 0;
            did_catchup = true;
            return true;
        }
        const int64_t tentative_merge_dst = delta_to_incoming / frame_step;
        
        // Check for large gap
        int64_t first_frame_after_old_window = 0;
        if (!CheckedAdd(
                stream_window_end, frame_step, first_frame_after_old_window)) {
            return false;
        }
        const bool large_gap_from_old =
            incoming_frame_start > first_frame_after_old_window;
        
        if (tentative_merge_dst > max_gap_frames || large_gap_from_old) {
            // Catch-up: reset window to incoming frame
            new_window_start = incoming_frame_start;
            merge_dst_frame = 0;
            did_catchup = true;
            
            if constexpr (DEBUG_LOGGING) {
                std::cout << "[StreamedMotionMerger] CATCH-UP: gap too large or old data expired" << std::endl;
            }
        } else {
            // Normal merge
            new_window_start = tentative_window_start;
            merge_dst_frame = tentative_merge_dst;
        }
        return true;
    }
    
    // Create new motion sequence with appropriate capacity
    std::shared_ptr<MotionSequence> CreateNewMotion(const IncomingData& data) const {
        auto new_motion = std::make_shared<MotionSequence>();
        new_motion->name = "streamed";
        
        int joints_to_reserve = data.num_joints;
        int bodies_to_reserve = 1;
        int body_quaternions_to_reserve = data.num_quat_bodies;
        int smpl_joints_to_reserve = data.num_smpl_joints;
        int smpl_poses_to_reserve = data.num_smpl_poses;
        
        new_motion->ReserveCapacity(
            MAX_MOTION_FRAMES,
            joints_to_reserve,
            bodies_to_reserve,
            body_quaternions_to_reserve,
            smpl_joints_to_reserve,
            smpl_poses_to_reserve
        );
        
        // Initialize body_part_indexes (typically just root for streaming)
        new_motion->SetBodyPartIndexes({0});
        
        return new_motion;
    }
    
    // Copy old data to new motion to fill gap before incoming data
    bool CopyOldDataToNewMotion(
        std::shared_ptr<MotionSequence> old_motion,
        int64_t old_window_start,
        std::shared_ptr<MotionSequence> new_motion,
        int64_t new_window_start,
        int64_t incoming_frame_start,
        int64_t frame_step,
        const IncomingData& data
    ) {
        if (!old_motion || old_motion->timesteps <= 0) {
            return true;
        }

        int64_t old_window_span = 0;
        int64_t old_window_end = 0;
        if (!CheckedMultiplyNonnegative(
                frame_step, old_motion->timesteps, old_window_span) ||
            !CheckedAdd(old_window_start, old_window_span, old_window_end)) {
            return false;
        }
        
        // Find overlap between old data and needed range
        const int64_t need_start_global = new_window_start;
        const int64_t need_end_global = incoming_frame_start;
        const int64_t overlap_start_global =
            std::max(need_start_global, old_window_start);
        const int64_t overlap_end_global =
            std::min(need_end_global, old_window_end);
        
        if (overlap_start_global >= overlap_end_global) {
            return true;  // No overlap
        }
        
        // Calculate copy parameters
        int64_t start_offset_old = 0;
        int64_t start_offset_new = 0;
        int64_t overlap_span = 0;
        if (!CheckedSubtract(
                overlap_start_global, old_window_start, start_offset_old) ||
            !CheckedSubtract(
                overlap_start_global, new_window_start, start_offset_new) ||
            !CheckedSubtract(
                overlap_end_global, overlap_start_global, overlap_span) ||
            start_offset_old < 0 || start_offset_new < 0 || overlap_span < 0 ||
            start_offset_old % frame_step != 0 ||
            start_offset_new % frame_step != 0 ||
            overlap_span % frame_step != 0) {
            return false;
        }

        const int64_t copy_src_idx_wide = start_offset_old / frame_step;
        const int64_t copy_dst_idx_wide = start_offset_new / frame_step;
        const int64_t copy_count_wide = overlap_span / frame_step;
        int64_t copy_src_end = 0;
        int64_t copy_dst_end = 0;
        if (!CheckedAdd(copy_src_idx_wide, copy_count_wide, copy_src_end) ||
            !CheckedAdd(copy_dst_idx_wide, copy_count_wide, copy_dst_end) ||
            copy_src_idx_wide < 0 || copy_dst_idx_wide < 0 ||
            copy_count_wide < 0 ||
            copy_src_end > old_motion->timesteps ||
            copy_dst_end > MAX_MOTION_FRAMES) {
            return false;
        }

        int copy_src_idx = 0;
        int copy_dst_idx = 0;
        int copy_count = 0;
        if (!NarrowToInt(copy_src_idx_wide, copy_src_idx) ||
            !NarrowToInt(copy_dst_idx_wide, copy_dst_idx) ||
            !NarrowToInt(copy_count_wide, copy_count)) {
            return false;
        }
        
        if constexpr (DEBUG_LOGGING) {
            std::cout << "[StreamedMotionMerger] Copying old data: "
                      << "global [" << overlap_start_global << ".." << (overlap_end_global-1) << "] → "
                      << "new_motion[" << copy_dst_idx << ".." << (copy_dst_idx + copy_count - 1) << "]" << std::endl;
        }
        
        // Copy joint data if present
        if (data.num_joints > 0 && old_motion->GetNumJoints() > 0) {
            int joints_to_copy = std::min(data.num_joints, old_motion->GetNumJoints());
            for (int i = 0; i < copy_count; ++i) {
                for (int joint = 0; joint < joints_to_copy; ++joint) {
                    new_motion->JointPositions(copy_dst_idx + i)[joint] = 
                        old_motion->JointPositions(copy_src_idx + i)[joint];
                    new_motion->JointVelocities(copy_dst_idx + i)[joint] = 
                        old_motion->JointVelocities(copy_src_idx + i)[joint];
                }
            }
        }
        
        // Copy body quaternions
        int old_quat_bodies = old_motion->GetNumBodyQuaternions();
        int quat_bodies_to_copy = std::min(data.num_quat_bodies, old_quat_bodies);
        for (int i = 0; i < copy_count; ++i) {
            for (int b = 0; b < quat_bodies_to_copy; ++b) {
                for (int q = 0; q < 4; ++q) {
                    new_motion->BodyQuaternions(copy_dst_idx + i)[b][q] = 
                        old_motion->BodyQuaternions(copy_src_idx + i)[b][q];
                }
            }
        }
        
        // Copy SMPL data if present
        if (data.num_smpl_joints > 0 && old_motion->GetNumSmplJoints() > 0) {
            int smpl_joints_to_copy = std::min(data.num_smpl_joints, old_motion->GetNumSmplJoints());
            for (int i = 0; i < copy_count; ++i) {
                for (int joint = 0; joint < smpl_joints_to_copy; ++joint) {
                    for (int xyz = 0; xyz < 3; ++xyz) {
                        new_motion->SmplJoints(copy_dst_idx + i)[joint][xyz] = 
                            old_motion->SmplJoints(copy_src_idx + i)[joint][xyz];
                    }
                }
            }
        }
        
        if (data.num_smpl_poses > 0 && old_motion->GetNumSmplPoses() > 0) {
            int smpl_poses_to_copy = std::min(data.num_smpl_poses, old_motion->GetNumSmplPoses());
            for (int i = 0; i < copy_count; ++i) {
                for (int p = 0; p < smpl_poses_to_copy; ++p) {
                    for (int xyz = 0; xyz < 3; ++xyz) {
                        new_motion->SmplPoses(copy_dst_idx + i)[p][xyz] = 
                            old_motion->SmplPoses(copy_src_idx + i)[p][xyz];
                    }
                }
            }
        }
        return true;
    }
    
    // Copy incoming data to motion sequence
    void CopyIncomingDataToMotion(
        const IncomingData& data,
        std::shared_ptr<MotionSequence> motion,
        int dst_frame_offset
    ) {
        // Copy joint data if present
        if (!data.joint_pos.empty() && !data.joint_vel.empty()) {
            for (int frame = 0; frame < data.num_frames; ++frame) {
                for (int joint = 0; joint < data.num_joints; ++joint) {
                    motion->JointPositions(dst_frame_offset + frame)[joint] = data.joint_pos[frame][joint];
                    motion->JointVelocities(dst_frame_offset + frame)[joint] = data.joint_vel[frame][joint];
                }
            }
        }
        
        // Copy body quaternions (always present)
        for (int frame = 0; frame < data.num_frames; ++frame) {
            for (int body = 0; body < data.num_quat_bodies; ++body) {
                for (int q = 0; q < 4; ++q) {
                    motion->BodyQuaternions(dst_frame_offset + frame)[body][q] = 
                        data.body_quat[frame][body][q];
                }
            }
        }
        
        // Copy SMPL joints if present
        if (!data.smpl_joints.empty()) {
            for (int frame = 0; frame < data.num_frames; ++frame) {
                for (int joint = 0; joint < data.num_smpl_joints; ++joint) {
                    for (int xyz = 0; xyz < 3; ++xyz) {
                        motion->SmplJoints(dst_frame_offset + frame)[joint][xyz] = 
                            data.smpl_joints[frame][joint][xyz];
                    }
                }
            }
        }
        
        // Copy SMPL poses if present
        if (!data.smpl_pose.empty()) {
            for (int frame = 0; frame < data.num_frames; ++frame) {
                for (int pose = 0; pose < data.num_smpl_poses; ++pose) {
                    for (int xyz = 0; xyz < 3; ++xyz) {
                        motion->SmplPoses(dst_frame_offset + frame)[pose][xyz] = 
                            data.smpl_pose[frame][pose][xyz];
                    }
                }
            }
        }
    }
};

#endif // STREAMED_MOTION_MERGER_HPP
