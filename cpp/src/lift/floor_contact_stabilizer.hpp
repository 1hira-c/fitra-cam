#pragma once
//
// Floor-contact stabilization for a Halpe26 lifted skeleton.
//
// Contact is detected from the sole keypoints, but the correction is applied
// as one rigid translation to the ankle + sole points.  This makes the default
// ankle-based VR foot tracker benefit from the same grounding as the WebUI
// skeleton without rotating or deforming the foot itself.  The knee and the
// rest of the leg are deliberately left alone: this is a bounded output-stage
// correction, not a leg-chain IK solve.

#include <array>

#include <opencv2/core.hpp>

#include "infer/types.hpp"

namespace fitra::lift {

struct FloorContactOptions {
    double floor_z_m = 0.0;
    double enter_height_m = 0.03;
    double exit_height_m = 0.06;
    double enter_speed_mps = 0.25;
    double exit_speed_mps = 0.80;
    double xy_anchor_tau_s = 0.25;
    double max_xy_correction_m = 0.03;
    double max_z_correction_m = 0.08;
    int missing_grace_frames = 2;
    double reset_dt_s = 0.10;
};

struct FootContactReport {
    bool contact = false;
    bool corrected = false;
    bool missing_grace = false;
    cv::Vec3f correction_m{0.0f, 0.0f, 0.0f};
};

struct FloorContactReport {
    // Index 0 = left, 1 = right.
    std::array<FootContactReport, 2> feet{};
};

class FloorContactStabilizer {
public:
    explicit FloorContactStabilizer(FloorContactOptions opts = {});

    // Mutates `skel` in place and reports the correction applied this frame.
    // COCO17 (no sole points) is an automatic no-op.
    FloorContactReport update(infer::Skeleton3D& skel, double dt_s);

    // Drop all contact/velocity history.  Call on idle/standby resume.
    void reset();

    const FloorContactOptions& options() const { return opts_; }

private:
    struct FootState {
        bool contact = false;
        bool has_prev_ankle = false;
        cv::Vec3f prev_raw_ankle{0.0f, 0.0f, 0.0f};
        double elapsed_since_prev_s = 0.0;
        cv::Vec2f anchor_xy{0.0f, 0.0f};
        cv::Vec3f last_correction{0.0f, 0.0f, 0.0f};
        int missing_frames = 0;
    };

    FloorContactOptions opts_;
    std::array<FootState, 2> state_{};
};

}  // namespace fitra::lift
