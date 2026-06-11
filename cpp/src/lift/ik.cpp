#include "lift/ik.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>

#include "lift/keypoint_format.hpp"
#include "lift/skeleton_def.hpp"

namespace fitra::lift {

namespace {

cv::Vec3d vec(const infer::Joint3D& a, const infer::Joint3D& b) {
    return {static_cast<double>(a.x) - b.x,
            static_cast<double>(a.y) - b.y,
            static_cast<double>(a.z) - b.z};
}

double norm(const cv::Vec3d& v) {
    return std::sqrt(v.dot(v));
}

cv::Vec3d normalize_or(const cv::Vec3d& v, const cv::Vec3d& fallback) {
    double n = norm(v);
    if (n < 1.0e-9) return fallback;
    return v * (1.0 / n);
}

void set_from(infer::Joint3D& j, const cv::Vec3d& p) {
    j.x = static_cast<float>(p[0]);
    j.y = static_cast<float>(p[1]);
    j.z = static_cast<float>(p[2]);
    j.valid = true;
}

cv::Vec3d point_of(const infer::Joint3D& j) {
    return {j.x, j.y, j.z};
}

double median(std::vector<double> vals) {
    if (vals.empty()) return 0.0;
    auto mid = vals.begin() + static_cast<std::ptrdiff_t>(vals.size() / 2);
    std::nth_element(vals.begin(), mid, vals.end());
    double m = *mid;
    if (vals.size() % 2 == 0) {
        auto mid2 = vals.begin() + static_cast<std::ptrdiff_t>(vals.size() / 2 - 1);
        std::nth_element(vals.begin(), mid2, vals.end());
        m = 0.5 * (m + *mid2);
    }
    return m;
}

}  // namespace

IkSolver::IkSolver() : IkSolver(Options{}) {}

IkSolver::IkSolver(Options opts) : opts_{opts} {
    if (opts_.has_subject_profile) {
        apply_subject_profile_locked(opts_.subject_profile);
    } else if (opts_.subject_height_m > 0.0) {
        apply_subject_height_model_locked();
    }
}

void IkSolver::apply_subject_height_model_locked() {
    const double h = opts_.subject_height_m;
    if (h <= 0.0) return;

    // Ratios are derived from AIST/HQL 3D Anthropometric Database 2003
    // young male/female means, normalized by stature and averaged into a
    // sex-neutral Japanese adult prior. COCO/Halpe joints are anatomical
    // approximations, so only robust limb/trunk segments are hard-coded.
    constexpr double kTorsoSide = 0.314;
    constexpr double kHipWidth = 0.199;
    constexpr double kShoulderWidth = 0.258;
    constexpr double kUpperArm = 0.189;
    constexpr double kForearm = 0.146;
    constexpr double kThigh = 0.226;
    constexpr double kLowerLeg = 0.224;

    locked_parent_len_.fill(0.0);

    // Joints whose parent index is identical under COCO17 and Halpe26.
    locked_parent_len_[7] = kUpperArm * h;    // l_elbow <- l_shoulder
    locked_parent_len_[8] = kUpperArm * h;    // r_elbow <- r_shoulder
    locked_parent_len_[9] = kForearm * h;     // l_wrist <- l_elbow
    locked_parent_len_[10] = kForearm * h;    // r_wrist <- r_elbow
    locked_parent_len_[13] = kThigh * h;      // l_knee <- l_hip
    locked_parent_len_[14] = kThigh * h;      // r_knee <- r_hip
    locked_parent_len_[15] = kLowerLeg * h;   // l_ankle <- l_knee
    locked_parent_len_[16] = kLowerLeg * h;   // r_ankle <- r_knee

    // Joints whose parent differs between COCO17 and Halpe26. enforce_lengths
    // iterates the active SkeletonDef, so the value we lock must match the
    // anatomical bone that the active parent tree expects -- otherwise the
    // shoulders or r_hip get yanked to a wrong distance on the very first
    // frame and bone_drift_pct stays inflated.
    if (active_keypoint_format() == KeypointFormat::Halpe26) {
        // Halpe26: 5/6 <- 18 (neck), 12 <- 19 (hip-center).
        locked_parent_len_[5]  = 0.5 * kShoulderWidth * h;  // neck -> l_shoulder
        locked_parent_len_[6]  = 0.5 * kShoulderWidth * h;  // neck -> r_shoulder
        locked_parent_len_[12] = 0.5 * kHipWidth * h;       // hip-center -> r_hip
    } else {
        // COCO17: 5 <- 11 (l_hip), 6 <- 12 (r_hip), 12 <- 11 (l_hip).
        locked_parent_len_[5]  = kTorsoSide * h;  // l_hip -> l_shoulder
        locked_parent_len_[6]  = kTorsoSide * h;  // r_hip -> r_shoulder
        locked_parent_len_[12] = kHipWidth * h;   // l_hip -> r_hip
    }

    // enforce_pair_lengths still anchors the shoulder pair distance, which is
    // anatomically equivalent under either topology (joints 5/6 are
    // l_shoulder/r_shoulder in both).
    locked_shoulder_width_ = kShoulderWidth * h;
    locked_ = true;
}

void IkSolver::apply_subject_profile_locked(const SubjectProfile& profile) {
    validate_subject_profile(profile);
    locked_parent_len_.fill(0.0);
    for (std::size_t child = 0; child < profile.bone_lengths_m.size(); ++child) {
        if (profile.bone_lengths_m[child] > 1.0e-6) {
            locked_parent_len_[child] = profile.bone_lengths_m[child];
        }
    }
    if (profile.hip_width_m > 1.0e-6 && locked_parent_len_[12] <= 1.0e-6) {
        locked_parent_len_[12] = profile.hip_width_m;
    }
    locked_shoulder_width_ = profile.shoulder_width_m;
    if (profile.subject_height_m > 0.0) {
        opts_.subject_height_m = profile.subject_height_m;
    }
    subject_id_ = profile.subject_id;
    profile_quality_status_ = profile.quality_status;
    profile_loaded_ = true;
    locked_ = true;
}

void IkSolver::apply_subject_height(double m) {
    if (m <= 0.0) return;
    std::lock_guard<std::mutex> g(mu_);
    for (auto& v : samples_) v.clear();
    observed_frames_ = 0;
    profile_loaded_ = false;
    subject_id_.clear();
    profile_quality_status_.clear();
    opts_.subject_height_m = m;
    apply_subject_height_model_locked();
}

infer::Skeleton3D IkSolver::update(const infer::Skeleton3D& input) {
    std::lock_guard<std::mutex> g(mu_);
    if (!locked_) {
        observe_lengths_locked(input);
        if (observed_frames_ >= opts_.bone_calib_frames) {
            lock_lengths_locked();
        }
    }
    infer::Skeleton3D out = input;
    if (!locked_) return out;
    for (int i = 0; i < std::max(1, opts_.iterations); ++i) {
        enforce_lengths(out);
        enforce_pair_lengths(out);
        enforce_hinges(out);
    }
    return out;
}

void IkSolver::observe_lengths_locked(const infer::Skeleton3D& skel) {
    const auto& def = active_skeleton_def();
    bool any = false;
    for (std::size_t child = 0; child < def.parents.size(); ++child) {
        int parent = def.parents[child];
        if (parent < 0) continue;
        const auto& a = skel.joints[static_cast<std::size_t>(parent)];
        const auto& b = skel.joints[child];
        if (!a.valid || !b.valid) continue;
        double len = norm(vec(a, b));
        if (len > 1.0e-4 && len < 2.0) {
            samples_[child].push_back(len);
            any = true;
        }
    }
    if (any) observed_frames_ += 1;
}

void IkSolver::lock_lengths_locked() {
    for (std::size_t child = 0; child < samples_.size(); ++child) {
        locked_parent_len_[child] = median(samples_[child]);
    }
    locked_shoulder_width_ = 0.0;
    locked_ = true;
}

void IkSolver::enforce_lengths(infer::Skeleton3D& skel) const {
    const auto& def = active_skeleton_def();
    for (std::size_t child = 0; child < def.parents.size(); ++child) {
        int parent = def.parents[child];
        if (parent < 0) continue;
        double target = locked_parent_len_[child];
        if (target <= 1.0e-6) continue;
        auto& p = skel.joints[static_cast<std::size_t>(parent)];
        auto& c = skel.joints[child];
        if (!p.valid || !c.valid) continue;
        cv::Vec3d dir = normalize_or(vec(c, p), {0.0, 0.0, -1.0});
        cv::Vec3d next{p.x + dir[0] * target,
                       p.y + dir[1] * target,
                       p.z + dir[2] * target};
        set_from(c, next);
    }
}

void IkSolver::enforce_pair_lengths(infer::Skeleton3D& skel) const {
    if (locked_shoulder_width_ <= 1.0e-6) return;
    auto& l = skel.joints[5];
    auto& r = skel.joints[6];
    if (!l.valid || !r.valid) return;
    cv::Vec3d lp = point_of(l);
    cv::Vec3d rp = point_of(r);
    cv::Vec3d mid = (lp + rp) * 0.5;
    cv::Vec3d dir = normalize_or(lp - rp, {1.0, 0.0, 0.0});
    set_from(l, mid + dir * (locked_shoulder_width_ * 0.5));
    set_from(r, mid - dir * (locked_shoulder_width_ * 0.5));
}

void IkSolver::enforce_hinges(infer::Skeleton3D& skel) const {
    const auto& def = active_skeleton_def();
    const double min_rad = opts_.min_hinge_deg * CV_PI / 180.0;
    const double max_rad = opts_.max_hinge_deg * CV_PI / 180.0;
    for (int joint : def.hinge_joints) {
        int parent = def.parents[static_cast<std::size_t>(joint)];
        int child = def.hinge_child(joint);
        if (parent < 0 || child < 0) continue;
        auto& a = skel.joints[static_cast<std::size_t>(parent)];
        auto& b = skel.joints[static_cast<std::size_t>(joint)];
        auto& c = skel.joints[static_cast<std::size_t>(child)];
        if (!a.valid || !b.valid || !c.valid) continue;

        cv::Vec3d ba = normalize_or(vec(a, b), {1.0, 0.0, 0.0});
        cv::Vec3d bc_raw = vec(c, b);
        double child_len = norm(bc_raw);
        if (child_len < 1.0e-6) continue;
        cv::Vec3d bc = bc_raw * (1.0 / child_len);
        double dot = std::clamp(ba.dot(bc), -1.0, 1.0);
        double angle = std::acos(dot);
        if (angle >= min_rad && angle <= max_rad) continue;

        double target = std::clamp(angle, min_rad, max_rad);
        cv::Vec3d perp = bc - ba * dot;
        if (norm(perp) < 1.0e-6) {
            perp = std::abs(ba[0]) < 0.9 ? cv::Vec3d{1.0, 0.0, 0.0}
                                         : cv::Vec3d{0.0, 1.0, 0.0};
            perp = perp - ba * ba.dot(perp);
        }
        perp = normalize_or(perp, {0.0, 1.0, 0.0});
        cv::Vec3d next_dir = ba * std::cos(target) + perp * std::sin(target);
        cv::Vec3d next{b.x + next_dir[0] * child_len,
                       b.y + next_dir[1] * child_len,
                       b.z + next_dir[2] * child_len};
        set_from(c, next);
    }
}

double IkSolver::bone_drift_pct(const infer::Skeleton3D& skel) const {
    std::lock_guard<std::mutex> g(mu_);
    return bone_drift_pct_locked(skel);
}

double IkSolver::bone_drift_pct_locked(const infer::Skeleton3D& skel) const {
    if (!locked_) return 0.0;
    const auto& def = active_skeleton_def();
    double sum = 0.0;
    int n = 0;
    for (std::size_t child = 0; child < def.parents.size(); ++child) {
        int parent = def.parents[child];
        double target = locked_parent_len_[child];
        if (parent < 0 || target <= 1.0e-6) continue;
        const auto& a = skel.joints[static_cast<std::size_t>(parent)];
        const auto& b = skel.joints[child];
        if (!a.valid || !b.valid) continue;
        sum += std::abs(norm(vec(a, b)) - target) / target * 100.0;
        n += 1;
    }
    if (locked_shoulder_width_ > 1.0e-6) {
        const auto& l = skel.joints[5];
        const auto& r = skel.joints[6];
        if (l.valid && r.valid) {
            sum += std::abs(norm(vec(l, r)) - locked_shoulder_width_) / locked_shoulder_width_ * 100.0;
            n += 1;
        }
    }
    return n > 0 ? sum / n : 0.0;
}

bool IkSolver::locked() const {
    std::lock_guard<std::mutex> g(mu_);
    return locked_;
}

bool IkSolver::profile_loaded() const {
    std::lock_guard<std::mutex> g(mu_);
    return profile_loaded_;
}

double IkSolver::subject_height_m() const {
    std::lock_guard<std::mutex> g(mu_);
    return opts_.subject_height_m;
}

std::string IkSolver::subject_id() const {
    std::lock_guard<std::mutex> g(mu_);
    return subject_id_;
}

std::string IkSolver::profile_quality_status() const {
    std::lock_guard<std::mutex> g(mu_);
    return profile_quality_status_;
}

}  // namespace fitra::lift
