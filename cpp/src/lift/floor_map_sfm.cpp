#include "lift/floor_map_sfm.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <set>
#include <utility>

namespace fitra::lift {

namespace {

using cv::Matx33d;
using cv::Matx44d;
using cv::Vec3d;

// Median of a copy (small vectors; clarity over the O(n) selection).
double median_of(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

// Robustly collapse a set of relative-pose samples to one pose: chordal mean,
// then drop samples beyond a MAD-scaled residual band (in both rotation angle
// and translation distance) and re-average the inliers. Falls back to the plain
// mean when there are too few samples to estimate a spread.
Matx44d robust_average(const std::vector<Matx44d>& s) {
    if (s.empty()) return Matx44d::eye();
    if (s.size() <= 2) return geom::average_poses(s);

    const Matx44d mean = geom::average_poses(s);
    const Vec3d mean_t = geom::trans_of(mean);

    std::vector<double> rot_res(s.size()), tr_res(s.size());
    for (std::size_t i = 0; i < s.size(); ++i) {
        rot_res[i] = geom::rotation_angle_deg(s[i], mean);
        tr_res[i]  = cv::norm(geom::trans_of(s[i]) - mean_t);
    }
    const double mr = median_of(rot_res);
    const double mt = median_of(tr_res);
    std::vector<double> dr(s.size()), dt(s.size());
    for (std::size_t i = 0; i < s.size(); ++i) {
        dr[i] = std::abs(rot_res[i] - mr);
        dt[i] = std::abs(tr_res[i] - mt);
    }
    const double rthr = mr + 3.0 * 1.4826 * median_of(dr) + 1e-6;
    const double tthr = mt + 3.0 * 1.4826 * median_of(dt) + 1e-6;

    std::vector<Matx44d> inliers;
    inliers.reserve(s.size());
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (rot_res[i] <= rthr && tr_res[i] <= tthr) inliers.push_back(s[i]);
    }
    if (inliers.empty()) return mean;
    return geom::average_poses(inliers);
}

// Minimal rotation mapping unit vector a → unit vector b (Rodrigues). Handles
// the (anti-)parallel degeneracies.
Matx33d rotation_between(const Vec3d& a, const Vec3d& b) {
    Vec3d u = a, v = b;
    const double na = cv::norm(u), nb = cv::norm(v);
    if (na < 1e-12 || nb < 1e-12) return Matx33d::eye();
    u *= 1.0 / na;
    v *= 1.0 / nb;
    Vec3d axis = u.cross(v);
    const double s = cv::norm(axis);
    const double c = u.dot(v);
    if (s < 1e-9) {
        if (c > 0.0) return Matx33d::eye();  // already aligned
        // 180°: rotate about any axis perpendicular to u.
        Vec3d perp = std::abs(u[0]) < 0.9 ? Vec3d(1, 0, 0) : Vec3d(0, 1, 0);
        axis = u.cross(perp);
        axis *= 1.0 / cv::norm(axis);
        // R = 2 n nᵀ − I (180° about axis).
        Matx33d R;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R(i, j) = 2.0 * axis[i] * axis[j] - (i == j ? 1.0 : 0.0);
        return R;
    }
    axis *= 1.0 / s;
    Matx33d K(0, -axis[2], axis[1], axis[2], 0, -axis[0], -axis[1], axis[0], 0);
    return Matx33d::eye() + K * s + (K * K) * (1.0 - c);
}

Matx33d rot_z(double rad) {
    const double cz = std::cos(rad), sz = std::sin(rad);
    return Matx33d(cz, -sz, 0, sz, cz, 0, 0, 0, 1);
}

// Smallest-eigenvalue eigenvector (plane normal) of the tag-centre covariance,
// plus the std along it (plane thickness). Mirrors floor_extrinsic_solver's
// min_axis_thickness. `centres` must hold ≥3 points.
Vec3d fit_plane_normal(const std::vector<Vec3d>& centres, double& rms_out) {
    Vec3d mean(0, 0, 0);
    for (const auto& p : centres) mean += p;
    mean *= 1.0 / static_cast<double>(centres.size());
    Matx33d cov = Matx33d::zeros();
    for (const auto& p : centres) {
        Vec3d d = p - mean;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) cov(i, j) += d[i] * d[j];
    }
    cov *= 1.0 / static_cast<double>(centres.size());
    cv::Matx33d evec;
    cv::Vec3d eval;
    cv::eigen(cov, eval, evec);  // descending; row 2 = smallest
    rms_out = std::sqrt(std::max(0.0, eval[2]));
    return Vec3d(evec(2, 0), evec(2, 1), evec(2, 2));
}

}  // namespace

bool build_floor_map_sfm(const std::vector<SfmFrame>& frames,
                         const SfmMapOptions& opts,
                         FloorTagMap& out_map,
                         SfmMapReport& report) {
    out_map = FloorTagMap{};
    report = SfmMapReport{};

    // --- 1. observed tag ids + per-frame relative poses → edge samples --------
    std::set<int> ids;
    // edge key (a<b) → samples of T_a←b (tag b expressed in tag a's frame).
    std::map<std::pair<int, int>, std::vector<Matx44d>> edge_samples;

    for (const auto& fr : frames) {
        // Keep only obs that passed the PnP-quality gate.
        std::vector<const SfmTagObs*> good;
        for (const auto& ob : fr.tags) {
            if (ob.reproj_rms_px <= opts.max_pose_rms_px) {
                good.push_back(&ob);
                ids.insert(ob.id);
            }
        }
        for (std::size_t i = 0; i < good.size(); ++i) {
            for (std::size_t j = i + 1; j < good.size(); ++j) {
                int a = good[i]->id, b = good[j]->id;
                const Matx44d* Ta = &good[i]->T_cam_tag.raw();
                const Matx44d* Tb = &good[j]->T_cam_tag.raw();
                if (a == b) continue;
                if (a > b) {  // canonical low→high
                    std::swap(a, b);
                    std::swap(Ta, Tb);
                }
                // T_a←b = (T_cam←a)⁻¹ · (T_cam←b)
                Matx44d rel = geom::invert_rigid(*Ta) * (*Tb);
                edge_samples[{a, b}].push_back(rel);
            }
        }
    }

    report.n_tags = static_cast<int>(ids.size());
    if (ids.empty()) {
        report.message = "no tag observations";
        return false;
    }

    // --- 2. robust per-edge relative pose + adjacency -------------------------
    std::map<std::pair<int, int>, Matx44d> edges;
    std::map<int, std::vector<int>> adj;
    for (auto& kv : edge_samples) {
        if (static_cast<int>(kv.second.size()) < opts.min_pair_views) continue;
        edges[kv.first] = robust_average(kv.second);
        adj[kv.first.first].push_back(kv.first.second);
        adj[kv.first.second].push_back(kv.first.first);
    }
    report.n_edges = static_cast<int>(edges.size());

    // Relative pose T_from←to for an arbitrary ordered pair, using the
    // canonical edge (and inverting when the stored direction is reversed).
    auto rel_pose = [&](int from, int to, Matx44d& out) -> bool {
        auto it = edges.find({std::min(from, to), std::max(from, to)});
        if (it == edges.end()) return false;
        out = (from < to) ? it->second : geom::invert_rigid(it->second);
        return true;
    };

    // --- 3. anchor + BFS placement --------------------------------------------
    int anchor = opts.anchor_tag_id;
    if (anchor < 0 || ids.find(anchor) == ids.end()) anchor = *ids.begin();
    report.anchor_id = anchor;

    std::map<int, Matx44d> world;  // tag id → T_world←tag
    world[anchor] = Matx44d::eye();
    std::deque<int> q{anchor};
    while (!q.empty()) {
        int cur = q.front();
        q.pop_front();
        auto ai = adj.find(cur);
        if (ai == adj.end()) continue;
        for (int nb : ai->second) {
            if (world.count(nb)) continue;
            Matx44d rel;  // T_cur←nb
            if (!rel_pose(cur, nb, rel)) continue;
            world[nb] = world[cur] * rel;  // T_world←nb
            q.push_back(nb);
        }
    }

    for (int id : ids) {
        if (!world.count(id)) report.unreached_ids.push_back(id);
    }
    report.connected = report.unreached_ids.empty();

    // --- 4. optional pose-averaging relaxation over redundant edges -----------
    if (opts.rotation_refine && world.size() > 2) {
        for (int iter = 0; iter < 10; ++iter) {
            for (int id : ids) {
                if (id == anchor || !world.count(id)) continue;
                std::vector<Matx44d> est;
                auto ai = adj.find(id);
                if (ai == adj.end()) continue;
                for (int nb : ai->second) {
                    if (!world.count(nb)) continue;
                    Matx44d rel;  // T_nb←id
                    if (!rel_pose(nb, id, rel)) continue;
                    est.push_back(world[nb] * rel);  // T_world←id via nb
                }
                if (!est.empty()) world[id] = geom::average_poses(est);
            }
        }
    }

    // --- 5. optional floor-plane re-gauge → FitraWorld (floor z=0, z-up) ------
    if (opts.fit_floor_plane && world.size() >= 3) {
        std::vector<Vec3d> centres;
        centres.reserve(world.size());
        for (auto& kv : world) centres.push_back(geom::trans_of(kv.second));
        double thick = 0.0;
        Vec3d n = fit_plane_normal(centres, thick);
        report.floor_plane_rms_m = thick;

        // Sign the normal to agree with the anchor tag's own +Z (floor-up).
        Vec3d anchor_z = geom::rot_of(world[anchor]) * Vec3d(0, 0, 1);
        if (n.dot(anchor_z) < 0.0) n = -n;

        // (a) align plane normal → world +Z (rotation about origin).
        Matx44d G = geom::compose(rotation_between(n, Vec3d(0, 0, 1)), Vec3d(0, 0, 0));
        for (auto& kv : world) kv.second = G * kv.second;

        // (b) cancel anchor in-plane yaw so its X axis is world +X.
        Vec3d xa = geom::rot_of(world[anchor]) * Vec3d(1, 0, 0);
        double yaw = std::atan2(xa[1], xa[0]);
        Matx44d Rz = geom::compose(rot_z(-yaw), Vec3d(0, 0, 0));
        for (auto& kv : world) kv.second = Rz * kv.second;

        // (c) translate so the anchor centre is the origin and the mean tag
        //     height is the floor (z=0).
        double mean_z = 0.0;
        for (auto& kv : world) mean_z += geom::trans_of(kv.second)[2];
        mean_z /= static_cast<double>(world.size());
        Vec3d ac = geom::trans_of(world[anchor]);
        Matx44d Ts = geom::compose(Matx33d::eye(), Vec3d(-ac[0], -ac[1], -mean_z));
        for (auto& kv : world) kv.second = Ts * kv.second;
    }

    // --- 6. emit FloorTagMap (sorted by id) -----------------------------------
    out_map.origin_tag_id = anchor;
    for (int id : ids) {
        auto it = world.find(id);
        if (it == world.end()) continue;  // unreached → omitted
        FloorTag t;
        t.id = id;
        t.size_m = opts.tag_size_m;
        t.T_world_tag = geom::T_world_marker::from_raw(it->second);
        out_map.tags.push_back(std::move(t));
    }

    if (!report.connected) {
        report.message = "co-visibility graph split: " +
                         std::to_string(report.unreached_ids.size()) +
                         " tag(s) not connected to the anchor";
    } else {
        report.message = "ok";
    }
    return report.connected;
}

}  // namespace fitra::lift
