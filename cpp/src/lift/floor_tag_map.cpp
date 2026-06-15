#include "lift/floor_tag_map.hpp"

#include <cmath>
#include <stdexcept>

#include "lift/apriltag_marker.hpp"  // tag_object_corners

namespace fitra::lift {

namespace {

constexpr char kSchema[] = "fitra_floor_tag_map_v1";

// roll(x), pitch(y), yaw(z) in degrees -> R = Rz(yaw) * Ry(pitch) * Rx(roll).
cv::Matx33d rpy_deg_to_R(double roll, double pitch, double yaw) {
    const double d2r = 3.14159265358979323846 / 180.0;
    const double cr = std::cos(roll * d2r), sr = std::sin(roll * d2r);
    const double cp = std::cos(pitch * d2r), sp = std::sin(pitch * d2r);
    const double cy = std::cos(yaw * d2r), sy = std::sin(yaw * d2r);
    cv::Matx33d Rx(1, 0, 0, 0, cr, -sr, 0, sr, cr);
    cv::Matx33d Ry(cp, 0, sp, 0, 1, 0, -sp, 0, cp);
    cv::Matx33d Rz(cy, -sy, 0, sy, cy, 0, 0, 0, 1);
    return Rz * Ry * Rx;
}

cv::Matx44d compose44(const cv::Matx33d& R, const cv::Vec3d& t) {
    cv::Matx44d m = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) m(r, c) = R(r, c);
        m(r, 3) = t[r];
    }
    return m;
}

cv::Vec3d read_vec3(const cv::FileNode& node) {
    cv::Mat m;
    node >> m;
    if (m.empty()) throw std::runtime_error("floor_tag_map: missing 3-vector");
    if (m.depth() != CV_64F) m.convertTo(m, CV_64F);
    m = m.reshape(1, 1);
    if (m.cols < 3) throw std::runtime_error("floor_tag_map: short 3-vector");
    return {m.at<double>(0, 0), m.at<double>(0, 1), m.at<double>(0, 2)};
}

}  // namespace

const FloorTag* FloorTagMap::find(int id) const {
    for (const auto& t : tags) {
        if (t.id == id) return &t;
    }
    return nullptr;
}

std::array<geom::Point3<geom::frame::FitraWorld>, 4> FloorTagMap::world_corners(
    const FloorTag& tag) const {
    std::array<cv::Point3f, 4> obj = tag_object_corners(tag.size_m);
    std::array<geom::Point3<geom::frame::FitraWorld>, 4> out;
    for (int i = 0; i < 4; ++i) {
        geom::Point3<geom::frame::Marker> p(
            static_cast<double>(obj[i].x),
            static_cast<double>(obj[i].y),
            static_cast<double>(obj[i].z));
        out[i] = tag.T_world_tag * p;
    }
    return out;
}

FloorTagMap floor_tag_map_load(const std::string& path) {
    cv::FileStorage fs{path, cv::FileStorage::READ};
    if (!fs.isOpened()) {
        throw std::runtime_error("failed to open floor tag map: " + path);
    }

    FloorTagMap map;
    cv::FileNode origin = fs["origin_tag_id"];
    if (!origin.empty()) map.origin_tag_id = static_cast<int>(origin.real());

    cv::FileNode tags = fs["tags"];
    if (tags.empty() || !tags.isSeq()) {
        throw std::runtime_error("floor tag map has no 'tags' sequence: " + path);
    }

    for (const auto& tn : tags) {
        FloorTag t;
        cv::FileNode idn = tn["id"];
        cv::FileNode szn = tn["size_m"];
        if (idn.empty() || szn.empty()) {
            throw std::runtime_error("floor tag entry missing id/size_m");
        }
        t.id = static_cast<int>(idn.real());
        t.size_m = static_cast<double>(szn.real());
        if (t.size_m <= 0.0) {
            throw std::runtime_error("floor tag size_m must be > 0");
        }

        cv::Vec3d tr = read_vec3(tn["t"]);
        cv::Matx33d R;
        cv::FileNode Rn = tn["R"];
        cv::FileNode rpyn = tn["rpy_deg"];
        if (!Rn.empty()) {
            cv::Mat Rm;
            Rn >> Rm;
            if (Rm.depth() != CV_64F) Rm.convertTo(Rm, CV_64F);
            if (Rm.rows * Rm.cols != 9) {
                throw std::runtime_error("floor tag R must have 9 elements");
            }
            Rm = Rm.reshape(1, 3);
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c) R(r, c) = Rm.at<double>(r, c);
        } else if (!rpyn.empty()) {
            cv::Vec3d rpy = read_vec3(rpyn);
            R = rpy_deg_to_R(rpy[0], rpy[1], rpy[2]);
        } else {
            R = cv::Matx33d::eye();  // floor tag default: lying flat, +Z normal
        }

        t.T_world_tag = geom::T_world_marker::from_raw(compose44(R, tr));
        map.tags.push_back(std::move(t));
    }

    if (map.tags.empty()) {
        throw std::runtime_error("floor tag map is empty: " + path);
    }
    return map;
}

void floor_tag_map_write(const std::string& path, const FloorTagMap& map) {
    cv::FileStorage fs{path, cv::FileStorage::WRITE};
    if (!fs.isOpened()) {
        throw std::runtime_error("failed to open floor tag map for write: " + path);
    }
    fs << "schema" << std::string(kSchema);
    fs << "unit" << std::string("m");
    fs << "coordinate_system"
       << std::string("world: x/y on floor, z up (FitraWorld); T_world_tag is tag->world");
    if (map.origin_tag_id >= 0) fs << "origin_tag_id" << map.origin_tag_id;

    fs << "tags" << "[";
    for (const auto& t : map.tags) {
        const cv::Matx44d& m = t.T_world_tag.raw();
        cv::Mat R = (cv::Mat_<double>(3, 3) <<
            m(0, 0), m(0, 1), m(0, 2),
            m(1, 0), m(1, 1), m(1, 2),
            m(2, 0), m(2, 1), m(2, 2));
        cv::Mat tr = (cv::Mat_<double>(1, 3) << m(0, 3), m(1, 3), m(2, 3));
        fs << "{";
        fs << "id" << t.id;
        fs << "size_m" << t.size_m;
        fs << "R" << R;
        fs << "t" << tr;
        fs << "}";
    }
    fs << "]";
    fs.release();
}

FloorTagMap floor_tag_grid(int rows, int cols, double pitch_m,
                           int first_id, double size_m) {
    if (rows <= 0 || cols <= 0 || pitch_m <= 0.0 || size_m <= 0.0) {
        throw std::runtime_error("floor_tag_grid: invalid dimensions");
    }
    FloorTagMap map;
    map.origin_tag_id = first_id;
    // Centre the grid on the world origin.
    const double x0 = -0.5 * (cols - 1) * pitch_m;
    const double y0 = -0.5 * (rows - 1) * pitch_m;
    int id = first_id;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            FloorTag t;
            t.id = id++;
            t.size_m = size_m;
            cv::Vec3d pos(x0 + c * pitch_m, y0 + r * pitch_m, 0.0);
            t.T_world_tag =
                geom::T_world_marker::from_raw(compose44(cv::Matx33d::eye(), pos));
            map.tags.push_back(std::move(t));
        }
    }
    return map;
}

}  // namespace fitra::lift
