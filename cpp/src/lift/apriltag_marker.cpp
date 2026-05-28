#include "lift/apriltag_marker.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>

namespace fitra::lift {

const MarkerFace* MarkerBoardConfig::find(int id) const {
    for (const auto& f : faces) {
        if (f.face_id == id) return &f;
    }
    return nullptr;
}

std::array<cv::Point3f, 4> tag_object_corners(double s) {
    float h = static_cast<float>(s * 0.5);
    // aruco corner order is TL, TR, BR, BL with marker X-right, Y-up, Z-out.
    return {
        cv::Point3f(-h,  h, 0.f),  // TL
        cv::Point3f( h,  h, 0.f),  // TR
        cv::Point3f( h, -h, 0.f),  // BR
        cv::Point3f(-h, -h, 0.f),  // BL
    };
}

bool solve_tag_pose(const std::array<cv::Point2f, 4>& corners,
                    double tag_size_m,
                    const cv::Mat& K,
                    const cv::Mat& dist,
                    cv::Matx44d& T_cam_face,
                    double& reproj_rms_px) {
    if (tag_size_m <= 0.0 || K.empty()) return false;

    std::array<cv::Point3f, 4> obj = tag_object_corners(tag_size_m);
    std::vector<cv::Point3f> objv(obj.begin(), obj.end());
    std::vector<cv::Point2f> imgv(corners.begin(), corners.end());

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(objv, imgv, K, dist, rvec, tvec, false,
                           cv::SOLVEPNP_IPPE_SQUARE);
    if (!ok) return false;

    cv::Mat R;
    cv::Rodrigues(rvec, R);  // 3x3 CV_64F
    T_cam_face = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) T_cam_face(r, c) = R.at<double>(r, c);
        T_cam_face(r, 3) = tvec.at<double>(r);
    }

    // Reprojection RMS over the 4 corners.
    std::vector<cv::Point2f> proj;
    cv::projectPoints(objv, rvec, tvec, K, dist, proj);
    double s2 = 0.0;
    for (int i = 0; i < 4; ++i) {
        cv::Point2f d = proj[i] - imgv[i];
        s2 += static_cast<double>(d.x) * d.x + static_cast<double>(d.y) * d.y;
    }
    reproj_rms_px = std::sqrt(s2 / 4.0);
    return true;
}

namespace {

constexpr int kDefaultDict = cv::aruco::DICT_APRILTAG_36h11;

cv::aruco::ArucoDetector make_detector(int dictionary_id) {
    if (dictionary_id < 0) dictionary_id = kDefaultDict;
    cv::aruco::Dictionary dictionary =
        cv::aruco::getPredefinedDictionary(dictionary_id);
    cv::aruco::DetectorParameters params;
    // Sub-pixel corner refinement matters for sub-mm extrinsic accuracy.
    params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    return cv::aruco::ArucoDetector(dictionary, params);
}

}  // namespace

AprilTagDetector::AprilTagDetector(MarkerBoardConfig cfg)
    : cfg_(std::move(cfg)),
      detector_(make_detector(cfg_.dictionary)) {
    // Persist the resolved dictionary id on cfg_ so config() reports it.
    if (cfg_.dictionary < 0) cfg_.dictionary = kDefaultDict;
}

std::vector<TagDetection> AprilTagDetector::detect(const cv::Mat& image,
                                                   const cv::Mat& K,
                                                   const cv::Mat& dist) {
    std::vector<TagDetection> out;
    if (image.empty()) return out;

    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;
    detector_.detectMarkers(gray, corners, ids, rejected);

    for (std::size_t i = 0; i < ids.size(); ++i) {
        const MarkerFace* face = cfg_.find(ids[i]);
        if (!face) continue;  // an ID we don't track — ignore

        TagDetection det;
        det.face_id = ids[i];
        for (int c = 0; c < 4; ++c) det.corners[c] = corners[i][c];
        det.pose_ok = solve_tag_pose(det.corners, face->tag_size_m, K, dist,
                                     det.T_cam_face, det.reproj_rms_px);
        out.push_back(std::move(det));
    }
    return out;
}

}  // namespace fitra::lift
