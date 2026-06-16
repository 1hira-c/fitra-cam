#include "lift/charuco_board.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <stdexcept>

namespace fitra::lift {

namespace {

constexpr int kDefaultDict = cv::aruco::DICT_4X4_50;

cv::aruco::CharucoBoard make_board(const CharucoBoardConfig& cfg) {
    if (cfg.squares_x < 2 || cfg.squares_y < 2) {
        throw std::runtime_error("charuco board needs squares_x/y >= 2");
    }
    if (cfg.square_len_m <= 0.0 || cfg.marker_len_m <= 0.0 ||
        cfg.marker_len_m >= cfg.square_len_m) {
        throw std::runtime_error("charuco marker_len must be 0 < marker < square");
    }
    int dict_id = cfg.dictionary < 0 ? kDefaultDict : cfg.dictionary;
    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(dict_id);
    return cv::aruco::CharucoBoard(
        cv::Size(cfg.squares_x, cfg.squares_y),
        static_cast<float>(cfg.square_len_m),
        static_cast<float>(cfg.marker_len_m), dict);
}

}  // namespace

CharucoBoardDetector::CharucoBoardDetector(CharucoBoardConfig cfg)
    : cfg_(std::move(cfg)),
      board_(make_board(cfg_)),
      detector_(board_) {
    if (cfg_.dictionary < 0) cfg_.dictionary = kDefaultDict;
}

CharucoView CharucoBoardDetector::detect(const cv::Mat& image) const {
    CharucoView v;
    if (image.empty()) return v;

    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    // detectBoard interpolates the chessboard corners from the detected ArUco
    // markers. const_cast: the OpenCV detector method is non-const but carries
    // no externally-visible mutation we depend on (params are fixed at build).
    auto& det = const_cast<cv::aruco::CharucoDetector&>(detector_);
    det.detectBoard(gray, v.corners, v.ids);
    return v;
}

void CharucoBoardDetector::match_points(const CharucoView& view,
                                        std::vector<cv::Point3f>& obj,
                                        std::vector<cv::Point2f>& img) const {
    obj.clear();
    img.clear();
    // Board-frame 3D positions of every inner chessboard corner (Z=0).
    const std::vector<cv::Point3f>& all = board_.getChessboardCorners();
    for (std::size_t i = 0; i < view.ids.size(); ++i) {
        const int id = view.ids[i];
        if (id < 0 || static_cast<std::size_t>(id) >= all.size()) continue;
        obj.push_back(all[id]);
        img.push_back(view.corners[i]);
    }
}

int CharucoBoardDetector::total_corners() const {
    return (cfg_.squares_x - 1) * (cfg_.squares_y - 1);
}

}  // namespace fitra::lift
