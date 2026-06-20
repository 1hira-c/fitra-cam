#pragma once
//
// V4L2 device enumeration for the setup wizard (docs/design/core-pipeline-setup-mode.md).
// Walks /dev/v4l/by-path/* and reports each capture-capable camera's pixel
// formats, discrete frame sizes, and frame rates via VIDIOC_ENUM_*. Pure ioctl
// — no CUDA/TensorRT/OpenCV — so the GPU-less Setup module can list cameras.

#include <string>
#include <vector>

namespace fitra::camera {

struct V4l2FrameSize {
    int width  = 0;
    int height = 0;
    std::vector<double> fps;   // discrete frame rates for this size (Hz)
};

struct V4l2Format {
    std::string fourcc;        // e.g. "MJPG", "YUYV"
    std::string description;   // driver-provided human label
    std::vector<V4l2FrameSize> sizes;
};

struct V4l2Device {
    std::string by_path;       // stable /dev/v4l/by-path/... symlink (open this)
    std::string dev_node;      // resolved /dev/videoN
    std::string card;          // VIDIOC_QUERYCAP card name
    std::string driver;        // VIDIOC_QUERYCAP driver name
    std::vector<V4l2Format> formats;
};

// Enumerate capture-capable cameras under /dev/v4l/by-path. Never throws: nodes
// that fail to open / query are skipped. Returns an empty vector if the by-path
// directory is absent. Sorted by by_path for a stable UI ordering.
std::vector<V4l2Device> enumerate_v4l2_cameras();

}  // namespace fitra::camera
