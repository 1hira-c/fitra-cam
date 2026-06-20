#include "camera/v4l2_enumerate.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <utility>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace fitra::camera {

namespace {

int xioctl(int fd, unsigned long req, void* arg) {
    int r;
    do {
        r = ::ioctl(fd, req, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

std::string fourcc_str(std::uint32_t f) {
    char c[4] = {static_cast<char>(f & 0xff),
                 static_cast<char>((f >> 8) & 0xff),
                 static_cast<char>((f >> 16) & 0xff),
                 static_cast<char>((f >> 24) & 0xff)};
    std::string s;
    for (char ch : c) if (ch >= 0x20 && ch < 0x7f) s += ch;
    return s;
}

std::vector<double> enum_frame_rates(int fd, std::uint32_t pixfmt, int w, int h) {
    std::vector<double> fps;
    v4l2_frmivalenum fi{};
    fi.pixel_format = pixfmt;
    fi.width  = static_cast<__u32>(w);
    fi.height = static_cast<__u32>(h);
    auto fract_fps = [](const v4l2_fract& f) -> double {
        return f.numerator > 0 ? static_cast<double>(f.denominator) /
                                 static_cast<double>(f.numerator)
                               : 0.0;
    };
    for (fi.index = 0; xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fi) == 0; ++fi.index) {
        if (fi.type != V4L2_FRMIVAL_TYPE_DISCRETE) {
            // Stepwise/continuous: a single descriptor with min/max intervals.
            // Synthesize the standard fps that fall inside [min,max] so the UI
            // still offers selectable rates (smallest interval = highest fps).
            const double fps_max = fract_fps(fi.stepwise.min);
            const double fps_min = fract_fps(fi.stepwise.max);
            for (double f : {120.0, 90.0, 60.0, 50.0, 30.0, 25.0, 15.0, 10.0, 5.0}) {
                if (f <= fps_max + 1e-6 && f >= fps_min - 1e-6) fps.push_back(f);
            }
            if (fps.empty() && fps_max > 0.0) fps.push_back(fps_max);
            break;
        }
        if (fi.discrete.numerator > 0) {
            fps.push_back(static_cast<double>(fi.discrete.denominator) /
                          static_cast<double>(fi.discrete.numerator));
        }
    }
    std::sort(fps.begin(), fps.end(), std::greater<double>());
    return fps;
}

std::vector<V4l2FrameSize> enum_frame_sizes(int fd, std::uint32_t pixfmt) {
    std::vector<V4l2FrameSize> sizes;
    v4l2_frmsizeenum fs{};
    fs.pixel_format = pixfmt;
    auto add_size = [&](int w, int h) {
        V4l2FrameSize sz;
        sz.width  = w;
        sz.height = h;
        sz.fps = enum_frame_rates(fd, pixfmt, w, h);
        sizes.push_back(std::move(sz));
    };
    for (fs.index = 0; xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fs) == 0; ++fs.index) {
        if (fs.type != V4L2_FRMSIZE_TYPE_DISCRETE) {
            // Stepwise/continuous: a single descriptor with a min/max/step range.
            // Synthesize the standard resolutions that fit so the UI dropdown is
            // not empty, always including the max (and min when distinct).
            const auto& sw = fs.stepwise;
            const int min_w = static_cast<int>(sw.min_width);
            const int min_h = static_cast<int>(sw.min_height);
            const int max_w = static_cast<int>(sw.max_width);
            const int max_h = static_cast<int>(sw.max_height);
            const int step_w = static_cast<int>(sw.step_width);
            const int step_h = static_cast<int>(sw.step_height);
            // A candidate must lie in [min,max] AND on the step grid, else the
            // driver rejects it at S_FMT (continuous reports step 1).
            auto fits = [&](int w, int h) {
                return w >= min_w && w <= max_w && h >= min_h && h <= max_h &&
                       (step_w <= 1 || (w - min_w) % step_w == 0) &&
                       (step_h <= 1 || (h - min_h) % step_h == 0);
            };
            add_size(max_w, max_h);
            for (auto wh : {std::pair{1920, 1080}, std::pair{1600, 1200},
                            std::pair{1280, 960}, std::pair{1280, 720},
                            std::pair{1024, 768}, std::pair{800, 600},
                            std::pair{640, 480}, std::pair{640, 360},
                            std::pair{320, 240}}) {
                if (fits(wh.first, wh.second) &&
                    !(wh.first == max_w && wh.second == max_h)) {
                    add_size(wh.first, wh.second);
                }
            }
            if (min_w != max_w || min_h != max_h) add_size(min_w, min_h);
            break;
        }
        add_size(static_cast<int>(fs.discrete.width),
                 static_cast<int>(fs.discrete.height));
    }
    return sizes;
}

std::vector<V4l2Format> enum_formats(int fd) {
    std::vector<V4l2Format> formats;
    v4l2_fmtdesc fd_desc{};
    fd_desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (fd_desc.index = 0; xioctl(fd, VIDIOC_ENUM_FMT, &fd_desc) == 0; ++fd_desc.index) {
        V4l2Format f;
        f.fourcc = fourcc_str(fd_desc.pixelformat);
        // V4L2 fixed-size __u8 buffers are documented NUL-terminated, but bound
        // the length defensively against a non-conformant driver over-reading.
        f.description = std::string(
            reinterpret_cast<const char*>(fd_desc.description),
            ::strnlen(reinterpret_cast<const char*>(fd_desc.description),
                      sizeof(fd_desc.description)));
        f.sizes = enum_frame_sizes(fd, fd_desc.pixelformat);
        formats.push_back(std::move(f));
    }
    return formats;
}

}  // namespace

std::vector<V4l2Device> enumerate_v4l2_cameras() {
    std::vector<V4l2Device> devices;
    const std::filesystem::path by_path{"/dev/v4l/by-path"};
    std::error_code ec;
    if (!std::filesystem::exists(by_path, ec) || ec) return devices;

    // Explicit increment with an error_code: a node disappearing mid-iteration
    // must break the loop, not throw filesystem_error out of the daemon.
    std::error_code iter_ec;
    for (std::filesystem::directory_iterator it{by_path, ec}, end;
         !ec && !iter_ec && it != end; it.increment(iter_ec)) {
        const std::filesystem::directory_entry& entry = *it;
        const std::string link = entry.path().string();
        // O_CLOEXEC: this is a multi-threaded daemon that execvp()s mode
        // children; an inherited fd would leak and keep the camera EBUSY.
        const int fd = ::open(link.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) continue;

        v4l2_capability cap{};
        if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) { ::close(fd); continue; }
        // Prefer per-device caps (a single hardware node exposes capture +
        // metadata sub-devices); fall back to the union capabilities.
        const std::uint32_t caps =
            (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps
                                                      : cap.capabilities;
        if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) { ::close(fd); continue; }

        V4l2Device dev;
        dev.by_path = link;
        std::error_code rec;
        auto resolved = std::filesystem::weakly_canonical(entry.path(), rec);
        dev.dev_node = rec ? link : resolved.string();
        dev.card = std::string(
            reinterpret_cast<const char*>(cap.card),
            ::strnlen(reinterpret_cast<const char*>(cap.card), sizeof(cap.card)));
        dev.driver = std::string(
            reinterpret_cast<const char*>(cap.driver),
            ::strnlen(reinterpret_cast<const char*>(cap.driver), sizeof(cap.driver)));
        dev.formats = enum_formats(fd);
        ::close(fd);
        devices.push_back(std::move(dev));
    }
    std::sort(devices.begin(), devices.end(),
              [](const V4l2Device& a, const V4l2Device& b) {
                  return a.by_path < b.by_path;
              });
    return devices;
}

}  // namespace fitra::camera
