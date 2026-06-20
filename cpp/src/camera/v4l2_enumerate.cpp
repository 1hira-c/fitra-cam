#include "camera/v4l2_enumerate.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <filesystem>

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
    for (fi.index = 0; xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fi) == 0; ++fi.index) {
        if (fi.type != V4L2_FRMIVAL_TYPE_DISCRETE) break;  // stepwise: skip
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
    for (fs.index = 0; xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fs) == 0; ++fs.index) {
        if (fs.type != V4L2_FRMSIZE_TYPE_DISCRETE) break;  // stepwise: skip
        V4l2FrameSize sz;
        sz.width  = static_cast<int>(fs.discrete.width);
        sz.height = static_cast<int>(fs.discrete.height);
        sz.fps = enum_frame_rates(fd, pixfmt, sz.width, sz.height);
        sizes.push_back(std::move(sz));
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
        f.description =
            reinterpret_cast<const char*>(fd_desc.description);
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

    for (const auto& entry : std::filesystem::directory_iterator(by_path, ec)) {
        if (ec) break;
        const std::string link = entry.path().string();
        const int fd = ::open(link.c_str(), O_RDONLY | O_NONBLOCK);
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
        dev.card   = reinterpret_cast<const char*>(cap.card);
        dev.driver = reinterpret_cast<const char*>(cap.driver);
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
