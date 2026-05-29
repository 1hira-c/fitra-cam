// Isolated Jetson hardware-NVJPEG decoder, built into libfitra_nvjpeg.so.
//
// WHY A SEPARATE .so (do not merge into the main binary):
//   Jetson's libnvjpeg.so exports the libjpeg API (jpeg_CreateDecompress, ...)
//   as UNVERSIONED global symbols, while OpenCV's imgcodecs imports the
//   VERSIONED libjpeg-turbo symbols (jpeg_*@LIBJPEG_8.0). If the MMAPI
//   NvJPEGDecoder is linked into the same image as cv::imdecode, the two
//   libjpeg ABIs collide and cv::imdecode breaks (empty image / struct-size
//   mismatch / crash). Isolating this code in a .so loaded with
//   dlopen(RTLD_DEEPBIND|RTLD_LOCAL) keeps libnvjpeg's jpeg_* symbols local to
//   this library so the main binary's cv::imdecode (the default MJPEG path)
//   keeps working. See docs/design/core-pipeline-nvjpeg-decode.md.
//
// Pipeline: MJPEG bytes -> NvJPEGDecoder::decodeToFd (HW, NVMM YUV422M) ->
// NvBufSurfTransform (VIC, YUV -> RGBA pitch-linear) -> map -> pack to BGR.
// All but the final pack runs on dedicated HW blocks, freeing the CPU from
// JPEG entropy decode.
//
// The handle is single-threaded (NvJPEGDecoder is not thread-safe); create one
// per decode thread, mirroring the per-camera Yolox context pattern.

#include "NvJpegDecoder.h"
#include "NvBufSurface.h"
#include "nvbufsurface.h"

#include <cstdint>
#include <cstring>
#include <vector>

namespace {

struct Handle {
    NvJPEGDecoder* dec        = nullptr;
    int            dst_fd     = -1;
    NvBufSurface*  dst_surf   = nullptr;
    int            w          = 0;
    int            h          = 0;
    bool           mapped     = false;
};

// Allocate (once) the pitch-linear RGBA destination for the transform and map
// it for CPU read. Returns false on failure.
bool ensure_dst(Handle* hd, int w, int h) {
    if (hd->dst_fd >= 0 && hd->w == w && hd->h == h) return true;
    if (hd->mapped && hd->dst_surf) { NvBufSurfaceUnMap(hd->dst_surf, 0, 0); hd->mapped = false; }
    if (hd->dst_fd >= 0) { NvBufSurf::NvDestroy(hd->dst_fd); hd->dst_fd = -1; }

    NvBufSurf::NvCommonAllocateParams ap;
    std::memset(&ap, 0, sizeof(ap));
    ap.width       = static_cast<uint32_t>(w);
    ap.height      = static_cast<uint32_t>(h);
    ap.layout      = NVBUF_LAYOUT_PITCH;
    // VIC supports only 32-bit packed RGB outputs (24-bit BGR is rejected by
    // NvBufSurfaceCreate/NvBufSurfTransform). Output RGBA; the caller does one
    // NEON cv::cvtColor(RGBA->BGR).
    ap.colorFormat = NVBUF_COLOR_FORMAT_RGBA;
    ap.memType     = NVBUF_MEM_SURFACE_ARRAY;
    ap.memtag      = NvBufSurfaceTag_VIDEO_CONVERT;
    if (NvBufSurf::NvAllocate(&ap, 1, &hd->dst_fd) != 0) { hd->dst_fd = -1; return false; }
    if (NvBufSurfaceFromFd(hd->dst_fd, (void**)&hd->dst_surf) != 0) return false;
    if (NvBufSurfaceMap(hd->dst_surf, 0, 0, NVBUF_MAP_READ) != 0) return false;
    hd->mapped = true;
    hd->w = w; hd->h = h;
    return true;
}

}  // namespace

extern "C" {

__attribute__((visibility("default")))
void* fitra_nvjpeg_create() {
    Handle* hd = new Handle();
    hd->dec = NvJPEGDecoder::createJPEGDecoder("fitra_nvjpeg");
    if (!hd->dec) { delete hd; return nullptr; }
    return hd;
}

// Decode `jpeg` (MJPEG bytes) on the HW NVJPEG block + VIC (YUV->RGBA). On
// success returns a pointer to the mapped RGBA8 output (zero-copy; valid until
// the next decode/destroy on this handle) and sets *w/*h/*pitch (row stride in
// bytes). Returns nullptr on failure. The caller does the single RGBA->BGR
// NEON conversion (cv::cvtColor) to keep RTMPose's BGR contract.
__attribute__((visibility("default")))
const unsigned char* fitra_nvjpeg_decode_rgba(void* handle,
                                              const unsigned char* jpeg,
                                              unsigned long n,
                                              int* w, int* h, int* pitch) {
    Handle* hd = static_cast<Handle*>(handle);
    if (!hd || !hd->dec || !jpeg || n == 0) return nullptr;

    int src_fd = -1; uint32_t pf = 0, dw = 0, dh = 0;
    if (hd->dec->decodeToFd(src_fd, const_cast<unsigned char*>(jpeg), n, pf, dw, dh) < 0)
        return nullptr;
    if (!ensure_dst(hd, static_cast<int>(dw), static_cast<int>(dh))) return nullptr;

    NvBufSurf::NvCommonTransformParams tp;
    std::memset(&tp, 0, sizeof(tp));
    tp.src_width = dw; tp.src_height = dh; tp.src_top = 0; tp.src_left = 0;
    tp.dst_width = dw; tp.dst_height = dh; tp.dst_top = 0; tp.dst_left = 0;
    tp.flag   = (NvBufSurfTransform_Transform_Flag)(NVBUFSURF_TRANSFORM_FILTER);
    tp.flip   = NvBufSurfTransform_None;
    tp.filter = NvBufSurfTransformInter_Nearest;
    if (NvBufSurf::NvTransform(&tp, src_fd, hd->dst_fd) != 0) return nullptr;

    NvBufSurfaceSyncForCpu(hd->dst_surf, 0, 0);
    auto& s = hd->dst_surf->surfaceList[0];
    *w     = static_cast<int>(dw);
    *h     = static_cast<int>(dh);
    *pitch = static_cast<int>(s.planeParams.pitch[0]);
    return static_cast<const std::uint8_t*>(s.mappedAddr.addr[0]);
}

__attribute__((visibility("default")))
void fitra_nvjpeg_destroy(void* handle) {
    Handle* hd = static_cast<Handle*>(handle);
    if (!hd) return;
    if (hd->mapped && hd->dst_surf) NvBufSurfaceUnMap(hd->dst_surf, 0, 0);
    if (hd->dst_fd >= 0) NvBufSurf::NvDestroy(hd->dst_fd);
    delete hd->dec;
    delete hd;
}

}  // extern "C"
