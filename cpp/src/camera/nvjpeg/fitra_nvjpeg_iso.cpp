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

#include <cuda_runtime.h>
#include <cudaEGL.h>

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
    // --- EGL -> CUDA bridge (M1, all-GPU front-end) ----------------------
    // The RGBA dst surface is registered into CUDA once via its EGL image;
    // the resulting pitch-linear device pointer aliases the same memory VIC
    // writes, so a CUDA kernel reads decode output with no host round-trip.
    // Registration is expensive, so it is cached and torn down only when the
    // surface is re-allocated (resolution change).
    bool               egl_registered = false;
    void*              egl_image      = nullptr;  // NvBufSurface EGL image
    CUgraphicsResource egl_res        = nullptr;  // CUDA-registered EGL resource
    void*              dev_ptr        = nullptr;  // pitch-linear RGBA8 CUDA device ptr
    int                dev_pitch      = 0;        // device row stride (bytes)
};

// Tear down the cached EGL->CUDA registration (before the dst surface is freed
// or re-allocated). Safe to call when nothing is registered.
void release_egl(Handle* hd) {
    if (!hd->egl_registered) return;
    if (hd->egl_res) { cuGraphicsUnregisterResource(hd->egl_res); hd->egl_res = nullptr; }
    if (hd->dst_surf) NvBufSurfaceUnMapEglImage(hd->dst_surf, 0);
    hd->egl_image = nullptr;
    hd->dev_ptr   = nullptr;
    hd->dev_pitch = 0;
    hd->egl_registered = false;
}

// Register the dst surface's EGL image into CUDA (once) and cache the
// pitch-linear device pointer. Returns false on any EGL/CUDA failure.
bool ensure_egl(Handle* hd) {
    if (hd->egl_registered) return true;
    if (!hd->dst_surf) return false;
    // cuGraphicsEGLRegisterImage (driver API) needs a CUDA context current on
    // this thread. The decode thread may have only touched HW NVJPEG + VIC so
    // far (no cudart call), so force the primary context to bind here. Cheap
    // and idempotent after the first call.
    cudaFree(0);
    if (NvBufSurfaceMapEglImage(hd->dst_surf, 0) != 0) return false;
    void* egl = hd->dst_surf->surfaceList[0].mappedAddr.eglImage;
    if (cuGraphicsEGLRegisterImage(&hd->egl_res, egl,
                                   CU_GRAPHICS_MAP_RESOURCE_FLAGS_NONE) != CUDA_SUCCESS) {
        NvBufSurfaceUnMapEglImage(hd->dst_surf, 0);
        hd->egl_res = nullptr;
        return false;
    }
    CUeglFrame f;
    if (cuGraphicsResourceGetMappedEglFrame(&f, hd->egl_res, 0, 0) != CUDA_SUCCESS ||
        f.frameType != CU_EGL_FRAME_TYPE_PITCH || f.planeCount < 1 ||
        f.frame.pPitch[0] == nullptr) {
        cuGraphicsUnregisterResource(hd->egl_res);
        hd->egl_res = nullptr;
        NvBufSurfaceUnMapEglImage(hd->dst_surf, 0);
        return false;
    }
    hd->egl_image      = egl;
    hd->dev_ptr        = f.frame.pPitch[0];
    hd->dev_pitch      = static_cast<int>(f.pitch);
    hd->egl_registered = true;
    return true;
}

// Allocate (once) the pitch-linear RGBA destination for the transform and map
// it for CPU read. Returns false on failure.
bool ensure_dst(Handle* hd, int w, int h) {
    if (hd->dst_fd >= 0 && hd->w == w && hd->h == h) return true;
    release_egl(hd);  // EGL registration is bound to the old surface; drop it first
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

// Decode `jpeg` on the HW NVJPEG block, VIC-transform YUV->RGBA into the cached
// dst surface, and ensure the EGL->CUDA registration is live. On success sets
// dw/dh and leaves hd->dev_ptr / hd->dev_pitch pointing at the RGBA device
// buffer. Shared by the M1 (decode_cuda) and M2 (preprocess) entry points.
bool decode_transform_egl(Handle* hd, const unsigned char* jpeg, unsigned long n,
                          uint32_t& dw, uint32_t& dh) {
    int src_fd = -1; uint32_t pf = 0;
    if (hd->dec->decodeToFd(src_fd, const_cast<unsigned char*>(jpeg), n, pf, dw, dh) < 0)
        return false;
    if (!ensure_dst(hd, static_cast<int>(dw), static_cast<int>(dh))) return false;

    NvBufSurf::NvCommonTransformParams tp;
    std::memset(&tp, 0, sizeof(tp));
    tp.src_width = dw; tp.src_height = dh; tp.src_top = 0; tp.src_left = 0;
    tp.dst_width = dw; tp.dst_height = dh; tp.dst_top = 0; tp.dst_left = 0;
    tp.flag   = (NvBufSurfTransform_Transform_Flag)(NVBUFSURF_TRANSFORM_FILTER);
    tp.flip   = NvBufSurfTransform_None;
    tp.filter = NvBufSurfTransformInter_Nearest;
    if (NvBufSurf::NvTransform(&tp, src_fd, hd->dst_fd) != 0) return false;
    return ensure_egl(hd);
}

}  // namespace

// Defined in fitra_nvjpeg_kernels.cu (same .so).
extern "C" int fitra_nvjpeg_preprocess_launch(
    const void* rgba_dev, int in_w, int in_h, int in_pitch_bytes,
    const double* M_inv6, int out_w, int out_h,
    const float* mean_bgr, const float* inv_std_bgr,
    float* dst_chw_dev, void* stream);

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

// Decode `jpeg` on the HW NVJPEG block + VIC (YUV->RGBA) exactly like
// fitra_nvjpeg_decode_rgba, then expose the RGBA result as a CUDA device
// pointer via the cached EGL->CUDA bridge (all-GPU front-end, M1). No host
// round-trip: *dev sees the same memory VIC wrote.
//   *w/*h        : image dimensions
//   *dev_pitch   : device row stride in bytes (>= w*4)
//   *dev         : pitch-linear RGBA8 CUDA device pointer (valid until the next
//                  decode/destroy on this handle) -- the all-GPU front-end input
//   *host        : CPU-mapped RGBA8 of the SAME surface (free; the surface is
//                  CPU-mapped for the regression check). M1 still builds BGR
//                  from this; M2 will drop it and consume *dev directly.
//   *host_pitch  : host row stride in bytes
// When check != 0, copies the device buffer back to host and writes the
// R-channel mean of the CPU map and of the device copy into *cpu_mean /
// *dev_mean so the caller can regression-check the bridge. Returns 0 on
// success, -1 on failure.
__attribute__((visibility("default")))
int fitra_nvjpeg_decode_cuda(void* handle,
                             const unsigned char* jpeg, unsigned long n,
                             int* w, int* h, int* dev_pitch, void** dev,
                             const unsigned char** host, int* host_pitch,
                             int check, double* cpu_mean, double* dev_mean) {
    Handle* hd = static_cast<Handle*>(handle);
    if (!hd || !hd->dec || !jpeg || n == 0) return -1;
    if (!w || !h || !dev_pitch || !dev || !host || !host_pitch) return -1;

    uint32_t dw = 0, dh = 0;
    if (!decode_transform_egl(hd, jpeg, n, dw, dh)) return -1;
    NvBufSurfaceSyncForCpu(hd->dst_surf, 0, 0);  // M1 reads RGBA on host for BGR
    const auto& s = hd->dst_surf->surfaceList[0];
    *w          = static_cast<int>(dw);
    *h          = static_cast<int>(dh);
    *dev_pitch  = hd->dev_pitch;
    *dev        = hd->dev_ptr;
    *host       = static_cast<const unsigned char*>(s.mappedAddr.addr[0]);
    *host_pitch = static_cast<int>(s.planeParams.pitch[0]);

    if (check && cpu_mean && dev_mean) {
        double cs = 0.0;
        for (uint32_t r = 0; r < dh; ++r) {
            const unsigned char* row = *host + static_cast<size_t>(r) * (*host_pitch);
            for (uint32_t c = 0; c < dw; ++c) cs += row[4 * c];  // R channel
        }
        *cpu_mean = cs / (static_cast<double>(dw) * dh);
        // ... vs device->host copy from the EGL CUDA pointer.
        std::vector<unsigned char> hb(static_cast<size_t>(dw) * 4 * dh);
        if (cudaMemcpy2D(hb.data(), static_cast<size_t>(dw) * 4,
                         hd->dev_ptr, hd->dev_pitch,
                         static_cast<size_t>(dw) * 4, dh,
                         cudaMemcpyDeviceToHost) == cudaSuccess) {
            double ds = 0.0;
            for (uint32_t r = 0; r < dh; ++r) {
                const unsigned char* row = hb.data() + static_cast<size_t>(r) * dw * 4;
                for (uint32_t c = 0; c < dw; ++c) ds += row[4 * c];
            }
            *dev_mean = ds / (static_cast<double>(dw) * dh);
        } else {
            *dev_mean = -1.0;
        }
    }
    return 0;
}

// Run the RTMPose preprocess kernel from the handle's LAST decode_cuda /
// decode_to_device RGBA output into `dst_chw_dev` (device, 3*out_h*out_w
// floats). Synchronizes so the result is ready when this returns (the per-camera
// worker hands the buffer to the central inference thread). M_inv6 / mean_bgr /
// inv_std_bgr as in fitra_nvjpeg_preprocess_launch. Returns 0 on success.
__attribute__((visibility("default")))
int fitra_nvjpeg_preprocess_from_last(void* handle,
                                      const double* M_inv6, int out_w, int out_h,
                                      const float* mean_bgr, const float* inv_std_bgr,
                                      float* dst_chw_dev) {
    Handle* hd = static_cast<Handle*>(handle);
    if (!hd || !hd->egl_registered || !hd->dev_ptr || !dst_chw_dev) return -1;
    if (fitra_nvjpeg_preprocess_launch(hd->dev_ptr, hd->w, hd->h, hd->dev_pitch,
                                       M_inv6, out_w, out_h, mean_bgr, inv_std_bgr,
                                       dst_chw_dev, nullptr) != 0)
        return -1;
    return (cudaStreamSynchronize(nullptr) == cudaSuccess) ? 0 : -1;
}

// Decode `jpeg` and expose only the RGBA CUDA device pointer (no CPU map / no
// sync) -- the pure all-GPU path. Use when the BGR host image is not needed
// (M3+). For now the worker uses decode_cuda (host map for YOLOX/calib) plus
// preprocess_from_last, so this is provided for completeness/M3.
__attribute__((visibility("default")))
int fitra_nvjpeg_decode_to_device(void* handle, const unsigned char* jpeg,
                                  unsigned long n, int* w, int* h,
                                  int* dev_pitch, void** dev) {
    Handle* hd = static_cast<Handle*>(handle);
    if (!hd || !hd->dec || !jpeg || n == 0) return -1;
    if (!w || !h || !dev_pitch || !dev) return -1;
    uint32_t dw = 0, dh = 0;
    if (!decode_transform_egl(hd, jpeg, n, dw, dh)) return -1;
    *w = static_cast<int>(dw); *h = static_cast<int>(dh);
    *dev_pitch = hd->dev_pitch; *dev = hd->dev_ptr;
    return 0;
}

__attribute__((visibility("default")))
void fitra_nvjpeg_destroy(void* handle) {
    Handle* hd = static_cast<Handle*>(handle);
    if (!hd) return;
    release_egl(hd);
    if (hd->mapped && hd->dst_surf) NvBufSurfaceUnMap(hd->dst_surf, 0, 0);
    if (hd->dst_fd >= 0) NvBufSurf::NvDestroy(hd->dst_fd);
    delete hd->dec;
    delete hd;
}

}  // extern "C"
