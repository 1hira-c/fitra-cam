// CUDA preprocessing kernels for the all-GPU front-end (M2), built into
// libfitra_nvjpeg.so alongside the HW decoder + EGL->CUDA bridge.
//
// The RTMPose preprocess (crop[bbox] + affine warp + BGR ImageNet normalize +
// HWC->CHW) is the last full-frame CPU pass on the HW-decode route. This kernel
// runs it on the GPU directly from the EGL-bridged RGBA device buffer, so the
// TRT input is produced with no host round-trip.
//
// Geometry: the host already computes the 2x3 inverse affine M_inv (dst->src)
// in RtmPose::preprocess_to_blob (it is needed anyway to remap keypoints back).
// cv::warpAffine samples each output pixel (ox,oy) from the source at
// M_inv * (ox,oy,1); this kernel does the identical sampling, so geometry
// matches OpenCV exactly. The only numeric difference vs the CPU path is float
// bilinear here vs OpenCV's fixed-point interpolation (sub-LSB), well within the
// keypoint L2 tolerance.
//
// Output channel order is [B, G, R] to match the rtmlib-exported RTMPose ONNX
// (same as the CPU preprocess_to_blob).

#include <cuda_runtime.h>
#include <cstdint>

namespace {

// One output pixel per thread. rgba is pitch-linear (in_pitch_px = pitch/4).
// Out-of-range bilinear neighbors contribute 0 (matches cv::warpAffine's
// default BORDER_CONSTANT=0).
__global__ void preprocess_rtmpose_kernel(
    const uchar4* __restrict__ rgba, int in_w, int in_h, int in_pitch_px,
    float a, float b, float c, float d, float e, float f,
    int out_w, int out_h,
    float mb, float mg, float mr, float ib, float ig, float ir,
    float* __restrict__ chw) {
    const int ox = blockIdx.x * blockDim.x + threadIdx.x;
    const int oy = blockIdx.y * blockDim.y + threadIdx.y;
    if (ox >= out_w || oy >= out_h) return;

    const float sx = a * ox + b * oy + c;
    const float sy = d * ox + e * oy + f;
    const int x0 = static_cast<int>(floorf(sx));
    const int y0 = static_cast<int>(floorf(sy));
    const float fx = sx - x0;
    const float fy = sy - y0;
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;

    float R = 0.f, G = 0.f, B = 0.f;
    // Accumulate the (up to) four neighbors, skipping any that fall outside the
    // source (per-neighbor zero border).
    auto tap = [&](int xi, int yi, float wt) {
        if (xi < 0 || xi >= in_w || yi < 0 || yi >= in_h) return;
        const uchar4 p = rgba[static_cast<size_t>(yi) * in_pitch_px + xi];
        R += wt * p.x;  // RGBA: x=R, y=G, z=B
        G += wt * p.y;
        B += wt * p.z;
    };
    tap(x0, y0, (1.f - fx) * (1.f - fy));
    tap(x1, y0, fx * (1.f - fy));
    tap(x0, y1, (1.f - fx) * fy);
    tap(x1, y1, fx * fy);

    const int npx = out_w * out_h;
    const int idx = oy * out_w + ox;
    chw[0 * npx + idx] = (B - mb) * ib;  // channel 0 = B
    chw[1 * npx + idx] = (G - mg) * ig;  // channel 1 = G
    chw[2 * npx + idx] = (R - mr) * ir;  // channel 2 = R
}

}  // namespace

extern "C" {

// Launch the RTMPose preprocess kernel on a device RGBA buffer, writing a
// normalized BGR CHW float tensor to `dst_chw_dev` (device, 3*out_h*out_w
// floats). Geometry is the caller-supplied inverse affine M_inv6 = {a,b,c,d,e,f}
// (row-major 2x3, dst->src). mean_bgr / inv_std_bgr are 3 floats each (B,G,R
// order). Runs on `stream` (0 = default); does NOT synchronize. Returns 0 on
// success, -1 on a launch error.
__attribute__((visibility("default")))
int fitra_nvjpeg_preprocess_launch(
    const void* rgba_dev, int in_w, int in_h, int in_pitch_bytes,
    const double* M_inv6, int out_w, int out_h,
    const float* mean_bgr, const float* inv_std_bgr,
    float* dst_chw_dev, void* stream) {
    if (!rgba_dev || !M_inv6 || !mean_bgr || !inv_std_bgr || !dst_chw_dev) return -1;
    if (in_w <= 0 || in_h <= 0 || out_w <= 0 || out_h <= 0 || (in_pitch_bytes & 3)) return -1;

    const dim3 block(16, 16);
    const dim3 grid((out_w + block.x - 1) / block.x, (out_h + block.y - 1) / block.y);
    preprocess_rtmpose_kernel<<<grid, block, 0, static_cast<cudaStream_t>(stream)>>>(
        static_cast<const uchar4*>(rgba_dev), in_w, in_h, in_pitch_bytes / 4,
        static_cast<float>(M_inv6[0]), static_cast<float>(M_inv6[1]),
        static_cast<float>(M_inv6[2]), static_cast<float>(M_inv6[3]),
        static_cast<float>(M_inv6[4]), static_cast<float>(M_inv6[5]),
        out_w, out_h,
        mean_bgr[0], mean_bgr[1], mean_bgr[2],
        inv_std_bgr[0], inv_std_bgr[1], inv_std_bgr[2],
        dst_chw_dev);
    return (cudaGetLastError() == cudaSuccess) ? 0 : -1;
}

// Correctness/bench entry: run the real kernel on a HOST RGBA image. Uploads
// rgba (pitch-linear) to the device, launches the kernel, copies the CHW result
// back to `out_chw_host` (3*out_h*out_w floats). Self-contained (allocates and
// frees its own device buffers). Returns 0 on success, -1 on any CUDA error.
__attribute__((visibility("default")))
int fitra_nvjpeg_preprocess_rgba_host(
    const unsigned char* rgba, int in_w, int in_h, int in_pitch_bytes,
    const double* M_inv6, int out_w, int out_h,
    const float* mean_bgr, const float* inv_std_bgr,
    float* out_chw_host) {
    if (!rgba || !out_chw_host) return -1;
    const size_t in_bytes  = static_cast<size_t>(in_pitch_bytes) * in_h;
    const size_t out_floats = static_cast<size_t>(3) * out_w * out_h;

    void*  d_rgba = nullptr;
    float* d_chw  = nullptr;
    int rc = -1;
    do {
        if (cudaMalloc(&d_rgba, in_bytes) != cudaSuccess) break;
        if (cudaMalloc(&d_chw, out_floats * sizeof(float)) != cudaSuccess) break;
        if (cudaMemcpy(d_rgba, rgba, in_bytes, cudaMemcpyHostToDevice) != cudaSuccess) break;
        if (fitra_nvjpeg_preprocess_launch(d_rgba, in_w, in_h, in_pitch_bytes,
                                           M_inv6, out_w, out_h,
                                           mean_bgr, inv_std_bgr, d_chw, nullptr) != 0) break;
        if (cudaDeviceSynchronize() != cudaSuccess) break;
        if (cudaMemcpy(out_chw_host, d_chw, out_floats * sizeof(float),
                       cudaMemcpyDeviceToHost) != cudaSuccess) break;
        rc = 0;
    } while (false);
    if (d_rgba) cudaFree(d_rgba);
    if (d_chw)  cudaFree(d_chw);
    return rc;
}

}  // extern "C"
