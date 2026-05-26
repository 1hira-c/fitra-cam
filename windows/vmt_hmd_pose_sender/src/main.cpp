//
// Phase 15 M1 — vmt_hmd_pose_sender
//
// SteamVR overlay/background app that polls the HMD pose via OpenVR
// (TrackingUniverseStanding) and forwards it to a fitra-cam Jetson over
// OSC/UDP as /fitra/hmd_pose.
//
// Usage:
//   vmt_hmd_pose_sender.exe --jetson 192.168.1.20 --port 39571 --rate-hz 60
//
// Exit codes:
//   0  — graceful Ctrl-C
//   1  — argument parse error
//   2  — could not open OSC socket
//   3  — fatal openvr error (after retry budget exhausted; user can re-run)
//

#include "osc_send.hpp"

#include <openvr.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_stop{false};

void on_signal(int) { g_stop.store(true); }

struct Args {
    std::string   jetson  = "127.0.0.1";
    std::uint16_t port    = 39571;
    double        rate_hz = 60.0;
};

bool parse_args(int argc, char** argv, Args& out, std::string& err) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto need = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                err = std::string("missing value for ") + name;
                return nullptr;
            }
            return argv[++i];
        };
        if (a == "--jetson") {
            const char* v = need("--jetson"); if (!v) return false;
            out.jetson = v;
        } else if (a == "--port") {
            const char* v = need("--port"); if (!v) return false;
            int p = std::atoi(v);
            if (p < 1 || p > 65535) { err = "--port out of range"; return false; }
            out.port = static_cast<std::uint16_t>(p);
        } else if (a == "--rate-hz") {
            const char* v = need("--rate-hz"); if (!v) return false;
            out.rate_hz = std::atof(v);
            if (out.rate_hz <= 0.0 || out.rate_hz > 240.0) {
                err = "--rate-hz must be in (0, 240]";
                return false;
            }
        } else if (a == "--help" || a == "-h") {
            std::printf(
                "vmt_hmd_pose_sender [options]\n"
                "  --jetson <ip>    target host (default 127.0.0.1)\n"
                "  --port   <p>     target UDP port (default 39571)\n"
                "  --rate-hz <hz>   send rate (default 60.0, max 240.0)\n");
            std::exit(0);
        } else {
            err = "unknown argument: " + a;
            return false;
        }
    }
    return true;
}

//
// Convert a vr::HmdMatrix34_t = [R | t] (row-major 3x4) into position and
// quaternion (xyzw). Standard "trace-based" extraction with branchless
// fallback for ill-conditioned rotations.
//
// SteamVR's GetDeviceToAbsoluteTrackingPose returns the device-to-absolute
// transform: a point p_local is at R * p_local + t in absolute space.
// We therefore read translation directly from m[*][3], and R from the
// top-left 3x3.
//
struct PosQuat {
    float x, y, z;
    float qx, qy, qz, qw;
};

PosQuat matrix_to_pos_quat(const vr::HmdMatrix34_t& m) {
    PosQuat out{};
    out.x = m.m[0][3];
    out.y = m.m[1][3];
    out.z = m.m[2][3];

    const float m00 = m.m[0][0], m01 = m.m[0][1], m02 = m.m[0][2];
    const float m10 = m.m[1][0], m11 = m.m[1][1], m12 = m.m[1][2];
    const float m20 = m.m[2][0], m21 = m.m[2][1], m22 = m.m[2][2];

    const float trace = m00 + m11 + m22;
    if (trace > 0.0f) {
        float s = std::sqrt(trace + 1.0f) * 2.0f;
        out.qw = 0.25f * s;
        out.qx = (m21 - m12) / s;
        out.qy = (m02 - m20) / s;
        out.qz = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        out.qw = (m21 - m12) / s;
        out.qx = 0.25f * s;
        out.qy = (m01 + m10) / s;
        out.qz = (m02 + m20) / s;
    } else if (m11 > m22) {
        float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        out.qw = (m02 - m20) / s;
        out.qx = (m01 + m10) / s;
        out.qy = 0.25f * s;
        out.qz = (m12 + m21) / s;
    } else {
        float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        out.qw = (m10 - m01) / s;
        out.qx = (m02 + m20) / s;
        out.qy = (m12 + m21) / s;
        out.qz = 0.25f * s;
    }
    return out;
}

bool try_init_openvr(std::string& err) {
    vr::EVRInitError ie = vr::VRInitError_None;
    vr::VR_Init(&ie, vr::VRApplication_Background);
    if (ie != vr::VRInitError_None) {
        err = vr::VR_GetVRInitErrorAsEnglishDescription(ie);
        return false;
    }
    // Force standing universe so the published pose frame matches the VMT
    // Driver frame seen by fitra-cam's VmtPublisher. Quest defaults to
    // standing; explicit set guards against seated configs.
    if (vr::VRCompositor()) {
        vr::VRCompositor()->SetTrackingSpace(vr::TrackingUniverseStanding);
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    std::string err;
    if (!parse_args(argc, argv, args, err)) {
        std::fprintf(stderr, "error: %s\n", err.c_str());
        return 1;
    }

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    fitra::hmd_sender::OscSender osc;
    if (!osc.open(args.jetson, args.port)) {
        std::fprintf(stderr, "error: %s\n", osc.last_error().c_str());
        return 2;
    }
    std::printf("[vmt_hmd_pose_sender] sending /fitra/hmd_pose to %s:%u at %.1f Hz\n",
                args.jetson.c_str(), args.port, args.rate_hz);

    const auto t_start = std::chrono::steady_clock::now();
    const auto period  = std::chrono::duration<double>(1.0 / args.rate_hz);

    // Slow retry loop for SteamVR (init can fail if SteamVR isn't up yet).
    int  retry_budget   = 60;  // 60 attempts × 30s sleep = up to 30 min wait
    bool openvr_inited  = false;
    auto next_tick = std::chrono::steady_clock::now();

    while (!g_stop.load()) {
        if (!openvr_inited) {
            if (try_init_openvr(err)) {
                std::printf("[vmt_hmd_pose_sender] openvr connected\n");
                openvr_inited = true;
            } else {
                if (--retry_budget < 0) {
                    std::fprintf(stderr, "openvr init failed (retries exhausted): %s\n",
                                 err.c_str());
                    return 3;
                }
                std::fprintf(stderr,
                    "[vmt_hmd_pose_sender] openvr init failed (%s); retry in 30s\n",
                    err.c_str());
                std::this_thread::sleep_for(std::chrono::seconds(30));
                continue;
            }
        }

        // Read all device poses (we only need HMD, but the API gives us all
        // at once).
        vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
        vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
            vr::TrackingUniverseStanding,
            0.0f,
            poses,
            vr::k_unMaxTrackedDeviceCount);

        const auto& hmd = poses[vr::k_unTrackedDeviceIndex_Hmd];
        const bool tracking_ok =
            (hmd.bDeviceIsConnected && hmd.bPoseIsValid &&
             hmd.eTrackingResult == vr::TrackingResult_Running_OK);

        fitra::hmd_sender::HmdSample sample{};
        sample.valid = tracking_ok;
        sample.timestamp_s = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - t_start).count();

        if (tracking_ok) {
            const PosQuat pq = matrix_to_pos_quat(hmd.mDeviceToAbsoluteTracking);
            sample.x  = pq.x;  sample.y  = pq.y;  sample.z  = pq.z;
            sample.qx = pq.qx; sample.qy = pq.qy; sample.qz = pq.qz; sample.qw = pq.qw;
        } else {
            sample.qw = 1.0f;  // identity (purely informational; receiver ignores when valid=0)
        }

        if (!osc.send(sample)) {
            std::fprintf(stderr, "[vmt_hmd_pose_sender] send failed: %s\n",
                         osc.last_error().c_str());
            // Don't exit on send failure; the user may bring the receiver back.
        }

        // SteamVR shutdown event handling.
        vr::VREvent_t ev;
        while (openvr_inited && vr::VRSystem()->PollNextEvent(&ev, sizeof(ev))) {
            if (ev.eventType == vr::VREvent_Quit) {
                std::printf("[vmt_hmd_pose_sender] SteamVR sent Quit; releasing.\n");
                vr::VR_Shutdown();
                openvr_inited = false;
                retry_budget  = 60;
                break;
            }
        }

        next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next_tick);
    }

    if (openvr_inited) {
        vr::VR_Shutdown();
    }
    std::printf("[vmt_hmd_pose_sender] bye\n");
    return 0;
}
