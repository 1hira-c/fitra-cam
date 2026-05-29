# vmt_hmd_pose_sender

> **Superseded.** This standalone overlay app was folded into the VMT fork's
> `vmt_manager` (HMD pose relay + registration arming + auto-launch) on
> 2026-05-27 — see the vr-output track changelog
> (`docs/tracks/vr-output.md`, "VMT 登録ゲート + sender の Manager 統合").
> This directory is kept only as a record of the `/fitra/hmd_pose` wire format
> documented below; you no longer build or run it for the live setup.

Windows-side SteamVR overlay app that publishes the HMD pose over OSC/UDP to a
fitra-cam Jetson.

The Jetson side (`HmdPoseReceiver` in `cpp/src/vmt/hmd_pose_receiver.cpp`)
listens on UDP port `39571` by default and feeds the pose into the auto
alignment solver. See `docs/archive/phase15-vmt-hmd-auto-align.md` for the
original end-to-end story.

## Wire format

Address `/fitra/hmd_pose` with typetag `,iffffffff`:

| Arg | Type   | Meaning                                        |
|----:|:-------|:-----------------------------------------------|
| 1   | int32  | `valid` (1 = HMD tracking OK, 0 = lost / off)  |
| 2   | float  | `timestamp_s` (monotonic seconds since launch) |
| 3-5 | float  | `x, y, z` — SteamVR Standing universe, metres  |
| 6-9 | float  | `qx, qy, qz, qw` — HMD orientation             |

The position/orientation frame is the SteamVR Standing universe (Y-up RH,
X-right, Z-back). fitra-cam transforms its own world frame (Z-up RH) into
the same frame via `world_pos_to_vmt` / `world_quat_to_vmt` before the
solver compares them.

## Build (Windows, MSVC)

Dependencies (both ship inside VirtualMotionTracker):

- `openvr-1.23.7/` — OpenVR SDK headers, `lib/win64/openvr_api.lib`,
  `bin/win64/openvr_api.dll`
- `oscpack_1_1_0/` — OSC client library

If you cloned VirtualMotionTracker next to fitra-cam
(`refs/VirtualMotionTracker/`), `cmake` finds both automatically. Otherwise
pass `-DOPENVR_DIR=...` and `-DOSCPACK_DIR=...`.

```cmd
cmake -S windows\vmt_hmd_pose_sender -B build_win ^
      -G "Visual Studio 17 2022" -A x64
cmake --build build_win --config Release
```

`openvr_api.dll` is copied next to `vmt_hmd_pose_sender.exe` automatically.

## Run

```cmd
build_win\Release\vmt_hmd_pose_sender.exe ^
    --jetson 192.168.1.20 ^
    --port   39571 ^
    --rate-hz 60
```

The app registers as a background SteamVR application (no overlay window).
If SteamVR is not running yet, it retries every 30 seconds for up to ~30
minutes before giving up.

## Firewall

Outbound UDP to the Jetson must be allowed. On the Jetson side, accept
inbound UDP on the chosen port (default `39571`). A typical Windows command:

```cmd
netsh advfirewall firewall add rule ^
    name="vmt_hmd_pose_sender outbound" dir=out ^
    program="C:\path\to\vmt_hmd_pose_sender.exe" ^
    action=allow protocol=UDP
```

## Caveats

- The pose frame moves whenever the user re-runs Room Setup in SteamVR. If
  Room Setup is repeated, restart `vmt_hmd_pose_sender.exe` so the auto
  alignment is recalculated against the new origin.
- Quest standalone defaults to the Standing universe; some seated SteamVR
  configurations may need adjustment. The app explicitly calls
  `SetTrackingSpace(TrackingUniverseStanding)` on startup.
