# vr-output: VMT pose relay wire spec

(作成日 2026-06-08 / 関連: `pose-3d-controller-marker-extrinsic.md`,
`vr-output-continuous-hmd-calibration.md`)

## 背景

VMT Manager 側が SteamVR/OpenVR から HMD pose を取り、fitra-cam Jetson へ送る経路は
既に VMT フォーク側へ統合済み。controller-marker extrinsic calibration では同じ SteamVR
Standing universe 上の controller pose も必要になる。

旧実装の `/fitra/hmd_pose` + `/fitra/controller_pose` 別ポート構成は PoC 用の暫定で、
運用上は非現実的。VMT Manager から **1 UDP port / 1 pose relay** として HMD と左右
controller を送る仕様へ統一する。

## 採用仕様

### Transport

- UDP / OSC 1.0
- 送信先 port: `39571`
- 送信先 host: VMT フォーク側が既に持つ Jetson IP 学習結果を使う
- 送信 rate: 60 Hz 既定、最大 120 Hz 程度まで
- OpenVR tracking space: `vr::TrackingUniverseStanding`
- OpenVR prediction: `fPredictedSecondsToPhotonsFromNow = 0.0f`

fitra-cam 側は latest-wins で受ける。packet loss は許容し、ACK / retransmit は不要。

### OSC message

新規 canonical message:

```text
address = "/fitra/tracked_pose"
typetag = ",iiiiffffffff"
args    = role(i32)
          device_index(i32)
          valid(i32)
          tracking_result(i32)
          timestamp_s(f32)
          x(f32) y(f32) z(f32)
          qx(f32) qy(f32) qz(f32) qw(f32)
```

`role`:

| value | meaning |
|---:|---|
| 0 | HMD |
| 1 | left controller |
| 2 | right controller |

`device_index`:

- OpenVR tracked device index。
- HMD は通常 `vr::k_unTrackedDeviceIndex_Hmd`。
- controller は `GetTrackedDeviceIndexForControllerRole(...)` の戻り値。
- role が未解決の場合は `-1`。

`valid`:

- `1` when `bDeviceIsConnected && bPoseIsValid`
- `0` otherwise

`tracking_result`:

- OpenVR `vr::ETrackingResult` の int 値。
- calibration-grade sample として使えるのは `vr::TrackingResult_Running_OK` のみ。
- `Running_OK` は `200`。

`timestamp_s`:

- VMT Manager 起動後の monotonic seconds。
- 同じ OpenVR poll tick から作った HMD / left / right は同じ timestamp を使う。
- wall-clock ではない。fitra-cam 側の stale 判定は受信時刻で行う。

`x y z qx qy qz qw`:

- SteamVR Standing universe の absolute pose。
- 単位は metres。
- 座標系は SteamVR/VMT と同じ Y-up right-handed frame。
- quaternion は `xyzw`。
- HMD 相対 pose ではない。
- invalid 時は `x=y=z=0, q=(0,0,0,1)` を送る。fitra-cam 側は invalid pose を幾何計算に使わない。

### Bundle

推奨は 1 poll tick ごとに OSC bundle で 3 message をまとめる:

```text
#bundle
  /fitra/tracked_pose role=0 ...
  /fitra/tracked_pose role=1 ...
  /fitra/tracked_pose role=2 ...
```

ただし、実装を簡単にするため同一 UDP port へ 3 datagram として送ってもよい。
fitra-cam 側は role ごとの latest-wins slot へ publish するだけなので、bundle は必須ではない。

## VMT Manager 側の取得手順

毎 tick で `GetDeviceToAbsoluteTrackingPose` を 1 回だけ呼ぶ。

```cpp
vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
    vr::TrackingUniverseStanding,
    0.0f,
    poses,
    vr::k_unMaxTrackedDeviceCount);
```

role index:

```cpp
const uint32_t hmd = vr::k_unTrackedDeviceIndex_Hmd;
const uint32_t left =
    vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(
        vr::TrackedControllerRole_LeftHand);
const uint32_t right =
    vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(
        vr::TrackedControllerRole_RightHand);
```

送信条件:

- role が unresolved (`vr::k_unTrackedDeviceIndexInvalid`) でも role message は送る。
  その場合 `device_index=-1`, `valid=0`, `tracking_result=0`, identity pose。
- role が resolved なら `device_index` はその index。
- `valid` は `bDeviceIsConnected && bPoseIsValid`。
- `tracking_result` は `poses[index].eTrackingResult`。
- pose matrix は `mDeviceToAbsoluteTracking` から position + quaternion へ変換する。

## fitra-cam 側の解釈

fitra-cam 側は `/fitra/tracked_pose` を 1 socket で受け、role ごとの bus に publish する。

- `role=0`: HMD alignment / continuous align 用
- `role=1`: left controller
- `role=2`: right controller

`running_ok`:

```text
valid != 0 && tracking_result == 200
```

controller-marker extrinsic calibration は設定された controller role のみを使う。
既定は `right_controller` とする。

stale 判定:

- role ごとに last receive time を持つ。
- 既定 threshold は 200 ms。
- stale の pose は `running_ok=false` と同等に扱う。

## Backward compatibility

移行期間のため、fitra-cam receiver は旧 message も受けられるようにする。

旧 HMD:

```text
address = "/fitra/hmd_pose"
typetag = ",iffffffff"
args    = valid(i32) timestamp_s(f32)
          x y z qx qy qz qw
```

旧 controller:

```text
address = "/fitra/controller_pose"
typetag = ",iiffffffff"
args    = valid(i32) tracking_result(i32) timestamp_s(f32)
          x y z qx qy qz qw
```

旧 controller message には role がないため、fitra-cam 側では設定された
`extrinsic_calib.controller_role` に publish する。旧別ポート構成は deprecated。

## 運用上の不変条件

- VMT Manager は `TrackingUniverseStanding` を使う。
- Session 中に SteamVR recenter / Room Setup をしない。
- HMD と controller は同じ OpenVR poll tick から取得する。
- controller-marker calibration では `Running_OK` 以外の controller sample を採用しない。
- 送信値は finite にする。NaN / Inf は fitra-cam 側で reject される。

## Milestone

- M1: **実装済 (2026-06-08)** — fitra-cam に `/fitra/tracked_pose` parser + role dispatch
  (`TrackedPoseReceiver`) を追加し、旧 message 互換を維持。
- M2: **実装済 (2026-06-08)** — `extrinsic_calib.controller_role` を config/CLI に追加し、
  別 controller port を deprecated。
- M3: VMT Manager 側が `/fitra/tracked_pose` で HMD / left / right を送信。
- M4: 実機で HMD continuous align と controller-marker extrinsic calibration が同時に動くことを確認。

## 検証

fitra-cam:

```bash
ctest --test-dir cpp/build --output-on-failure -R 'tracked_pose|hmd_pose|controller_pose|auto_alignment|extrinsic_calib'
```

Windows / VMT Manager:

- Quest HMD + 両 controller 接続後、fitra-cam の `/stats3d` で HMD pose が `valid=true`,
  `age_ms < 100`。
- extrinsic calibration UI で controller tracking が `OK` になり、タグが見えて静止している時に
  `GOOD` へ遷移する。
- left/right を入れ替えて持った場合、設定した `controller_role` の controller だけが採用される。
