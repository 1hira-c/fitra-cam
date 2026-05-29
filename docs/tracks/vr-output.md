# Track: vr-output

カメラ由来の 3D pose / tracker を **VR (SlimeVR Server / SteamVR) に流す経路**。
現状最もアクティブなトラック。

## 現状 (2026-05-27)

2 つの出力経路が **同時 enable 可能**で、いずれも pose-3d トラックの `TrackerExtractor`
(単一 producer) を read-only consumer として共有する:

1. **SlimeVR Firmware UDP** (`--slimevr-out`, port 6969) — 回転のみ。位置は SlimeVR 側 IK が
   骨格 + HMD から再構築。10 trackers を named display。
2. **VMT (Virtual Motion Tracker) → SteamVR 直結** (`--vmt-out`) — 位置 + 回転。SlimeVR Server を
   飛ばして SteamVR Driver に直結。VMT_10..VMT_19 を `/VMT/Room/Driver` OSC 60Hz。

### 設計原則 / live な制約

- **10 trackers の TrackerRole 順は固定**: LeftUpperArm / RightUpperArm / Chest / Hip /
  LeftUpperLeg / RightUpperLeg / LeftLowerLeg / RightLowerLeg / LeftFoot / RightFoot。
  SlimeVR `TrackerPosition` enum に完全一致 (骨盤は `HIP(6)`、`WAIST(5)` は auto-assign されない)。
- **Bridge relay (Jetson → Windows .NET relay → Named Pipe → SlimeVR Server) は没**。理由は
  SteamVR 起動中の `\\.\pipe\SlimeVRInput` 排他 + 座標系整合の不安定。位置を VR に流す要求は
  VMT 経路で解決済み。実装一式は `archive/botsu-phase12-bridge-relay` ブランチに凍結。
- **座標変換**: `world_*_to_vmt` は SteamVR Y-up RH frame target。archive Bridge と完全同型。
- **VMT alignment**: HMD pose (SteamVR) を取り込み chest tracker との 2D Procrustes で yaw+xyz を
  自動算出。Y (HMD 頭頂 vs chest 胴体中心の個人差 0.35–0.55m) のみ手動 slider 運用。
- **VMT 登録タイミング**: Quest 接続前に VMT が登録されると SteamVR コントローラを奪う。Driver の
  `WaitForHmd=true` で HMD+両コントローラ検知まで `RegisterToVRSystem` を arm 保留。
- **VMT フォーク側の実装**: Driver ゲート / Manager / HMD pose sender は VMT フォーク
  (Windows, `vmt_driver.sln` / `vmt_manager.sln`) に存在。fitra-cam 側はスキーマ受信のみで無改修。

### 検証

`ctest -R 'vmt|firmware_protocol|tracker_extract|hmd_pose|auto_alignment'` +
Windows 実機 (SlimeVR Server GUI / SteamVR + VMT Manager + VRChat FBT)。
詳細な合格基準は [`cpp-migration-plan.md` 検証戦略表](../cpp-migration-plan.md) の旧 Phase 11/14/15/15.5 行。

## Changelog (新しい順)

### 2026-05-29 — 出力レイテンシ M1: frame-rate 非依存 smoothing (キーストーン)
GPU フロントエンドでパイプラインが詰まった後、E2E の支配項は VR 出力の 60Hz×2 ホップ
(avg +16.7ms / worst ~33ms)。e2e-latency M4 で hop1 をイベント駆動 (opt-in) にしたが、
smoothing が **dt 非依存の固定 alpha** のままで、ソースレート同期だと高 fps で過平滑になる潜在バグが
あった。`apply_quat_smoothing`/`apply_pos_smoothing` を `alpha_eff = 1-(1-base_alpha)^(dt/nominal)` の
frame-rate 非依存形に一般化 (`run_loop` の実測 dt / nominal dt を配線)。固定レート (`dt==nominal`) は
従来と完全一致 (既定ゼロリスク)、イベント駆動は過平滑解消。これがレート引き上げ・イベント駆動を
安全にするキーストーン。`test_tracker_extract_pos` に rate-independence テスト追加 (dt/2 の 2 ステップ ==
dt の 1 ステップ 他)、ctest 9/9。実機 judder / e2e 数値検証 + イベント駆動既定化 + publisher hop2 は
被写体 (`ik_locked`)+SteamVR 要のため M2 送り。
→ [design/vr-output-latency.md](../design/vr-output-latency.md)

### 2026-05-27 — VMT 登録ゲート + sender の Manager 統合
Driver `WaitForHmd` ハードゲートで Quest 接続前の登録レースを解消 (コントローラ奪取回避)。
`vmt_hmd_pose_sender` を廃止し `vmt_manager` に吸収 (HMD pose 中継 + 登録 arm + auto-launch)。
Jetson IP は Driver が OSC `remoteEndpoint` から自動学習。fitra-cam は無改修。
→ [archive/phase15.5-vmt-registration-gate.md](../archive/phase15.5-vmt-registration-gate.md)

### 2026-05-26 — HMD pose 駆動の自動 VMT alignment
SteamVR HMD pose を `/fitra/hmd_pose` UDP で受信 (`HmdPoseReceiver` → `HmdPoseBus`)、
chest tracker との対応から `AutoAlignmentSolver` (cv::SVD 2D Procrustes) で yaw+xyz を自動算出。
T ポーズ瞬時キャリブ + 3 秒歩行精度モードの 2 操作。Web UI `/api/vmt/alignment/auto/*`。
→ [archive/phase15-vmt-hmd-auto-align.md](../archive/phase15-vmt-hmd-auto-align.md)

### 2026-05-25 — VMT 経由 SteamVR 直結
位置 + 回転を VMT 経由で SteamVR Driver に直結 (SlimeVR Server を飛ばす)。Bridge relay 没の
代替経路。`VmtPublisher` を `TrackerExtractor` の read-only consumer として並列接続、
Firmware UDP と同時 enable 可。OSC 1.0 wire writer を旧実装から `fitra::vmt` に復元。
→ [archive/phase14-vmt-steamvr.md](../archive/phase14-vmt-steamvr.md)

### 2026-05-22 — Bridge relay 経路を没
位置を VR に流す Bridge relay (Named Pipe) は SteamVR との排他 + 座標系問題で不採用。
`archive/botsu-phase12-bridge-relay` に凍結。位置経路は後の VMT で復活。
(roll 品質改善 M1 は pose-3d トラックへ。)
→ [archive/phase12-slimevr-bridge-relay.md](../archive/phase12-slimevr-bridge-relay.md)

### 2026-05-21 — SlimeVR ネイティブ Firmware UDP 連携
初版 VMC over OSC が SlimeVR で連番表示になり body-part assign 不能 → Firmware UDP (port 6969)
へ移行。10 trackers を named display。Handshake → SensorInfo×10 → 60Hz RotationData + Heartbeat。
MAC は hostname SHA-1 で安定化 (再起動後も persistence)。
→ [archive/phase11-slimevr-integration.md](../archive/phase11-slimevr-integration.md)
