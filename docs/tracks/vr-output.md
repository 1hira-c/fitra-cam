# Track: vr-output

カメラ由来の 3D pose / tracker を **VR (SlimeVR Server / SteamVR) に流す経路**。
現状最もアクティブなトラック。

## 現状 (2026-05-29)

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
- **VMT alignment**: HMD pose (SteamVR) を取り込み 2D Procrustes で yaw+xz を自動算出。
  単発(T ポーズ / 3 秒歩行、chest 対応)に加え、**常時バックグラウンドの `ContinuousAligner`**
  が頭部優先・chest フォールバックで継続サンプリング → reservoir → clamped EMA で半継続追従。
  サンプル品質は脊椎/首ボーンの垂直性で重み付け。Y (HMD 頭頂 vs chest 中心の個人差 0.35–0.55m)
  のみ手動 slider 運用。
- **VMT 登録タイミング**: Quest 接続前に VMT が登録されると SteamVR コントローラを奪う。Driver の
  `WaitForHmd=true` で HMD+両コントローラ検知まで `RegisterToVRSystem` を arm 保留。
- **VMT フォーク側の実装**: Driver ゲート / Manager / HMD pose sender は VMT フォーク
  (Windows, `vmt_driver.sln` / `vmt_manager.sln`) に存在。fitra-cam 側はスキーマ受信のみで無改修。

### 検証

`ctest -R 'vmt|firmware_protocol|tracker_extract|hmd_pose|auto_alignment|continuous_aligner'` +
Windows 実機 (SlimeVR Server GUI / SteamVR + VMT Manager + VRChat FBT)。
詳細な合格基準は [`cpp-migration-plan.md` 検証戦略表](../cpp-migration-plan.md) の旧 Phase 11/14/15/15.5 行。

## Changelog (新しい順)

### 2026-05-29 — 自動・半継続 HMD キャリブレーション
Phase 15 の単発 alignment を常時バックグラウンド化。起動時から HMD と「信頼性高く
報告された頭部(不安定時は chest 中点にフォールバック)」を継続サンプリングし、空間
reservoir に代表値を蓄積 → 定期 `solve_motion` → clamped EMA で alignment を自動収束・
追従(Y は手動 slider 維持)。サンプル品質の主要因に**脊椎/首ボーンの垂直性**(直立ほど
高得点)を採用。新規 `ContinuousAligner`(`fitra_vmt`)、`--vmt-continuous-align`(既定 ON)、
`/api/vmt/alignment/auto/continuous/*` + `/stats3d` ブロック。
→ [design/vr-output-continuous-hmd-calibration.md](../design/vr-output-continuous-hmd-calibration.md)

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
