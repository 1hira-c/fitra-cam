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

### 2026-06-08 — One Euro の GitHub レビュー修正 (バグ修正)
PR #25 の gemini / Copilot レビュー指摘を反映。design doc なし(changelog のみ)。
- **(gemini HIGH / 実バグ)** 位置 One Euro の速度推定 `pos_dx_hat` 更新に外れ値ゲートが
  効いておらず、三角測量グリッチの巨大 `dx` で速度状態が汚染 → 直後の数フレームで
  カットオフが開き静止ジッタが素通りしていた。速度更新にも `(1-gate)` を適用し、回帰テスト
  (`test_one_euro_outlier_gate_does_not_pollute_speed`)を追加。
- `TrackerExtractorOptions` の One Euro 既定係数が `MainConfig` のチューニング値と不一致
  だった点を同値化(位置 1.0/4.0、回転 1.5/1.5)+「main 側で上書きされる」旨をコメント明記。
- design doc の「しきい値の根拠」に現行既定値(初期値ではない)の注記を追加、`one_euro_alpha`
  のエッジケースコメントの優先順位明確化、`test_main_config` のコメント実態合わせ。
- 完了の定義に従い `docs/cpp-migration-plan.md` 検証戦略表に One Euro 行を追加。

### 2026-06-08 — One Euro 既定値を実測チューニング値に更新 (閾値調整)
`configs/medium_3d.yaml` で詰めた One Euro 係数を `MainConfig` の既定値へ昇格。位置は
`mincutoff 0.8→1.0` / `beta 0.4→4.0`、回転は `mincutoff 1.0→1.5` / `beta 0.3→1.5`
(`dcutoff` は両軸 1.0 据え置き)。初期既定の `beta` は m/s・rad/s スケールに対し小さすぎ、
動作時もカットオフが開ききらず遅延が残っていたため引き上げ。`main_config.hpp` の既定値と
`main.cpp --help` の表記を同値に更新。design doc なし(閾値調整のため changelog のみ)。

### 2026-06-03 — One Euro フィルタによる動静適応スムージング
座位静止時のトラッカー揺れに対処。固定 α EMA(α=0.5 ≈ カットオフ 9.5Hz)は静止の滑らかさと
動作追従を両立できないため、位置(per-axis)・回転(測地角速度ベース)とも **One Euro
(速度適応カットオフ)** に置換。静止時は低カットオフで強くスムージング、動作時は `beta·速度`
でカットオフを開いて遅延なく追従。既存の swing/twist 分離・parent-yaw transport・hip-relative
hold・外れ値ゲート(8–16 m/s freeze)は温存(swing/twist 本体を per-tracker alpha の `impl` に
抽出、固定 α 版は bit-identical で既存 ctest 無傷)。既定 ON、`--vr-no-one-euro` で旧 EMA に
フォールバック、`beta=0` で固定カットオフ EMA に縮退。`three_d.vr_*` YAML / `--vr-{pos,quat}-*`
CLI を追加。新規 ctest(位置 6 / 回転 3 / config 1)。Phase 14 で見送った One Euro の昇格。
→ [design/vr-output-one-euro-filter.md](../design/vr-output-one-euro-filter.md)

### 2026-06-03 — 継続キャリブのレビュー修正 (バグ修正)
Codex + GitHub (gemini / Copilot) レビューで顕在化した点を修正。design doc なし(changelog のみ)。
- `SampleReservoir::key_of`: 負座標で符号付き左シフト UB(VMT x/z は通常移動で負になる)→ uint32 経由 pack。負4象限が別セルになる回帰テスト追加。
- `continuous_align`(と既存の `hmd`)ステータスが `/stats3d` にしか載らず、WebUI は `/ws3d` バンドル(`state.bundle3d`)しか読まないため「自動追従」トグルが恒久 disabled だった → `publisher_loop` の ws3d ブロードキャストにも fragment を載せた。
- `make_sample`: 非有限入力(NaN/Inf)を reject。reservoir 汚染と `key_of` の float→int キャスト UB を防止。
- `ramp`: `zero_at == full_at` の退化帯を step 関数化(「full_at で 1」契約を満たす)。
- 自動追従 OFF 時に reservoir を `clear()`(OFF→ON で古セルを使った solve を防止)。HMD 速度計算の dt に下限(`>1e-4`)。
- gemini の「`joints[19]` で範囲外アクセス」指摘は誤検知(`joints` は固定長 `std::array<,26>`、coco17 でも index 19 は valid=false の zero-init)。

### 2026-05-30 — 継続キャリブの cold-start ブースト
初期収束が遅すぎる(実機で 1 分以上歩かないと位置が合わない)問題に対処。原因は fine の
step clamp(`max_pos_step 0.05m` / `max_yaw_step 2°` per 2s resolve)が起動時の大きな初期
ズレまで律速していたこと。純関数 `update_lock_state` でロック状態を導入し、未収束の間は
coarse クランプ(`coarse_max_pos_step 0.50m` / `coarse_max_yaw_step 30°` / `blend 0.6`)で
速く粗収束 → 近接した solve が連続(`lock_streak 3`)したら fine クランプに latch(ジャンプ
防止は維持)。VMT 再センタリング等の大乖離・runtime トグルで coarse へ復帰。`/stats3d` と
Web UI に `locked` を追加。新規 ctest `test_lock_state`。
→ [design/vr-output-continuous-hmd-calibration.md](../design/vr-output-continuous-hmd-calibration.md)(cold-start 追補)

### 2026-05-29 — 自動・半継続 HMD キャリブレーション
Phase 15 の単発 alignment を常時バックグラウンド化。起動時から HMD と「信頼性高く
報告された頭部(不安定時は chest 中点にフォールバック)」を継続サンプリングし、空間
reservoir に代表値を蓄積 → 定期 `solve_motion` → clamped EMA で alignment を自動収束・
追従(Y は手動 slider 維持)。サンプル品質の主要因に**脊椎/首ボーンの垂直性**(直立ほど
高得点)を採用。新規 `ContinuousAligner`(`fitra_vmt`)、`--vmt-continuous-align`(既定 ON)、
`/api/vmt/alignment/auto/continuous/*` + `/stats3d` ブロック。
→ [design/vr-output-continuous-hmd-calibration.md](../design/vr-output-continuous-hmd-calibration.md)

### 2026-05-29 — OSC パディングの単一 insert 化 + gate 定数の static_assert (挙動不変)
(1) `OscWriter::emit_osc_string` の 4-byte 境界パディングを `push_back` ループから単一
`insert(end, 1 + pad4(...), '\0')` に置換。出力バイト列は同一 (`test_vmt_osc_writer` golden 通過)。
(2) `tracker_extract.cpp` の smoothstep gate 定数 (`kRollSin*` / `kPosVelGate*` / `kPelvisYawGate*`)
に `low < high` を固定する `static_assert` を追加 — 将来の境界反転がコンパイル時に弾かれる。
値は不変。微最適化のため design doc なし (changelog のみ)。

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

### 2026-05-29 — 出力レイテンシ M2: 被写体実測 — VR ペーシングは lever でない (負の結果)
被写体 in view + calib + subject02 で `e2e_capture_to_send_ms` を A/B 実測。**extractor を event-driven に
しても publisher を 60→120Hz にしても e2e は不動 (~34-35ms)** — 理論「60Hz×2 = +16-33ms」は実機では
非該当 (extractor は三角測量にほぼ同期、hop2 も支配項でない)。一方 **nvjpeg 全 GPU フロントエンドで
cap→pub 21→13ms、e2e 34→26ms (−8ms)**。photon→send を削るのはパイプラインのみと確定。残 VR 側 ~13ms は
`sync_window=15ms` + 処理で rate 非依存。よって VR ペーシングのレイテンシ目的変更は見送り (M1 smoothing は
過平滑バグ correctness 修正として維持)。VR レイテンシを下げる手は 3D 設定の `cameras.pixel_format: nvjpeg`
(per-machine config は gitignored、雛形 `configs/live_2cam_3d.yaml.example` に既定記載 / CLI `--pixel-format nvjpeg`)。
judder の体感比較は HMD 主観評価として残課題。
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
