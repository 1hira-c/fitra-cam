# Track: pose-3d

2D keypoint から **3D pose / bone tracker** を起こす経路。lift / IK / Kalman / roll 品質 /
subject calibration。vr-output トラックの上流 (= tracker の単一 producer) を担う。

## 現状 (2026-05-27)

`SlimeTrackerBus` + `TrackerExtractor` が tracker snapshot の **単一 producer**。
Firmware UDP / VMT publisher / WebUI viz が同じ smoothing 履歴を共有する。

### 設計原則 / live な制約

- **degeneracy gate は相対しきい**: `quat_from_forward_up` の degeneracy 判定は `sin θ`
  ベース (`kRollSinLow=0.15` / `kRollSinHigh=0.30`)。絶対 norm しきいは使わない。
  primary が degenerate になる向き (水平腕・伸展脚) では roll を freeze に倒す。
- **lateral pin anti-pattern**: secondary lateral pin (neck-shoulder 等) で roll を稼ぐ構造は
  「立位伸展で 90° roll が一気に入る」症状の原因。`upper_arm` / `upper_leg` を同型の 1-stage 構造に
  揃え、tertiary を `Vec3f{0,0,0}` sentinel にして freeze へ倒す経路を確立済み。
- **smoothing の state 所有は TrackerExtractor に集約**: 回転 (`prev_quat_`) も位置 EMA
  (`apply_pos_smoothing`) も同じ場所で持つ。publisher 側に smoothing state を分散させない。
- **subject profile schema は厳格分離**: `fitra_subject_profile_v1` (COCO17) と `v2` (Halpe26) は
  マイグレーションせず再キャリブを要求 (keypoint topology は core-pipeline トラック参照)。
- **フル IK は backstop 設計のみ**: Tier A swing-twist + ROM clamp + 角速度 clamp + constrained
  Kalman / Tier C Bullet ragdoll の設計メモは起票済だが、degeneracy gate が実用品質に達したため
  取り込みは保留。→ [archive/phase13-full-ik.md](../archive/phase13-full-ik.md)

### 検証

`ctest -R 'tracker_extract|firmware_protocol'` (21 ケース) + 実機目視 (WebUI の
per-tracker AxesHelper×10 / `#trackers-table` の state 色分け、`/stats3d`)。
合格基準は [`cpp-migration-plan.md` 検証戦略表](../cpp-migration-plan.md) の旧 Phase 13 行。

## Changelog (新しい順)

### 2026-05-27 — コントローラ固定 AprilTag extrinsic: M1 transport + M2 ソルバ/検出コア
案C (コントローラ固定マーカー + VR world 連結) のソフト側コアを着手・実装。実機を要する
M0 (PnP 残差 / SLAM ドリフト実測) と live 収集 UI (M2 後半 / M4) は手戻りリスクを承知で後回し、
ハードに依存しない 3 モジュールを単体テスト付きで先行実装した:
- **M1 transport**: `vmt/controller_pose_receiver.{hpp,cpp}` — `/fitra/controller_pose`
  (`,iiffffffff` = HMD の `,iffffffff` に `eTrackingResult` を 1 個追加) を HMD と並列の
  latest-wins bus で受信。`running_ok()` ゲート (`bPoseIsValid && Running_OK`)。OSC decode
  helper を `vmt/osc_decode.hpp` に抽出し HMD 経路と共有。
- **M2 ソルバ**: `lift/extrinsic_solver.{hpp,cpp}` — `A_i·X = Z·B_i` を
  `cv::calibrateRobotWorldHandEye` (AX=ZB) に写像 (A=T_cam←marker / B=T_world←controller /
  Z=T_cam←world / X=T_marker←controller=Y_face⁻¹)。(camera,face) グループ毎に解き、面間で
  T_cam←world を集約 + spread を品質指標化、自己残差も算出。
- **M2 検出**: `lift/apriltag_marker.{hpp,cpp}` — `DICT_APRILTAG_36h11` 検出 + 面 ID→サイズ
  設定 + `SOLVEPNP_IPPE_SQUARE` で T_cam←face。`objdetect` を OpenCV link に追加。
- **収集ループ骨格 + 配線**: `pipeline/extrinsic_calib_session.{hpp,cpp}` — frame tap → 検出 →
  controller pose ペアリング → モーションゲート (線速/角速しきい) → (cam,face) 毎バースト平均 →
  `ExtrinsicSample` 蓄積 → 終了時 solve + `CalibrationSet` 書き出し。`calib_io::write_calibration`
  新設。`main`/`main_config` に `--extrinsic-calib` / `extrinsic_calib:` セクションと
  `ControllerPoseReceiver` 起動を配線 (subject wizard と frame tap 排他)。
- **Crow 配線 + WebUI**: `crow_server` に `set_extrinsic_calib_session` +
  `/api/excal/{state,start,stop,solve,extrinsics}` (subject wizard の `/api/calib/*` と同型) +
  `/extrinsic-calib` 静的配信。フロントエンド `web/extrinsic_calibration/` — Start/Stop/Solve、
  理由付き Capture gate (GOOD/MOVING/NO_TAG/NO_POSE)、per-camera ライブ検出 (face·reproj·age)、
  被覆マトリクス (cam×face, min_samples でセル色)、Solve 後 per-camera 結果。`state_json` に
  `num_cams`/`min_samples`/`faces`/`gate_reason`/`detections` 追加。
- **3D 検証シーン**: `scene.html`+`scene.js` (three.js, vendor 流用) が `/api/excal/extrinsics`
  (intrinsics + `T_cam_world` + center) を読み、各カメラ 6DoF を共通 world frame に frustum 配置。
  session は `on_frame` で per-camera 最新検出を保持 + `gate_reason_`/`extrinsics_json` を公開。
- 検証: `ctest -R 'extrinsic_solver|apriltag_marker|controller_pose_receiver|extrinsic_calib_session|main_config|crow_excal'`。
  合成データで厳密復元 (1e-9 m / 1e-6°)・相対 extrinsic 恒等・ノイズ <1cm/<1°、tag
  render→detect→PnP 往復、収集ループの gate/burst + end-to-end solve→write→reload、
  レンダ tag→on_frame の検出/gate_reason、`extrinsics_json` 内容、ループバック実 HTTP で
  `/api/excal/*` + 静的配信 (collect/scene)。
- **未** (実機 or 大掛かりで保留): Windows sender の controller 送出、per-camera ライブ**映像**
  オーバーレイ (検出ステータスはテキスト表示済、映像配信は別途)、M4 live solver 重畳、M3 BA
  (閉形式が合成厳密復元 + 動機が M0 実測待ちのため保留)、M0 実測。収集制御は headless 既定 +
  WebUI/API 手動制御。
→ [design/pose-3d-controller-marker-extrinsic.md](../design/pose-3d-controller-marker-extrinsic.md)

### 2026-05-24〜25 — roll 品質詰め + WebUI tracker 可視化 + per-tracker stats
「観察基盤を先に作る → データで仮説確定 → 構造修正」の順で立位伸展時の 90° roll を解消。
`SlimeTrackerBus` + `TrackerExtractor` を新設し publisher を consumer に refactor。
degeneracy 判定を相対しきい (sin θ) 化、upper_arm を 1-stage 構造に統一。
WebUI に AxesHelper×10 + per-tracker rolling stats (ang_vel p50/p95, freeze_pct 等)。
→ [archive/phase13-quality-refinement.md](../archive/phase13-quality-refinement.md)

### 2026-05-22 — roll 品質改善 M1 (confidence-modulated smoothing)
`tracker_extract.cpp` の二の腕 / 大腿 / 足の up を多段選択 + confidence-modulated smoothing に
書き換え。二の腕ひねり症状の解消 + 腕完全伸展時の roll twist 振動収束を実機確認。
(同 phase の Bridge relay 経路は vr-output トラックで没。)
→ [archive/phase12-slimevr-bridge-relay.md](../archive/phase12-slimevr-bridge-relay.md)

### 〜2026-05-20 — IK pose calibration + subject profile
IK ベースの pose calibration と subject profile (体格パラメータ) 永続化。
→ [archive/phase8-ik-pose-calib.md](../archive/phase8-ik-pose-calib.md) /
  [archive/phase8-subject-profile-runbook.md](../archive/phase8-subject-profile-runbook.md)

### 初期 — 3D IK + Kalman lift
2D keypoint → 3D lift の IK + Kalman フィルタ基盤。
→ [archive/phase7-3d-ik-kalman.md](../archive/phase7-3d-ik-kalman.md)
