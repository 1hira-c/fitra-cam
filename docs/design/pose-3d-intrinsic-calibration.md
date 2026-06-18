# pose-3d: C++ 内部パラメータ (intrinsic) カメラ校正 + 歪みモデル明示

(着手 2026-06-16 / 関連: [pose-3d-floor-apriltag-extrinsic.md](pose-3d-floor-apriltag-extrinsic.md) の前提工程)

## 背景 / 動機

extrinsic 校正 (案C/案D) はカメラ intrinsics (K, dist) を前提にするが、**intrinsic 校正は
Python (`calibrate_intrinsics_charuco.py`, ChArUco, pinhole 専用 `calibrateCameraCharuco`) と
レガシー web (FastAPI :8020) にしか無く、C++ flow / WebUI には未統合**。さらに intrinsics YAML
スキーマは **歪みモデルのフラグを持たず dist 1×5 pinhole 固定**。

一方、案D の floor solver は `cv::fisheye::undistortPoints` 経路 (`--floor-fisheye`) を持つが、
これを有効にすると **5 係数 pinhole の dist を 4 係数魚眼 API に食わせる“裏付けのないフラグ”**
になり壊れる。実機 ELP AR0234 は強い魚眼なので、本来は魚眼モデルで校正したい。

完了条件:
- intrinsics YAML が **歪みモデル (pinhole/fisheye) を自己記述**し、全 consumer がそれに従う。
- **無引数/`--config` 起動から、intrinsics が無ければ最初に intrinsic 校正へ**入れる
  (setup の step 0)。WebUI で被覆を見ながら収集→solve→YAML 出力。
- 魚眼 intrinsics を C++ で生成でき、案D extrinsic + 三角測量まで魚眼で一貫する。

CLAUDE.md「python/ に機能追加しない」方針に従い、producer は C++ 側に新設する
(Python script は当面併存・参照のみ)。

## 検討した案

- **producer を C++ に新設** / Python script に魚眼追加は方針外 (python/ は動かす最小パッチのみ)。
- **歪みモデルを YAML に明示 (`distortion_model`)** / dist 係数数 (4 vs 5) からの暗黙判定は脆い
  (pinhole も 4 係数を取り得る)。ファイルを自己記述にし、consumer は係数数でなくモデルで分岐。
  CLI `--floor-fisheye` / `--intrinsic-model` は override に降格。
- **ChArUco を採用** / 素の checkerboard より部分遮蔽・斜めに強く、既存 Python と同じ盤を流用できる。
  C++ は `cv::aruco::CharucoDetector` + `board.matchImagePoints` で obj/img 対応を作り、
  pinhole は `cv::calibrateCamera`、fisheye は `cv::fisheye::calibrate`。旧
  `cv::aruco::calibrateCameraCharuco` は使わない (4.8 で matchImagePoints 経路が素直)。
- **全カメラ同時収集・per-camera 独立 solve** / 1 台ずつ回すより運用が速い。各カメラが自分の
  ビュー集合を貯め、solve は per-camera。出力 YAML は既存の per-camera intrinsics map。
- **ビュー多様性ゲート** / 連続フレームをそのまま貯めると冗長で条件数が悪い。被覆 (画像内の
  charuco 分布) + 直近採用ビューとの差分 (board pose の並進/回転 or コーナー重心移動) でゲートし、
  散らばったビューだけ採用。
- **別 RunMode `calib-intrinsic`** / 既存モードに相乗りしない (収集対象・solver・出力が独立)。
  案C/D と同じ「収集 session + replay + WebUI + flow」骨格を踏襲し infra を再利用。
- **replay 等価**: 案C/D と同じ `tools/excal_record` 形式 (ctrl は無視) を流用し、live↔replay を
  bit-exact 比較。新録画ツールは作らない。

## 採用設計

### 歪みモデル基盤 (I1, 全 consumer の土台)
- `lift::Intrinsics` に `std::string distortion_model = "pinhole"` を追加。`calib_io`
  load/write/validate が読み書き (absent → pinhole で後方互換)。validate: fisheye は dist 4 係数、
  pinhole は ≥4。`distortion_model` は "pinhole" | "fisheye" のみ許可。
- consumer をモデル駆動に:
  - **floor_extrinsic_solver** / **floor_calib_session**: per-camera `fisheye =
    (model=="fisheye")` (`--floor-fisheye` は全 cam 強制 override)。
  - **triangulator** / **apriltag_marker**: model=="fisheye" のとき `cv::fisheye::undistortPoints`
    /`projectPoints` を使う (pinhole は従来通り)。これで魚眼 intrinsics が run/extrinsic 全経路で
    一貫する。

### ChArUco 検出 (I2)
- `lift/charuco_board.{hpp,cpp}`: `CharucoBoardConfig{squares_x,squares_y,square_len_m,
  marker_len_m,dictionary}` → `cv::aruco::CharucoBoard` + `CharucoDetector`。
  `detect(image) → {charuco_corners(px), charuco_ids}`。`object_image_points(corners,ids) →
  (objPoints Z=0, imgPoints)` を `board.matchImagePoints` で。

### 収集 + solve (I3)
- `pipeline/intrinsic_calib_session.{hpp,cpp}`: `on_frame(cam,bgr)` で charuco 検出 → 多様性ゲート →
  per-camera にビュー (corners+ids) 蓄積。`solve_and_write`: per-camera に matchImagePoints →
  pinhole=`calibrateCamera` / fisheye=`fisheye::calibrate`(`RECOMPUTE_EXTRINSIC|FIX_SKEW`) → K/dist/
  rms → `CalibrationSet` (distortion_model 付き) を `write_calibration`。`state_json` は per-camera
  の採用ビュー数・被覆・rms。
- **受け入れゲート (2026-06-17 追加)**: solve 後、`rms_px > max_rms_px` (既定 1.5) または K の
  `|fx-fy|/max(fx,fy) > max_fxfy_aniso` (既定 0.25) なら **そのカメラを失敗扱いにして書き出さない**。
  盤面寸法の転置 (squares_x/y) や square/marker/dict の取り違えは「solve は通るが rms 数百 px・
  異方 K」の退化解になり、これを書き出すと extrinsic/triangulation を静かに壊す (実例: ChArUco
  盤面 5×7↔7×5 転置で rms 203px↔0.72px)。CLI `--intrinsic-max-rms` / YAML `intrinsic_calib.max_rms_px`。

### RunMode / CLI / replay (I4)
- `RunMode::CalibIntrinsic`、`--calib-intrinsic` / `--intrinsic-out` / `--intrinsic-model
  {pinhole,fisheye}` / board params (`--charuco-*`) / `--intrinsic-replay`。
- `app/mode_calib_intrinsic`: replay 無人 + live (Crow)。`app/intrinsic_calib_runner` +
  既存 `ExcalInputSource` 流用 (ctrl 無視)。被覆/rms を stdout JSON。
- flow: `kExitFlowToCalibIntrinsic`、`precheck_mode_switch` (board params + 出力先)、
  `daemon` の `initial_mode` で intrinsics 不在 → CalibIntrinsic を先頭段、module_argv 配線。

### WebUI (I5)
- `web/intrinsic_calibration/` (vanilla) を Crow が静的配信、`/api/incal/{state,start,stop,solve}`。
  被覆ヒートマップ的フィードバック + start/stop/solve。`flow.js`/`useFlowWatch` の
  `PAGE_FOR_MODE` に `calib-intrinsic → /intrinsic-calib`。

### 不変条件
- intrinsics YAML は **distortion_model 自己記述**。consumer は係数数でなくモデルで分岐。
- intrinsic 校正は **per-camera 独立**。解像度は校正時のものを記録 (extrinsic と違い intrinsic は
  解像度依存)。

## Milestone
- **I1** distortion_model 基盤 (calib_io + 全 consumer をモデル駆動) — fisheye footgun 解消。
- **I2** ChArUco 検出。
- **I3** intrinsic 収集 session + solve。
- **I4** RunMode + CLI/replay + flow exit code/precheck。
- **I5** WebUI ページ + Crow ルート。
- **I6** daemon 統合 (setup step0) + 本 doc + track/migration 更新。

## 検証
- ctest: `test_calib_io` (distortion_model round-trip + fisheye/pinhole validate)、
  `test_charuco_board` (合成盤の検出 + matchImagePoints)、`test_intrinsic_calib_session`
  (合成投影ビュー → solve → K/dist/rms が GT 近傍、pinhole/fisheye 両方)、`test_intrinsic_replay`
  (live↔replay)、`test_main_config` (calib-intrinsic の YAML/CLI/mode/precheck)。
- 実機: ChArUco を各カメラに見せ被覆を埋める → solve → rms_px < 0.5 目安、出力 YAML を
  案D/案C extrinsic + run 三角測量に投入して破綻しないこと。fisheye モデルで端の歪みが改善。

## 残課題
- 高 distortion での pinhole↔fisheye 自動推奨 (rms 比較) は将来。
- setup ウィザード (readiness API + ランディング) は別トラック。本 doc は intrinsic を flow の
  step0 に差すところまで。
