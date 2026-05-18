# Phase 10 — 3 カメラ対応 + マルチカメラキャリブ C++ 化 + ライブ ChArUco 可視化

## Context

Phase 6b で C++ パイプライン (V4L2 → YOLOX → RTMPose) が 2 カメラ aggregate 170 fps、Phase 7 で 3D triangulation + IK、Phase 8 で被験者プロファイル wizard が動いている。一方、**マルチカメラキャリブレーション (内部 ChArUco + 外部 床点 PnP) は依然 Python (`python/scripts/calibrate_intrinsics_charuco.py` / `measure_extrinsics_web.py`) に残っており**、運用フローが Python ↔ C++ で分断されている。Python の `--web` モードは MJPEG オーバーレイ (`<img src="/stream/cam0">`) を ~10 fps で配信する仕組みで、エンコード負荷で遅く視認性も劣る。

Phase 10 のゴール (ユーザー確定):

1. **3 カメラ end-to-end**: 既存 main は `--cam0/1/2` を持つが、Phase 8 wizard と `dump_keypoints_3d` は `cam0/cam1` ハードコード。3 カメラ運用で全機能を通す。
2. **マルチカメラキャリブを C++ 化 (内部+外部 両方)**: ChArUco 内部キャリブと床点 PnP 外部キャリブをともに C++ + Crow に移植。出力 YAML スキーマ `fitra_cam_calibration_v1` は不変、`cpp/src/lift/calib_io.cpp` がそのまま読める形を維持。Python ツールは参照実装として残置。
3. **MJPEG オーバーレイ廃止、ブラウザ WS + Canvas でライブ ChArUco 表示**: 検出結果 (corners, ids, board pose, reproj err, coverage) を WebSocket JSON で 30 Hz 配信し、ブラウザ Canvas でカメラ実寸座標系にベクター描画。視覚的にわかりやすく、サブピクセル精度のフィードバックで高品質なサンプル収集ができる。
4. **Phase 9 (Halpe26) とは独立**: Phase 10 はキーポイントフォーマット非依存。`kNumKeypoints` 等は現状維持。

---

## ハイレベル設計

### モード分離
`cpp/build/main` に `--mode {pose | calib-intrinsics | calib-extrinsics}` を追加 (デフォルト `pose`)。
- `pose`: 現行動作 (TRT 推論 + Web UI + Phase 8 wizard)
- `calib-intrinsics`: GPU/TRT 起動なし、V4L2 → JPEG decode → ChArUco 検出 → WS 配信 + Solve API
- `calib-extrinsics`: 静止画 (or 1枚キャプチャ) + クリック注釈 → PnP solve

### 流用パターン
- **`CalibrationSession` の state machine** (Phase 8) を ChArUco/Extrinsics 両セッションで再利用。重い処理は別スレッド (`launch_*_thread()`)、lock 配下で受理サンプルを `std::move` でスワップ。
- **`crow_server.cpp::register_calibration_routes_()`** と同じ登録パターンで `register_charuco_routes_()` / `register_extrinsics_routes_()` を追加。
- **原子的 YAML 書き出し** (`tmp + fs::rename`) を `latest_profile.yaml` の Phase 8 実装から踏襲し、`write_calibration_yaml` 内部に共通化。
- **JSON は `std::ostringstream` で手書き** (依存追加せず、Phase 8 と同じ方針)。

### 早期検証用の垂直スライス
**Milestone 3 単独で動く "ChArUco 観測のみ + WS 配信" バイナリ** を最初に通す。calibration 解算/YAML 書き出しを後回しにし、3 カメラの V4L2 並列 + ChArUco 検出 + WS の 30 Hz + ブラウザ Canvas オーバーレイ までを一気に通してリスクを潰す。

---

## マイルストーン

### M1 — 3 カメラ制約の解除

**修正ファイル**:
- `cpp/src/main.cpp` — Phase 8 wizard の `n_cams == 2` 判定を `n_cams >= 2` に緩める (L409–423 付近)。`cam_paths.resize(3)` と `for (auto& path : cam_paths) if (path.empty()) continue;` のループは既に N=1..3 対応済み。
- `cpp/src/pipeline/calibration_session.{hpp,cpp}` および `cpp/tools/dump_keypoints_3d.cpp` の汎化は M7 で。M1 では起動拒否を外す土台のみ。

**検証**: `--cam0/--cam1/--cam2 --enable-3d --calib ...` で起動が拒否されないこと。

---

### M2 — `calib_io` の書き込み API 追加

**修正ファイル**: `cpp/src/lift/calib_io.{hpp,cpp}`

**追加 API**:
```cpp
struct CalibrationWriteOptions {
    std::string schema = "fitra_cam_calibration_v1";
    std::string unit = "m";
    std::string coordinate_system = "world: x/y measured on floor, z up; extrinsics are T_cw";
    std::string created_at;        // RFC3339; default = now(UTC)
    std::string metadata_json;     // pre-serialized JSON line
};

void write_calibration_yaml(
    const std::string& path,
    const std::map<std::string, Intrinsics>& runtime_intrinsics,
    const std::map<std::string, Intrinsics>* capture_intrinsics,
    const std::map<std::string, Extrinsics>* extrinsics,
    const std::string& quality_json_inline,   // serialized; empty -> "quality: {}"
    const CalibrationWriteOptions& opts);
```

**出力フォーマット**: Python `calibration_io.py::_matrix_yaml` と完全互換。`!!opencv-matrix` ブロック、`%.12g` 精度、`extrinsics: {}` empty 時の括弧出力、`metadata_json: "..."` の 1 行埋め込みまで再現。

**検証**: 既存 `calibrations/intrinsics.yaml` を `load_calibration` で読み込み → `write_calibration_yaml` で書き戻し → 再ロードで K, dist, rms_px が `< 1e-9` 一致。Python `load_intrinsics_yaml` でも同 YAML を読めることを確認。

---

### M3 — ChArUco 観測 + WebSocket 配信 (MJPEG 不使用)

**新規ファイル**:
- `cpp/src/calib/charuco_worker.{hpp,cpp}` — per-camera 検出ワーカー
- `cpp/src/calib/charuco_session.{hpp,cpp}` — セッションオーケストレータ
- `cpp/tools/calibrate_intrinsics.cpp` — CLI エントリ (M4 で完成)

**修正ファイル**:
- `cpp/src/CMakeLists.txt` — `fitra_calib` ライブラリを追加 (`OpenCV::objdetect`, `OpenCV::calib3d`, `OpenCV::imgcodecs` リンク)
- `cpp/src/web/crow_server.{hpp,cpp}` — `register_charuco_routes_()` フック
- `cpp/src/main.cpp` — `--mode calib-intrinsics` 分岐, ChArUco 関連フラグ

**`CharucoWorker` 設計**:
- メンバ: `V4l2Capture` + `JpegDecoder` (既存) + `cv::aruco::CharucoDetector` + `cv::aruco::CharucoBoard`
- 1 スレッド `worker_loop()`: V4L2 dequeue → JPEG decode → グレー化 → `detectBoard()` → 受理判定 → `accepted_corners_/_ids_` 蓄積 + coverage_mask 更新 + 最新 detection 1 スロット書き込み
- 受理判定 (Python L341–354 同等): `collecting && ids.rows >= min_corners && now - last_accept >= sample_interval && accepted < target`
- `coverage_mask_` = `(H/16 × W/16) 8UC1`, 受理時に corner 所属セル ++。`coverage_pct = nonzero / total`。
- OpenCV ArUco API 版差: `#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR==4 && CV_VERSION_MINOR>=7)` で新 API (`objdetect/charuco_detector.hpp` の `CharucoDetector`) と legacy (`interpolateCornersCharuco`) を分岐。Jetson 環境は 4.8 系で新 API 想定。

**`CharucoSession` 設計**:
- 状態: `kIdle / kCollecting / kPaused / kSolving / kDone / kFailed`
- API: `session_payload()`, `start()`, `pause()`, `reset_all()`, `reset(cam_id)`, `solve_blocking()`, `latest_bundle_json()` (WS publisher 用)
- Solve 処理は `std::thread` で逃がし、Phase 8 の `launch_finalize_/launch_analyzer_` パターンを踏襲 (lock 内で `std::move` でスワップ → lock 外で重処理)。
- 入力構造体 `CharucoBoardSpec{squares_x, squares_y, square_len_m, marker_len_m, dict_name, samples, min_corners, sample_interval_sec, runtime_width, runtime_height}` を CLI/REST 共通化。

**WS スキーマ (`/ws/charuco`, 30 Hz)**:
```jsonc
{
  "seq": <uint>,
  "ts_ms": <double>,
  "state": "collecting" | "paused" | "idle" | "solving" | "done" | "failed",
  "samples_target": <int>,
  "min_corners": <int>,
  "runtime": {"width": int, "height": int},
  "board": {"squares_x":int, "squares_y":int, "square_len":float, "marker_len":float, "dict":"DICT_4X4_50"},
  "cameras": [
    {
      "id": "cam0",
      "device": "/dev/v4l/...",
      "capture_w": int, "capture_h": int,
      "accepted": int,
      "collecting": bool,
      "last_frame_age_ms": float,
      "latest": {
        "ts_ms": float,
        "corners": [[x,y], ...],        // ChArUco corners, capture pixel coords, %.3f
        "ids":     [int, ...],
        "markers": [[[x,y],[x,y],[x,y],[x,y]], ...], // optional, --charuco-no-markers で抑制
        "marker_ids": [int, ...],
        "n_corners": int,
        "accepted_now": bool             // 1フレームだけ true
      },
      "coverage_pct": float,
      "coverage_grid": {"cols":int,"rows":int,"hits":[int, ...]},  // <=200 要素
      "last_solve_rms_px": float,
      "error": null | string
    }
  ]
}
```
- 1 メッセージは 640×480 で 3 カメラ × ~70 corners ≈ 1 KB 以下。MJPEG 経路 (~3 MB/s) と比較し帯域 15 分の 1。
- `corners` は `%.3f` 丸めで送信 (サブピクセル精度を保ちつつ帯域節約)。

**REST**: `/calib-intrinsics` (HTML), `/calib-intrinsics/<path>` (static), `/api/charuco/{session,start,pause,reset,solve}`, WS `/ws/charuco`。

**CLI フラグ追加**:
- `--mode calib-intrinsics`
- `--charuco-squares-x N --charuco-squares-y N --charuco-square-len F --charuco-marker-len F`
- `--charuco-dict NAME` (default `DICT_4X4_50`)
- `--charuco-samples N` (default 25)
- `--charuco-min-corners N` (default 8)
- `--charuco-sample-interval F` (default 0.35)
- `--out PATH` (default `calibrations/intrinsics.yaml`)
- `--charuco-capture-w/--charuco-capture-h` (推論時の `--width/--height` とは独立)

**検証**: `wscat -c ws://localhost:8000/ws/charuco` で 1 秒 ≥ 25 メッセージ受信、`n_corners > 0`、ブラウザ Canvas で 3 カメラのコーナーが描画される。

---

### M4 — Intrinsic 解算 + YAML 書き出し

**修正ファイル**: `cpp/src/calib/charuco_session.cpp` (`solve_blocking()` 完成), `cpp/tools/calibrate_intrinsics.cpp`

**Solve フロー**:
1. 全 worker から `accepted_corners_/_ids_` を lock 経由でローカルへ move
2. cam ごとに `CharucoBoard::matchImagePoints()` でループ → `cv::calibrateCamera(objAll, imgAll, imageSize, K, dist, rvecs, tvecs, 0)` を呼ぶ
3. capture 解像度の K, dist, RMS を埋める (`source="charuco_capture"`)
4. `scale_intrinsics_for_runtime()` を Python (calibrate_intrinsics_charuco.py 内) から C++ に移植: `sx = runtime_w / capture_w; sy = runtime_h / capture_h; K[0,*] *= sx; K[1,*] *= sy; rms_px *= (sx+sy)/2`
5. `quality_json` 文字列を組み立て: `{intrinsic_camera_count, camN: {capture_rms_px, runtime_rms_px, samples_used}}`
6. `write_calibration_yaml(out_path_tmp, runtime, capture, nullptr, quality, opts)` → `fs::rename(tmp, out_path)` で原子的差し替え

**検証**: 既存 Python YAML から accepted corner セットをエクスポート (デバッグ用 `--charuco-image cam0=path` モード) し、C++ で solve した K と Python の K を比較、`||K - K_python||_F < 0.5 px`。

---

### M5 — 床点 PnP Extrinsic ツール (ロジック)

**新規ファイル**:
- `cpp/src/calib/world_points.{hpp,cpp}` — `world_points.json` パーサ
- `cpp/src/calib/extrinsics_session.{hpp,cpp}` — Session ロジック
- `cpp/tools/calibrate_extrinsics.cpp` — CLI エントリ

**`WorldPoints` パース**: Python `load_world_points + _expand_floor_grid` の逐語移植。
- nlohmann/json は依存追加せず、`cv::FileStorage` を JSON モードで使用 (`cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON`)
- `floor_grid.x_m / y_m / z_m` 配列を展開 → `grid_r{row:02d}_c{col:02d}` ID で `points` リストに追加
- `point_ids[row][col]` 2 次元配列をクライアントの "Fit Grid" 用に保持
- `measurements.camera_heights_m`, `baselines_m` も保持 (quality JSON 出力時に使用)

**`ExtrinsicsSession` 設計**:
- 状態: `kIdle → kCapturing → kAnnotating → kSolving → kDone / kFailed`
- 入力:
  - `intrinsics_yaml_path` (M4 の成果物)
  - `world_points_json_path` (`calibrations/world_points.json`)
  - `output_dir` (`calibrations/measure_session/`)
  - カメラ集合: `--image cam0=path` (静止画) または `--cam cam0=/dev/v4l/...` (ライブで 1 枚抜き取り、Python `capture_still` 相当)
- 永続データ:
  - `<output_dir>/images/cam0.jpg ...`
  - `<output_dir>/annotations.json` (Python 互換: `{cameras:{camN:{point_id:[x,y]}}, grid_observations:{}, points:[...], floor_grid:{...}}`)
  - `<output_dir>/overlays/camN_reprojection.jpg` (任意)
  - `<output_dir>/cam_params.yaml` (最終結果)

**`solve_camera_pose` 移植** (Python `measure_extrinsics_web.py` L207–309 を 1:1 移植):
- 平面判定: `obj.col(2)` の ptp < 1e-6 で planar
- planar: `cv::solvePnPGeneric(SOLVEPNP_IPPE)` で全候補列挙 → 各候補で `projectPoints` + `Rodrigues` → `camera_center = -R^T t`
- 失敗時フォールバック: `cv::solvePnPRansac(SOLVEPNP_EPNP, reproj=8.0)` → `cv::solvePnP(SOLVEPNP_ITERATIVE)`
- スコア = 平均再投影誤差 + (measured_height_m があれば `|center.z - measured| * 25.0`、`center.z < -0.05` で `+1000`)
- ベスト候補に `solvePnPRefineLM` をかける
- 戻り: `Extrinsics{method="measured_floor_points_pnp", T_cw, camera_center_w, rvec, tvec}` + Quality {used_points, used_grid_points, reproj_{rms,mean,p95,max}_px, estimated_height_m, measured_height_m, height_error_m}

**検証**: 既存 `calibrations/measure_session/annotations.json` を C++ 版に入力し、cam0/cam1 の `T_cw` が Python 出力と Frobenius ノルムで `< 1e-5` 一致。

---

### M6 — Extrinsic REST + WS 配信

**修正ファイル**: `cpp/src/web/crow_server.{hpp,cpp}`, `cpp/src/calib/extrinsics_session.cpp`

**REST**:
| Route | Method | 動作 |
|---|---|---|
| `/calib-extrinsics`, `/calib-extrinsics/<path>` | GET | static |
| `/calib_images/<file>` | GET | output_dir/images の静止画を返す (Python と同パス) |
| `/api/extrinsics/session` | GET | カメラ一覧 + 画像 URL + world_points + floor_grid + 既存 annotations + last_quality |
| `/api/extrinsics/annotations` | POST | annotations.json に原子的 rename で保存 |
| `/api/extrinsics/solve` | POST | solve スレッド起動 → 完了で cam_params.yaml + overlays + quality.json |
| `/api/extrinsics/state` | GET | solve 進捗 polling |
| WS `/ws/extrinsics` | – | solve 進捗の即時通知 (任意) |

**検証**: annotations POST → ファイルの中間 `.tmp` 経由 rename を確認。solve POST → `cam_params.yaml` が `load_calibration` でロードできる。

---

### M7 — Phase 8 Subject Wizard を N カメラ化

**修正ファイル**:
- `cpp/src/pipeline/calibration_session.{hpp,cpp}`
- `cpp/tools/dump_keypoints_3d.cpp`
- `web/subject_calibration/{index.html,app.js}`

**`calibration_session` の変更**:
- `std::array<std::array<std::vector<FrameItem>, 2>, kPoseCount> buffers_;` → 内側を `std::vector` にして N カメラ可変長化
- `CalibPoseRecord::frames/fps` を `std::array<*,2>` → `std::vector<*>` に
- `set_camera_count(std::size_t n)` を追加し、preflight 前に main から呼ぶ
- L341 `if (cam_idx >= 2) return;` → `if (cam_idx >= n_cams_) return;`
- L352–356 の "両カメラ満タンか" → `std::all_of` で N カメラ判定
- L460–477 の `raw/<pose>_cam{0,1}.mp4` ループ → `for (size_t ci=0; ci<n_cams_; ++ci)`
- L496–498 の `"cameras":[{"id":"cam0"},{"id":"cam1"}]` を動的生成
- L688–694 `cam0_buf/cam1_buf` → `std::vector<int>`
- `state_json()` の `"buffered/frames/fps"` 配列を可変長に

**`dump_keypoints_3d.cpp`**:
- `--cam1-frame-offset` → `--cam-frame-offset N=K` (repeat 可能) に変更 (旧形式は後方互換エイリアス)
- L552 `require_camera_ids({"cam0","cam1"})` → `pose_session.json` の `camera_count`/`cameras[]` から動的に
- L274 の `idx < 2` ループ、L280–283 の `raw/<pose>_camN.mp4` を N 化
- Triangulator は既に N 対応のため修正最小

**`web/subject_calibration/app.js + index.html`**:
- `rec_bar0/rec_count0/rec_bar1/rec_count1` 等の cam0/cam1 ハードコード DOM → `state.cameras.length` から動的生成 (JS で `<div data-cam="camN">` をループ生成)
- 2 ペイン CSS グリッド → `repeat(auto-fit, minmax(...))` の動的グリッドに

**検証**: 3 カメラで `--calibrate` → `raw/standing_cam{0,1,2}.mp4` の 3 本出力、`pose_session.json.camera_count == 3`。2 カメラ構成での回帰も確認。

---

### M8 — ChArUco Web UI (MJPEG なし)

**新規ファイル**: `web/calib_intrinsics/{index.html,app.js,styles.css}` (既存 `web/calibration/` は撤去せず Python ツール用に残置)

**UI 構造**: ヘッダに `Start/Pause/Reset/Solve` ボタン + ステータス。`<section id="cameraGrid">` 内に `<article.camera>` × N、各 article に `<canvas data-cam="camN" width=640 height=480>` (capture 解像度) と `<div.hud>` (受理数/coverage/last RMS)。

**app.js 描画戦略**:
- WS `/ws/charuco` を購読、メッセージ毎 (30 Hz) に各 camera について:
  1. `ctx.clearRect`
  2. **Coverage オーバーレイ**: `coverage_grid.hits` を低明度の青で塗る (受理が濃いセル ほど明るく)
  3. **マーカー輪郭**: `markers[]` の各 quad を薄い緑の polyline
  4. **ChArUco corners**: `latest.corners` を半径 2 px の白丸 + ID 順の隣接連結線 (赤)
  5. ID ラベル小文字
  6. HUD: `accepted/target corners=N rms=last_solve_rms_px coverage=XX%`
  7. `latest.accepted_now == true` の瞬間は周囲 1 フレーム黄色フラッシュ
- **背景画像は読み込まない (MJPEG 撤廃)**。Canvas を capture 解像度で確保し、CSS `transform: scale(...)` でカード幅にフィット。
- データ更新は WS レート、描画は `requestAnimationFrame` で 60 Hz (latest-wins)。

**検証**: 3 カメラで 60 秒間ボードを動かし、Canvas がフリーズしない。DevTools の WS 帯域が **< 200 KB/s** (MJPEG 3 cam の ~3 MB/s と比較し 15 分の 1)。

---

### M9 — Extrinsic Web UI (クリック → world point)

**新規ファイル**: `web/calib_extrinsics/{index.html,app.js,styles.css}` (既存 `web/calibration/index.html,app.js` をベースにコピー、Python ツール用に originals は残置)

**主な変更点**:
- API パス: `/api/session` → `/api/extrinsics/session` 他
- 画像 URL: `/calib_images/...` (Crow が同じパスをマウント)
- **"Fit Grid" 機能はそのまま流用**: 8 パラメータ正規方程式 (`fitHomography + projectHomography`) で 4+ アンカーから DLT で H を求めて欠損 grid 点を補完。Z=0 floor の仮定下で Python と一致。
- SVG クリック・キー操作は既存ロジック流用

**検証**: 4 点クリック → "Fit Grid" → 残り点が grid に展開。Solve → quality バッジと `cam_params.yaml` パス表示。

---

### M10 — 回帰テスト + ドキュメント

**新規ファイル**:
- `cpp/tools/test_calib_roundtrip.cpp` (ユニットテスト)
- `docs/phase10_calibration.md` (運用ドキュメント)

**テスト範囲**:
- `write_calibration_yaml` → `load_calibration` round-trip (K/dist/T_cw < 1e-9)
- Python `load_intrinsics_yaml` で C++ 産出 YAML が読める (cv::FileStorage 互換)
- `WorldPoints::parse_json` の grid 展開が Python と一致
- `solve_camera_pose` の 4 点既知解一致

**回帰確認 (実機)**:
- Phase 6b の 170 fps ベンチ (`recent_pose_fps`) を `--mode pose` で再測定
- Phase 1 correctness (`outputs/recorded_rtmpose/20260515_064342/raw_cam0.mp4` 30 frames, max kpt L2 < 1 px)
- 3 カメラでの Phase 8 subject wizard 完走 (2 カメラ運用も合わせて回帰)

**ドキュメント**:
- 3 モード (`--mode pose / calib-intrinsics / calib-extrinsics`) の使い方
- ChArUco board の物理スペック例 (`squares_x=8 squares_y=11 square_len=0.045 marker_len=0.034`)
- WS スキーマリファレンス
- Python ツール → C++ ツール移行表

---

## 推奨実行順序 (依存順)

1. **M1** — `n_cams == 2` 制約解除 (小、即座に効く)
2. **M2** — `calib_io` 書き込み API (以降のすべての土台)
3. **M3** — ChArUco 観測 + WS 配信 (★垂直スライス α; ここで一度デモ可能)
4. **M8** — ChArUco Web UI (M3 と並行可)
5. **M4** — Intrinsic 解算 + YAML
6. **M5 → M6** — Extrinsic ツール + REST
7. **M9** — Extrinsic Web UI (既存 web/calibration を borrow)
8. **M7** — Phase 8 wizard の N カメラ化 (独立、M1 が前提)
9. **M10** — 回帰 + ドキュメント

---

## クリティカルファイル

**新規**:
- `cpp/src/calib/{charuco_worker,charuco_session,extrinsics_session,world_points}.{hpp,cpp}`
- `cpp/tools/{calibrate_intrinsics,calibrate_extrinsics,test_calib_roundtrip}.cpp`
- `web/{calib_intrinsics,calib_extrinsics}/{index.html,app.js,styles.css}`
- `docs/phase10_calibration.md`

**修正**:
- `cpp/src/main.cpp` — `--mode` 分岐, 制約解除
- `cpp/src/CMakeLists.txt`, `cpp/tools/CMakeLists.txt`
- `cpp/src/lift/calib_io.{hpp,cpp}` — 書き込み API
- `cpp/src/web/crow_server.{hpp,cpp}` — ChArUco / Extrinsics ルート登録
- `cpp/src/pipeline/calibration_session.{hpp,cpp}` — N カメラ汎化
- `cpp/tools/dump_keypoints_3d.cpp` — N カメラ汎化
- `web/subject_calibration/{index.html,app.js}` — N カメラ汎化

**不変** (Python は参照実装として温存):
- `python/scripts/calibrate_intrinsics_charuco.py`, `measure_extrinsics_web.py`, `calibration_io.py`
- `web/calibration/` (Python ツールが参照)
- `cpp/src/lift/calib_io.cpp::load_calibration` のスキーマ

---

## リスクと緩和策

| ID | リスク | 緩和 |
|---|---|---|
| R1 | OpenCV ArUco API の版差 (4.7+ 新 vs 4.5 legacy) | `#if CV_VERSION_*` を `charuco_session.cpp` 1 箇所に集約。Jetson は 4.8 で新 API ベース |
| R2 | ChArUco coverage 偏り (Python は枚数しか見ない) | `coverage_grid` 16×12 を持ち、`--charuco-uniform-coverage` opt-in で coverage 不足セルのみ受理。デフォルト OFF で Python 互換 |
| R3 | Canvas ↔ capture pixel 座標 | Canvas を capture 解像度で確保し CSS `transform: scale` でフィット。WS は `%.3f` 丸めで送信 |
| R4 | "Fit Grid" homography の数値安定 | DLT 正規方程式は既存 JS をそのまま使用。共線アンカー警告を UI で出す |
| R5 | YAML 同時更新の競合 | `tmp + fs::rename` の atomic 書き出し (Phase 8 流用) |
| R6 | 並列 Session の競合 | `--mode` で排他、同一プロセスで 1 モードのみ起動 |
| R7 | V4L2 解像度競合 | `--charuco-capture-w/h` を分離。MJPEG fourcc 固定 |
| R8 | GPU 不要モードの起動コスト | `--mode calib-*` では TRT/CUDA を一切初期化しない経路を main.cpp に追加 |
| R9 | サブピクセル精度 | `cv::aruco::CharucoDetector` の cornerSubPix (11×11 default) を有効活用、`%.3f` 送信 = 1/1000 px 精度 |
| R10 | Python YAML round-trip | `capture_intrinsics` ブロックを `write_calibration_yaml` で必ず emit、`extrinsics: {}` 空時の表記も再現 |

---

## 検証 (end-to-end)

実機 3 カメラ (cam0=`2.3:1.0`, cam1=`2.4:1.0`, cam2=新規 by-path) で 1 セッション通し:

1. `./cpp/build/main --mode calib-intrinsics --cam0 ... --cam1 ... --cam2 ... --charuco-samples 30 --out calibrations/intrinsics.yaml`
   → ブラウザで Canvas に 3 ペインの ChArUco オーバーレイ、各カメラ 30 サンプル収集 → Solve → `intrinsics.yaml` 生成
2. `./cpp/build/main --mode calib-extrinsics --intrinsics calibrations/intrinsics.yaml --world-points calibrations/world_points.json --cam0 ... --cam1 ... --cam2 ... --out calibrations/measure_session/cam_params.yaml`
   → 1 枚キャプチャ → ブラウザでクリック注釈 → Fit Grid → Solve → `cam_params.yaml` 生成
3. `./cpp/build/main --mode pose --cam0 ... --cam1 ... --cam2 ... --enable-3d --calib calibrations/measure_session/cam_params.yaml --calibrate --calib-subject-id subj01 --calib-subject-height-m 1.72`
   → 3 カメラの skeleton + Phase 8 wizard が走り、`raw/standing_cam{0,1,2}.mp4` 3 本録画、`latest_profile.yaml` 生成
4. `./cpp/build/main --mode pose --cam0 ... --cam1 ... --cam2 ... --enable-3d --calib ... --subject-id subj01` で 3 カメラライブ実行、`recent_pose_fps` aggregate 値を計測 (Phase 6b ベースラインから大幅劣化していないこと)

**合格基準**:
- 3 カメラ ChArUco solve の K, dist が Python と `||K - K_python||_F < 0.5 px`
- 3 カメラ extrinsic solve の T_cw が Python と Frobenius ノルム `< 1e-5`
- WS 帯域 `< 200 KB/s` (3 cam, MJPEG ≈ 3 MB/s の 15 分の 1)
- Canvas オーバーレイ更新レイテンシ `< 50 ms`
- Phase 1 correctness (kpt L2 < 1 px) と Phase 6b の aggregate fps 維持
