# pose-3d: 床 AprilTag 既知配置 PnP による extrinsic 校正 (案D)

(着手 2026-06-15 / research: [floor-apriltag-sfm-map.md](../research/floor-apriltag-sfm-map.md) /
上流: [pose-3d-controller-marker-extrinsic.md](pose-3d-controller-marker-extrinsic.md))

## 背景 / 動機

多カメラ extrinsic 校正は controller-marker hand-eye (案C) が実装済みだが、共通 frame が
VR world (動的) なので **Quest 3 の SLAM 原点ドリフトが inter-camera extrinsic に直接乗る**
のが律速。案D は床に AprilTag を既知配置し、各カメラを静的マップへ個別 localize することで
VR を extrinsic チェーンから外す代替経路。VR 非起動環境でも校正でき、再現性で案C を上回る
余地がある。

2026-06-13 の実機検証 (`python/scripts/detect_apriltags.py`) で **検出可否のみ**確認済み:
MJPG 高解像度 + CLAHE(`createCLAHE(2.0,(8,8))`) で安定検出、最悪カメラから edge_px ≥ 30
(安全 35) が配置条件。本作業はその次工程 — マップ定義・PnP solver・収集セッション・CLI/replay・
WebUI 選択統合 — のコア実装。

完了条件: 高解像度 intrinsics があれば録画 → replay で extrinsics YAML を出せ、WebUI から案C/案D
を選択式で起動でき、出力が既存 `Triangulator` でそのまま使えること。

## 検討した案

- **配置 (a) 既知配置 PnP を採用** / (b) スマホ SfM は将来。(a) はタグ間隔を実測して貼るだけで
  SfM 不要・最も堅い。(b) は本命方向だが COLMAP/自前 BA が要り実装規模が大きい。**両者は
  「マップへの localize」コア (`floor_extrinsic_solver`) を共有**し、(b) は `FloorTagMap` を
  生成するだけ — solver/session/CLI は無改変で乗る (分離線をここに引いた)。
- **別 solver / 別 session / 別 RunMode にした** (案C へ `--method` を足さない)。案C は
  controller・hand-eye(AX=ZB)・VmtWorld・motion gate を持つが、案D はそのどれも要らず world は
  FitraWorld。共有すると死にフィールド/分岐だらけになる。型 (`ExtrinsicSolution` ↔
  `FloorExtrinsicSolution`) も意味が違うので分けた。共有するのは AprilTag 検出と calib_io 書出のみ。
- **world = FitraWorld を直接定義** (案C のような VmtWorld→FitraWorld 基底変換が不要)。床マップ
  自身が world frame なので solvePnP の出力 `T_cam←world` をそのまま `T_cw` に書ける。これが
  案C との最大の構造差で、`solve_and_write` が単純になる。
- **マップ形式は OpenCV FileStorage YAML** (nlohmann_json でなく)。calib_io / intrinsics と同じ
  ファミリで `load/write` が対称に書け、cv::Mat の入出力が素直。**注意: FileStorage は
  `%YAML:1.0` が先頭行必須で、先頭コメント行を許さない** (検証で確認) → 例ファイルにヘッダ
  コメントは付けられない。形式説明は本 doc に置く。
- **WebUI 選択 = flow-switch に乗せる** / 新規 `/api/excal/select-method` は作らない。方式選択は
  本質的にモード切替なので、既存の flow daemon 再 spawn 経路 (exit code → daemon → ページ
  redirect) をそのまま使う。select-method 案は「同一プロセス内で session/受信/出力 frame を
  動的に差し替える」必要が出て、案C/案D を 1 binary に混ぜる結果になり没。
- **解像度非依存を利用**: extrinsic (`T_cw` = カメラ物理姿勢) は解像度に依存しない。校正は
  同画角・高解像度 + 高解像度 intrinsics で PnP し、出力 YAML にはランタイム解像度 intrinsics を
  載せる (`floor_intrinsics` ↔ `floor_out_intrinsics`/`calib`)。両者は別解像度でよい。

## 採用設計

### データフロー
```
FloorTagMap(YAML, 実測 or 将来SfM)         intrinsics(校正解像度)
        │                                          │
        ▼                                          ▼
FloorCalibSession.on_frame(cam,bgr,ts) ─ AprilTagDetector(CLAHE) ─ (cam,tag)→corner 蓄積/算術平均
        │ solve_and_write
        ▼
solve_floor_extrinsics(per-cam: 全タグの 2D↔3D を集約 → solvePnP) → T_cam_world(FitraWorld)
        │ 無変換 (basis change なし)
        ▼
calib_io.write_calibration(out: ランタイム intrinsics + T_cw) → Triangulator がそのまま消費
```

### 主要ファイル
- `cpp/src/lift/floor_tag_map.{hpp,cpp}` — `FloorTag{id,size_m,T_world_tag}` / `FloorTagMap`
  (`find`, `world_corners`=aruco 順)。FileStorage I/O (`R` 9要素 or `rpy_deg` を受理、write は `R`)、
  `floor_tag_grid` ヘルパ。`geom/frames.hpp` に `T_world_marker` を追加。
- `cpp/src/lift/floor_extrinsic_solver.{hpp,cpp}` — **localize コア (案a/b 共有)**。
  `solve_floor_extrinsics(cams,map,opts)`: カメラ毎に集約 `cv::solvePnP` (魚眼は
  `fisheye::undistortPoints` 経由)、再投影 RMS、**平面縮退検出** (world 3D 点群の最小主成分の
  厚み < `planar_warn_thickness_m`)。
- `cpp/src/pipeline/floor_calib_session.{hpp,cpp}` — 静的前提の収集。motion gate 無し、
  `(cam,tag)` ごとにコーナー算術平均、`burst_min` 到達のみ solver へ。`state_json` は
  `method:"floor"` と検出/coverage/再投影/平面縮退を返す。`set_on_solved` で flow 切替。
- `cpp/src/app/floor_calib_runner.{hpp,cpp}` / `floor_live_input.{hpp,cpp}` — `ExcalInputSource`
  を流用 (controller pose は破棄、frame timestamp のみ)。live は bus 不要の `FloorLiveInput`。
- `cpp/src/app/mode_calib_extrinsic_floor.{hpp,cpp}` — `RunMode::CalibExtrinsicFloor` の runner。
  replay は完全 unattended、live は Crow + `FloorCalibSession` を attach。再投影誤差を stdout JSON。
- `cpp/src/web/crow_routes_setup.cpp` `register_floor_calib_routes` — `/extrinsic-calib` +
  `/api/excal/{state,start,stop,solve}` を floor session で再利用 (controller 専用の
  `/api/excal/poses` は非登録)。
- WebUI: `web/extrinsic_calibration/{index.html,app.js}` に方式トグル + floor 分岐、
  `web-ui/public/flow.js` / `web-ui/src/hooks/useFlowWatch.ts` の `PAGE_FOR_MODE` に
  `calib-extrinsic-floor → /extrinsic-calib`、redirect 判定を target ページ比較に変更
  (同一ページ共有でリロードループを防ぐ)。

### 不変条件
- 出力 `T_cw` は **FitraWorld (Z-up, 床=Z=0)**、無変換。`coordinate_system` 文字列で明示。
- `FloorTagMap` は生成元 (実測/SfM) に依存しない契約。`size_m` が (b) のスケール基準。
- 案C と案D は別プロセス (flow-switch 再 spawn) でしか動かないので `/api/excal/*` 名を共有してよい。

### マップ YAML 形式
FileStorage YAML。先頭は `%YAML:1.0` 必須 (コメント行を前置しない)。各タグ:
`{ id, size_m, R:[9] | rpy_deg:[roll,pitch,yaw](deg), t:[x,y,z] }`。床タグは `R=I, z=0`、面外
スタンドは任意 6DoF。原点タグ中心=world 原点・+Z up。検証済み例: `configs/floor_tag_map.yaml.example`。

## Milestone

- **M1** AprilTagDetector に CLAHE オプション (案C/D 共通)。
- **M2** `FloorTagMap` + FileStorage I/O + grid ヘルパ。
- **M3** `floor_extrinsic_solver` (多タグ PnP・再投影・平面縮退)。
- **M4** `FloorCalibSession` (burst 平均・solve_and_write 無変換書出)。
- **M5** `floor_calib_runner` + replay 等価テスト。
- **M6** `RunMode::CalibExtrinsicFloor` + CLI/replay (headless) + 再投影レポート。
- **M7** WebUI 選択式起動 (flow-switch + Crow floor ルート + vanilla 方式トグル)。
- **M8** (b) SfM 拡張点をヘッダ doc に明記 (コードは無改変)。
- **M9/M10** ビルド配線 + 本 doc + track changelog + migration-plan 更新。

## 検証

- ctest: `test_floor_tag_map` (I/O round-trip・world_corners・grid)、`test_floor_extrinsic_solver`
  (round-trip ノイズ無で復元 < 1e-3deg/float 精度・平面縮退検出・px ノイズ・点数不足)、
  `test_floor_calib_session` (ingest→solve→write→reload で `T_cw` が GT 一致・fitra Z-up)、
  `test_floor_calib_replay` (direct imdecode ↔ ExcalReplayInput の蓄積一致)、`test_main_config`
  (YAML/CLI/mode/validate)。
- 実機 (intrinsics 取得後): 高解像度 + CLAHE で床配置を撮影 → `excal_record` 録画 →
  `--floor-replay` で solve → 再投影 RMS (目標 < 3px、できれば < 1px) と平面縮退フラグ確認 →
  出力を `Triangulator` に渡して三角測量成立を確認。
- WebUI: daemon 起動 → /extrinsic-calib で方式トグル → 案D 再 spawn・floor UI 表示 →
  start/solve で YAML 出力 → run へ自動遷移。案C ⇄ 案D 往復。

## 残課題

- **intrinsics 取得 (前提工程)**: 同画角・高解像度の魚眼 intrinsics を
  `python/scripts/calibrate_intrinsics_charuco.py` で取得。PnP は校正解像度の K/dist、出力 YAML は
  ランタイム解像度。`T_cw` 解像度非依存で両立。
- **(b) スマホ SfM** (本命方向): `lift/floor_map_sfm.*` が `FloorTagMap` を生成すれば本コアに乗る。
  research doc に残置。
- **floor 検証シーン** (`/extrinsic-calib/scene.html`): 現状 floor は `/api/excal/extrinsics` 非提供。
  必要なら floor 用 extrinsics_json を追加。
- 実機での再投影 RMS / 平面縮退の実値はまだ未取得 (intrinsics 取得後に確定)。
