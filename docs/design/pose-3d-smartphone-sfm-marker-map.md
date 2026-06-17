# pose-3d: スマホ動画から床 AprilTag マップを生成する (案D mode (b) SfM)

(着手 2026-06-17 / 上流: [pose-3d-floor-apriltag-extrinsic.md](pose-3d-floor-apriltag-extrinsic.md) M8 /
intrinsic: [pose-3d-intrinsic-calibration.md](pose-3d-intrinsic-calibration.md))

## 背景 / 動機

床 AprilTag extrinsic 校正 (案D) は **`FloorTagMap` (各タグの `T_world_tag`) を入力に取る**。
これまでは mode (a) = 巻尺で実測して YAML を手書きする経路しかなく、「**タグ間距離の入力が面倒**」
が実運用の律速だった。案D の設計時点で mode (b) = スマホ SfM が本命と位置づけられ、コア
(`floor_extrinsic_solver`) は生成元非依存に作ってあった (分離線は floor-apriltag-extrinsic doc
「検討した案」参照)。本作業はその mode (b) — **動画から `FloorTagMap` を自動生成** — の実装。

前提は「リグカメラとは別の手持ちカメラ (スマホ) で、固定光学状態 (focus/zoom 固定・手振れ補正
OFF) で床マーカーを舐めるように撮る」こと。マップは **どのカメラで作っても物理配置そのもの** な
ので、手持ちで作ったマップを後から固定リグが localize する運用が成立する (マップは world 定義、
カメラ非依存)。各タグの実寸 (例 114.5mm) が**メトリックスケールを固定**するため、特徴点 SfM の
ようなスケール不定性は無く、巻尺は不要になる。

完了条件: (1) ChArUco 動画からスマホ intrinsics を出せる、(2) マーカー動画から `FloorTagMap`
(既存スキーマ) を自動生成でき、(3) 生成マップが既存 `solve_floor_extrinsics` でそのまま localize
できる (再投影で検証)。

## 検討した案

- **多フレーム pose-graph 連結を採用** / 単一アンカーフレーム PnP は不可。実測 (samples) では
  1 フレーム平均 1.8 タグ・最大 5 タグで、**単一フレームに全 8 タグは映らない**。共可視グラフは
  連結なので、フレーム毎の共可視ペアの相対姿勢を蓄積 → アンカーから BFS で全タグを配置する。
- **実装は C++ (既存 `fitra_lift` 再利用)** / Python ではない。検出 (`AprilTagDetector` /
  `CharucoBoardDetector`)・PnP (`solve_tag_pose`)・I/O (`floor_tag_map_write`/`calib_io`)・
  検証 (`solve_floor_extrinsics`) が全て C++ 4.8 (modern aruco) に既存。Python cv2 は 4.5.4
  (legacy aruco) かつ「python/ に機能追加しない」方針のため不採用。
- **外部 BA / COLMAP は導入しない**。共可視グラフが連結なら spanning-tree 連結 + 冗長エッジでの
  pose 平均 (緩和) で十分な初期マップが出る。Ceres 等の新規依存はコスト過大。BA は精度を詰める
  将来拡張として残す (残課題)。
- **エッジは raw `cv::Matx44d` で計算**。`geom::Transform` は型で frame を区別するが、2 つの
  `T_cam_marker` (どちらも `From=Marker`) は型付き `operator*` で連結できない (tag i と tag j を
  型が区別しない)。pose-graph 演算は `geom::compose/invert_rigid/average_poses/rotation_angle_deg`
  の raw ヘルパで行い、最終結果だけ `T_world_marker::from_raw` で包む。
- **ロバスト集約は MAD トリム平均**。`geom` に SE(3) median ヘルパは無い。各エッジで chordal
  平均 (`average_poses`) → 回転角/並進距離の残差を MAD でゲート → 内点を再平均。
- **床平面再ゲージを既定 ON**。床タグは同一平面上なので、全タグ中心に平面フィット (共分散最小
  固有ベクトル = 法線、`floor_extrinsic_solver` の `min_axis_thickness` と同手法) し、法線→+Z・
  アンカー面内 X→world X・床→z=0 に再ゲージ。出力が FitraWorld (z-up, 床=z=0) 契約に直接乗る。
  アンカー単独の PnP 傾きノイズを全タグの平面で均す効果もある。

## 採用設計

### データフロー
```
[A] ChArUco 動画 ──VideoCapture──▶ CharucoBoardDetector ──▶ IntrinsicCalibSession(pinhole)
                                                              └─▶ intrinsics.yaml (スマホ)
[B] マーカー動画 + intrinsics ──▶ AprilTagDetector(CLAHE) ──▶ per-frame {id, T_cam←tag, corners}
        │ build_floor_map_sfm: 共可視ペア相対姿勢を蓄積 → 各エッジ MAD トリム平均
        │                      → アンカー BFS → pose 平均緩和 → 床平面再ゲージ
        ▼
   FloorTagMap (FitraWorld z-up) ──floor_tag_map_write──▶ floor_tag_map.yaml
        │ (holdout フレームで自己検証)
        ▼
   solve_floor_extrinsics({holdout cam}, map) ──▶ 再投影 RMS レポート
```

### 主要ファイル
- `cpp/src/lift/floor_map_sfm.{hpp,cpp}` (新規・純幾何, IO/画像なし) — `SfmTagObs`/`SfmFrame`/
  `SfmMapOptions`/`SfmMapReport` と `build_floor_map_sfm(frames,opts,&map,&report)`。例外を投げず
  `report.connected` で成否、分割時は到達成分のみ map に詰め `unreached_ids` を返す。
- `cpp/tools/charuco_intrinsic_video.cpp` (新規) — ChArUco 動画 → スマホ intrinsics。
  `IntrinsicCalibSession` を無改変で再利用 (多様性ゲート + `cv::calibrateCamera`/`fisheye` +
  `calib_io::write_calibration`)。フレーム源だけ `cv::VideoCapture`。
- `cpp/tools/sfm_floor_map.cpp` (新規) — マーカー動画 + intrinsics → `floor_tag_map.yaml` +
  holdout 検証レポート。`--ids/--tag-size/--anchor/--stride/--max-rms/--holdout-frac/--no-*`。
- `cpp/tools/test_floor_map_sfm.cpp` (新規 ctest) — 合成データで連結復元・床フィット・
  メトリックスケール保存・ノイズ耐性・分割グラフ報告・`solve_floor_extrinsics` 往復。

### 不変条件
- マップのスケールは各タグの `tag_size_m` (PnP) が固定 — SfM スケール不定性は無い。
- 出力は `FloorTagMap` の既存スキーマそのもの (`fitra_floor_tag_map_v1`)。生成元 (実測/SfM) は
  `floor_extrinsic_solver` から不可視 — 案D コアは無改変。
- `build_floor_map_sfm` は純関数 (画像 I/O 無し)。検出フロントエンドはツール側。

### スマホ intrinsics の注意
- iPhone (Blackmagic, 手振れ補正 OFF) のメインレンズは概ね矩形像 → **pinhole** (リグの魚眼 ELP
  とは別物)。intrinsics の解像度は撮影解像度のまま (例 2160×1214) で、PnP は同解像度で行う。
- **ChArUco ボード寸法は実物と一致必須**。本サンプルは盤面が **7×5** (`configs/intrinsic_calib.yaml`
  の `5×7` は転置していた)。`squares_x/squares_y` が転置すると marker ID→3D 対応が崩れ、K が
  異方 (fx≠fy)・RMS 数百 px の退化解になる (検証で確認: 5×7→RMS 203px / 7×5→RMS 0.72px)。

## Milestone

- **M1** `lift/floor_map_sfm` (pose-graph コア) + 合成 ctest。
- **M2** `charuco_intrinsic_video` (`IntrinsicCalibSession` 流用)。
- **M3** `sfm_floor_map` (検出 → build → 書出 → holdout 検証)。
- **M4** 本 doc + track changelog。

## 検証

- ctest `test_floor_map_sfm`: 合成 8 タグ・単一フレーム非全可視・共可視連結のもとで、連結復元
  (回転 < 1e-3deg / 並進 < 1e-4m, アンカーゲージ基準)、床フィット (|z| < 1e-6・plane_rms < 1e-6)、
  メトリックスケール保存、ノイズ (~0.4deg/4mm) で回転 < 2deg、分割グラフで `connected=false` +
  不到達 ID 列挙、`solve_floor_extrinsics` 往復 (再投影 < 1e-2px・姿勢一致)。
- 実サンプル (samples/, iPhone 2160×1214):
  - intrinsic: `charuco_intrinsic_video --squares-x 7 --squares-y 5 --square 0.056 --marker 0.042
    --model pinhole` → **RMS 0.83px**, fx≈fy≈2585, 主点≈画像中心。
  - map: `sfm_floor_map --ids 20-27 --tag-size 0.1145 --anchor 20` → **8/8 タグ連結**, 27 エッジ,
    plane_rms **6.7mm** (床面に同一平面), 約 1.2m×1.0m の妥当な配置 — **実測ゼロ**。
  - holdout 検証: 3+ タグ (well-conditioned) で再投影 median **5.3px** / p90 7.7px。2 タグ frame は
    coplanar PnP の二値曖昧性で外れ値の裾を持つ (床のみマップの既知の限界)。

## 残課題

- **精度**: pose-graph 連結 + 平均緩和のみ。バンドル調整 (全フレームのカメラ姿勢 + タグ姿勢を
  同時最適化) で再投影を詰められる (将来)。手持ちフレームの動きボケ・ローリングシャッターも
  holdout 再投影に乗るため、上の 5px はマップ品質の悲観的上界。
- **平面縮退**: 床のみマップは coplanar。固定リグの height/tilt 可観測性には**面外スタンドタグ**
  併用が必要 (案D 既述)。スタンドタグも本ツールで同時に SfM 配置できる (6DoF をそのまま出力)。
- **WebUI / daemon 統合**: 現状はオフライン CLI ツール。将来 setup フローに「スマホでマップ作成」
  step を足す余地 (intrinsic step と同様)。
- **盤面寸法の config 不一致**: `configs/intrinsic_calib.yaml` の ChArUco が `5×7` 表記だが実物は
  `7×5`。リグの intrinsic も同じ転置で退化していた疑いがある — 別途リグ再校正の要確認。
