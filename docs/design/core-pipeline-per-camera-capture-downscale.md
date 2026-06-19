# core-pipeline: per-camera capture 解像度 + ソフト downscale

(着手日 2026-06-18 / 関連メモ: 新USB3.0カメラは640で中央crop)

## 背景 / 動機

3 台目として増設した USB3.0 個体 (`Global Shutter Camera` serial `2601240001`) は、
既存 2 機 (USB2.0 / bus 2.3・2.4) と **低解像度時の挙動が逆**だった。実機キャプチャで確認:

- 既存機: どの解像度でも同じ広角 = 低解像度は full-frame **downscale**。
- 新カメラ: 1920×1200 / 1600×1200 / 1280×960 は広角のまま広い画角だが、
  **640×480 だけセンサ中央を center-crop** に切替わり、極端に狭い画角(ズーム)になる。

パイプラインは全カメラ共通の 640×480 ランタイムを前提に組まれており
(`MainOptions::width/height` は単一)、新カメラを 640×480 で開くと一台だけ狭画角になり
三角測量の共視ボリュームが破綻する。さらに `tools/scale_intrinsics` / `calib_io::scale_intrinsics`
の「640 は full sensor の線形 downscale」前提も、この個体では崩れる
(center-crop は fx/fy 不変・cx/cy オフセットで、線形スケールが誤りになる)。

**完了条件**: 任意のカメラを「downscale が効く高解像度で V4L2 キャプチャ → decode 直後に
共通ランタイム解像度へソフト縮小」できるようにし、新カメラを 1280×960 撮影 → 640×480 縮小で
既存機と画角・座標系を揃える。intrinsic は 1280×960 で校正し `scale_intrinsics` で 640 へ線形縮小。

## 検討した案

### 案A: 全カメラを高解像度化 (没)
`width/height` を全体で 1280×960 に上げる。配管は単純だが、既存 2 機は USB2.0 の xHCI isoc
帯域上限に張り付いており (memory: USBカメラのバス上限)、解像度を上げると 2 台運用が壊れる。
新カメラだけ高解像度にしたいので不採用。

### 案B: per-camera キャプチャ解像度 + ソフト downscale (採用)
カメラ単位で V4L2 キャプチャ解像度を上書きでき、decode 段で共通ランタイム解像度へ縮小。
高解像度化を必要なカメラだけに局所化でき、既存機・ダウンストリームは無改造。
**環境固有のハードコードを避け、`cam{N}_capture_width/height` の汎用オプションにする**
(ユーザー要望: この環境では新カメラのみだが、カメラのオプションで切替可能に)。

### 案C: ソフト縮小せず per-camera 解像度のまま三角測量 (没)
新カメラを 1280×960 のまま流し、その K も 1280 のまま triangulate (K が合えば座標スケールは
不問)。しかしダウンストリーム (multi_pipeline の snapshot `w/h`, drawer, web overlay) が
全カメラ単一解像度を前提にしており、per-camera 座標スケールを持ち回る大改修になる。
ユーザー方針どおり共通 640 に揃える案B を採る。

### intrinsic: 640 直接校正 vs 1280 校正→線形縮小 (後者を採用)
当初「640 直接が最小誤差」と考えたが誤り。高解像度で校正する方が ChArUco コーナーの
サブピクセル検出が精細で再投影誤差(px)が小さく、真の downscale なら線形スケールで
その精度をそのまま 640 へ継承できる。`scale_intrinsics` のアスペクト比ガード (4:3→4:3) も通る。
→ **1280×960 校正 → `scale_intrinsics(.,640,480)`** を採用。

### GPU front-end との両立 (縮小カメラは BGR 経路に降ろす)
nvjpeg HW デコーダ (`NvJpegHwDecoder`) はスケール機能を露出せず、device RGBA に native 解像度で
デコードし preprocess kernel がそこを直読みする。device 経路のまま縮小するには .so 改修
(VIC スケール) が要る。当面は **縮小カメラだけ all-GPU front-end を無効化** し、
HW デコードした BGR を `cv::resize` で縮小 → CPU YOLOX + CPU RTMPose prebake に乗せる。
HW JPEG デコード(重い部分)は維持。GPU preprocess を失う CPU コストは 1 台ぶんで許容。
.so 側 VIC スケール decode は将来最適化として残課題。

## 採用設計

### 解像度の二層化 (`camera::V4l2Options`)
- `width`/`height` = **出力(実効)解像度**。ダウンストリームが読む値。既存意味を変えない。
- `cap_width`/`cap_height` = **実 V4L2 キャプチャ解像度**。`0` = 上書き無し(=出力と同じ=縮小無し)。
- ヘルパ `capture_w()/capture_h()` = `cap_*>0 ? cap_* : width/height`。

`V4l2Capture::start` は `capture_w()/capture_h()` で `VIDIOC_S_FMT`。ドライバが解像度を
丸めた場合、縮小有り (`cap_width>0`) なら `cap_*` を、縮小無しなら従来どおり `width/height` を
実値で更新。YUYV のバッファ解釈 (`v4l2_capture` / `frame_source`) も capture 次元を使う。

### 縮小ステップ (`FrameSource::decode_loop`)
`downscaling = (capture_w()!=width || capture_h()!=height)`。
- decode_loop 冒頭で `downscaling` なら `device_pose_=false; yolox_device_=false`
  (BGR scratch + CPU prebake を強制; use_hw でも `decode()` で BGR を得る)。
- BGR `scratch` 確定後・YOLOX 前に `cv::resize(scratch, scratch, {width,height}, INTER_AREA)`、
  `fw=width; fh=height`。以降 bbox / M_inv / keypoint / drawer / triangulation は全て 640 空間。

`options().width/height` = 640 のままなので multi_pipeline snapshot 他は無改造。

### config (`MainOptions` / YAML / CLI)
- `MainOptions`: `std::array<int,3> cam_cap_width{}, cam_cap_height{}` (0=無し)。
- YAML `cameras`: `cam{0,1,2}_capture_width` / `cam{0,1,2}_capture_height` を allowed に追加。
- CLI: `--cam{N}-capture WxH` (例 `--cam2-capture 1280x960`)。
- `camera_builder`: index 付きループにし `o.cap_width/height = opts.cam_cap_*[i]` を設定。

### 校正との関係
- **intrinsic 校正**: 縮小を設定せず native 1280×960 で実行 → K を 1280 で取得 → `scale_intrinsics`
  で 640 へ。
- **extrinsic / live**: 縮小を有効化し共通 640 で動かす (intrinsics も 640 にスケール済み)。
  両者の解像度が一致するので extrinsic 校正・推論で座標系が揃う。

## Milestone

- M1: `V4l2Options` 二層化 + `V4l2Capture` capture 次元対応 + `FrameSource` downscale。
- M2: config (MainOptions/YAML/CLI) + `camera_builder` 配管。
- M3: run config (`configs/medium_3d*.yaml` 等) に cam2 と capture 上書きの記入例、
  新カメラ intrinsic を 1280 校正→640 スケールで投入。
- M4 (2026-06-19): **HW NVJPEG 破損フレーム guard** (`looks_like_jpeg()`)。多カメラの
  USB 帯域飽和で truncated/garbage MJPEG が出ると HW NVJPEG が **segfault** する (CPU
  `cv::imdecode` は耐える)。SOI(FFD8)/EOI(FFD9)+長さを検査し、HW decoder に渡す前に drop。
  medium_3d 等あらゆる nvjpeg 運用のクラッシュ耐性も上がる。
- M5 (2026-06-19): **per-camera `pixel_format`** (`cam{N}_pixel_format`)。混在 decode 経路
  (例: 混雑する HW ブロックを避けて特定カメラだけ YUYV / nvjpeg) を可能に。
- M6 (2026-06-19): **VIC スケール decode**。`fitra_nvjpeg_decode_to_device` に
  `target_w/h` を追加し、VIC の YUV→RGBA 変換で 1280→640 を同時実行。縮小カメラも
  all-GPU front-end に残せる (CPU resize/prebake 不要)。M1 の cv::resize は YUYV/CPU
  /HW-BGR フォールバック経路の安全網として残置。

## 検証

- ビルド: `cmake --build cpp/build -j`。
- 実機: `--cam2 <新カメラ> --cam2-capture 1280x960 --width 640 --height 480` で起動し、
  新カメラのプレビュー画角が既存機と同等(広角)で、640 で center-crop の狭画角にならないこと。
- 縮小無し既存 2 機の挙動・fps が回帰しないこと (CPU 経路に落ちないこと)。
- intrinsic: 1280 校正 → `scale_intrinsics` 640 で再投影 RMS が妥当域。

## 残課題

- ~~**3カメラ 60fps の GPU 直列化天井**~~ **(2026-06-19 解決 — ただし当初診断は誤りだった)**:
  当初「null ストリーム直列化が天井」と診断したが、実測 e2e breakdown で**否定された** —
  中央 RTMPose は `bake->pose=1.1ms` で律速でなく、真の律速は worker 側 YOLOX の毎フレーム
  検出 (無人時に `cached_bboxes_.empty()` が毎フレーム YOLOX を強制) だった。検出スケジュール
  修正 + per-handle CUDA ストリーム + 検出位相ずらしで cam0/cam2 は 60fps 安定。正しい原因
  究明・修正は `docs/design/core-pipeline-3cam-60fps-smoothing.md`。
  (cam1 の recv 50-60 振れはカメラ/UVC 配信特性として別タスクに分離。)
- ハード同期なし多カメラの時間 sync: 3-way 一致は 2-way より低歩留まりで、tight 窓だと
  tri が間引かれ、広い窓 (例 30ms) だとモーションのカメラ間ズレが出る本質的トレードオフ。
- 縮小カメラの YUYV 経路 (cvtColor 1280 + INTER_AREA resize) の CPU コスト実測。
