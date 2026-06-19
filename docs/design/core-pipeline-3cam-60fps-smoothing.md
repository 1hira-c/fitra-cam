# core-pipeline: 3カメラ 60fps 安定化 (GPU 直列化解消 + 検出スケジュール + 位相ずらし)

(着手日 2026-06-19 / 関連: core-pipeline-per-camera-capture-downscale.md の残課題, メモ project-3cam-rig-layout)

## 背景 / 動機

3カメラリグ (cam0/cam2=ELP 640, cam1=USB3.0 1280→640) を 60fps で回すと、pose 出力が
~46-50fps で頭打ち・pending 増大し、3カメラ sync が外れて tri_fps がガタついた。
**CPU はアイドル (load ~1.0/6コア)** なので GPU 側の問題と判断。

当初 (per-camera-capture-downscale の残課題に記載) は「per-camera RTMPose 前処理の
`cudaStreamSynchronize(nullptr)` による GPU null ストリーム直列化が天井」と診断していたが、
**実測の e2e breakdown でこの診断は主因ではなかったと判明** — 真の律速は別だった。本ドキュメントは
正しい原因究明とその修正を記録する。

## 実測による原因究明 (e2e breakdown)

中央ループの per-stage breakdown を読むと:

| stage | 当初 (無人) | 解析 |
|---|---|---|
| cap→dec | 11.4ms | キャプチャ待ち + decode |
| **dec→det** | **18.1ms** | **YOLOX 検出。これが支配的** |
| det→bake | 0.0ms | RTMPose 前処理 (GPU) — 実質ゼロ |
| bake→pose | 1.1ms | 中央 RTMPose バッチ推論 — **律速ではない** |
| cap→pub | 30.6ms | 合計 ≈ 33fps の壁 |

判明した事実:
1. **中央 RTMPose は律速でない** (`bake->pose=1.1ms`、3人バッチで ~1ms)。
   → null ストリーム直列化は主因ではなかった。
2. **真の律速は worker 側の YOLOX (`dec->det=18ms`)**。平均 18ms は「ほぼ毎フレーム
   YOLOX が走っている」ことを意味する (det_frequency=10 なら平均 ~1.8ms のはず)。
3. 原因は `frame_source.cpp` の検出条件:
   ```cpp
   bool do_detect = (frame_idx_ % det_frequency == 0) || cached_bboxes_.empty();
   ```
   **検出ゼロ (無人 or 未検出) だと `cached_bboxes_` が空のまま → 毎フレーム YOLOX**。
   3カメラが毎フレーム YOLOX_s 640 (~18ms) を共有 GPU で奪い合い ~48fps に張り付く。
   「アイドル時こそ最重量 GPU op を全力で回す」という逆転した挙動だった。

## 検討した案と採用

### 修正1: per-handle 非ブロッキング CUDA ストリーム (採用; 直列化は副次的に解消)
`fitra_nvjpeg_iso.cpp` の `Handle` に専用 `cudaStream_t` (`cudaStreamNonBlocking`) を持たせ、
RTMPose 前処理カーネルの launch + sync を **NULL ストリーム → per-handle ストリーム**へ。
NULL (legacy default) ストリームは全 non-blocking ストリーム (TRT エンジンは全て
non-blocking) と暗黙同期するため、各カメラの前処理が全 GPU バリアになっていた。worker は
publish 前に自分のストリームを sync するので正当性不変。
**効果**: `det->bake` が 0.00ms に (前処理が並列化されゼロコスト化)。ただし当初の診断と異なり
これ単独では 60fps に届かなかった (主因は YOLOX だった)。**残す価値はある** (前処理が他カメラ・
中央推論をブロックしなくなる)。

### 修正2: 検出スケジュール修正 (採用; 本命)
`|| cached_bboxes_.empty()` を削除し、`frame_idx_ % det_frequency == phase` のみに。
空検出でも毎フレーム再検出せず、次の検出周期まで待つ (det_frequency=10 @60fps ≈ 167ms で
人物入場を捕捉、十分応答的)。frame 0 は検出するので起動時取得は即時。
**効果 (実測, 無人)**: `dec->det` 18ms→2.6ms、`cap->pub` 30.6→7.9ms、
cam0/cam2 recent_pose ~48→**59-61 (60fps 張り付き)**。

### 修正3: 検出位相ずらし (採用; ジッタ対策)
修正2 後も recent_pose が 53-59 で振れた。GPU は飽和していない (YOLOX は 40% 程度) のに不安定 =
**3カメラの YOLOX 検出フレームが揃うとバーストして詰まる**。`FrameSource::Options::det_phase` を
追加し、`camera_builder` でカメラ i に `phase = i * det_frequency / N` を割当
(3カメラ・freq 10 → frame 0/3/6、~50ms 間隔)。検出バーストが時間軸に分散。
**効果 (実測, 無人)**: cam0/cam2 recent_pose 59-61 で**安定** (振れが解消)。

## 残課題: cam1 のキャプチャレート

cam0/cam2 は安定 60fps。残る不安定は **cam1 単独で、`recv` (キャプチャ自体) が 50-60 で振れる**。
パイプラインではなくカメラ/UVC 配信特性:
- USB は SuperSpeed (5000M) で正常リンク (`lsusb -t` 確認) → **帯域起因ではない**。
- MJPEG 1280×960: 平均 ~54fps だが圧縮量依存でジッタ大 (49.8-59.96)。
- YUYV 1280×960: 安定 ~53fps (フレームサイズ一定)。広告は 60fps 対応だが 53 頭打ち
  (センサ読み出し or この Jetson ポートの isoc スケジューリング起因と推定。memory
  project-usb-camera-bus-limit と同系)。
- **スループットはほぼ同じ。MJPEG はジッタを買うだけ** → 三角測量には **YUYV (安定) が有利**。

→ cam1 は YUYV 運用を推奨 (`run_3d_floor.yaml`)。本当の指標は tri_fps / 3D の滑らかさで、
三角測量は cam1 が sync window を外しても cam0+cam2 の2視点でフォールバックしうる。
cam1 の 53→60 capture チューニング (isoc 枠 / モード探索) は別タスク (ハード軸)。

## 検証

- ビルド: `cmake --build cpp/build -j`。
- 実機 (無人, 3カメラ 60fps, cam1=nvjpeg): cam0/cam2 recent_pose 59-61 安定、
  pending 横ばい。cam1 は recv 50-60 (カメラ起因)。
- 人物ありは det が det_frequency に従うので同等のスループット (検出が埋まっても10フレームに1回)。
- 数値同一性: 修正1 はカーネルの実行ストリームのみ変更 (math 不変) → `fitra_gpu_preprocess_check`
  の kpt L2 に無影響。

## 残課題 (続き)

- cam1 capture 60fps 安定化 (上記)。
- 中央ループ multi-camera の 2ms poll sleep をイベント駆動 (N ソース shared wakeup) 化 —
  poll ジッタ除去の follow-up (core-pipeline-e2e-latency の単一カメラ CV 化の multi 版)。
