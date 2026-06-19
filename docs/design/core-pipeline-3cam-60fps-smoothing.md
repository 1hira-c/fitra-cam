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

## cam1 capture 頭打ちの追及 (2026-06-19 深掘り)

cam1 (USB3.0 1280×960) が 53fps 頭打ち + 3カメラで 50-60 ジッタ。原因を順に切り分けて確定:

### (i) カメラは無実
純 v4l2-ctl (パイプライン非経由) で計測:
- YUYV 1280×960 @60: **60.2fps steady** (要求 @120 でも **119fps**)。
- MJPEG 1280×960 @60: 54-58fps (圧縮量依存で**ソース時点から**ジッタ)。
→ カメラは YUYV で 60(120) を余裕で出す。MJPEG のジッタは圧縮固有。

### (ii) 毎フレーム 2.46MB mmap (コード) — 修正済 (別コミット)
`V4l2Capture::worker_loop` が毎フレーム新規 `Frame` の空 vector に 2.46MB(YUYV) を `assign`。
glibc malloc は >128KB を mmap/munmap で扱うため毎フレーム mmap+ページフォルト+munmap → ~53 に拘束。
capture スレッド専有 `spare_data_` を `latest_->data` とスワップ (capacity 再利用)。消費側も
`FrameSource::decode_loop` の `Frame raw` をループ外へ。→ 単一カメラで 52.78→**60 steady**。

### (iii) 電源モード — 環境是正
nvpmodel が 25W + schedutil で cpu4/5 が 0.73GHz 省エネコアに落ち、スレッドが載るとジッタ。
**真の最大は MAXN_SUPER (mode 2)**。このデバイスは mode 番号が振り直されており
`0`=15W(最弱) / `1`=25W / `2`=MAXN_SUPER。**CLAUDE.md の `nvpmodel -m 0` は誤り**
(Orin Nano Super では 15W 固定になる)。`-m 2` + `jetson_clocks` で全コア 1.728GHz 固定。

### (iv) 残る 3カメラ時のドロップ = USB ホストコントローラ競合 (ハード)
(ii)(iii) 後も 3カメラ時に cam1 のみ `recv` が 55-60 で揺れる。`buf.sequence` ギャップ計装
(`FITRA_CAPTURE_DEBUG`) で確定:
- cam0/cam2 (640): driver_dropped = **0**。cam1 (1280): 蓄積する (帯域に比例)。
  YUYV ~4.5% / MJPEG ~2.4% / 単一カメラ時 0%。
- **CPU でもバッファでもない** (n_buffers 16 でも不変。我々は常に1枚しか保持しないので
  ドライバ枠は尽きない → ギャップ = フレームが USB から来ていない)。
- トポロジ: 単一 xHCI `3610000.usb` が3カメラを収容 — cam1 は Bus002(USB3) の Realtek
  USB3.0 ハブ、ELP×2 は Bus001(USB2) の USB2.0 ハブ。**同一コントローラを共有**し cam1 の
  高帯域 1280 転送が競合。memory project-usb-camera-bus-limit の xHCI 枠制約と同根。

→ **実用影響は軽微**: cam1 がドロップしたフレームは cam0+cam2 の2視点で三角測量フォールバック
できるので tri_fps は落ちない (当初の「pending 爆発で使用不可」からは別世界)。
**推奨**: cam1=YUYV (decode 軽量 stage_ms 1.8ms・HW NVJPEG 非競合・圧縮ジッタなし、ドロップは
やや多いが 3D 影響は誤差)。真の cam1 単独 60 が要るなら**別 USB コントローラ `3550000.usb` 側の
ポートへ cam1 を物理隔離** (要・再キャリブレーション) — ハード軸の follow-up。

## 検証

- ビルド: `cmake --build cpp/build -j`。
- 実機 (無人, 3カメラ 60fps, MAXN_SUPER): cam0/cam2 recent_pose 59-61 安定・pending 横ばい・
  driver_dropped 0。cam1 は recv 55-60・pending 安定 (USB 競合のドロップ 2-4%、上記 (iv))。
  cap→pub 30→**5.6ms**。
- 人物ありは det が det_frequency に従うので同等のスループット (検出が埋まっても10フレームに1回)。
- 数値同一性: 修正1 はカーネルの実行ストリームのみ変更 (math 不変) → `fitra_gpu_preprocess_check`
  の kpt L2 に無影響。

## 残課題 (続き)

- cam1 の USB 競合ドロップ完全解消はハード軸 (別コントローラ `3550000.usb` へ隔離、または
  cam1 帯域削減)。現状はソフト最適 (mmap 撤廃済) + 2視点フォールバックで実用十分。
- 中央ループ multi-camera の 2ms poll sleep をイベント駆動 (N ソース shared wakeup) 化 —
  poll ジッタ除去の follow-up (core-pipeline-e2e-latency の単一カメラ CV 化の multi 版)。
