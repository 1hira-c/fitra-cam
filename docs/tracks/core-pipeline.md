# Track: core-pipeline

カメラ取り込み → TensorRT 推論 (YOLOX + RTMPose) → Web 配信の **基盤パイプライン**と性能。
C++ 移行 (旧 Phase 0–6) で確立し、現在は安定。pose-3d / vr-output トラックの土台。

## 現状

3 USB カメラ並列、aggregate **170 fps** (旧 Phase 3 baseline 比 5.5×) を達成済み。
keypoint topology は COCO17 / Halpe26 を `--keypoint-format` で切替。

### アーキ仕様の source of truth

アーキ図 (capture thread × N / 単一 TRT context / 単一 CUDA stream / SPSC queue size 1 drop-old /
Crow WS 30Hz)、リポジトリレイアウト、依存表 (FetchContent header-only) は
[`cpp-migration-plan.md`](../cpp-migration-plan.md) が今も live な仕様。CLAUDE.md「Architecture」も参照。

### live な制約 (要点)

- **単一 TRT context・単一 CUDA stream**: Python 版の per-camera セッション分離とは逆。
  context 切替コストをゼロにするのが移行の肝。
- **latest-frame-wins capture**: SPSC queue size 1 で drop-old。リアルタイム鮮度 > 全フレーム処理。
- **FP16 RTMPose drift**: 低スコア keypoint (score<0.5) が Y で 100–200px ずれることがある。
  FP32 なら max kpt L2 ≈ 1.15px。INT8/FP16 を扱うときは Phase 1 と同じ動画で再現テスト必須。
- **YOLOX end2end + TRT 10.3 INT8 不可**: mmdeploy end2end ONNX の NMS/TopK/Where chain が
  calibration `slot::decode` assert に当たる。calibrator infra は RTMPose INT8 計画で salvage 予定。
- **keypoint topology**: `kMaxKeypoints=26` で配列統一、`kp_count` で論理点数を伝搬。
  subject profile schema (v1/v2) はマイグレーションしない (pose-3d トラック参照)。

### 検証

`tools/correctness_check` で Python ORT と bbox IoU>0.99 / kpt L2<1px。
ベンチ・合格基準は [`cpp-migration-plan.md`](../cpp-migration-plan.md) の検証戦略表 + 各 Phase 着地メモ。

## Changelog (新しい順)

### 2026-05-29 — 全 GPU フロントエンド M3 Step B: frame_source 統合 + cvtColor 撤去
device 経路を `decode_to_device` (host map / RGBA→BGR cvtColor なし) に切替え、検出フレームで
`Yolox::infer_device`、calib/retain_bgr 時のみ `decode_keep_device`。**残っていた最後の per-frame
CPU フルパス (full-frame cvtColor) を撤去**。フレーム寸法は decode 戻り値で追跡、device decode 失敗時は
BGR+CPU フォールバック (BGR なし時は pose スキップで空 Mat deref 回避)。**2cam 90fps@VGA: CPU
1.28→0.98 cores (mjpeg 1.83 比 −46%)、cap→pub 12.8→11.7ms** で 1 コアを切った。SIGINT rc=0/0.32s、
ctest 9/9、CHW/keypoint/bbox correctness 維持。残 CPU は後段推論 + capture → M4/M5。
→ [design/core-pipeline-gpu-frontend.md](../design/core-pipeline-gpu-frontend.md)

### 2026-05-29 — 全 GPU フロントエンド M3 Step A: YOLOX 前処理 CUDA カーネル
YOLOX letterbox 前処理 (resize+HWC→CHW, 正規化なし, 114 pad) を GPU カーネル化。`cv::resize` の
half-pixel convention + edge clamp を再現。`Yolox::infer_device(fill)` で engine 入力 device バッファを
カーネル直書き (静的 shape, H2D なし, kernel と enqueue を同一 stream で順序付け)。YOLOX は per-camera
worker の TRT context なので cross-thread なし。loader に `decode_to_device`/`preprocess_yolox_into`/
`yolox_device_capable`。`gpu_preprocess_check` に bbox モード追加 (host `infer` vs device、IoU マッチ +
corner L2)。実機 **bbox corner L2 = 0.0px (8/8 matched)** — letterbox 微差を FP16 が量子化吸収
(device-first で stale false-pass 排除)。ctest 9/9。Step B で frame_source 統合 + full-frame cvtColor 撤去。
→ [design/core-pipeline-gpu-frontend.md](../design/core-pipeline-gpu-frontend.md)

### 2026-05-29 — 全 GPU フロントエンド M2 Step B: device CHW を TRT 入力直結
前処理カーネル出力を host を介さず TRT 入力に渡す配線。`TrtEngine::copy_input_region_from_device`
(D2D, offset 対応) + `RtmPose::PrebakedRequest.chw_dev` で `run_one_prebaked` を device バッチ経路化
(H2D 消滅)。逆アフィンのみ `RtmPose::compute_m_inv` で CPU 算出。.so に `fitra_nvjpeg_preprocess_from_last`
/ `decode_to_device`、loader に `device_capable`/`decode_keep_device`/`preprocess_into`。レース対策に
per-camera `DeviceChwPool` (`shared_ptr<DeviceChwBuf>` deleter がプールを生かし、worker は consumer 保持中の
バッファを触らない = host copy-on-pop の device 版)、取得失敗時は CPU prebake フォールバック。検証:
`gpu_preprocess_check` の keypoint モードで host `infer` vs device `infer_prebaked` を照合、confident
keypoint L2 **avg 0.34px / worst 1.18px** (低スコアは FP16 既知の argmax 不安定で除外)。実機 (単一カメラ
nvjpeg fake-bbox): **det→bake 4.1→1.1ms / cap→pub ~20→~15ms**、30fps 維持、SIGINT 0.52s、ctest 9/9。
**2cam 90fps@VGA A/B** (`FITRA_DISABLE_GPU_PREPROCESS=1` で CPU prebake 強制比較): GPU フロントエンドが
**nvjpeg の ~1.8 コア床を初めて割った** — CPU 1.77→**1.28 cores (−28%)**、cap→pub 16.4→**12.8ms**。
「高 fps では色変換が床」とした nvjpeg doc 結論に対し per-person warp/normalize の GPU 移行が効くと実証。
BGR は YOLOX/calib 用に残置 (full-frame cvtColor 除去は M3/M4)。
→ [design/core-pipeline-gpu-frontend.md](../design/core-pipeline-gpu-frontend.md)

### 2026-05-29 — 全 GPU フロントエンド M2 Step A: RTMPose 前処理 CUDA カーネル
前処理 (crop+resize+normalize+HWC→CHW) を GPU 化する `.cu` カーネルを隔離 .so 内に追加
(`enable_language(CUDA)` を Jetson 分岐で有効化、Orin sm_87)。幾何は CPU が算出する `M_inv` を
カーネルに渡し `cv::warpAffine` と完全一致 (`getAffineTransform` device 再実装不要)。RGBA→正規化 BGR を
CHW=[B,G,R] に出力、per-neighbor ゼロ境界。correctness ツール `tools/gpu_preprocess_check` で録画動画
raw_cam0.mp4 を CPU `preprocess_to_blob` と CHW 比較: **mean abs 0.0028 / L2 0.0046 / worst max 0.058**
(OpenCV 固定小数補間 1/32 量子化の床 ≈0.07 以内、カーネルの方が高精度)。ctest 9/9。Step B で TRT 入力
device 直結 + prebake 配線 + H2D 消滅。
→ [design/core-pipeline-gpu-frontend.md](../design/core-pipeline-gpu-frontend.md)

### 2026-05-29 — 全 GPU フロントエンド M1: EGL→CUDA ブリッジ常設化
スパイクで実証した EGL→CUDA 経路を隔離 .so (`libfitra_nvjpeg.so`) に常設化。`ensure_egl`
(`NvBufSurfaceMapEglImage`→`cuGraphicsEGLRegisterImage`→`GetMappedEglFrame`) で RGBA decode 出力を
**確保時 1 回だけ** CUDA device ptr に register・キャッシュ (`cudaFree(0)` で primary context を
デコードスレッドに bind、解像度変更時のみ teardown)。新 C API `fitra_nvjpeg_decode_cuda` を追加
(`decode_rgba` 本番経路は不変)。loader は `FITRA_NVJPEG_EGL=1` で opt-in し 300 フレームごとに
device↔CPU map の R-mean を回帰ログ。.so に `CUDA::cudart`/`CUDA::cuda_driver` をリンク。実機
(単一カメラ 640×480@30, nvjpeg): **device→host R-mean が CPU map と完全一致 (diff=0)**、device ptr 安定、
30fps 維持、SIGINT 0.42s クリーン終了、既定 mjpeg 経路無影響、ctest 9/9。M1 は足場 (BGR は host map から)、
M2 で device ptr 直結。
→ [design/core-pipeline-gpu-frontend.md](../design/core-pipeline-gpu-frontend.md)

### 2026-05-29 — (設計) 全 GPU フロントエンドを起票 + EGL→CUDA ブリッジ実証
nvjpeg の「高 fps では CPU 色変換が床」知見を受け、decode→前処理→TRT を host 経由なしで回す設計を
起票。make-or-break の **EGL→CUDA interop をスパイクで実証** (NvBufSurfaceMapEglImage→
cuGraphicsEGLRegisterImage→CUeglFrame の pitch-linear device ptr、cudaMemcpy2D で CPU マップと画素
一致)。VIC の CUDA メモリ直出力は非対応と確認し EGL 経路採用。RTMPose 前処理は回転なし(crop+scale)で
CUDA bilinear 吸収可。実装は未着手 (M1〜)。
→ [design/core-pipeline-gpu-frontend.md](../design/core-pipeline-gpu-frontend.md)

### 2026-05-29 — MJPEG HW デコード (`--pixel-format nvjpeg`) M1 実装
Jetson HW NVJPEG ブロックで MJPEG をデコード。libnvjpeg の無バージョン jpeg_* が OpenCV の
libjpeg-turbo と衝突するため、`NvJPEGDecoder` + `NvBufSurfTransform` を**独立 .so
(`libfitra_nvjpeg.so`)** に隔離し本体から `dlopen(RTLD_DEEPBIND|RTLD_LOCAL)`。既定 mjpeg(CPU) 経路は
非リンクで無影響。実測 (単一カメラ 640×480@30): cap→pub mjpeg 22.5 / yuyv 17.1 / **nvjpeg 20.7ms**、
色は CPU decode と meanAbsDiff<1。真価は CPU を entropy decode から解放する点 (multi-cam 向け)。
→ [design/core-pipeline-nvjpeg-decode.md](../design/core-pipeline-nvjpeg-decode.md)

### 2026-05-29 — (設計) MJPEG GPU デコード (HW NVJPEG) を新目標化
E2E レイテンシ調査中に「MJPEG の CPU decode (~6.7ms) を GPU に逃がす」目標が派生。実機調査で
CUDA `nvjpeg.h` は未搭載・**Jetson MMAPI `NvJPEGDecoder` が唯一の HW 経路**と確定（出力は YUV、
MMAPI common-class ソースの取り込みが要）。`--pixel-format nvjpeg` 追加で段階導入する設計を起票。
実装は未着手 (別コミット予定)。
→ [design/core-pipeline-nvjpeg-decode.md](../design/core-pipeline-nvjpeg-decode.md)

### 2026-05-29 — E2E レイテンシ計測基盤 + YUYV 切替 + sleep 除去 + VR イベント駆動
ステージ別レイテンシ TS を `DecodedFrame`/`Skeleton3DSnapshot` に inline 化し、central loop の
3 秒 breakdown を `cap→dec/dec→det/det→bake/bake→pose/pose→pub` 拡張、VR 側に
`e2e_capture_to_send_ms` を追加。`--pixel-format {mjpeg,yuyv}` / `--n-buffers` を CLI/YAML 化
(`Frame.jpeg`→`data`、YUYV は `cv::cvtColor` 経路)。capture/decode/central の 2ms poll sleep を
condition_variable に置換 (単一カメラ)。`--vr-extract-event-driven` で TrackerExtractor を
三角測量フレーム駆動に (opt-in, default off)。実機計測: 単一カメラ 640×480@30 で
**YUYV は MJPEG 比 cap→pub −5.5ms** (cap→dec 6.7→0.95ms、decode 消滅) かつ 30fps 維持。
M4 VR e2e は被写体 (`ik_locked`) 要のため残課題。
→ [design/core-pipeline-e2e-latency.md](../design/core-pipeline-e2e-latency.md)

### 2026-05-20 — COCO17 → Halpe26 keypoint 移行
`--keypoint-format {coco17,halpe26}` で CLI 切替。`SkeletonDef` active format singleton
(`cpp/src/lift/keypoint_format.hpp`)。subject profile v1/v2 を厳格分離。
→ [archive/phase9-halpe26-migration.md](../archive/phase9-halpe26-migration.md)

### 2026-05-15 — Phase 0–6: C++ 移行 + 性能 (aggregate 170 fps 達成)
Python 退避 + C++ skeleton (P0) → TRT engine ラッパ + correctness (P1) → 1cam e2e (P2) →
3cam + Crow Web (P3) → FP16/INT8/pinned-memory (P4) → per-cam YOLOX (P5) → 90fps push 170fps (P6)。
詳細・着地メモ・ベンチ表は [`cpp-migration-plan.md`](../cpp-migration-plan.md) に保存。

> Phase 10 (3 カメラ + C++ ライブキャリブ) はスキップ。設計メモのみ
> [archive/phase10-cpp-live-calib.md](../archive/phase10-cpp-live-calib.md) に残置。
