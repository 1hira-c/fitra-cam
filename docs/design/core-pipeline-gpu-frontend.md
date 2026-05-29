# core-pipeline: 全 GPU フロントエンド (decode→前処理→TRT を host 経由なしで)

(着手日 2026-05-29 / 派生元: [`core-pipeline-nvjpeg-decode.md`](core-pipeline-nvjpeg-decode.md) の
「高 fps では CPU 色変換が床」知見、[`core-pipeline-e2e-latency.md`](core-pipeline-e2e-latency.md) の
ステージ計測 / migration-plan の Phase 6「GPU 前処理」を具体化)

## 背景 / 動機

E2E レイテンシ + nvjpeg の実測で、**CPU に残るのは前処理だけ**と判明した:
- decode (HW NVJPEG 化済み) の後、**RGBA→BGR フルフレーム変換が CPU**（VIC は 24-bit BGR 非対応）。
- RTMPose 前処理 = **warpAffine(crop+scale) + normalize + HWC→CHW が CPU**（Phase 6b で per-cam
  ワーカーに分散）。YOLOX も letterbox+HWC→CHW が CPU。
- 各推論の **入力 H2D コピー**。

実測の含意: nvjpeg の CPU オフロードは中 fps 帯まで (-24% @30fps×2) だが、90fps×2 では full-frame
CPU 変換が床になり相殺 (mjpeg 1.81 / nvjpeg 1.79 cores)。**残る CPU パスを全部 GPU に寄せれば
高 fps でも CPU が空き、H2D も消え、レイテンシも下がる。** ホストへ戻すのは最終 keypoint だけ。

完了基準: RTMPose(+YOLOX) の入力生成が decode 出力から TRT 入力 device バッファまで **host を一度も
経由しない**。correctness は CPU 参照と keypoint L2 が許容内。高 fps で CPU が capture+後段のみに
落ち、`cap→pose` レイテンシが H2D 分縮む。

## 検討した案

### A. CUDA nvJPEG (`nvjpeg.h`) で JPEG→CUDA 直行 → 没
最短経路だが **この JetPack に CUDA `nvjpeg.h` ヘッダが無い** (`libnvjpeg.so` は MMAPI 版のみ、
`find / -name nvjpeg.h` 空)。実装不能。

### B. VIC 出力を CUDA メモリ (`NVBUF_MEM_CUDA_DEVICE/UNIFIED`) に直接 → 没
`surfaceList[].dataPtr` がそのまま CUDA device ptr になれば EGL 不要。だが実機で
NvBufSurfTransform が **"Surface type not supported for transformation"** で reject
(CUDA_DEVICE/UNIFIED とも)。SURFACE_ARRAY の dataPtr は CUDA から読めない (unregistered)。没。

### C. EGL register で NvBufSurface→CUDA (採用・実証済み)
`NvBufSurfaceMapEglImage` → `cuGraphicsEGLRegisterImage` → `cuGraphicsResourceGetMappedEglFrame`
→ `CUeglFrame.frame.pPitch[0]` が **pitch-linear CUDA device ptr**。DeepStream/MMAPI サンプル 04 の
定番経路。**スパイクで実証** (下記)。採用。

### D. 現状維持 (CPU 前処理) → 没
Phase 6b で 170fps を出した構成だが、高 fps で CPU 律速 + H2D 残存。動機の通り高 fps を解放できない。

## 実証済み事実 (スパイク, 2026-05-29, /tmp)

- **EGL→CUDA ブリッジ成立**: 640×480 MJPEG を decodeToFd→NvBufSurfTransform(RGBA, SURFACE_ARRAY)
  →EglImage→`cuGraphicsEGLRegisterImage`→`GetMappedEglFrame` で `frameType=CU_EGL_FRAME_TYPE_PITCH`,
  planes=1, pitch=2560(=640×4), `pPitch[0]`=CUDA device ptr を取得。`cudaMemcpy2D` D2H 成功、
  **CPU マップと画素一致 (R-mean 128.4 = 128.4)**。CUDA カーネルが decode 出力を直読みできることを確認。
- **RTMPose 前処理は回転なし**: `warp_matrix` は "rotation angle = 0"、中心 crop + アスペクト調整
  スケールのみ。→ **単純 CUDA bilinear crop+resize で吸収可** (一般 affine 不要)。VIC の crop+scale でも可。
- `cuGraphicsEGLRegisterImage` は重い → 再利用 dst バッファに対し**確保時 1 回だけ register**しキャッシュ
  (サンプル 04 の dma_egl_map と同パターン)。

## 採用設計

### データフロー
```
MJPEG ─VIC/NVJPEG decodeToFd─▶ NvBufSurface(YUV422,NVMM)
        ─NvBufSurfTransform─▶ NvBufSurface(RGBA, pitch, SURFACE_ARRAY)   ← 既存 .so 経路を流用
        ─EglImage+cuGraphicsEGLRegisterImage(確保時1回)─▶ CUeglFrame(CUDA device ptr)
   [ここから host 経由なし・全 CUDA stream 上]
   YOLOX:   CUDA kernel(letterbox resize + normalize + HWC→CHW) ─▶ YOLOX TRT 入力(device)
   RTMPose: CUDA kernel(crop[bbox]+resize→input_w×input_h + 正規化 + HWC→CHW) ─▶ RTMPose TRT 入力(device)
                                                       ─▶ SimCC ─▶ argmax+inverse-affine(GPU/軽量) ─▶ keypoint(host, 小)
```

### 所有権・構造
- **隔離 .so (`libfitra_nvjpeg.so`) を拡張**: EGL register + CUDA 前処理カーネルもこの .so 内に置く
  (libjpeg-8b 隔離の dlopen 境界をそのまま使う)。C API を `decode → device CHW` まで拡張するか、
  「decode して登録済み CUeglFrame の device ptr を返す」+ カーネルは本体側 (要 nvcc) かを M1 で決める。
- **TRT 入力を device バッファ直結**: 現状 `RtmPose`/`Yolox` は host blob → `copy_input_from_host`
  (H2D)。カーネル出力 device ptr を TRT 入力 binding に直接させ H2D を消す
  (`trt_engine` に「入力は既に device」モードを足す)。
- **VIC は GPU SM と別ブロック**: decode/transform/crop-resize を VIC に寄せれば TRT 推論の SM を
  奪わない。normalize+HWC→CHW の CUDA kernel のみ SM 使用 (軽量)。

### 不変条件
- latest-frame-wins (SPSC size 1, drop-old) は維持。
- correctness: keypoint が CPU 経路と L2 許容内 (full-range YCbCr / 正規化係数を CPU と一致させる)。
- EGL image / CUgraphicsResource / CUDA context は**生成スレッドに束縛**。per-cam ワーカーで
  decode+register+kernel を回すなら、そのスレッドで CUDA context を current にする。

## Milestone
- **M1 ✅ (2026-05-29)**: EGL→CUDA interop を .so に常設化 (register キャッシュ)。decode→device CUeglFrame を取得し、
  まず **既存 CPU 経路と並行**して device→host で取り出し画素一致を回帰確認 (足場)。
  - 実装: `fitra_nvjpeg_iso.cpp` に `ensure_egl`(=`NvBufSurfaceMapEglImage`→`cuGraphicsEGLRegisterImage`→
    `GetMappedEglFrame`、`cudaFree(0)` で primary context をデコードスレッドに bind、確保時 1 回 register・
    解像度変更時のみ `release_egl`) + 新 C API `fitra_nvjpeg_decode_cuda` (device ptr + host map + check 用
    R-mean を返す)。`decode_rgba` 本番経路は不変。loader 側は `FITRA_NVJPEG_EGL=1` で opt-in、300 フレーム
    ごとに device↔CPU の R-mean を回帰ログ。.so は `CUDA::cudart`/`CUDA::cuda_driver` をリンク。
  - 実機検証 (単一カメラ 640×480@30, nvjpeg): device ptr 安定 (register キャッシュ確認)、
    **device→host R-mean が CPU map と完全一致 (diff=0)** を毎チェックで確認。30fps 維持、SIGINT 0.42s で
    rc=0 のクリーン終了 (EGL teardown ハングなし)、既定 mjpeg(CPU) 経路は無影響、ctest 9/9 green。
  - M1 時点では BGR 出力は依然 host map から生成 (足場)。M2 で device ptr を直接消費し host を落とす。
- **M2**: **RTMPose 前処理 CUDA カーネル** (crop+resize+normalize+HWC→CHW)。出力 device バッファを
  TRT 入力直結 (`trt_engine` の device-input モード)。keypoint L2 を CPU 参照と照合。H2D 消滅を確認。
  - **決定 (2026-05-29)**: カーネルは **隔離 .so 内に `.cu` で実装** (nvcc 有効化)。EGL image・CUDA context が
    既に .so 内にあり親和性が自然、libjpeg 隔離の dlopen 境界も維持。幾何は **CPU が算出する `M_inv`
    (逆アフィン 2×3) をそのままカーネルに渡す** — `cv::warpAffine` は dst 画素 (ox,oy) を src の
    `M_inv·(ox,oy,1)` からサンプルするので、同じ `M_inv` を渡せば OpenCV と幾何が完全一致。`getAffineTransform`
    の device 再実装が不要。回転なし(crop+scale)だが一般 2×3 で書けるため将来の回転にもそのまま対応。
  - **Step A ✅ (2026-05-29)**: `fitra_nvjpeg_kernels.cu` に `preprocess_rtmpose_kernel`
    (1 出力画素 1 スレッド、float bilinear、per-neighbor ゼロ境界 = `cv::warpAffine` の BORDER_CONSTANT 相当、
    RGBA→正規化 BGR を CHW=[B,G,R] に出力)。launch C API `fitra_nvjpeg_preprocess_launch` (stream 受け、
    同期なし) + テスト用 host API `fitra_nvjpeg_preprocess_rgba_host`。CMake で Jetson 分岐に
    `enable_language(CUDA)` + `CUDA_ARCHITECTURES 87`、`-fvisibility` は `$<COMPILE_LANGUAGE:CUDA>` で
    `-Xcompiler` 経由。correctness ツール `tools/gpu_preprocess_check` (録画動画 raw_cam0.mp4 を 8 分割サンプル
    × 5 bbox、CPU `preprocess_to_blob` と CHW を比較)。**実測: mean abs 0.0028 / mean L2 0.0046 / worst max
    0.058** — worst は OpenCV の固定小数補間 (INTER_BITS=5, 重み 1/32 量子化) の床
    (255·(1/64)/std ≈ 0.07) 以内で、本 float カーネルの方がむしろ高精度。channel order / 幾何バグなら
    diff は 1〜4 オーダーになるので 0.1 を閾値に。ctest 9/9。
  - **Step B (次)**: `decode_cuda` の device RGBA ptr → カーネル → TRT 入力 device 直結
    (`trt_engine` に device-input モード追加、batch 各 item を engine 入力バッファ offset へ書く or
    `setTensorAddress`)。per-cam prebake 経路を device CHW に切替。keypoint L2 を実機/録画で照合し H2D 消滅を確認。
- **M3**: **YOLOX 前処理 CUDA カーネル** (letterbox+normalize+HWC→CHW) 同様に device 直結。
- **M4**: アーキ移行 — Phase 6b の per-cam CPU 前処理を撤去し GPU 経路へ。EGL/CUDA context の
  スレッド親和性、register ライフサイクル、multi-cam の resource キャッシュを整理。
- **M5 (任意)**: SimCC argmax + inverse-affine を GPU 化し host 転送を keypoint のみに最小化。

## 検証
- correctness: 同一フレームで GPU 前処理経路 vs 現行 CPU 経路の RTMPose keypoint L2 (許容内)。
  色は full-range YCbCr→RGB を CPU `cv::imdecode` と meanAbsDiff<1 で既に確認済み (nvjpeg doc)。
- レイテンシ: `core-pipeline-e2e-latency.md` の計測基盤で `det→bake`/`bake→pose`/`cap→pub` を
  CPU 経路と比較。H2D 消滅分の短縮を確認。
- CPU: 90fps×2 で mjpeg-CPU / nvjpeg-CPU / GPU-frontend を比較。GPU 経路で CPU が capture+後段のみに
  落ちることを確認 (nvjpeg doc の表に GPU-frontend 列を追加)。
- ctest 維持、`--pixel-format mjpeg` (cv::imdecode) 無影響、SIGINT クリーン終了。

## 残課題 / リスク
- **EGL/CUDA context のスレッド管理**が最大の落とし穴 (per-cam ワーカー × EGL image × CUDA stream)。
- **GPU SM 競合**: normalize/CHW カーネルが TRT 推論と SM を取り合う (VIC 寄せで緩和、kernel は軽量)。
- **nvcc / CUDA ビルド**: 現状 CMake は `.cu` を持たない (CXX のみ)。カーネルを .cu にするなら
  CUDA language を有効化、または PTX/driver API で回避するか M1 で決める。
- **register ライフサイクル**: dst バッファ再確保時 (解像度変更) に unregister/re-register。
- **Phase 6b アーキとの整合**: 前処理を GPU に戻すと per-cam ワーカーの役割が decode+register に縮む。
  170fps 達成の前提が変わるので multi-cam スループットを再計測 (M4)。
- フォールバック: GPU 経路が使えない環境では既存 CPU 経路 (mjpeg/yuyv) を残す。
