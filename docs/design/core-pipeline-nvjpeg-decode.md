# core-pipeline: MJPEG GPU デコード (Jetson HW NVJPEG)

(着手日 2026-05-29 / 派生元: [`core-pipeline-e2e-latency.md`](core-pipeline-e2e-latency.md) の
E2E レイテンシ調査中に「MJPEG decode を GPU に逃がす」目標が立った /
migration-plan の Phase 6 残課題「NVJPEG GPU decode」を具体化するもの)

## 背景 / 動機

E2E レイテンシ計測 ([`core-pipeline-e2e-latency.md`](core-pipeline-e2e-latency.md)) で、MJPEG 経路の
`cap→dec` が **~6.7ms**（CPU `cv::imdecode` / libjpeg-turbo）と判明。YUYV 切替でこれを ~0.95ms に
できたが、YUYV は USB 帯域を食うため**複数台・高解像度では破綻する**。

「帯域は MJPEG のまま低く保ちつつ、decode 遅延と CPU を削る」=**MJPEG を GPU (HW NVJPEG) で
デコードする**第3の道が、調査中に現実的目標として浮上した。これは migration-plan アーキ図の
"NVJPEG decode (GPU)" を初めて実装に落とすもの。

完了基準: MJPEG 入力で `cap→dec` が CPU 経路比で有意に短縮し（目標 <2ms 程度）、CPU 占有が下がり、
correctness (RTMPose keypoint) が CPU decode と一致（L2 許容内）。

## 実機調査結果 (2026-05-29, この Jetson)

- **CUDA nvJPEG は不可**: `libnvjpeg.so` は `/usr/lib/aarch64-linux-gnu/nvidia/` に在るが
  CUDA の `nvjpeg.h` ヘッダが無い → CUDA nvJPEG API 直叩きは不可。
- **Jetson MMAPI `NvJPEGDecoder` が唯一の HW 経路**: `/usr/src/jetson_multimedia_api/` に
  `include/NvJpegDecoder.h` + sample `06_jpeg_decode` + `NvBufSurface.h` / `nvbufsurface.h` /
  `nvbufsurftransform.h` が揃う。`NvJPEGDecoder` は内部で `libnvjpeg.so` を叩く HW デコーダ。
- **API**:
  - `decodeToFd(int& fd, in_buf, in_len, uint32_t& pixfmt, w, h)` — JPEG を HW メモリへ decode し
    DMA-buf FD + V4L2 pixfmt (YUV420/422/444) を返す。**速い経路**。
  - `decodeToBuffer(NvBuffer**, ...)` — sw メモリへ変換込みで遅い（コメント明記）。
  - **出力は YUV**（BGR ではない）。RTMPose は BGR + warpAffine を要求するので変換が要る。
- **ビルド統合**: `NvJPEGDecoder` は単純な共有 lib リンクでは使えず、MMAPI の "common classes"
  **ソース** (`/usr/src/jetson_multimedia_api/samples/common/classes/`) を自前ビルドに取り込む必要:
  最低 `NvJpegDecoder.cpp` `NvElement.cpp` `NvBuffer.cpp` `NvLogging.cpp` (+依存)。
  include は `/usr/src/jetson_multimedia_api/include` と `include/libjpeg-8b`、
  link は `libnvjpeg.so` + `libnvbufsurface` + (使うなら) `libnvbufsurftransform`。

## 検討する案

### A. NvJPEGDecoder `decodeToFd` → NvBufSurfTransform (GPU YUV→RGBA+resize) → RTMPose
HW decode → GPU で色変換とリサイズを 1 回。**理想形**。RTMPose の crop+scale が回転なし
（標準 top-down pose は通常そう）なら、NvBufSurfTransform の crop+scale が prebake の warpAffine を
**吸収**でき、decode+crop+resize+正規化を GPU 完結にできる可能性（det→bake の 4.1ms も削れる）。
ただし NvBuffer/DMA-buf ライフサイクル管理 + CUDA/EGL interop が要り統合コスト最大。
warpAffine が回転を含む場合は NvBufSurfTransform では代替不可（要確認）。

### B. NvJPEGDecoder `decodeToFd` → map → CPU `cvtColor(YUV→BGR)` → 既存 prebake
decode だけ HW 化し、以降は現状の cv::Mat 経路を流用。統合は A より軽い。ただし YUV→BGR の
CPU コストと NvBuffer→host コピーが乗るので、`cap→dec` の短縮幅は A より小さい（decode 本体は
消えるが色変換が残る）。**第1マイルストーンとして妥当**（HW decode の効果を切り分けて測れる）。

### C. GStreamer `nvjpegdec` / `nvv4l2decoder mjpeg=1` パイプライン
research/multi-camera-ingest.md の当初構想。GStreamer 依存を丸ごと導入する必要があり、
生 V4L2 + 単一 SPSC slot の現アーキと衝突。**没**（アーキ不整合・統合最重）。

### 没にした前提
- 「CUDA nvJPEG (`nvjpeg.h`) で簡単に GPU decode」→ **ヘッダ未搭載で不可**（上記調査）。
  migration-plan の当該記述は正しかった。MMAPI 経由が必須。

## 採用方針 (暫定)

**B を M1 で入れて HW decode の効果を計測 → A を M2 で狙う**（warpAffine 回転有無を確認した上で）。
段階導入により、NvBuffer ライフサイクルや correctness の問題を小さく切り分けられる。
`pixel_format` に 3 つ目の選択肢 `nvjpeg` (= MJPEG + HW decode) を足すのが自然
（`mjpeg`=CPU decode / `yuyv`=非圧縮 / `nvjpeg`=HW decode）。`JpegDecoder` を
インターフェース化し CPU/HW 実装を差し替える。

## Milestone (案)
- **M1**: CMake に MMAPI common-class ソース取り込み + `NvJpegDecoder` ラッパ。
  `--pixel-format nvjpeg` で decodeToFd→CPU cvtColor→既存 prebake。`cap→dec` を計測比較、
  correctness を CPU decode と照合。
- **M2**: NvBufSurfTransform で GPU 色変換+リサイズ。可能なら prebake warpAffine を吸収。
  `det→bake` も含めた短縮を計測。
- **M3**: NvBuffer プール化 + バッチ（複数カメラ同時 decode）で multi-cam スループット確認。

## 検証
- ビルド: MMAPI ソース取り込み後も `cmake --build` クリーン、既存 ctest 維持。
- correctness: 同一 MJPEG フレームで HW decode vs CPU `cv::imdecode` の RTMPose keypoint L2 を比較
  （許容内か）。色変換の YUV サブサンプリング差に注意。
- レイテンシ: `core-pipeline-e2e-latency.md` の計測基盤 (`cap→dec` / `cap→pub`) で
  CPU MJPEG / YUYV / HW nvjpeg の 3 者比較表を作る。

## 残課題 / リスク
- NvBuffer / DMA-buf FD のライフサイクルと SPSC slot (size 1, drop-old) の整合
  （FD をいつ閉じるか、コピーするか）。
- RTMPose prebake の warpAffine が回転を含むか（含むと案 A の NvBufSurfTransform 吸収が不可）。
- HW NVJPEG ブロックは 1 基。複数カメラで decode が直列化しないか（バッチ/並列度の確認）。
- correctness: HW decode の色再現が libjpeg-turbo と微妙に異なる可能性（keypoint への影響を計測）。
