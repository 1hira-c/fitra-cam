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

## 実装スパイク所見 (2026-05-29, /tmp で検証・リポジトリ未変更)

実カメラの 640×480 MJPEG フレーム 1 枚で MMAPI `NvJPEGDecoder` を直接叩いて検証した。

1. **ビルド recipe 確定・動作**: MMAPI common-class ソース
   (`NvJpegDecoder.cpp`/`NvElement.cpp`/`NvElementProfiler.cpp`/`NvLogging.cpp`/`NvBuffer.cpp`) を
   `-I.../include -I.../include/libjpeg-8b` でコンパイル、`-lnvjpeg -lnvbufsurface` リンクで HW decode
   成功。
2. **レイテンシ実測 (640×480)**: `decodeToFd` = **0.9ms** だが出力は **NVMM (block-linear, CPU 直
   map 不可)** → CPU アクセスには NvBufSurfTransform が必須。`decodeToBuffer` = **2.0ms** で
   CPU アクセス可 (sw メモリ)。いずれも CPU `cv::imdecode` (~5ms) より速い。出力フォーマットは
   `'YM16'` = **YUV422M (3 面プレーナ 4:2:2、Y=640×480 / U,V=320×480)**。
3. **★最大の制約: libjpeg ABI 衝突**。`libnvjpeg.so` は **無バージョンの `jpeg_*` をグローバル
   export**、一方 OpenCV imgcodecs は `jpeg_*@LIBJPEG_8.0` (system libjpeg-turbo) を要求。
   `NvJpegDecoder.cpp` を本体に直リンクすると **`cv::imdecode` (= 既定の MJPEG CPU 経路) が破綻**
   (空画像 / struct size mismatch 656 vs 776 / segfault)。
   - **解決策 (検証済み)**: NvJPEG 機能を **独立 .so に隔離**し、本体から
     `dlopen(..., RTLD_NOW|RTLD_LOCAL|RTLD_DEEPBIND)` で読む。`RTLD_DEEPBIND` が肝で、.so 内の
     `jpeg_*` 解決を自前依存 (libnvjpeg) 優先にし、本体の libjpeg-turbo と分離する。これで
     **nvjpeg decode と `cv::imdecode` が同一プロセスで両立**することを実証。
     (`RTLD_LOCAL` 単独では未定義シンボルがグローバルへ解決され不可。)
4. **色変換の罠**: HW decode の **輝度 Y は CPU decode と完全一致 (meanAbsDiff 0.01)** = decode
   自体は正しい。しかしプレーナ YUV422 → BGR の手動変換が CPU decode と合わない (meanAbsDiff ~57)。
   full-range/studio-range、U/V 入替を試しても解消せず、チロマ実値が中立シーンで ~64 (本来 ~128)
   と読める。プレーン意味論/レンジが噛み合わず**手動 CPU 変換は脆い**。

### 方針見直し
上記 3・4 より、当初の案 B (decodeToBuffer + 手動 CPU cvtColor) は (a) decodeToFd より遅く
(2.0 vs 0.9ms)、(b) 色変換が脆い、ため**非推奨**に格下げ。**`decodeToFd` (0.9ms, NVMM) +
NvBufSurfTransform で GPU 色変換 (YUV422→RGBA/BGR)** が、色も正しく速いため本命 (旧案 A)。
NvBufSurfTransform は crop+scale も担えるので prebake warpAffine 吸収 (det→bake 削減) も射程。

## 採用方針 (改訂)

`pixel_format` に 3 つ目 `nvjpeg` (= MJPEG capture + HW decode) を追加。capture fourcc は MJPEG の
まま、FrameSource の decode 分岐のみ差し替える。実装構造:
- **隔離 .so (`libfitra_nvjpeg.so`)**: `NvJPEGDecoder` + NvBufSurface + NvBufSurfTransform を内包し、
  `-fvisibility=hidden` で C API (`fitra_nvjpeg_create/decode_bgr/destroy`) のみ export。
  本体は `dlopen(RTLD_DEEPBIND|RTLD_LOCAL)` で読む。これにより既定の MJPEG CPU 経路 (cv::imdecode)
  を一切壊さない。
- **decodeToFd → NvBufSurfTransform → BGR(or RGBA) pitch-linear → map → cv::Mat** で色を正しく取得。
- `JpegDecoder` をインターフェース化し CPU / HW(.so) 実装を差し替え。

## Milestone (改訂)
- **M0 (完了, スパイク)**: 実機で HW decode 動作・レイテンシ・libjpeg ABI 衝突と
  `RTLD_DEEPBIND` 隔離・Y 一致を検証。recipe 確定。
- **M1 (完了, 2026-05-29)**: 隔離 .so (`libfitra_nvjpeg.so`) を CMake 別ターゲット化
  (MMAPI common-class ソース取り込み + `-fvisibility=hidden` + C API、RPATH で nvidia/tegra 解決、
  `main` と同ディレクトリ出力)。decodeToFd → NvBufSurfTransform(RGBA) → BGR を .so 内で完結
  (`fitra_nvjpeg_iso.cpp`)。本体に dlopen ローダ `NvJpegHwDecoder`
  (`camera/nvjpeg_decoder.{hpp,cpp}`, `RTLD_DEEPBIND|RTLD_LOCAL`) を追加、`frame_source` decode 分岐
  + `--pixel-format nvjpeg` 配線 (capture は MJPEG のまま)。下記実測参照。
- **M2 (一部完了, 2026-05-29)**: decode 出力を .so からゼロコピー (mapped RGBA ポインタ+pitch を返却)
  にし、手書きスカラー RGBA→BGR ループを除去。BGR 化は本体側で **NEON `cv::cvtColor`** 1 回に。
  `cap→dec` 5.0→4.0ms。**ただし VIC は 24-bit BGR 出力非対応** (NvBufSurfaceCreate/NvTransform が
  reject、実機で確認) のため RGBA→BGR のフルフレーム CPU 変換が残り、90fps では CPU 床のまま
  (下表)。真の高 fps 解放は GPU 前処理 (= NvBufSurfTransform の crop+scale で prebake warpAffine を
  吸収し full-frame 変換を消す / RTMPose 入力を GPU で作る) が必要。これは migration-plan の Phase 6
  「GPU 前処理」に合流する大きめの作業として M3 へ送る。
- **M3 (未着手)**: GPU 前処理 — VIC で decode→crop→resize を 1 パス化し full-frame CPU 変換を
  排除 (cached bbox で crop)。+ NvBuffer プール化・複数カメラ同時 decode。

## 検証 (M1 実測, 2026-05-29)

ビルド: `cmake --build` クリーン、ctest 9/9 pass。`libfitra_nvjpeg.so` は `fitra_nvjpeg_*` のみ
export (jpeg_* 隠蔽確認)、RPATH で依存解決。

**回帰チェック (最重要)**: 同一バイナリで `--pixel-format mjpeg` (cv::imdecode) が decode 失敗 0 件で
正常動作 → dlopen 隔離が効き既定経路を壊していない。クリーン停止: nvjpeg/mjpeg とも SIGINT で
~0.3s 終了 (ハングなし)。

correctness: 同一 640×480 フレームで HW decode(decodeToFd+NvBufSurfTransform→BGR) vs CPU
`cv::imdecode` の **meanAbsDiff B=0.70/G=0.31/R=0.46 (max22)** = 実用上一致 (max22 はチロマ補間の
丸め差)。

レイテンシ (単一カメラ 640×480@30, `--bench-fake-bbox`, yolox_s.fp16+rtmpose_m.fp16, 定常):

| 経路 | cap→dec | cap→pub | recv fps | 備考 |
|---|---|---|---|---|
| mjpeg (CPU libjpeg-turbo) | 6.7ms | 22.5ms | 30 | CPU で entropy decode |
| yuyv (非圧縮) | 0.95ms | 17.1ms | 30 | decode 不要だが USB 帯域大 |
| **nvjpeg (HW)** | **4.7ms** | **20.7ms** | 30 | MJPEG 帯域維持・CPU を entropy decode から解放 |

**所見**: 単一カメラの E2E 短縮は mjpeg-CPU 比 −1.8ms と中庸 (HW decode 0.9ms 自体は速いが、
NvBufSurfTransform + map sync + RGBA→BGR の CPU repack が乗る; cap→dec はさらに slot 待ちを含む)。
**真の価値は CPU オフロード** (entropy decode が HW ブロックへ)。純レイテンシ最小は単一カメラなら依然 YUYV。

**multi-cam 実測 (2 カメラ /dev/video0+video2, 640×480@30, 6 コア機, プロセス CPU を
/proc/PID/stat で計測)**:

| 経路 (2 cam) | プロセス CPU | recv/cam | recent_pose/cam | cap→dec | cap→pub |
|---|---|---|---|---|---|
| mjpeg (CPU) | **0.90 cores** | 30.0 | ~30 | 7.4ms | 26.9ms |
| nvjpeg (HW) | **0.72 cores** | 30.0 | ~30 | 5.1ms | 24.8ms |

2 カメラで **CPU −0.18 cores (約 20%)**、両 cam 30fps 維持・E2E も −2ms。CPU オフロードが実測で
確認できた。削減幅が控えめなのは RGBA→BGR の **CPU repack が 2×残る**ため。
(注: `jetson_clocks` 未設定の素の状態での相対比較。)

**高 fps (90fps×2) では CPU 差がほぼ消える (要注意の知見)**:

| 経路 (2 cam @90fps) | プロセス CPU | recv/cam | recent_pose/cam | cap→pub |
|---|---|---|---|---|
| mjpeg (CPU) | 1.81 cores | 88 | ~83 | 16.7ms |
| nvjpeg (HW) | 1.78 cores | 88 | ~81 | 17.5ms |

30fps で −0.18 cores だった差が 90fps では **ほぼゼロ**。処理フレームが ~166/s に増えると、
libjpeg decode の CPU 増加分を **nvjpeg 側の full-frame 色変換 (166×307k px/s) が相殺**し、かつ全体 CPU
が RTMPose prebake 支配になりデコード差の比率が縮むため。

**M2 後 (ゼロコピー + NEON cvtColor, 2 cam):**

| | mjpeg | nvjpeg | 差 |
|---|---|---|---|
| 30fps×2 | 0.89 cores | **0.68** | −0.21 (−24%) |
| 90fps×2 | 1.81 cores | 1.79 | −0.02 (≈0) |

スカラーループ除去で `cap→dec` は 5.0→4.0ms に改善したが、**24-bit BGR が VIC 非対応で RGBA→BGR の
full-frame CPU 変換が消せず**、90fps の CPU 床は変わらなかった。
**結論 (実測確定)**: nvjpeg は **中 fps 帯まで CPU を明確に削る** (≤30fps で −24%) が、カメラ最大 90fps では
色変換が床になり相殺。完全な高 fps 解放には **GPU 前処理** (full-frame CPU 変換の排除) が必須で、
これは Phase 6 GPU 前処理として M3 送り。実運用は RTMPose が ~30–80fps なので中 fps 帯の CPU 余力が
そのまま効く。当初の「fps が上がるほど一方的に有利」想定は実測が覆した。

**追記 (2026-05-29, GPU フロントエンド M2 後)**: 上記「GPU 前処理が必須」を
[`core-pipeline-gpu-frontend.md`](core-pipeline-gpu-frontend.md) M2 で実装し実測確認。RTMPose 前処理
(per-person warp+normalize) を GPU カーネルに移し pose 入力を device 直結 (H2D 消滅) した結果、
**90fps×2 で初めて ~1.8 コアの床を割った**:

| 経路 (2 cam @90fps, 同一手法) | プロセス CPU | det→bake | cap→pub |
|---|---|---|---|
| mjpeg (CPU) | 1.83 cores | 2.9ms | 16.7ms |
| nvjpeg CPU prebake | 1.77 cores | 3.5ms | 16.4ms |
| **nvjpeg GPU フロントエンド (M2)** | **1.28 cores** | **0.5ms** | **12.8ms** |

GPU 前処理で **−30% (vs mjpeg) / −28% (vs nvjpeg-CPU)**、E2E `cap→pub` −3.7ms。これは「色変換が床」
だった nvjpeg 単体に対し、**per-person warp/normalize の GPU 移行**こそが高 fps の CPU を解放すると
裏付けた。残り 1.28 コアは full-frame RGBA→BGR cvtColor (YOLOX/calib 用に残置) + YOLOX + capture が
律速で、M3 (YOLOX 前処理 GPU 化) / M4 で削る。

## 残課題 / リスク
- **dlopen 隔離の本体組み込み**: `.so` を CMake で別ターゲット化し、本体は実行時 dlopen。
  ビルド時はリンクしない (NEEDED にすると libnvjpeg がグローバル scope に入り cv::imdecode 破綻)。
- NvBuffer / DMA-buf FD のライフサイクルと SPSC slot (size 1, drop-old) の整合（いつ閉じる/コピー）。
- RTMPose prebake の warpAffine が回転を含むか（含むと NvBufSurfTransform 吸収が不可）。
- HW NVJPEG ブロックは 1 基。複数カメラで decode が直列化しないか（バッチ/並列度の確認）。
- 色変換は **GPU NvBufSurfTransform に寄せ済み**（手動 CPU 変換はプレーン意味論/レンジで脆いと判明）。
  ただし最終の **RGBA→BGR repack が CPU** (307k px/frame)。M2 で BGRx 直出し or GPU 完結 or
  RTMPose 入力を 4ch 化して repack 除去を検討 (cap→dec をさらに削れる)。
- 非 Jetson 環境ではこの .so をビルド対象から外すガード済み (`EXISTS NvJpegDecoder.h` 判定、
  CMake `FITRA_HAVE_NVJPEG`)。.so 不在時は dlopen 失敗 → 起動時エラー (既定 mjpeg は無影響)。
