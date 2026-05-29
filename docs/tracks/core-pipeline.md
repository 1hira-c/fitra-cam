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
