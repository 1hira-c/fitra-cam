# Gemini Code Assist スタイルガイド（設計・アーキレビュー）

このリポジトリでの Gemini Code Assist の主担当は**設計・アーキテクチャ観点**です。
コード品質は Copilot、正確性/数値は Codex が主担当のため、Gemini は以下を最優先で見てください。
共通のレビュー指針は `docs/review-guidelines.md`、規約は `CLAUDE.md` を参照。

## 基本

- **レビューコメントは日本語**で書く。
- 一般論より、本リポジトリ固有の設計判断・制約に照らした指摘を優先する。
- 確信度の低い指摘はその旨を明示する。

## 重点レビュー観点

### 1. 設計ドキュメント整合・完了定義
- 非自明な設計判断・トレードオフ・複数マイルストーンを伴う変更に、`docs/design/<track>-<topic>.md`（背景・**却下した選択肢とその理由**・採用構造・検証）が伴っているか。閾値調整やバグ修正のような単発作業は changelog 1 行で良い。
- **完了の定義**: コード変更だけでは不十分。track doc（`docs/tracks/<track>.md`）changelog の日付入りエントリ、（非自明なら）design doc、（アーキ/検証に触れるなら）`docs/cpp-migration-plan.md` の更新が揃っているか。揃っていなければ指摘する。

### 2. モジュール横断影響
- `cpp/src/` の各モジュール（`camera` / `infer` / `lift` / `tracking` / `vmt` / `pipeline` / `web` / `config` / `util`）間の依存関係・契約破壊を見る。
- 一方の変更が他モジュールの前提（データ形式、スレッド境界、所有権）を壊していないか。

### 3. 性能含意
- ホットパスでの不要なアロケーション・コピー・ロック・同期。集約 **170 fps**（USB 3 カメラ）目標への影響。
- **single TRT context / single CUDA stream** 設計（モデルごとに共有 1 つ。カメラ単位で複製しない）を崩す変更でないか。
- **latest-frame-wins**（SPSC キューサイズ 1・drop-old、リアルタイム鮮度優先）を崩していないか。

### 4. 互換性・スキーマ
- `web/dual_rtmpose/` の **JSON スキーマ互換**（`python/scripts/dual_rtmpose_web.py` の publisher が定義）を壊していないか。フロントは vanilla HTML/JS。
- subject profile v1（COCO17）/ v2（Halpe26）はマイグレーションしない設計。
- VR 出力の **固定 10 TrackerRole 順序**と VMT index の対応を変えていないか。

## 知っておくべき制約

- 数値目標: YOLOX bbox IoU **> 0.99** / RTMPose kpt L2 **< 1.0 px**（Python ORT 参照に対し）。FP16 RTMPose は低スコア kpt にドリフト（Y ±100〜200px）既知 — FP16/INT8 を触る変更は参照動画再検証が必要。
- Jetson 制約: OpenCV/TensorRT は apt 版のみ（`pip install opencv/tensorrt` 禁止）、カメラは `/dev/v4l/by-path`。
- コミット規約: `feat(<track>):` / `fix(<track>):` / `docs(<track>):`（track = `core-pipeline` / `pose-3d` / `vr-output`）。`feat(phaseN):` は旧式。
