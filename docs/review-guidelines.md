# レビュー指針（AIレビュワー共通）

このドキュメントは、本リポジトリで利用する AI レビュワーの**共通の真実源**です。各ツールの
ネイティブ設定ファイル（`AGENTS.md` / `.github/copilot-instructions.md` /
`.gemini/styleguide.md`）はこのファイルを参照します。

## 全体方針

- **レビューコメントは日本語**で書く（`CLAUDE.md` の PR 説明＝日本語方針に合わせる）。
  Conventional Commits のプレフィックスや関数名・パス等の技術トークンは英語のまま混在で良い。
- 一般論の指摘より、**本リポジトリ固有の制約**（後述）に照らした指摘を優先する。
- 重箱の隅より**読み手の意思決定を変える指摘**を優先し、確信度の低いものはその旨を明示する。

## 役割分担（多角的レビューの設計）

3 つのレビュワーに**相補的な観点**を割り当て、重複を減らしつつ網羅性を上げる。

| レビュワー | タイミング | 主担当の観点 |
|---|---|---|
| **Codex CLI** | ローカル（push 前・コード実行可） | 正確性 / 根本原因 / 数値回帰（IoU・kpt L2・fps）/ CUDA・TensorRT メモリ安全 / ビルド整合 / リアルタイム不変条件 |
| **GitHub Copilot** | PR | コード品質 / 可読性 / 命名 / エラーハンドリング / セキュリティ / テスト網羅 / コミット・ブランチ規約遵守 |
| **Gemini Code Assist** | PR | 設計・アーキ / 設計ドキュメント整合 / モジュール横断影響 / 性能含意 / Web JSON スキーマ互換 |

各レビュワーは主担当外でも重大な問題に気づけば指摘して良いが、**主担当の観点を最優先**する。

## 全レビュワーが知るべき制約

### 数値検証目標（`docs/cpp-migration-plan.md` 検証戦略表）
- YOLOX bbox IoU **> 0.99**、RTMPose keypoint L2 **< 1.0 px**（高スコア kpt のみ）を Python ORT 参照に対して維持。
- 集約スループット **170 fps**（USB 3 カメラ）を退行させない。
- **FP16 RTMPose は低スコア kpt にドリフト**（Y ±100〜200px）が既知。FP16/INT8 を触る変更は参照動画で再検証必須。

### Jetson 全体制約（`~/CLAUDE.md`）
- OpenCV / TensorRT は **apt 提供版のみ**。**`pip install opencv/tensorrt` は禁止**（システムパッケージを壊す）。NumPy は 1.x。
- カメラは **`/dev/v4l/by-path`** の固定名で参照（`/dev/video*` 直指定や `by-id` への変更はハードウェア再検証なしに行わない）。
- 性能計測前に `nvpmodel -m 0 && jetson_clocks`。

### リアルタイム不変条件（`docs/tracks/core-pipeline.md`）
- **latest-frame-wins**: SPSC キューサイズ 1・drop-old。フレーム取りこぼしより**リアルタイム鮮度を優先**する設計。古いフレームを溜める変更は不変条件違反。
- **single TRT context / single CUDA stream**: モデルごとに共有コンテキスト 1 つ。カメラ単位の TRT コンテキスト複製はしない。
- V4L2 は `VIDIOC_REQBUFS` でカメラあたり 4 バッファ。

### 設計ドキュメント先行文化と完了定義（`CLAUDE.md`）
- 非自明な設計判断・トレードオフ・複数マイルストーンを伴う作業は `docs/design/<track>-<topic>.md` が**必須**（背景・**却下した選択肢とその理由**・採用構造・検証）。閾値調整やバグ修正のような単発作業は changelog 1 行のみ。
- **完了の定義**: コードだけでは不十分。track doc（`docs/tracks/<track>.md`）の changelog に日付入りエントリ＋（非自明なら）design doc＋（アーキ/検証に触れるなら）`docs/cpp-migration-plan.md` 更新まで揃って完了。

### コミット / ブランチ規約（`CLAUDE.md`）
- コミット: `feat(<track>):` / `fix(<track>):` / `docs(<track>):`（scope = track 名 = `core-pipeline` / `pose-3d` / `vr-output`）。リポジトリ横断メタは bare `docs:` 等。プレフィックスは英語、サマリ本文は日本語。
- ブランチ: `<track>/<topic>`（番号なし）。`Develop` か同 track の先行ブランチから分岐。
- `cpp-phaseN` / `feat(phaseN):` は**歴史的パターン。新規では使わない**。

## 観点詳細

### Codex CLI（ローカル・実行可）
- **正確性と根本原因**: 対症療法でなく原因を突く。エッジケース・並行性・所有権。
- **数値回帰**: 前処理/後処理のアフィン・SimCC argmax・デコード math が Python 参照とビット一致（許容誤差内）か。IoU/L2/fps を劣化させていないか。
- **CUDA / TensorRT メモリ安全**: デバイスメモリのライフタイム、ストリーム同期、`cudaError` チェック、エンジン再生成条件。
- **ビルド整合**: `cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release && cmake --build cpp/build -j` が通るか。`ctest`（例 `ctest -R 'tracker_extract|firmware_protocol|kalman'` 等、track 別）。
- **リアルタイム不変条件**: latest-frame-wins / single TRT context / SPSC size 1 を壊していないか。

### GitHub Copilot（PR・コード品質）
- 可読性・命名・関数分割・周辺コードとの一貫性（コメント密度・命名・イディオムを合わせる）。
- エラーハンドリング・リソース解放・例外安全・null/境界。
- セキュリティ（入力検証、外部送信、機密の取り扱い）。
- テスト網羅（新規ロジックに対応する ctest があるか）。
- コミット/ブランチ規約・言語規約の遵守。

### Gemini Code Assist（PR・設計/アーキ）
- 設計ドキュメント整合: 非自明な変更に design doc があるか、track changelog・完了定義を満たすか。
- モジュール横断影響: `cpp/src/{camera,infer,lift,tracking,vmt,pipeline,web,config,util}` 間の依存・契約破壊。
- 性能含意: ホットパスのアロケーション・コピー・同期、170 fps 目標への影響。
- 互換性: `web/dual_rtmpose/` の **JSON スキーマ互換**（`dual_rtmpose_web.py` の publisher が定義）、subject profile v1/v2、固定 10 TrackerRole 順序。
