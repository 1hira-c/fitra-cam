# AGENTS.md — Codex 作業ガイド

`fitra-cam` は Jetson Orin Nano Super 上で YOLOX 人物検出 + RTMPose 17/26 keypoint 2D pose を
複数 USB カメラで動かし、3D lift / IK を経て VMT→SteamVR の VR トラッカー出力
する C++/TensorRT プロジェクト。Python (ONNX Runtime) 実装は**数値参照・フォールバック**として保持。
Codex は調査、実装、検証、必要なドキュメント更新までを一貫して行う。

## 最初に使うコマンド

変更前に `git status --short` を確認し、無関係な作業ツリー変更を変更・破棄しない。
実装変更後は影響範囲に応じて次を実行する。存在するテスト名は先に `ctest -N` で確かめる。

```bash
# C++: 構成、ビルド、基本確認、全テスト
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
ctest --test-dir cpp/build                       # 全テスト
ctest --test-dir cpp/build -R 'tracker_extract|kalman'   # pose-3d 系
ctest --test-dir cpp/build -R 'vmt|hmd_pose|auto_alignment|continuous_aligner'  # vr-output 系
./cpp/build/main --help
ctest --test-dir cpp/build                       # 全テスト

# C++: 代表的な focused test
ctest --test-dir cpp/build -N
ctest --test-dir cpp/build -R 'tracker_extract|kalman'
ctest --test-dir cpp/build -R 'vmt|hmd_pose|auto_alignment|continuous_aligner'

# Web UI
pnpm --dir web-ui typecheck
pnpm --dir web-ui build

# Python 参照・フォールバック
./python/scripts/setup_jetson_env.sh
. python/.venv/bin/activate
python python/scripts/dual_rtmpose_web.py --device auto
```

実行できなかった検証は、実行しなかった理由と残るリスクを報告する。pose-3d、カメラ、推論、
VR 出力の実機挙動に関わる変更は、ローカルテストだけを実機品質の証拠とみなさない。

## プロジェクトの地図

- `cpp/`: 主実装。C++/TensorRT + Jetson Multimedia API。
- `python/`: Python ONNX Runtime の数値参照とフォールバック。新機能は追加せず、動作維持だけを行う。
- `web-ui/`: Vite + React + TypeScript の主 Web UI。パッケージマネージャは `pnpm`。
- `web/calibration/`: legacy の vanilla-JS 校正ツール。`web-ui/` とは別系統。
- `docs/tracks/`: 現在の作業単位と changelog。`docs/design/`: 非自明な実装の設計記録。
  `docs/research/`: 未実装の探索。`docs/archive/phaseN-*.md`: 凍結履歴であり編集禁止。

作業の一次情報は [`CLAUDE.md`](CLAUDE.md) と
[`docs/tracks/README.md`](docs/tracks/README.md)、対象 track doc である。パイプライン、数値処理、
Web API、非自明な設計を変更する前には、関連する `docs/design/` と
`docs/cpp-migration-plan.md` を読む。

## 常に守ること

- `cpp/` のリアルタイム処理は **latest-frame-wins**。SPSC キューはサイズ 1、古いフレームを捨て、
  処理完遂より鮮度を優先する。
- TensorRT はモデルごとに共有 context 1 つ、CUDA stream 1 つ。カメラごとに TRT context を複製しない。
- C++ の affine、SimCC argmax、inverse-affine decode は Python 参照と数値的に一致させる。
  検証目標は YOLOX IoU `> 0.99`、高スコア keypoint の L2 `< 1.0 px`、集約 170 fps。
- FP16 RTMPose の低スコア keypoint ドリフトは既知。FP16 / INT8 や関連する推論処理を変更したら、
  参照動画で数値・実機の両方を再検証する。
- Web UI または設定を変更する場合、C++ Crow、Python fallback、既存クライアントとの REST / WebSocket
  JSON スキーマ互換性を保つ。
- 新規ロジックや変更した不変条件には、可能な限り回帰テストを追加する。

## 境界

### 常に行う

- 表面的な回避策より根本原因を修正し、境界値、並行性、所有権、ライフタイム、古い状態の残留を確認する。
- CUDA / TensorRT の変更では、デバイスメモリの寿命、stream 同期、戻り値、エンジン再生成条件を確認する。
- Jetson 固有のパッケージ、カメラ、電力モードは `~/CLAUDE.md` の現行手順に従う。

### 事前確認が必要

- 新たな実行時依存の追加、REST / WebSocket JSON スキーマの破壊的変更、デプロイ・CI 設定の変更。
- 外部送信、権限昇格、破壊的操作、仕様を大きく変える判断。

### 行わない

- OpenCV / TensorRT を pip で導入しない。
- `docs/archive/phaseN-*.md` を書き換えない。
- 秘密情報を追加、出力、コミットしない。

## ブランチ、コミット、ドキュメント

- ブランチは `<track>/<topic>`。通常は `Develop` から切り、未マージでも安定した同 track の先行ブランチが
  あればその先端に積んでよい。新たな `cpp-phaseN` / `feat(phaseN):` は作らない。
- コミットはビルド不能な中間状態を作らない意味のある単位にする。新規作業は
  `feat(<track>):` / `fix(<track>):` / `docs(<track>):` を使う。リポジトリ横断のメタ文書は bare `docs:`。
- ユーザー向け返信、進捗、コミットの prefix 後の件名・本文、PR 説明は日本語。
  コードコメント、識別子、例外文、`docs/` 本文は各ファイルの既存慣習に合わせる。
- 非自明な設計判断、トレードオフ、複数 milestone を伴う変更には
  `docs/design/<track>-<topic>.md` を作る。背景、採用・不採用案と理由、構造・不変条件、milestone、検証を残す。
- 実装作業の完了時には、対象 track doc に日付入り changelog を追記する。アーキテクチャまたは検証戦略に
  触れた場合は `docs/cpp-migration-plan.md` も更新する。

このファイルには毎回必要なコマンド、境界、既知の落とし穴だけを置く。詳細な設計説明や稀な手順は、
上記の一次ドキュメントに置く。
