# AGENTS.md — Codex CLI 向け指示

`fitra-cam` は Jetson Orin Nano Super 上で YOLOX 人物検出 + RTMPose 17/26 keypoint 2D pose を
複数 USB カメラで動かし、3D lift / IK を経て VR トラッカー出力（SlimeVR Firmware UDP / VMT→SteamVR）
する C++/TensorRT プロジェクト。Python (ONNX Runtime) 実装は**数値参照・フォールバック**として保持。

詳細・規約の一次情報は **`CLAUDE.md`** と **`docs/tracks/README.md`**。本ファイルはその要約と、
Codex がローカルレビュー時に重点を置く観点をまとめる。

## ビルド / テスト

```bash
# C++（主実装）
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
ctest --test-dir cpp/build                       # 全テスト
ctest --test-dir cpp/build -R 'tracker_extract|firmware_protocol|kalman'   # pose-3d 系
ctest --test-dir cpp/build -R 'vmt|hmd_pose|auto_alignment|continuous_aligner'  # vr-output 系

# Python（参照 / フォールバック・新機能追加はしない）
./python/scripts/setup_jetson_env.sh && . python/.venv/bin/activate
python python/scripts/dual_rtmpose_web.py --device auto
```

Jetson 制約: OpenCV/TensorRT は **apt 版のみ**（`pip install opencv/tensorrt` 禁止）、カメラは
`/dev/v4l/by-path`、性能計測前に `nvpmodel -m 0 && jetson_clocks`。詳細は `~/CLAUDE.md`。

## コミット / ブランチ / 言語

- コミット: `feat(<track>):` / `fix(<track>):` / `docs(<track>):`（scope = `core-pipeline` / `pose-3d` / `vr-output`）。プレフィックスは英語、**サマリ・本文は日本語**。
- ブランチ: `<track>/<topic>`（番号なし）。`cpp-phaseN` は歴史的パターンで新規では使わない。
- ユーザー向け返信・PR 説明は日本語。コード内コメント/識別子はファイルの既存慣習（英語多数派）に従う。
- **完了の定義**: コードだけでは不十分。track doc changelog の日付入りエントリ＋（非自明なら）`docs/design/<track>-<topic>.md`＋（アーキ/検証に触れるなら）`docs/cpp-migration-plan.md` 更新まで。

## レビュー観点（Codex の主担当）

ローカルでコードを実行できる強みを活かし、以下を重点的に見る（共通指針は
**`docs/review-guidelines.md`** 参照、コメントは日本語）:

1. **正確性・根本原因**: 対症療法でなく原因。エッジケース・並行性・所有権・ライフタイム。
2. **数値回帰**: 前処理/後処理（アフィン変換、SimCC argmax、inverse-affine デコード）が Python ORT 参照とビット一致（許容誤差内）か。YOLOX IoU **> 0.99** / RTMPose kpt L2 **< 1.0 px** / 集約 **170 fps** を退行させていないか。**FP16 RTMPose は低スコア kpt ドリフト既知** — FP16/INT8 を触る変更は参照動画で再検証。
3. **CUDA / TensorRT メモリ安全**: デバイスメモリのライフタイム、ストリーム同期、`cudaError`/TRT 戻り値チェック、エンジン再生成条件。
4. **ビルド整合**: 上記 cmake/ctest が通るか、未ビルド中間状態を生まないコミット粒度か。
5. **リアルタイム不変条件**: **latest-frame-wins**（SPSC size 1・drop-old）、**single TRT context / single CUDA stream** を壊していないか。
