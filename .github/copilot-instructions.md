# Copilot Instructions（コードレビュー指示）

> Copilot code review はこのファイルの**先頭 4,000 文字のみ**読む。重要点を先頭に凝縮。

## レビュー方針

- **コメントは日本語**で書く。
- このリポジトリでの Copilot の主担当は**コード品質**: 可読性・命名・エラーハンドリング・セキュリティ・テスト網羅・規約遵守。正確性/数値や設計/アーキは他レビュワー（Codex / Gemini）が主担当。重大なら指摘して良いが主担当を優先。

## 重点チェック項目

1. **可読性・一貫性**: 命名、関数分割、周辺コードとの整合（コメント密度・命名・イディオムを合わせる）。マジックナンバー、重複、過度なネスト。
2. **エラーハンドリング・リソース**: 例外安全、`cudaError`/TRT/V4L2 など戻り値チェック漏れ、メモリ/FD/ストリームの解放漏れ、null・境界・整数オーバーフロー。
3. **セキュリティ**: 入力検証、外部送信時の取り扱い、機密情報のハードコード、安全でない文字列/バッファ操作。
4. **テスト網羅**: 新規ロジックに対応する `ctest` があるか。回帰を守るテストか。
5. **規約遵守**:
   - コミット: `feat(<track>):` / `fix(<track>):` / `docs(<track>):`（track = `core-pipeline` / `pose-3d` / `vr-output`）。プレフィックスは英語、サマリは日本語。`feat(phaseN):` は旧式で禁止。
   - ブランチ: `<track>/<topic>`。
   - コード内コメント/識別子は当該ファイルの既存慣習（英語多数派）に従う。

## 具体例（指摘してほしい正/誤パターン）

戻り値チェック漏れ（誤）→ 指摘する:
```cpp
cudaMemcpyAsync(dst, src, n, cudaMemcpyHostToDevice, stream);  // 誤: 戻り値未チェック
CUDA_CHECK(cudaMemcpyAsync(dst, src, n, cudaMemcpyHostToDevice, stream));  // 正
```

latest-frame-wins 違反（誤）→ 指摘する:
```cpp
queue.push(frame);                 // 誤: 滞留させ古いフレームを処理 = 鮮度優先の不変条件違反
latest.store(frame); /* drop-old */ // 正: 最新フレームで上書き
```

## リポジトリ前提（誤った前提でのレビューを避ける）

- 主実装は **C++/TensorRT**（`cpp/`、CMake 3.22+ / g++11 / CUDA 12.6 / TensorRT 10.3）。`python/` は **数値参照・フォールバック**で新機能追加はしない。`web/dual_rtmpose/` は Canvas フロント（JSON スキーマ互換を保つ）。
- `cpp/src/` 構成: `camera`（V4L2+nvjpeg）/ `infer`（TRT, YOLOX, RTMPose）/ `lift`（3D, Kalman, IK）/ `tracking` / `vmt` / `pipeline` / `web`（Crow）/ `config` / `util`。
- 守るべき不変条件（破る変更は指摘）:
  - **latest-frame-wins**: SPSC キューサイズ 1・drop-old。リアルタイム鮮度優先。
  - **single TRT context / single CUDA stream**（モデルごとに共有 1 つ。カメラ単位で複製しない）。
  - 数値目標: YOLOX IoU **> 0.99** / RTMPose kpt L2 **< 1px** / 集約 **170 fps**。FP16 は低スコア kpt ドリフト既知。
- Jetson 制約: OpenCV/TensorRT は **apt 版のみ**（`pip install opencv/tensorrt` 禁止）、カメラは `/dev/v4l/by-path`。
- **完了の定義**: コードだけでは不十分。track doc changelog ＋（非自明なら）`docs/design/` の design doc が必要。

---

補足（review では読まれない可能性あり / Chat・agent 用）: 共通のレビュー指針は
`docs/review-guidelines.md`、規約は `CLAUDE.md`、検証目標は `docs/cpp-migration-plan.md`。
