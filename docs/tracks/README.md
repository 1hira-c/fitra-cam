# 開発トラック (work tracks)

旧来の **phase 番号制 (Phase 0, 1, ... 15.5)** を廃止し、**ドメイン別トラック制**に移行した
(2026-05-27)。

## なぜ変えたか

phase 番号は「ゴールが事前に決まった一回限りの C++ 移行ロードマップ」(Phase 0–6) には
最適だったが、移行完了後の継続開発でも同じ単位を使い続けた結果、

- 番号が無限に増える (`15.5` のような小数が出た時点で単位が破綻)
- 実体は **少数の長命な subsystem を何度も再訪している**だけなのに、番号制だと
  「前の phase をまた手直ししている」という churn に見える
- subsystem ごとの文脈が phase11 / 12 / 14 / 15 / 15.5 の doc に散り、各 doc が毎回
  前提を再説明していた

という歪みが出た。トラック制では、同じ subsystem への再訪が **そのトラックの changelog 追記**
として一本の流れで読める。

## トラック一覧

| トラック | 範囲 | 状態 | 旧 Phase |
|---|---|---|---|
| [core-pipeline](core-pipeline.md) | capture / TRT 推論 / Web / 性能 / keypoint topology | 移行完了・安定 | 0–6, 9 |
| [pose-3d](pose-3d.md) | 3D lift / IK / Kalman / roll 品質 / subject calibration | 継続改善 | 7, 8, 12-M1, 13 |
| [vr-output](vr-output.md) | VMT / SteamVR alignment | 最もアクティブ | 14, 15, 15.5 |

## ドキュメントの構成

- 各トラック doc = **「現状 (scope / 設計原則 / live な制約)」 + 逆時系列の changelog**。
  changelog の各エントリは要約 + リンク (新規作業は [`docs/design/`](../design/)、
  過去作業は [`docs/archive/`](../archive/) の当時の詳細 doc)。
- **設計判断を伴う作業は [`docs/design/<track>-<topic>.md`](../design/) に設計 doc を必ず残す**
  (旧 phaseN doc の役割。テンプレ・運用は [`docs/design/README.md`](../design/README.md))。
  changelog はその要約に留める。軽い単発作業は changelog 1 行で完結。
- [`docs/cpp-migration-plan.md`](../cpp-migration-plan.md) は **C++ 移行 (Phase 0–6) の歴史記録 + アーキ仕様**
  として凍結保存。core-pipeline のアーキ図・リポレイアウト・依存表はここが今も source of truth。
- [`docs/archive/phaseN-*.md`](../archive/) は当時の phase 詳細設計 doc。**書き換えない**(歴史記録)。
  新しい作業はトラック doc の changelog に追記する。

## 運用 (ブランチ / コミット)

CLAUDE.md「Branching and commits」を参照。要点:

- **ブランチ**: `<track>/<topic>` (例 `vr-output/registration-gate`, `pose-3d/roll-quality`)。
  番号は振らない。
- **コミット prefix**: `feat(<track>):` / `fix(<track>):` / `docs(<track>):`
  (例 `feat(vr-output): ...`)。トラック名が scope。
- **完了の定義**: 該当トラック doc の changelog に entry を追記し、必要なら
  `cpp-migration-plan.md` の検証戦略表 / アーキ記述を更新する。
