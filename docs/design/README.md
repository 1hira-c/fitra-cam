# 設計ドキュメント (design docs)

**非自明な設計判断を伴う作業の設計記録**を置く。旧 phase 番号制で `docs/phaseN-*.md` が
担っていた役割を、番号ではなくトラック名で引き継ぐもの。何度も後の自分を救ってきた一級の
資産なので、軽い作業に埋もれさせず独立 doc として残す。

## いつ書くか

- **必須**: 複数 milestone を伴う / トレードオフがある / 構造を変える作業
  (= 従来 phaseN doc を書いていたレベル)。
- **不要**: しきい値調整・単発バグ修正など。トラック changelog 1 行で完結させる。

## 命名

`<track>-<topic>.md` (番号なし)。例: `vr-output-registration-gate.md`,
`pose-3d-roll-quality.md`。`<track>` は [`../tracks/`](../tracks/) の 3 つ
(`core-pipeline` / `pose-3d` / `vr-output`) または新設トラック名。

## トラック doc との関係

- 設計 doc = **深い記録**(なぜ・検討・構造)。
- [`tracks/<track>.md`](../tracks/) の changelog エントリ = その**要約 + リンク**。
- 旧 `cpp-migration-plan.md` (計画 + 検証) ↔ `phaseN-*.md` (設計詳細) と同じ分業。

## 他フォルダとの境界

| フォルダ | 役割 |
|---|---|
| `docs/design/` | **実装する/した作業**の設計記録(生きている) |
| `docs/research/` | **未実装**の検討・評価ノート(前向き探索) |
| `docs/archive/` | 旧 phase 番号制の設計 doc(凍結・書き換えない) |

実装に着手したら `research/` のノートは `design/` の doc に昇格させてよい
(research 側はリンクで残す)。

## テンプレート

```markdown
# <track>: <topic>

(着手日 YYYY-MM-DD / 関連 backlog・research へのリンク)

## 背景 / 動機
なぜ今これをやるのか。旧挙動の何が問題で、何を達成したら完了か。

## 検討した案
採用案だけでなく **没にした案と理由** を必ず残す。
(例: Bridge relay 没 = SteamVR Named Pipe 排他 / lateral pin = degeneracy で 90° roll)
これが「後で同じ轍を踏む / 再発明する」のを防ぐ核。

## 採用設計
データフロー・所有権・不変条件・主要しきい値とその根拠。
キーとなる関数 / ファイルは inline で示す。

## Milestone
M1, M2, ... = コミット境界。各 M で何が動くようになるか。

## 検証
ctest 対象 / 実機手順 / 合格基準。

## 残課題
次トラック候補・backlog 化したもの。
```
