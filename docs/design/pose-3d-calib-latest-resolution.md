# pose-3d: 校正成果物の run-time latest 解決 + setup での latest クリア

(着手日 2026-06-27 / 関連: [pose-3d-flow-daemon.md](pose-3d-flow-daemon.md),
[core-pipeline-setup-mode.md](core-pipeline-setup-mode.md),
[[feedback-calib-generated-not-required]])

## 背景 / 動機

校正ファイル（intrinsics / extrinsics）は**校正で生成される成果物**。にもかかわらず:
- `--enable-3d requires --calib PATH`（main_config.cpp）が **生成物のパスをユーザーに要求**する。
- daemon に生の config を渡すと `three_d.calib` 既定が空で、上記でいきなり死ぬ。

ユーザー指摘:「そもそも生成するファイルなのに無ければいけないのがおかしい（デフォルトで
値が入っている）」。正しいモデルは:

```
run しようとする
  └ calib が 引数/yaml にある? ──yes→ それを使う（明示が最優先）
        └ no → latest を見る
              ├ ある → それで run
              └ ない → 校正フロー（setup→intrinsic→extrinsic→subject）へ routing（死なない）
setup（手動起動）
  └ latest ポインタを消す → 全部 set し直す（手動入力は最小）→ 校正が新 latest を作る → run へ
```

完了条件: `--calib` を書かなくても 3D run が立ち上がる（未校正なら校正へ routing、hard-error
しない）。明示パス（既存リグの `extrinsics_1.yaml` 等）は最優先で尊重し壊さない。手動 setup は
latest ポインタを消して再校正に倒す（履歴の timestamp 実体は残す）。

## 検討した案

- **没（2026-06-27 午前, [[feedback-calib-generated-not-required]]）: `latest.yaml` を config の
  既定値にする** — `calib`/`excal_out` 既定を `calibrations/<kind>/latest.yaml` にし、precheck で
  **存在必須**にした。結果 (1) 実在しない latest を要求して error/crash、(2) legacy フォールバックが
  無印名しか見ず実体 `_1` を orphan、(3) setup でクリアしないので「作り直し」にならない。
  撤回済み。**latest を『setup 時の既定値』として焼いたのが誤り。**
- **採用: `latest` は run-time 解決フォールバック（config に焼かない）+ setup でクリア** — 下記。
  `calib`（読み）は config 既定**空のまま**。run/daemon が「明示 > latest > 校正へ」で**解決**する。

## 採用設計

### latest は「書き込み先の既定」であって「読みの既定」ではない

- **書き込み先**（calibration の出力）: `excal_out` / `floor_out` / `intrinsic_out` の既定を
  `calibrations/<kind>/latest.yaml` にする。`write_calibration_versioned` が「ファイル名が
  `latest.yaml` のとき sibling `<ts>.yaml` を書き latest symlink を atomic 貼り替え、それ以外は
  in-place」。**書き込み先に既定を持つのは妥当**（どこかに書く必要がある）。
- **読み**（`three_d.calib`）: config 既定**空**。run 時に解決:
  `effective_extrinsics(opts) = calib.empty() ? excal_out : calib`。
  → 明示 calib があればそれ、無ければ書き込み先（＝ latest symlink）。**config に読みの既定値を
  焼かない**ので「生成物なのに既定パスが入っている」アンチパターンを踏まない。
- 存在判定は **解決後パス**に対して行い、無ければ **校正へ routing**（daemon の既存
  initial_mode が intrinsic→extrinsic→subject→run を生成連鎖）。`--enable-3d requires --calib`
  の hard-fail は**撤廃**（解決して空＝未校正＝校正へ、で扱う）。

### 解決ヘルパー（`config` 層）

| 用途 | 解決順 |
|---|---|
| extrinsics 読み (run/三角測量/daemon 判定) | `calib` ＞ `excal_out`(=latest) |
| extrinsic-calib の intrinsics 入力 | `excal_intrinsics` ＞ `intrinsic_out`(=latest) ＞ effective extrinsics(intrinsics 内包) |
| floor-calib の intrinsics 入力 | `floor_intrinsics` ＞ `intrinsic_out`(=latest) ＞ effective extrinsics |

明示（CLI/yaml）は常に最優先。`precheck_mode_switch` の `source_ready` も解決後パスで存在判定する
（前回の precheck 漏れも同時に解消）。daemon の `excal_out != calib` 警告は、calib 空＝自動結合
なので解決後で比較し誤発火を止める。

### 手動 setup で latest クリア

`run_mode_setup` 開始時に `calibrations/<kind>/latest.yaml` **symlink を削除**（`<ts>.yaml`
実体は履歴として残す）。手動 setup に入る＝「作り直す」を確定させ、後続の intrinsic/extrinsic/
subject 校正が新しい latest を生成する。auto-setup（カメラ未構成で daemon が選ぶ）は消す latest が
無いので実質 no-op。クリアは `lift::clear_calib_latest(path)`（symlink のみ unlink）。

### UI（手動入力最小限）

`SetupPage`: 校正の読みパス欄を出さない（calib は解決される）。`normalizeCalibPaths` は
`three_d.calib` を**空のまま**にする（excal_out へ強制結合しない）。書き込み先は既定の latest で
良く、ユーザーは触らない。明示したい上級者向けに「詳細設定」だけ残す（任意）。

## Milestone

- **M1（backend）**: `write_calibration_versioned` + `clear_calib_latest` + 解決ヘルパー、
  既定（excal_out/floor_out/intrinsic_out → latest 書き込み先、calib は空のまま）、
  `--enable-3d requires --calib` 撤廃、daemon/setup/threed/calib-input/precheck を解決経由に、
  setup 開始で latest クリア。ctest（解決順 / versioned write / clear がポインタのみ削除）。
- **M2（frontend）**: `normalizeCalibPaths` が calib を空に保つ、setup から読みパス欄を退避。

## 検証

- ctest: `test_calib_io` に versioned write / `resolve`（明示>latest>空）/ `clear_calib_latest`
  （symlink のみ消え `<ts>.yaml` は残る）。
- 実機: `--enable-3d` + calib 空 + latest 無し → **error せず校正へ routing**。校正後 latest が
  でき run が立つ。手動 setup 再入 → latest 消えて再校正。明示 `extrinsics_1.yaml` config →
  そのまま使われる（無変更）。
- `cmake --build` / `pnpm build`。

## 残課題

- 履歴 `<ts>.yaml` の prune（N 世代）未対応。backlog。
- symlink 非対応 FS は想定外（Jetson ext4）。
