# pose-3d: TrackerExtractor source freshness

(着手日 2026-07-07 / 対象: `TrackerExtractor` → `SlimeTrackerBus` → WebUI / SlimeVR / VMT)

## 背景

`TrackerExtractor` は `Skeleton3DBus` から 3D skeleton を読み、SlimeVR/VMT/WebUI 用の 10 tracker
を作る単一 producer。`three_d.vr_extract_event_driven` が true の時は `Skeleton3DBus::wait_for_update`
で新しい 3D 更新を待つ設計だったが、実装は戻り値を捨てていた。

そのため 3D 更新が来ていない timeout でも `snapshot()` を再読込し、同じ stale skeleton をもう一度
`extract_trackers` / st_filter / One Euro に投入していた。WebUI と OSC/VMT の両方で気になる揺れや引っかかりが
出る場合、出力先ではなくこの単一 producer の入力 freshness が共通原因になり得る。

## 検討した案

1. **timeout ごとに nominal dt で publish 継続**: 旧挙動に近く、WebUI の無信号は避けられるが、
   stale snapshot を st_filter / One Euro に再投入する根本問題が残るため不採用。
2. **timeout 即 invalid clear**: freeze は最短で消せるが、通常の 30Hz source / 60Hz extractor でも timeout が
   挟まりやすく、実フレーム間で不要な invalid が混ざるため不採用。
3. **quiet 閾値後に単発 invalid clear**: timeout は re-filter せず、3D bus が明確に止まった時だけ clear する。
   `stale_clear_after=250ms` は 30Hz source の約 7.5 frame に相当し、短い jitter では発火せず、ユーザーが
   freeze として知覚し始める前に診断と invalid clear を出す現実的な妥協として採用。

## 採用設計

- `Skeleton3DBus` の内部 `update_seq_` と更新時刻を `Skeleton3DSnapshot` に載せる。
- event-driven extractor は snapshot 取得後の `update_seq` を前回処理済み source と比較し、新規更新だけを
  tracker frame として処理する。`wait_for_update` から `snapshot()` までの間に新規 update が来た場合も
  `last_update_seq = snap.update_seq` で追従し、同じ source を次 loop で再処理しない。
- timeout だけの loop は publish しない。3D bus が `stale_clear_after_ms`（既定 250ms）以上 quiet の時だけ、
  stale clear として invalid tracker publish を 1 回だけ流す。初回 update 前に 3D bus が止まっているケースでも
  同じ stale clear を出し、`tracker_stream` が無信号にならないようにする。
- forced clear を出す直前に snapshot の `update_seq` を再確認し、timeout 後に新規 update が到着していれば
  clear をキャンセルして fresh frame として処理する。
- fixed-rate extractor は比較用に残す。旧挙動への切替は `--no-vr-extract-event-driven`。
- `/ws3d` と `/stats3d` に `tracker_stream` を追加し、source freshness を可視化する。
- stale 突入・復帰は edge-trigger でログする。ただし idle 中は運用ノイズになるためログ抑制する。

## `tracker_stream`

`SlimeTrackerBus` の JSON fragment に以下を追加する。

- `mode`: `event` / `fixed`
- `source_update_seq`: `Skeleton3DBus` の内部更新番号
- `source_pose_seq`: `Skeleton3DSnapshot::seq`
- `source_age_ms`: tracker publish 時点の source age
- `filter_dt_ms`: 今回の smoothing/filter step に渡した dt
- `fresh_hz`: event-driven では fresh source 間隔の EMA、fixed では `stats.tri_fps`
- `suppressed_wakeups`: event-driven で re-filter せず抑止した timeout wakeup 数
- `refiltered_duplicates`: fixed-rate で同じ source snapshot を再フィルタした tick 数
- `stale_clears`: 3D bus quiet による forced invalid clear の累積数
- `source_stale`: event-driven では forced invalid clear publish、fixed-rate では source age が
  `stale_clear_after_ms` を超えた状態

この block は top-level field なので `/ws3d` と `/stats3d` の両方で同じ値を観測できる。WebUI の 3D stats
panel では `trk_*` 行として表示する。

## 既定値

2026-07-07 以降、`three_d.vr_extract_event_driven` は default on。これは VR レイテンシ削減目的ではなく、
同じ 3D snapshot を複数回フィルタへ投入しない correctness/default freshness のため。比較や切り分けでは
`--no-vr-extract-event-driven` で fixed-rate producer に戻せる。

## Milestone

- **M1**: event-driven timeout の stale snapshot 再処理停止、`tracker_stream` 追加、default-on / `--no-` CLI。
- **M2**: PR review follow-up。wait→snapshot race と clear race を塞ぎ、初回 update 前の無信号を stale diagnostics
  で可視化。stale clear / fixed duplicate の回帰テストを追加。

## 検証

- `test_tracker_extractor`: event-driven timeout が stale tracker snapshot を republish しないことを固定。
- `test_tracker_extractor`: 初回 update 前の stale diagnostics、stale clear の単発性、fresh 復帰、fixed-mode
  duplicate/stale flag を固定。
- `test_main_config`: default on、YAML false、CLI on/off の precedence を固定。
- 既存 tracker/st_filter/main_config 系 ctest でスムージング・設定の回帰を確認する。

## 残課題

- `source_age_ms` は publish 時点の値なので、stale clear 後に 3D bus が長時間止まると WebUI 表示は次 publish まで
  更新されない。必要なら UI 側で local elapsed を加算する。
