# pose-3d: TrackerExtractor source freshness

(着手日 2026-07-07 / 対象: `TrackerExtractor` → `SlimeTrackerBus` → WebUI / SlimeVR / VMT)

## 背景

`TrackerExtractor` は `Skeleton3DBus` から 3D skeleton を読み、SlimeVR/VMT/WebUI 用の 10 tracker
を作る単一 producer。`three_d.vr_extract_event_driven` が true の時は `Skeleton3DBus::wait_for_update`
で新しい 3D 更新を待つ設計だったが、実装は戻り値を捨てていた。

そのため 3D 更新が来ていない timeout でも `snapshot()` を再読込し、同じ stale skeleton をもう一度
`extract_trackers` / st_filter / One Euro に投入していた。WebUI と OSC/VMT の両方で気になる揺れや引っかかりが
出る場合、出力先ではなくこの単一 producer の入力 freshness が共通原因になり得る。

## 修正方針

- `Skeleton3DBus` の内部 `update_seq_` と更新時刻を `Skeleton3DSnapshot` に載せる。
- event-driven extractor は `wait_for_update` が true を返した時だけ tracker frame を処理する。
- timeout だけの loop は publish しない。3D bus が 250ms 以上 quiet の時だけ、stale clear として invalid
  tracker publish を 1 回だけ流す。
- fixed-rate extractor は比較用に残す。旧挙動への切替は `--no-vr-extract-event-driven`。
- `/ws3d` と `/stats3d` に `tracker_stream` を追加し、source freshness を可視化する。

## `tracker_stream`

`SlimeTrackerBus` の JSON fragment に以下を追加する。

- `mode`: `event` / `fixed`
- `source_update_seq`: `Skeleton3DBus` の内部更新番号
- `source_pose_seq`: `Skeleton3DSnapshot::seq`
- `source_age_ms`: tracker publish 時点の source age
- `filter_dt_ms`: 今回の smoothing/filter step に渡した dt
- `fresh_hz`: event-driven では fresh source 間隔の EMA、fixed では `stats.tri_fps`
- `duplicate_ticks`: timeout や fixed-rate duplicate source の累積数
- `stale_clears`: 3D bus quiet による forced invalid clear の累積数
- `source_stale`: forced invalid clear publish かどうか

この block は top-level field なので `/ws3d` と `/stats3d` の両方で同じ値を観測できる。WebUI の 3D stats
panel では `trk_*` 行として表示する。

## 既定値

2026-07-07 以降、`three_d.vr_extract_event_driven` は default on。これは VR レイテンシ削減目的ではなく、
同じ 3D snapshot を複数回フィルタへ投入しない correctness/default freshness のため。比較や切り分けでは
`--no-vr-extract-event-driven` で fixed-rate producer に戻せる。

## 検証

- `test_tracker_extractor`: event-driven timeout が stale tracker snapshot を republish しないことを固定。
- `test_main_config`: default on、YAML false、CLI on/off の precedence を固定。
- 既存 tracker/st_filter/main_config 系 ctest でスムージング・設定の回帰を確認する。
