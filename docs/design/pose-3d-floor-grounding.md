# pose-3d: floor-contact grounding (床接地)

(着手日 2026-07-04 / 関連: spatial-filtering の **M-D floor anchor**(保留分)を実装、
[pose-3d-spatial-filtering](pose-3d-spatial-filtering.md)、heel-sink は 2D 持病
[../tracks/pose-3d.md] の踵精度課題)

## 背景 / 動機

実機 WebUI で足の床貫通(heel-sink)を確認。録画クリップの world Z 実測(floor extrinsic の calib):
- **still**: `r_big_toe` median **−8.2mm・99.8% が床下**、heels 20–29mm、ankle ~80mm(正しく床上)。
- **walk_around**: stance 中に toe/heel が **−30〜−62mm** 床貫通(frac_below0 8–25%)。

heel-sink は RTMPose の踵/足首精度由来の **2D 持病**で 2D 段では直せないと確定済(memory)。時間フィルタ(M-C)でも
位置フロアは下がらない。→ **lift 段で床平面へ接地拘束する**のが唯一の対処層。完了条件: 足 sole 点の床貫通を消し、
静止/歩行 stance の足ジッタを低減、既存パス非退行(既定 OFF で byte 不変)。

## 検討した案

- **(没) `vmt.disable_below_floor` の拡張**: これは床下トラッカーを**表示 gate で隠すだけ**で pose を拘束しない。欲しいのは
  3D pose そのものの接地。
- **(没) floor extrinsic を lift 段へ配線**: **不要と判明(本設計の肝)**。triangulator の出力 world は floor AprilTag 契約
  (`floor_tag_map.hpp`)で **fitra Z-up・床=Z=0**。VMT の disable_below_floor も `pos.z<0`=床下前提。よって床平面は
  **定数 Z=`floor_z_m`(既定 0)** で、`joint.z` がそのまま床距離。extrinsic を lift へ渡す plumbing は一切要らない。
- **(v1 見送り) XY stance freeze**: 接地足の XY も固定すると slip を消せるが、誤検出時に足が地面に貼り付いて body が
  滑る失敗(locomotion-stability で既知)を招くリスク大。v1 は **Z のみ**。
- **(非対象) ankle の接地**: ankle は脚関節で stance でも ~8cm 床上。接地させると IK の脚長を壊す。**sole 点(toe/heel)のみ**。
- **(採用) lift 最終段の pure `floor_grounding`**: 下記。

## 採用設計

`cpp/src/lift/floor_grounding.{hpp,cpp}`(rigid_fit と同型の pure lift ヘルパ)。
`apply_floor_grounding(skel, state, dt_s, opts)` が Halpe26 の **sole 点 {20,21 big-toe, 22,23 small-toe, 24,25 heel}**
に対し(COCO17 は `kp_count`=17 でこれらを持たず自動 no-op):

1. **床下 clamp(常時・stateless)**: `z < floor_z → z = floor_z`。床下は物理的に不可能なので**無条件に正しい**。heel-sink を直接除去。
2. **stance snap(prev 必要)**: `floor_z ≤ z < floor_z + snap_band_m` かつ 生足速度 `< stance_vel_mps` の点を床へ **Z のみ** snap。
   速度は生(接地前)位置の frame 間差分で判定、prev は生値を保持(接地を state へ帰還させない)。

- **配置**: `multi_pipeline::maybe_update_3d` の **Kalman + IK の後(最終 3D 段)**。後段スムーザに再度沈められず、
  post-IK の sole 微調整は **output-only**(Kalman/IK state に帰還しない=drift 蓄積なし)。dt は Kalman と同じ。
- **flag**: `three_d.floor_grounding`(既定 **OFF** = byte 不変)+ `floor_z_m`/`floor_stance_vel_mps`/`floor_snap_band_m`。
  CLI `--floor-grounding`。`output`... なし。`threed_builder` 経由で `ThreeDConfig` へ。`FloorGroundingState` は
  `MultiCameraDriver` member、idle resume(`handle_idle_transition`)で `.reset()`(kalman.reset() と並置)。
- **オフライン**: `dump_keypoints_3d --floor-grounding [--floor-z-m/--floor-snap-band-m/--floor-stance-vel-mps]` で同一拘束を
  最終段に適用し A/B 可能。

## Milestone

- **M-D** ✅ 済 (2026-07-04): 上記 core + `multi_pipeline` 配線 + config + harness + ctest。

## 検証

- **ctest** `test_floor_grounding`(9 ケース): 床下 clamp / 床上不変 / stance snap / swing 非 snap / COCO17 no-op /
  floor_z オフセット / invalid skip / **ankle 非接地** / 冪等。full build + ctest **34/34**。
- **実機データ**(`still` 300f を harness `--floor-grounding`): `r_big_toe` **−8.2mm・99.8%床下 → 0.0mm・0%床下**、
  heels 20–29mm → 0mm(stance snap)、ankle ~77–79mm 不変。**床貫通ゼロ化を確認**。

## 残課題

- **XY stance freeze**(slip 検出込み)で接地足の水平ジッタ/slip も抑える(v1 は Z のみ)。
- **snap_band/stance_vel チューニング**: still で l_big_toe は ~12mm 残り(band 内でも速度が閾値を超える frame があり不snap)。
  歩行の動的接地(踏み込み/離地)での体感確認。
- **床が Z=0 でない校正**(controller-marker 等)では `floor_z_m` で吸収 or 既定 OFF のまま。
- 実機 A/B(WebUI)で歩行時の足の見た目・接地感を確認して既定 ON 化を検討。
