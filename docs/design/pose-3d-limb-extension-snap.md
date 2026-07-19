# pose-3d: 四肢伸展スナップと足先方向推論

(着手日 2026-07-13 / 関連: [pose-3d-locomotion-stability](pose-3d-locomotion-stability.md))

## 背景 / 動機

肘・膝がほぼ伸び切った姿勢では、三角測量と IK 後にも上下のボーン方向が数度ずれ、VMT の
tracker 位置・回転と SlimeVR の回転が「わずかに曲がった肢」として残る。また、脚が一直線に
なると `hip→knee` と `knee→ankle` が共線になり、thigh / shin の twist は膝の曲げ面から観測
できない。既存経路はこの縮退時に前回 roll を保持するため安全だが、立位で足先を左右へ向けても
脚の向きへ反映されにくい。

完了条件は、(1) ほぼ伸展した腕・脚の tracker-facing chain を一貫した直線へ射影できる、
(2) 伸展脚の thigh / shin twist を足先方向から同じ基準で復元できる、(3) 欠損時に捏造 roll を
作らず既存 hold へ戻る、(4) 製品既定 ON としつつ、個別 kill switch と低レベルの旧挙動
baseline を残して実機 A/B できること。

## 検討した案

- **上流 `IkSolver` で Skeleton3D 自体を直線化**: `/ws3d`、subject calibration、bone drift、
  Kalman の入力まで変わり、VR 出力だけの補正として影響範囲が大きいので不採用。
- **tracker 回転だけを直線化**: rotation-only の SlimeVR には効くが、位置も送る VMT では tracker
  の位置と向きが食い違うため不採用。
- **tracker 用の私有 skeleton copy を直線化**: 上流を不変に保ち、VMT の位置と両出力の回転を
  同じ幾何から作れるため採用。
- **伸展 twist を world-Z や骨盤横軸で補う**: 観測していない膝向きや torso との絶対結合を作る。
  過去の roll 品質改善で排除した lateral/world-axis pin と同じ問題なので不採用。
- **heel/toe の足底面を使う**: heel 3D keypoint は既存実測でノイズが大きい。既存 foot tracker と
  同じ `ankle→big_toe` のみを使い、同じ平滑化係数を共有する案を採用。

## 採用設計

`TrackerExtractor` の `extract_trackers` 内で、次の順に処理する。

1. 元の post-IK skeleton から腕 2 本・脚 2 本の flexion（完全伸展 = 0°）を測る。
2. 屈伸方向つきの「逆向き」per-limb hysteresis を `ExtractContext` が保持する。伸ばす途中は
   `enter=20°` 以下で早めに入り、曲げる途中は `exit=12°` 以上で早めに抜ける。中間帯で単純な
   閾値交換をすると毎フレーム再吸着し得るため、非 snap 中は直近の最大 flexion、snap 中は最小
   flexion を保持し、最大 2°の移動量と連続 2 valid frame で方向を確認してから遷移する。最初の
   sample は遷移方向への移動を必須とし、次の sample は閾値外を維持していれば静止でも確認を
   完了する。逆方向へ戻った場合と欠損時は確認を reset する。初回は方向不明なので `exit=12°`
   以下だけを snap とする。idle→resume では他の smoothing/anchor と一緒にこの状態も clear する。
3. snap ON の伸展肢だけ skeleton を私有コピーし、parent を固定したまま
   `parent→distal` 軸上へ hinge / distal を配置する。2 本の現行ボーン長は保存し、脚では toe / heel
   を ankle の移動量だけ剛体平行移動して足部方向を保存する。
4. toe-direction ON の伸展脚では `ankle→big_toe` を脚軸の直交平面へ射影し、既存の adjacent-chain
   up 規約（屈曲時の `knee→ankle` / `knee→hip` の直交成分）と連続になるよう符号を反転して、
   thigh / shin の共通 up hint とする。これを省くと伸展への切替で 180° roll flip する。roll
   confidence は既存 foot と同じ `0.3` に落とし、toe jitter を強く追わない。
5. toe が invalid、脚軸と平行、または長さが退化した場合は foot 推論を使わず、既存の
   forward-only quaternion + held roll + parent-yaw transport へ戻る。

公開設定は `three_d.limb_extension_snap` と `three_d.extended_leg_toe_direction`（個別・製品既定 ON）、
および `extension_snap_enter_deg` / `extension_snap_exit_deg`。CLI は対応する
`--limb-extension-snap`、`--extended-leg-toe-direction`、OFF 用の
`--no-limb-extension-snap` / `--no-extended-leg-toe-direction`、`--extension-snap-*-deg`。
`extract_trackers()` の低レベル引数既定だけは旧出力との field-level 比較 baseline として両機能
OFF を維持し、実行時は `MainOptions` / `TrackerExtractorOptions` から ON を明示的に渡す。
`0 <= exit < enter < 90` を config validation と抽出 API の両方で守る。

## Milestone

- **M1**: tracker 私有コピー上の腕・脚 snap、共有 hysteresis、設定配線。
- **M2**: 足先方向による thigh/shin twist と欠損フォールバック。
- **M3**: ctest、設定 round-trip、設計/track changelog、実機 A/B 用 YAML 例。

## 検証

- `test_tracker_extract`: `TrackerExtractorOptions` の製品既定 ON と低レベル両機能 OFF の field-level
  完全一致、腕/脚の共通軸・位置・骨長保存、入力 skeleton 非変更、20° enter / 12° exit の方向確認、
  1-frame spike と欠損による確認中断、閾値越え後の静止による enter / exit 確認完了、
  中間帯での動作反転、toe-based thigh/shin twist、toe 欠損 hold、屈曲脚の非介入。
- `test_main_config`: 製品既定 ON、YAML/CLI の個別 OFF、emit-load round-trip、閾値順序の validation。
- 回帰: `ctest -R 'tracker_extract|firmware_protocol|vmt_protocol|main_config'` と full `ctest`。
- 実機 A/B: 同一の立位、T-pose、腕前方伸展、足先内外旋、歩行、しゃがみを OFF / snap-only /
  toe-only / both の 4 条件で比較する。合格条件は、伸展中の上下脚 forward 一致、足先内外旋への
  thigh/shin twist 追従、境界 chatter と歩行中の誤 snap が目視で増えないこと。

## 残課題

- `enter=20°` / `exit=12°`、方向確認最大 `2°`、toe roll weight `0.3` は初期値。製品既定は ON とし、
  実機で問題が出た場合は YAML の個別 `false` または `--no-*` で即時切り戻す。
- Halpe26 は手指方向を持たないため、伸展腕の twist は既存 held roll + chest yaw transport のまま。
  controller/hand orientation を入力できる場合だけ、腕にも観測ベースの twist source を追加検討する。
