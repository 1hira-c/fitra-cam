# pose-3d: 立位伸展移動時の足置き去り解消 (locomotion-stability)

(着手日 2026-05-28 / 関連トラック: pose-3d, vr-output)

## 背景 / 動機

**症状**: 立位で足を伸ばしたまま胴体を移動すると、VR avatar の足が世界座標に置き去りになる
(VRChat FBT で「足が地面に貼り付いて体だけ滑る」絵)。

**根本原因** (3 層が複合):

1. **Kalman が per-joint 独立** (`cpp/src/lift/kalman.cpp` 旧版): `SkeletonKalman` は
   各 joint を独立に 6D constant-velocity で予測。hip の velocity は ankle の prediction
   に伝播しない。
2. **IK は bone 長のみを enforce** (`cpp/src/lift/ik.cpp:203-218`): `enforce_lengths`
   は親子両方 valid な時のみ動く。ankle が `valid=false` のまま IK を通っても触らない。
3. **tracker_extract の位置 freeze が world 絶対**
   (`cpp/src/slimevr/tracker_extract.cpp:467-486` 旧版): `valid=false` の瞬間
   `curr.pos ← prev_pos[i]` で world 絶対値を hold。hip が動いても tracker は動かない。
   位置 EMA に confidence / velocity gating が無く `base_alpha=0.5` 固定。

「立位伸展 + 移動」は 2D 検出で ankle/big_toe が motion blur と自己遮蔽で `valid=false` に
落ちやすい configuration なので、上記 3 つが全部効いて完全 freeze が起きる。

完了条件: 立位伸展で胴体を 1 m 横移動した時、SlimeVR Server / VRChat FBT で
足 tracker が hip に追従して動くこと。既存 ctest が通り、`/stats3d` の `freeze_pct` が
回帰しないこと。

## 検討した案

### 案A: tracker_extract 局所改善のみ — 対症療法、効果限定

`valid=false` の hold を hip 相対化 + velocity gate + 足限定 FK fallback。Kalman / IK は
無変更。**メリット**: 低リスク、`valid=false` 局面の完全 freeze は確実に消える。
**デメリット**: `valid=true` でも Kalman が独立予測なので、measurement が遅延 / ノイジー
な瞬間に hip → ankle 連動が構造的には成立しない。

### 案B: kinematic-tree Kalman — 採用 (案A と同時取り込み)

Kalman state を root (hip_center) world + 子関節 parent-relative offset の chain 構造に
再構築。hip の world 移動が child の FK 出力に直接乗る (parent_world + offset)。
**メリット**: 構造的に解決。`valid=true/false` 問わず chain で動く。**デメリット**:
state 設計を変えるので tuning パラメータ (q_pos_offset / q_vel_offset) が追加、tests
新設が必要。

### 案C: IK 出力をそのまま tracker source に — 没

`enforce_lengths` は bone 長しか保証せず absolute position の整合は親子両方 valid
依存。Kalman の絶対位置 (frozen) が dominant のままで、効果薄。

### 案D: foot grounding / locomotion prior — 保留

接地推定 + 軸切替 IK で物理的に正しい歩行再現。コスト数週間規模。案 B + 案 A で
症状が十分に減る見込みなので別トラック候補に棚上げ。

## 採用設計

### M1: tracker_extract 層 (案A)

**変更ファイル**:
- `cpp/src/slimevr/tracker_extract.{hpp,cpp}`
- `cpp/src/slimevr/tracker_extractor.{hpp,cpp}`
- `cpp/tools/test_tracker_extract.cpp` / `test_tracker_extract_pos.cpp` (テスト拡張)

**追加した state 構造**:
- `PosSmoothingContext` — hip cache (`current_hip_pos` / `prev_hip_pos`), 各 tracker の
  `last_raw_pos` + `has_last_raw`, `dt_s`。`TrackerExtractor::pos_ctx_` に格納。
- `ExtractContext` + `FootAnchor[2]` — 足限定の FK fallback 用 anchor
  (knee→ankle 方向 + tibia 長, ankle→toe 方向 + foot 長)。`TrackerExtractor::extract_ctx_`
  に格納。

**新しい不変条件**:
- `apply_pos_smoothing` は `ctx.hip_valid && ctx.prev_hip_valid && ctx.has_last_raw[i]` の
  とき hip 相対 hold。それ以外は world 絶対 hold (既存挙動)。Waist tracker は
  `prev_pos ≡ hip_center` なので offset ≈ 0 で自然に hip 追従するため特別扱い不要。
- velocity gate は **`last_raw_pos` (前周期の生 curr) と curr** の delta で測る。
  prev_pos (EMA 平均) との距離だと EMA 収束途中で誤 trigger するため。
- 足の `FootAnchor` 更新は **ankle/toe ともに real measurement のフレーム** に限る。
  synth 結果の dir/length で自己 drift させない。

**主要しきい**:
- `kPosVelGateLow_mps = 8.0`, `kPosVelGateHigh_mps = 16.0` — 16 m/s は sprint 上限の
  ~12 m/s に margin。jitter は通常 >> 16 m/s なので確実に gate。
- `kFootFkSmoothingWeight = 0.15` (vs 通常 `kFootSmoothingWeight = 0.3`) — FK 合成中は
  quat EMA を prev に寄せる。実 measurement 復帰までの過渡を滑らかに保つ。

### M2: kinematic-tree Kalman (案B)

**変更ファイル**:
- `cpp/src/lift/kalman.{hpp,cpp}` (大改修)
- `cpp/tools/test_kalman_chain.cpp` (新規) + `cpp/tools/CMakeLists.txt`

**state 構造**:
- Root (parent == -1): 6D world `[px, py, pz, vx, vy, vz]`。
- 他: 6D parent-relative offset `[ox, oy, oz, vox, voy, voz]`。
- `world_pos` を per-joint にキャッシュし、出力 / child correct step で参照。

**process noise**:
- `q_pos / q_vel` は root world 用、`q_pos_offset / q_vel_offset` は child offset 用。
  default は同値からスタート (既存 tuning と連続性確保)。`q_pos_offset` を下げると
  chain がより rigid になり、observation 抜けに対する drift が減る。

**update flow**:
1. `ensure_topology()` で root (parents[i] == -1 の最初) を探し、BFS で
   parent-before-child の topo_order を構築。Halpe26 の parents[] は index 順に
   sorted されていない (parents[0]=18 など) ので必須。
2. topo_order 順に各 joint を predict + correct + emit world。
3. Child の correct は `residual = z_world - parent_world - x_offset` で offset 空間で
   行う (H 行列は world と同形)。
4. Parent が未初期化の child は skip — hallucinated world を出さない。

**API 互換**:
- `SkeletonKalman::update(measurement, dt_s) → Skeleton3D` の signature は不変。
- `multi_pipeline.cpp` の呼び出しは無変更。

## Milestone

- **M1** (commit `6c419fc`): tracker_extract に hip 相対 hold + velocity gate + FK fallback
- **M2** (commit `fc5a3d6`): Kalman を hip 起点の kinematic chain に再構築
- **M3** (このコミット): design doc + track changelog + 実機検証

各 M は単独で意味のあるユニット (M1 だけマージしても症状の半分は消える)。

## 検証

### ctest (10 ケース全通過済)

```
ctest --test-dir cpp/build --output-on-failure
```

- `test_triangulator`, `test_firmware_protocol`, `test_tracker_extract`,
  `test_vmt_osc_writer`, `test_vmt_protocol`, `test_hmd_pose_receiver`,
  `test_auto_alignment`, `test_main_config` — 既存テスト、全通過
- `test_tracker_extract_pos` — M1 で 4 ケース追加、全通過
  - `test_hip_relative_hold_on_invalid`: hip 移動 + ankle invalid で tracker が hip に追従
  - `test_first_valid_frame_skips_velocity_gate`: 初フレームの (0,0,0) sentinel から
    の遷移を outlier 扱いしない
  - `test_velocity_gate_attenuates_jump`: 300 m/s スパイクで alpha ≈ 0
  - `test_velocity_gate_passes_plausible_motion`: 5 m/s 動作は通常 EMA で通る
- `test_tracker_extract` — M1 で 2 ケース追加、全通過
  - `test_foot_fk_fallback_uses_last_anchor`: anchor seeded 後、ankle 失点でも foot
    valid (FK 合成)、confidence ≤ 0.20
  - `test_foot_fk_fallback_needs_seed`: 1 フレーム目から ankle 失点だと foot invalid
- `test_kalman_chain` — M2 で新規、4 ケース全通過
  - root motion が unobserved child に伝播
  - child measurement で offset 補正
  - long missing → reset → 再 init
  - 親未初期化の child は skip

### 実機検証 (未実施 — Jetson + Windows 環境必要)

1. Jetson で `./cpp/build/main` 起動 + WebUI:
   - 立位伸展で 1 m 横移動 → 足 AxesHelper×10 が hip と共に追従
   - 通常歩行 3 歩 → 足が踏み出し時に独立に動く (chain Kalman が child measurement を尊重)
   - 急な手の振り → upper_arm tracker が振動しない (velocity gate)
   - `/stats3d` JSON で `freeze_pct` / `dropout_count` が回帰してないこと
2. Windows VMT 経路で VRChat FBT 入室 + ジャンプ + しゃがみ + 歩行のフルパス
3. 合格基準:
   - 立位伸展 1 m 横移動で foot tracker の world 移動量 ≥ 0.7 m
   - 通常歩行 `freeze_pct` ≤ baseline + 5pp
   - VRChat 内で「足が地面に置き去り」感が消える (目視)

## 残課題

- **foot grounding / locomotion prior** (案 D): 接地推定 + 軸切替 IK で「床貫通する swing」や
  「接地足の slip」が残る場合のみ着手。M1+M2 で症状が十分減れば不要。別トラック
  `pose-3d/foot-grounding` 候補。
- **`q_pos_offset` / `q_vel_offset` の tuning**: 現在は root と同値スタート。実機評価で
  「chain が rigid すぎる / 緩すぎる」が出たら個別に調整。design doc に決定根拠を残す。
- **FK fallback の足以外への拡張**: 現状は ankle/big_toe のみ。手首・肘も同様に anchor を
  持てば腕の振りで wrist が落ちても tracker が無効化されない。需要が顕在化してからで
  良い。
