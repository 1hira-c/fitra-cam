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

### M4: swing/twist 分離による roll-only hold (2026-05-29)

**残った症状**: M1 (hip 相対 hold) で foot tracker の*位置*は hip に追従するように
なったが、立位伸展で**足先は動くのに太もも・すね・上腕の bone がまったく回らない**。

**直接原因**: roll 縮退時に bone の*向き*ごと freeze していた。`upper_leg` / `lower_leg`
/ `upper_arm` は roll を決める up hint (thigh: `ankle-knee`、shin: `hip-knee`、arm:
`wrist-elbow`) が forward と平行に近づくと縮退し、`quat_from_forward_up()` が sin θ
ゲートで `false` → `valid=false` → `apply_quat_smoothing` が **quaternion 全体**を前
フレームで hold。失われるのは roll (bone 軸まわりの twist) だけのはずなのに、pitch/yaw
(= bone がどこを向くか) まで一緒に止まる。forward (`knee-hip` 等、端点だけで決まる) は
常に信頼できるのに roll が測れないという理由で向きごと freeze していた。

**検討した案**:
- **①world-up で roll を代用** (旧 tertiary world Z 相当) — 却下。立位伸展 / 直座りで
  「膝裏が天井向き」の捏造 roll が confidence 1.0 で書かれる、まさに M1 以前に潰した
  アンチパターンの復活。
- **②Euler 分解で roll 軸だけ hold** — 却下。pitch 90° 付近で gimbal lock、特異点処理が
  煩雑。
- **③swing/twist 分解 (採用)**: 相対回転 `prev⁻¹·curr` を bone forward (local +Z) 軸で
  swing (pitch/yaw) と twist (roll) に分解し、独立 alpha で slerp。roll 縮退時は twist
  alpha を 0 にして roll を前フレーム保持、swing は満額追従。roll は parallel transport
  で連続。特異点は ~180° swing のみ (twist=identity に縮退して回避)。

**変更ファイル**:
- `cpp/src/slimevr/tracker_extract.hpp`: `SlimeTracker` に `swing_confidence`
  (swing ゲート) を追加。`roll_confidence` を twist 専用ゲートに意味変更。
- `cpp/src/slimevr/tracker_extract.cpp`:
  - `build_tracker`: forward 縮退 (真の欠損 → `valid=false`) と roll 縮退 (forward 有効
    → `valid=true` + `roll_confidence=0` + forward-only quat) を区別。`fallback_up_for`
    で forward と非平行な world 軸を up 代用にする (roll は twist alpha=0 で捨てられるので
    代用 up の roll 値は出力に無関係)。
  - `foot_tracker`: `swing_confidence = kFootSmoothingWeight` も渡す (foot は overall
    low-pass を維持)。
  - `apply_quat_smoothing`: swing/twist 分解スムージング。`swing_confidence ==
    roll_confidence` の時は従来どおり単一 slerp の **fast path** (rigid bone と foot は
    ビット同一 = 回帰ゼロ)。

**不変条件**:
- swing 成分は curr の roll に不変 (`Q' = Q·Rz(φ)` で swing 不変、twist のみ変化)。
  よって縮退時に forward-only quat へ入れた任意 roll は twist alpha=0 で完全に捨てられる。
- `roll_confidence=0` かつ `swing_confidence=1` で「roll hold + 向き追従」。
  両者 0 で完全 freeze (従来の `valid=false` hold 相当)。

### M5: parent-yaw transport による held roll の横向き追従 (2026-05-29)

**残った症状**: M4 で伸展肢の*向き* (swing) は追従するようになったが、実機で
「伸展状態で xyz 移動は問題ないが**回転がだめ、特に横を向いたとき**」という報告。

**直接原因**: 立位で脚・腕を伸ばすと bone forward (`knee-hip` / `ankle-knee` /
`elbow-shoulder`) が**ほぼ鉛直**になる。この configuration で「横を向く」= 鉛直軸まわりの
yaw 回転は、**bone forward 軸まわりの回転 = まさに twist (roll)** に一致する。M4 は roll を
world 絶対で hold するので、伸展肢が鉛直なときの体 yaw がちょうど hold 軸に乗り、bone の
向きが最初の yaw で固まる。swing は forward (鉛直) が変わらないので恒等のまま。膝/肘を曲げると
roll が観測でき症状が出ない = 「特に横向き (= 鉛直肢) で」の報告と一致する。

**検討した案**:
- **①rigid pelvis pin の復活** (roll を骨盤 yaw に confidence 1.0 で絶対結合) — 却下。
  M4 / upper_leg で潰した「捏造 roll を全幅で書く」アンチパターンそのもの。
- **②parent-yaw transport (採用)**: M1 が*位置*でやった「world 絶対 hold → hip 相対 hold」の
  **回転版**。観測可能な親 tracker の orientation 変化 `D = P_curr·P_prev⁻¹` を、roll を
  hold している split-branch tracker の prev に**左から**掛けて transport する。swing が
  forward を観測値に再整合するので、bone forward 軸まわりの成分 (鉛直肢では yaw) だけが roll
  として残り、親の pitch/roll は swing が吸収する → yaw だけを明示抽出する必要がない。
  これは絶対 pin ではなく**差分 (delta) 結合**: 相対 roll オフセットは保存され、変化量だけ
  乗る。M1 が「足 = hip」ではなく「hip の移動量だけ足を動かす」だったのと同じ構図。
  **参照は肢ごと**: 腕は **chest** (肩甲帯)、脚は **waist** (骨盤) に乗る。骨盤は脊椎の捻りで
  胸とは独立に yaw できるので、腕に骨盤を使うと体幹回旋を誤追従する。腕は chest が観測できない
  ときのみ waist に fallback (固まるよりマシ)。

**変更ファイル**:
- `cpp/src/slimevr/tracker_extract.cpp`:
  - `apply_quat_smoothing`: ループ前に waist / chest それぞれ (`prev_quat[ref]` × 当該フレーム
    raw `curr[ref]`) から transport delta を一度ずつ計算 (`make_transport_delta` lambda)。
    split branch (`sa != ta`) の `p` に、肢に応じた参照 (腕→chest / 脚→waist) の delta を
    `carry`-blend して左から掛ける。fast path (rigid bone / foot / chest・waist 自身) は
    不変 → ビット同一。
  - **`carry = 1 - ta/sa`**: parent-yaw prior と bone 自身の roll 観測の相補ブレンド。
    完全 hold (`ta=0`) は full transport、roll が観測可能になるほど (`ta→sa`) transport を
    弱め、親に対して相対回転する肢を強制追従させない。
  - **magnitude gate** `kPelvisYawGateLow_rps=8 / High=16` (rad/s): 横向き時に `hip_axis` が
    カメラ depth 軸に乗って骨盤 yaw 推定が暴れる / 三角測量 glitch の局面で、暴れた delta を
    identity に向けて減衰。位置 velocity gate と同じ思想。

**不変条件**:
- transport delta は loop 前に確定 (chest / waist の prev が上書きされる前に capture) →
  tracker 処理順に非依存。
- 参照する親が当該フレーム invalid なら transport 無効 → M4 の world 絶対 hold に degrade
  (腕は chest→waist の fallback あり)。
- split branch 限定なので rigid bone / foot / chest・waist は回帰ゼロ (fast path 不変)。

## Milestone

- **M1** (commit `6c419fc`): tracker_extract に hip 相対 hold + velocity gate + FK fallback
- **M2** (commit `fc5a3d6`): Kalman を hip 起点の kinematic chain に再構築
- **M3** (commit `9c31811`): design doc + track changelog
- **M4** (commit `f49cc0b`): swing/twist 分離による roll-only hold (脚・腕が向きに追従)
- **M5** (このコミット): parent-yaw transport で held roll を横向きに追従させる (腕→chest / 脚→waist)

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
- `test_tracker_extract` — M4 で挙動変更 + 1 ケース追加、全通過
  - `test_roll_hold_keeps_swing` (新規): `roll_confidence=0` / `swing_confidence=1` で
    swing が新 forward に追従しつつ、curr の roll に依らず結果が一致 (roll hold)。
    full confidence では roll を追従する対比も検証。
  - roll 縮退テスト (`test_thigh_standing_knee_straight` / `直座り` /
    `confidence_zero_all_degenerate`) を `valid=false` → `valid=true` +
    `roll_confidence=0` + forward 追従に更新。
  - `test_smoothing_freezes_under_low_confidence` を「両ゲート 0 で完全 freeze」に更新。
- `test_tracker_extract` — M5 で 2 ケース追加、全通過
  - `test_pelvis_yaw_transport_held_roll` (新規): 鉛直 forward の held-roll 脚 +
    waist が +30° yaw → 脚 up が pelvis yaw に乗って回る (`up → (-sin30, cos30, 0)`、
    forward は鉛直のまま)。waist invalid の control では transport 無効で prev roll 保持。
  - `test_arm_chest_leg_waist_transport` (新規): chest を +30°、waist を -40° と逆向きに
    yaw → held-roll の腕は chest (+30°)、脚は waist (-40°) にそれぞれ追従。参照が肢ごとに
    独立していることを検証。
- `test_kalman_chain` — M2 で新規、4 ケース全通過
  - root motion が unobserved child に伝播
  - child measurement で offset 補正
  - long missing → reset → 再 init
  - 親未初期化の child は skip
- PR #21 AI レビュー対応で 2 ケース追加 + 防御的修正、全通過
  - `test_pelvis_yaw_transport_no_overshoot` (`test_tracker_extract`): waist を一定 +30° yaw に
    保ち `alpha=0.5` で 24 フレーム → held 脚 up が `Θ`(30°) に収束し `2Θ`(60°) へオーバー
    シュートしないことを `min_dot=0.999` で検証。M5 の単フレームテストは `alpha=1` のため
    transport を `alpha_rate` でスケールしない回帰 (`Θ/alpha` まで回る) を見逃す。
  - `test_hip_dropout_clears_prev_hip_valid` (`test_tracker_extract_pos`): hip dropout 後の
    valid-hip フレームで stale な `prev_hip_pos` を使った re-anchor が起きないこと
    (`prev_hip_valid` を毎 tick 反映) を検証。
  - `SkeletonKalman` の防御的バウンズチェック (`i` をループ先頭で検査 / `parent` の範囲検査 /
    `ensure_topology` の `kMaxKeypoints` 超過で throw)。

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
