# Track: pose-3d

2D keypoint から **3D pose / bone tracker** を起こす経路。lift / IK / Kalman / roll 品質 /
subject calibration。vr-output トラックの上流 (= tracker の単一 producer) を担う。

## 現状 (2026-05-29)

`SlimeTrackerBus` + `TrackerExtractor` が tracker snapshot の **単一 producer**。
Firmware UDP / VMT publisher / WebUI viz が同じ smoothing 履歴を共有する。Kalman は
**kinematic-tree (root = hip_center, children = parent-relative offset)** で動く。

### 設計原則 / live な制約

- **degeneracy gate は相対しきい**: `quat_from_forward_up` の degeneracy 判定は `sin θ`
  ベース (`kRollSinLow=0.15` / `kRollSinHigh=0.30`)。絶対 norm しきいは使わない。
  primary が degenerate になる向き (水平腕・伸展脚) では **roll (twist) だけ**を hold する
  (向き = swing は追従させる。下記 swing/twist 分離参照)。
- **swing/twist 分離スムージング**: `apply_quat_smoothing` は相対回転を bone forward
  (local +Z) で swing (pitch/yaw) と twist (roll) に分解し、独立 alpha で slerp。roll 縮退時は
  `roll_confidence=0` で twist を前フレーム保持しつつ swing は満額追従するので、伸展した脚・腕
  でも bone の向きが freeze しない。`swing_confidence == roll_confidence` の時は単一 slerp の
  fast path (rigid bone / foot はビット同一)。
- **roll 縮退は valid=false にしない**: forward が有効で up hint だけ縮退した bone は
  `build_tracker` が forward-only quat + `roll_confidence=0` で `valid=true` を返す
  (真の欠損 = forward 縮退のみ `valid=false`)。代用 up の roll 値は twist alpha=0 で捨てられる。
- **lateral pin anti-pattern**: secondary lateral pin (neck-shoulder 等) で roll を稼ぐ構造は
  「立位伸展で 90° roll が一気に入る」症状の原因。`upper_arm` / `upper_leg` を同型の 1-stage 構造に
  揃え、tertiary を `Vec3f{0,0,0}` sentinel にして roll hold へ倒す経路を確立済み。
  world-Z で roll を代用する案は「膝裏が天井向き」の捏造 roll を生むため不採用。
- **smoothing の state 所有は TrackerExtractor に集約**: 回転 (`prev_quat_`) も位置 EMA
  (`apply_pos_smoothing`) も同じ場所で持つ。publisher 側に smoothing state を分散させない。
- **held roll は parent-yaw transport**: roll を hold 中の伸展肢 (split branch) は、観測可能な
  親 tracker の orientation 変化 `D=P_curr·P_prev⁻¹` を prev に左から掛けて transport する。
  立位伸展で bone forward が鉛直のとき体 yaw は bone 軸まわり回転 = roll に一致し、world 絶対
  hold だと「横を向くと向きが固まる」。swing が forward を観測値に再整合するので forward 軸成分
  (鉛直肢では yaw) だけが roll に残る。これは**差分結合** (M1 hip 相対 hold の回転版) で、却下した
  rigid parent pin (絶対結合) とは別。**参照は肢ごと**: 腕は chest (肩甲帯)、脚は waist (骨盤)。
  骨盤は脊椎の捻りで胸と独立に yaw するので腕に骨盤は使わない (腕は chest 不在時のみ waist へ
  fallback)。`carry=1-ta/sa` で観測と prior を相補ブレンド、`kPelvisYawGate 8–16 rad/s` で親の
  yaw 推定が暴れる横向き局面の暴走 delta を減衰。
- **位置 hold は hip 相対**: `valid=false` の tracker は world 絶対値で freeze せず、
  hip_center 相対 offset を保って current hip にプロジェクトする。立位伸展で 2D 検出が
  motion blur で落ちても足が世界座標に取り残されない。Waist は `prev_pos ≡ hip_center` で
  offset ≈ 0 なので自然に hip 追従。
- **velocity gate は consecutive raw delta で測る**: 位置 EMA の outlier 検出は `prev_pos`
  (EMA 平均) ではなく `last_raw_pos` (前周期生 curr) と curr の delta で行う。EMA 収束途中の
  遅延を outlier と誤判定しないため。8–16 m/s smoothstep で alpha を attenuate。
- **FK fallback は足限定 + real-frame anchor**: ankle/big_toe が単発で落ちた瞬間は
  `FootAnchor` (knee→ankle dir + tibia 長, ankle→toe dir + foot 長) から再合成。Anchor の
  更新は fully measured フレームに限定し、合成中の dir で自己 drift させない。
- **Kalman は kinematic-tree**: root joint (hip_center under Halpe26 / l_hip under COCO17) は
  world 6D state、それ以外は parent-relative offset 6D state。出力は `world = parent_world + offset`
  の FK 再構成。hip 移動が child の world に自然に伝播する (per-joint 独立は廃止)。
  Process noise は root と offset で分離 (`q_pos` / `q_pos_offset`)。
- **subject profile schema は厳格分離**: `fitra_subject_profile_v1` (COCO17) と `v2` (Halpe26) は
  マイグレーションせず再キャリブを要求 (keypoint topology は core-pipeline トラック参照)。
- **フル IK は backstop 設計のみ**: Tier A swing-twist + ROM clamp + 角速度 clamp + constrained
  Kalman / Tier C Bullet ragdoll の設計メモは起票済だが、degeneracy gate + chain Kalman が
  実用品質に達したため取り込みは保留。→ [archive/phase13-full-ik.md](../archive/phase13-full-ik.md)

### 検証

`ctest -R 'tracker_extract|firmware_protocol|kalman'` (29 ケース) + 実機目視 (WebUI の
per-tracker AxesHelper×10 / `#trackers-table` の state 色分け、`/stats3d`)。
合格基準は [`cpp-migration-plan.md` 検証戦略表](../cpp-migration-plan.md) の旧 Phase 13 行に
加え、立位伸展 1m 横移動で foot tracker world 移動量 ≥ 0.7m / `freeze_pct` baseline +5pp 以内。

## Changelog (新しい順)

### 2026-05-29 — parent-yaw transport (横向き時の伸展肢 roll 追従)
M4 (roll-only hold) 後の実機報告「伸展状態で xyz 移動は OK だが回転がだめ、特に横を向いたとき」に
対応。立位伸展で bone forward が鉛直になると体 yaw が bone 軸まわり回転 = roll に一致し、world 絶対
hold だと横向きで向きが固まる。`apply_quat_smoothing` のループ前に親 tracker の orientation 変化を
計算し、roll を hold 中の split-branch tracker の prev に左から transport (M1 hip 相対 hold の
回転版 = 差分結合)。swing が forward を再整合するので親の pitch/roll は吸収され yaw 成分だけが
roll に残る。**参照は肢ごと** — 腕は chest (肩甲帯)、脚は waist (骨盤)。骨盤は脊椎の捻りで胸と
独立に yaw するので腕に骨盤は使わない。`carry=1-ta/sa` で観測との相補ブレンド、
`kPelvisYawGate 8–16 rad/s` で横向き時の親 yaw 推定暴走を減衰。fast path (rigid bone / foot /
chest・waist) はビット同一で回帰ゼロ。
→ [design/pose-3d-locomotion-stability.md](../design/pose-3d-locomotion-stability.md) M5

### 2026-05-29 — roll-only hold (脚・腕が向きに追従)
M1 の hip 相対 hold で足の*位置*は hip 追従するようになったが、立位伸展で足先は動くのに
太もも・すね・上腕の bone が回らない症状が残っていた。roll 縮退時に bone の向きごと freeze
していたのが原因。`apply_quat_smoothing` を swing/twist 分離に書き換え、`roll_confidence` を
twist 専用ゲートに、`swing_confidence` を新設。roll が測れない伸展肢でも swing (pitch/yaw) は
追従し twist (roll) だけ前フレーム保持する。`swing_confidence==roll_confidence` で従来の単一
slerp に縮約する fast path で rigid bone / foot は回帰ゼロ。
→ [design/pose-3d-locomotion-stability.md](../design/pose-3d-locomotion-stability.md) M4

### 2026-05-28 — locomotion stability (足置き去り解消 + chain Kalman)
立位伸展で胴体を動かしたときに足 tracker が world に取り残される症状を 3 層 (tracker_extract /
Kalman / IK) のうち上 2 層で解決。tracker_extract に hip 相対 hold + velocity gate + 足限定
FK fallback を追加し、Kalman を per-joint 独立から hip 起点の kinematic-tree (root world +
child parent-relative offset) に再構築。
→ [design/pose-3d-locomotion-stability.md](../design/pose-3d-locomotion-stability.md)

### 2026-05-24〜25 — roll 品質詰め + WebUI tracker 可視化 + per-tracker stats
「観察基盤を先に作る → データで仮説確定 → 構造修正」の順で立位伸展時の 90° roll を解消。
`SlimeTrackerBus` + `TrackerExtractor` を新設し publisher を consumer に refactor。
degeneracy 判定を相対しきい (sin θ) 化、upper_arm を 1-stage 構造に統一。
WebUI に AxesHelper×10 + per-tracker rolling stats (ang_vel p50/p95, freeze_pct 等)。
→ [archive/phase13-quality-refinement.md](../archive/phase13-quality-refinement.md)

### 2026-05-22 — roll 品質改善 M1 (confidence-modulated smoothing)
`tracker_extract.cpp` の二の腕 / 大腿 / 足の up を多段選択 + confidence-modulated smoothing に
書き換え。二の腕ひねり症状の解消 + 腕完全伸展時の roll twist 振動収束を実機確認。
(同 phase の Bridge relay 経路は vr-output トラックで没。)
→ [archive/phase12-slimevr-bridge-relay.md](../archive/phase12-slimevr-bridge-relay.md)

### 〜2026-05-20 — IK pose calibration + subject profile
IK ベースの pose calibration と subject profile (体格パラメータ) 永続化。
→ [archive/phase8-ik-pose-calib.md](../archive/phase8-ik-pose-calib.md) /
  [archive/phase8-subject-profile-runbook.md](../archive/phase8-subject-profile-runbook.md)

### 初期 — 3D IK + Kalman lift
2D keypoint → 3D lift の IK + Kalman フィルタ基盤。
→ [archive/phase7-3d-ik-kalman.md](../archive/phase7-3d-ik-kalman.md)
