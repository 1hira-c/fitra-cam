# Track: pose-3d

2D keypoint から **3D pose / bone tracker** を起こす経路。lift / IK / Kalman / roll 品質 /
subject calibration。vr-output トラックの上流 (= tracker の単一 producer) を担う。

## 現状 (2026-05-28)

`SlimeTrackerBus` + `TrackerExtractor` が tracker snapshot の **単一 producer**。
Firmware UDP / VMT publisher / WebUI viz が同じ smoothing 履歴を共有する。Kalman は
**kinematic-tree (root = hip_center, children = parent-relative offset)** で動く。

### 設計原則 / live な制約

- **degeneracy gate は相対しきい**: `quat_from_forward_up` の degeneracy 判定は `sin θ`
  ベース (`kRollSinLow=0.15` / `kRollSinHigh=0.30`)。絶対 norm しきいは使わない。
  primary が degenerate になる向き (水平腕・伸展脚) では roll を freeze に倒す。
- **lateral pin anti-pattern**: secondary lateral pin (neck-shoulder 等) で roll を稼ぐ構造は
  「立位伸展で 90° roll が一気に入る」症状の原因。`upper_arm` / `upper_leg` を同型の 1-stage 構造に
  揃え、tertiary を `Vec3f{0,0,0}` sentinel にして freeze へ倒す経路を確立済み。
- **smoothing の state 所有は TrackerExtractor に集約**: 回転 (`prev_quat_`) も位置 EMA
  (`apply_pos_smoothing`) も同じ場所で持つ。publisher 側に smoothing state を分散させない。
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

`ctest -R 'tracker_extract|firmware_protocol|kalman'` (27 ケース) + 実機目視 (WebUI の
per-tracker AxesHelper×10 / `#trackers-table` の state 色分け、`/stats3d`)。
合格基準は [`cpp-migration-plan.md` 検証戦略表](../cpp-migration-plan.md) の旧 Phase 13 行に
加え、立位伸展 1m 横移動で foot tracker world 移動量 ≥ 0.7m / `freeze_pct` baseline +5pp 以内。

## Changelog (新しい順)

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
