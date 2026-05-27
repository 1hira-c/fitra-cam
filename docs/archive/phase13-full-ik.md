# Phase 13 候補設計メモ — IK 完全化(twist 制御 + バイオメカ制約 + 物理 ragdoll)

> **ステータス (2026-05-23 起票)**: 調査段階。本書は Phase 13 着手時の起点となる方向性メモ。`docs/phase13-full-ik.md` への正式昇格は後日。
>
> **推奨**: Tier A(制約付き smoothing 層)→ Tier C(Bullet 物理 ragdoll prototype)の段階構築。Tier B(FABRIK chain solver)は SlimeVR の IK を借りられている間は不要と判断、保留。

## Context

Phase 12 M1 で roll の up ベクトル多段選択 + confidence-modulated slerp smoothing まで実装したが、M2-M7(SlimeVR Bridge relay 経由で位置情報を送って IK に効かせる経路)は SteamVR と `\\.\pipe\SlimeVRInput` の排他問題で凍結。M1 のみで Phase 12 を締めた状態 ([[project-phase12-bridge-frozen]] 参照)。

現状の IK は `cpp/src/slimevr/tracker_extract.cpp` の **forward/up ベクトルから quaternion を直接構築する解析的方法** + `cpp/src/lift/ik.cpp` の **骨長制約 + hinge clamp (5°-175°)** が全て。具体的に欠けているもの:

1. **twist 軸が独立自由度として扱われていない** — forward × up の外積で自動決定されるため、肘内転や手首ひねりが解像されない
2. **関節可動域 (ROM) 制約が hinge のみ** — 肩 ball-and-socket、腰 saddle のような複合関節モデルが無い
3. **角速度 / 角加速度の物理制約が無い** — confidence 低下時の飛び・暴れに対して slerp α の調整のみで対抗
4. **chain solver が無い** — 各 bone が独立に計算され、子関節の整合は SlimeVR 側 IK に依存

ユーザー指摘の通り、これら 4 つを統合した「フル IK」は学術・産業の両方で確立されている領域(PhysCap, SimPoE, OpenSim AUKS, KHMP, Final IK 等)。本メモはそれを fitra-cam に段階的に組み込む方針案。

Jetson Orin Nano Super の負荷見積もり(別途確認済):
- Bullet (CPU) 1 ragdoll → 推定 **0.3-1.0 ms/step**(古い CPU で 4 ragdoll = 2ms ベンチから外挿)
- 既存パイプライン(YOLOX + RTMPose 2-3cam, TRT FP16)= 10-15 ms / frame
- 60Hz = 16.6 ms / frame 内に物理含めて収まる余裕あり
- **Bullet (CPU)** を採用、PhysX GPU は推論 GPU と競合するため不採用

## 全体方針: Tier A → Tier C 段階実装

### Tier A(M1-M3): 制約付き smoothing 層

既存 analytical orientation はそのまま、その**後段**に制約レイヤを差し込む。chain solver には触れない。

最小投資で「飛び抑制 + 人体の物理らしさ」を獲得する。Tier C に進まなくてもこの時点で実用品質は大幅改善するため、Phase 13 単体の到達点としても成立。

### Tier C(M4-M6): Bullet 物理 ragdoll prototype

PhysCap 風の **inner-kinematic IK → outer-physics correction** 二段構成。Bullet (CPU) で 1 ragdoll を別 thread で回し、Tier A 出力を観測(target)として PD controller 経由で物理整合 quaternion を吐く。

### M7: 安定性評価 + 本採用判断

Tier C が Jetson 上で安定 60Hz を維持し、定性的に「人体らしさ」が向上していれば本流昇格。難航すれば Tier A 締めで Phase 13 完了とし、Tier C は次フェーズに送る。

## マイルストーン

### M1 — swing-twist 分解 + ROM clamp

`tracker_extract.cpp` の出力 quaternion を `q = q_swing ⊗ q_twist` に分解し、各成分に解剖学的制約をかける。

- **swing**: bone direction を z 軸として cone 制約(肩 60°、肘 120° pitch、腰 30° 等の解剖学的値)
- **twist**: 1DOF の hinge limit(前腕 ±90°、上腕 ±60° 等)
- 制約値は新規 `cpp/src/slimevr/joint_constraints.{hpp,cpp}` に集約、tracker_id → limit のテーブル

`apply_quat_smoothing()` の **前段**に挿入。smoothing は制約後の値に対して適用。

### M2 — 角速度 / 角加速度 clamp

前周期 quaternion との差分 `Δq` から角速度 ω を推定、上限超過時 clamp。さらに前周期 ω との差分から角加速度 α を推定、これも clamp。

- 上限値は文献的な人体最大角速度(肩 ~360°/s, 肘 ~720°/s, 手首 ~1500°/s 等)を `joint_constraints.cpp` に同居
- confidence 連動で上限を縮める(`max_omega *= confidence` で低 confidence 時はより厳しく制限)

### M3 — confidence-aware constrained Kalman filter

per-tracker quaternion 状態空間モデル。状態 `x = (q, ω)`、観測 `z = q_observed`。

- measurement noise covariance を `1/confidence` でスケール
- process noise を角加速度上限から逆算
- 新規 `cpp/src/slimevr/quat_kalman.{hpp,cpp}`
- M2 の clamp は Kalman の hard constraint として組み込み(constrained UKF / Lie-group EKF アプローチ)

ここまでが Tier A の終端。**この時点で「twist 独立 + ROM + 速度/加速度 + confidence 連動」がすべて揃う。**

### M4 — Bullet 統合(ragdoll セットアップ)

Bullet 3.x を `cpp/CMakeLists.txt` の FetchContent に追加(header-only ではないが軽量ビルド)。

- 新規 `cpp/src/ik/bullet_ragdoll.{hpp,cpp}` — 14-15 個の `btCapsuleShape` + `btConeTwistConstraint` / `btGeneric6DofSpring2Constraint`
- 骨長は既存 `cpp/src/lift/ik.cpp` の SubjectProfile から共有
- ROM 値は M1 で定義した `joint_constraints` を再利用
- Bullet は **CPU の別 thread** で 60Hz step、推論 thread とはロックフリーで状態交換

### M5 — PD controller + 観測駆動

各関節に PD controller を設置。Tier A 出力(constrained Kalman の filtered quaternion)を target、Bullet の現在 quaternion を current として `torque = Kp·(target − current) − Kd·ω`。

- stiffness `Kp` / damping `Kd` は解剖学的に詰める(参考: SlimeVR の `LegTweaks.kt`、PhysCap 論文の Table 2)
- 2D keypoint 信頼度を Kp 倍率に反映(低 conf joint は緩く targeting → 物理慣性が支配)

### M6 — PhysCap 風 inner-IK / outer-Sim 統合

kinematic refine(Tier A)→ physics step(Bullet)→ output quaternion の 2 段パイプラインを完成。

- ground contact / floor 制約を `btStaticPlaneShape` で追加(foot-plant、貫通防止)
- Bullet 出力 quaternion を最終 SlimeVR tracker 出力として `tracker_extract.cpp` の publisher に渡す
- M3 の Kalman 状態は Bullet の物理状態で再初期化(divergence 防止)

### M7 — 安定性 + 遅延評価、本採用判断

- `tools/pose_bench.cpp` を拡張、3 cam × IK 込みの latency / fps を実測
- `outputs/recorded_rtmpose/20260515_064342/raw_cam{0,1}.mp4` で定性確認(roll 安定 + 飛び抑制 + foot slide 減少)
- 不採用なら Tier A までで `docs/phase13-full-ik.md` を締める

## 重要ファイル

### 既存(参照 + 改修)

- `cpp/src/slimevr/tracker_extract.{hpp,cpp}` — analytical orientation の出口。M1-M3 で後段にレイヤ追加、M6 で Bullet 出力に差し替え
- `cpp/src/lift/ik.{hpp,cpp}` — 骨長 + hinge。M4 で SubjectProfile 経由で骨長共有
- `cpp/src/lift/skeleton_def.{hpp,cpp}` — Halpe26 / COCO17 topology。ROM 定義に活用
- `cpp/src/main.cpp` — CLI option 追加(`--full-ik={off,a,c}` 等)
- `python/scripts/pose_pipeline.py` — Tier A 部分の Python reference 実装場所(数値整合用)

### 新規

- `cpp/src/slimevr/joint_constraints.{hpp,cpp}` — ROM テーブル、swing-twist 分解、角速度上限(M1, M2)
- `cpp/src/slimevr/quat_kalman.{hpp,cpp}` — constrained Kalman(M3)
- `cpp/src/ik/bullet_ragdoll.{hpp,cpp}` — Bullet ラッパ、ragdoll セットアップ、PD controller(M4-M6)
- `docs/phase13-full-ik.md` — Phase 12 と同じ体裁で起こす(着手時)

### 依存追加

- Bullet 3.x via `FetchContent`(M4 以降のみ。Tier A 単独完了時は不要)

## 検証

- **既存 eval ビデオ**(`outputs/recorded_rtmpose/20260515_064342/raw_cam{0,1}.mp4`)で定性的に roll 安定 + 飛び抑制 + foot slide 減少を確認
- **Python reference との数値整合**: Tier A は Python 側で再実装して bit-level 整合確認(Phase 1 の `tools/correctness_check` 同様)。Tier C は数値一致ではなく **physical plausibility metric**(joint velocity / accel histogram、foot-slide RMS、ground penetration)で評価
- **Jetson 60Hz リアルタイム維持**: `tools/pose_bench.cpp` を拡張、3 cam × IK 込みの latency / fps を実測。3 cam 時 30Hz fallback を許容するかは M7 で判断
- **SlimeVR 連携**: Phase 11 の Firmware UDP 経路で実機 SlimeVR Server に送り、avatar 描画で破綻が出ないこと(発散・テレポート無し)を確認

## Phase 計画書との連動

着手時に以下を実施:

- `docs/phase13-full-ik.md` を Phase 12 ドキュメントの体裁(Context / アーキテクチャ / トラッカー定義 / マイルストーン / 完了条件)で起こす
- `docs/cpp-migration-plan.md` の「段階実装」「検証戦略」両表に Phase 13 行を追加(完了判定はこの 2 表更新まで含む — CLAUDE.md "Phase completion" 規約)
- ブランチは現在の `cpp-phase13` をそのまま使用、M ごとに 1 commit を原則

## 参考文献

- PhysCap (Shimada et al. 2020) — https://arxiv.org/abs/2008.08880
- SimPoE (Yuan et al. 2021) — https://arxiv.org/abs/2104.00683
- D3L (2023) — https://arxiv.org/abs/2306.06406
- SMPL-IKS (IJCV 2025) — https://link.springer.com/article/10.1007/s11263-025-02574-5
- KHMP (2026) — https://arxiv.org/pdf/2603.21327
- Bullet ragdoll benchmark — https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=3930
- Final IK / VRIK TwistRelaxer — http://root-motion.com/finalikdox/html/index.html
- SlimeVR `IKSolver.kt` / `LegTweaks.kt`(`refs/slimevr/SlimeVR-Server/server/core/src/main/java/dev/slimevr/tracking/processor/skeleton/`)
