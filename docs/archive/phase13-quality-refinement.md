# Phase 13 — roll 品質詰め + WebUI tracker 可視化 + per-tracker stats

> **方針 (2026-05-24 着手 / 2026-05-25 締め)**: Phase 12 M1 で smoothstep-based confidence smoothing を入れた後でも、立位伸展時に大腿 / 脛 / 上腕が **一気に 90° 近く roll する** 症状が WebUI 上で目視確認された。Phase 13 では (a) 症状を定量化する **WebUI 上の tracker 向き可視化 + per-tracker stats** を作り、(b) そのデータで仮説検証してから (c) **degeneracy gate 自体の強化** + **secondary lateral pin / world-Z fallback の撤去** で対症する、という観察 → 仮説検証 → 構造修正のサイクルで進めた。
>
> **完了状態**: 4 commits で本流に乗る。Bridge relay 経由の位置情報送出は Phase 12 で没にしたまま、Phase 11 Firmware UDP 経路 (10 トラッカー、回転のみ) の上での品質改善に閉じる。
>
> - **Tier A/C のフル IK** (`docs/phase13-full-ik.md`) は backstop プランとして起票済 (commit `3613ade`)、Phase 14 候補。本 Phase 13 で扱う degeneracy gate / 1-stage 化が実用品質に達したため、Tier A M1 (swing-twist + ROM clamp) の取り込みは保留した
> - **M3 (max_freeze lifecycle)** は不採用。実機評価で `valid=false` 時の publisher skip + SlimeVR Server 側の前周期保持で実用上の不都合がなかったため

## Context

Phase 12 M1 で `tracker_extract.cpp` の up ベクトル多段選択 + confidence-modulated smoothing を入れた状態で実機評価を回したところ、立位 (= 脚を伸ばし切る) で:

1. **大腿 (UpperLeg) が一気に 90° roll する** — Phase 12 で「直座り (= 床に脚伸ばし) で primary が thigh 軸と並行 → freeze するはず」と設計したが、実際には **smoothstep の leakage zone (0 < confidence < 1)** で primary が weak pull を継続的に出し、累積 drift で 90° 飛んでいた。`leakage_pct = 100%`、`conf_avg = 0.15-0.31` で sustained。
2. **脛 (LowerLeg) も同じく 90° 回る** — shin は `roll_confidence = 1.0` 固定で smoothing modulation が無く、`quat_from_forward_up` の degeneracy 判定 `norm(cross) < 1e-6` は near-parallel に対してほぼ無効。`state = active`、`conf = 1.0` のまま `ang_vel p95 = 4-13 rad/s` で激しく暴れていた。
3. **上腕 (UpperArm) も同型** — 腕を水平に伸ばし切ると primary (wrist - elbow) が degenerate、secondary (neck - shoulder, = 胸郭 lateral) が full confidence で支配 → 上腕 roll が胸郭 yaw に rigid 共有 (体を捻ると腕も一緒に回る anti-pattern)。Phase 12 で大腿から撤去した lateral pin と同型。

ゴール:

**(A) 観察基盤の構築** — `cpp-migration-plan.md` の 検証戦略 Phase 11 までは数値スカラ (sent_rotations、 reproj_err 等) しか露出しておらず、「どの tracker がどの程度暴れているか」を見る術がなかった。WebUI 上に per-tracker AxesHelper + per-tracker rolling stats table を出して、肉眼 + 数値で評価できる土台を作る。

**(B) 構造修正** — (A) のデータで仮説確定後、degeneracy gate を sin θ-based に拡張 + 閾値を 8.6°/17.5° に引き上げ + 上腕の secondary/tertiary fallback を撤去。Phase 12 の thigh と同じ「1-stage primary + zero sentinel」パターンに揃える。

## アーキテクチャ

```
   Skeleton3DBus (既存)
     └ Halpe26 3D joints in world frame
                │
                ▼  (60 Hz)
   TrackerExtractor (新規 Phase 13 M1)
     ├ extract_trackers() — 10 SlimeTracker (world quat)
     ├ apply_quat_smoothing() — slerp with confidence-modulated alpha
     ├ per-tracker rolling stats (Phase 13 M2):
     │   * angular velocity rad/s p50/p95
     │   * roll_confidence avg, leakage_pct, freeze_pct
     │   * freeze_current_ms / freeze_max_ms / dropout_count
     └ SlimeTrackerBus::publish(trackers, stats)
                │
       ┌────────┴────────┐
       ▼                  ▼
   NativePublisher      CrowServer
   (Firmware UDP)       /ws3d, /stats3d JSON
     │ optional         │ embeds `trackers[]` fragment
     ▼                  ▼
   SlimeVR Server     Three.js viewer (WebUI)
                      ├ skeleton mesh
                      ├ per-tracker AxesHelper × 10 (向き / scale = confidence)
                      └ per-tracker stats table (state 色付き / leakage / freeze)
```

設計の肝:

- **TrackerExtractor が唯一の producer**: `extract_trackers + apply_quat_smoothing` の呼び出しは TrackerExtractor 1 箇所のみ。`NativePublisher` は smoothed 値を bus から consume するだけに refactor し、`prev_quat_` を移譲。WebUI viz と Firmware UDP 送信が **同じ smoothed history を共有** する設計
- **観察と修正が分離**: M1 + M2 を入れても挙動は変わらない (純粋に viz/stats のみ)。`18ef73e` (degeneracy gate) + `08140f7` (上腕 1-stage 化) が初めて挙動を変える commit
- **WebUI は Three.js infrastructure 既存活用**: AxesHelper / OrbitControls / 座標変換 (world Z-up → Y-up) は Phase 7 以前から実装済。10 個の per-tracker Group を scene に常設し、毎フレーム position + quaternion を代入

## トラッカー定義 (10 本構成、Phase 11 と同じ)

`extract_trackers()` の 10 本構成は Phase 11 / Phase 12 M1 から不変。Phase 13 で変更したのは **forward / up の構築ロジック** だけ:

| # | TrackerRole | forward | up (Phase 13 後) | Phase 12 M1 との差分 |
|---|---|---|---|---|
| 0,1 | Left/RightUpperArm | elbow − shoulder | **wrist − elbow のみ** (1-stage, fallback なし) | secondary (neck-shoulder) / tertiary (world Z) を Vec3f{0,0,0} sentinel に変更 |
| 2 | Chest | ⊥(shoulder_axis × spine) | neck − hip_center | 変更なし (rigid pin、roll_confidence=1.0 固定) |
| 3 | Waist | ⊥(hip_axis × spine) | spine | 変更なし |
| 4,5 | Left/RightUpperLeg | knee − hip | ankle − knee のみ | 変更なし (Phase 12 M1 で実装済) |
| 6,7 | Left/RightLowerLeg | ankle − knee | hip − knee | 変更なし。**degeneracy gate (sin θ < 0.15) で保護** されるようになった |
| 8,9 | Left/RightFoot | big_toe − heel | knee − ankle (tibia 軸) | 変更なし |

## マイルストーン

| M | 状態 | 内容 | 完了基準 |
|---|---|---|---|
| M1 | 完了 (`ba9ec2c`) | `SlimeTrackerBus` + `TrackerExtractor` 新規 / `NativePublisher` refactor (consume bus) / `Skeleton3DBus::make_bundle_json` に extra_fields_json 追加 / WebUI に per-tracker AxesHelper × 10 + `show trackers` toggle | C++ build + ctest pass、`./main --enable-3d --keypoint-format=halpe26 --port 8000` 起動でブラウザ上に 10 axes が表示 |
| M2 | 完了 (`71899c8`) | `SlimeTrackerStats` 構造追加 / per-tracker rolling stats (angular_velocity p50/p95、leakage_pct、freeze_pct、freeze_ms、dropout_count) を TrackerExtractor で計算 / bundle JSON / WebUI table に露出 | ctest pass、ブラウザで table が steady-state で更新、`#trackers-tbody` の 10 行に state 色 (active / leakage / frozen) が反映 |
| 修正 1 | 完了 (`18ef73e`) | `quat_from_forward_up` の degeneracy 判定を `sin θ < kRollSinLow` に変更。`kRollSinLow` 0.05 → **0.15**、`kRollSinHigh` 0.20 → **0.30** に引き上げ | ctest pass、実機評価で thigh / shin の立位伸展時 leakage_pct = 100% → freeze_pct = 100% に推移、ang_vel p95 < 1 rad/s |
| 修正 2 | 完了 (`08140f7`) | `upper_arm` ラムダの secondary (neck - shoulder) / tertiary (world Z) を `Vec3f{0,0,0}` sentinel に変更 (`upper_leg` と同じ 1-stage)、`joints_valid` から `kNeck` を外す。T-pose の wrist を Z=1.42 → 1.27 (forearm 12° drop) に変更して T-pose で primary 非 degenerate を維持 | ctest pass、実機評価で水平腕の upper arm が体を捻った時に rigid 共有しない (前周期 roll 保持)、ang_vel p95 < 1 rad/s |
| backstop | 完了 (`3613ade`) | `docs/phase13-full-ik.md` (Tier A swing-twist + ROM / Tier C Bullet ragdoll) を起票済として保存。Phase 13 で degeneracy gate 系統が実用品質に達したため、Tier A M1 の Phase 13 内取り込みは保留 | doc 単独で Phase 14 起点として使えるレベル |
| M3 (max_freeze lifecycle) | **不採用** | held → disconnected の 2 段階モデル + `--slimevr-tracker-max-freeze-ms` CLI フラグ | 実機評価で「`valid=false` で publisher skip → SlimeVR Server 側の前周期保持」が実用上問題なかったため見送り。必要になれば後日 |
| M5 docs | 完了 (本 commit) | `docs/phase13-quality-refinement.md` 新規 + `docs/cpp-migration-plan.md` の段階実装 / 検証戦略行追加 | doc review |

## 主要定数 (Phase 13 改訂後)

`cpp/src/slimevr/tracker_extract.cpp` のローカル定数:

```cpp
// Phase 12 M1 → Phase 13 修正後
constexpr float kRollSinLow  = 0.15f;  // sin 8.6° — degeneracy gate
constexpr float kRollSinHigh = 0.30f;  // sin 17.5° — full-confidence ceiling
// pick_up_multistage は sin θ < kRollSinLow を degenerate、
// sin θ > kRollSinHigh で confidence=1.0、間は smoothstep 中間域。
// 実用ポーズ (歩行 bend > 30° / しゃがみ > 60° / 着座 ~90°) は確実に full
// confidence 域、立位伸展 (knee bend 0-5°) は確実に freeze 域 になる。

constexpr float kFootSmoothingWeight = 0.3f;  // 変更なし (足は heel KP ノイズ
                                              // 回避のため固定 throttle)
```

`detail::quat_from_forward_up()` の degeneracy 判定:

```cpp
// Phase 13 修正: 旧 `norm(right_raw) < 1.0e-6f` (絶対しきい、ほぼ完全並行のみ)
//                ↓
// 新 `right_norm < kRollSinLow * up_norm` (sin θ-based 相対しきい)
//
// これで pick_up_multistage を経由しない rigid tracker (shin / foot / chest /
// waist) も同等に near-parallel から守られる。
float right_norm = norm(right_raw);
float up_norm    = norm(up_raw);
if (up_norm < 1.0e-6f || right_norm < kRollSinLow * up_norm) {
    out_wxyz = cv::Vec4f{1, 0, 0, 0};
    return false;
}
```

## /stats3d スプライス

Phase 11 で導入した `slimevr` ブロックに加え、Phase 13 で各 tracker のローリング統計を embedded として追加 (`make_tracker_bundle_fragment`):

```json
"trackers": [
  {"role": "LeftUpperArm",
   "pos": [x,y,z], "quat_wxyz": [w,x,y,z], "valid": true,
   "roll_confidence": 0.95,
   "stats": {
     "ang_vel_p50": 0.27, "ang_vel_p95": 0.57,
     "conf_avg": 1.00,
     "leakage_pct": 0.0, "freeze_pct": 0.0,
     "freeze_current_ms": 0, "freeze_max_ms": 0,
     "dropouts": 0
   }},
  ...
],
"tracker_stats_window_frames": 120
```

window は `TrackerExtractorOptions::stats_window` で制御可能、default 120 frames (= 2 s at 60 Hz)。

## 検証

### 単体 (実機不要)

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
ctest --test-dir cpp/build --output-on-failure -R 'tracker_extract|firmware_protocol'
# 全 21 ケース pass:
#   test_quat_from_forward_up_degenerate
#   test_t_pose_extracts_all_ten (T-pose 膝 10 cm 前 / 手首 15 cm 下 で全 10 valid)
#   test_thigh_seated_extended_straight_knee (直座り freeze)
#   test_thigh_standing_knee_straight (立位 freeze)
#   test_thigh_walking_knee_30 (60° flex で primary active)
#   test_thigh_lateral_ankle_uses_primary (横ずれ 15 cm で primary active)
#   ...
```

### 統合 (Jetson 単体、Windows 機不要)

```bash
./cpp/build/main --enable-3d --keypoint-format=halpe26 --port 8000 \
    --cam0 /dev/v4l/by-path/... --cam1 /dev/v4l/by-path/... \
    --calib calibrations/<id>/calibration.yaml \
    --det-engine engines/yolox.engine --pose-engine engines/rtmpose.engine
```

ブラウザで `http://<jetson-ip>:8000/`:

1. 3D viewer に 10 個の AxesHelper が表示される (R/G/B = X/Y/Z 軸)
2. 被験者の動きに axes が追従、`show trackers` checkbox で ON/OFF 切替
3. `#trackers-tbody` table が 30 Hz で更新
4. 立位伸展で thigh / shin / upper arm 4 軸が `state=frozen` (赤背景)、`ang_vel p95 < 1 rad/s`
5. 歩行 / しゃがみで全 10 行が `state=active`、`ang_vel p95 < 2 rad/s`
6. しゃがみ ↔ 立位の遷移で state が **smooth に切替** (oscillation なし)

### E2E (Phase 11 Firmware UDP 経路、Windows SlimeVR 機が必要)

```bash
./cpp/build/main --enable-3d --keypoint-format=halpe26 \
    --slimevr-out --slimevr-host=<windows-ip> \
    --cam0 ... --cam1 ... --calib ... --det-engine ... --pose-engine ...
```

SlimeVR Server GUI で 10 トラッカーが名前付き表示、avatar が:

- 立位伸展で脚 / 腕が突然 90° 回らない
- 体を捻ったときに水平腕が胸郭と一緒に rigid 共有で回らない (前周期 roll 持続)
- しゃがみ → 立位の遷移で smooth (snap なし)
- 歩行 / つま先立ち / 直座り (= 床に脚伸ばし) で破綻なく追従

## 完了条件

1. M1: `./main --enable-3d --port 8000` で WebUI に 10 axes + stats table 表示 ✓ (`ba9ec2c`)
2. M2: stats が steady-state で更新、leakage/freeze の WebUI ラベルが意味のある値を返す ✓ (`71899c8`)
3. 修正 1: 立位伸展時の thigh / shin の 90° snap が消え、`ang_vel p95 < 1 rad/s` ✓ (`18ef73e`、実機確認)
4. 修正 2: 水平腕の upper arm の 90° snap + 胸郭 rigid 共有が消え、`ang_vel p95 < 1 rad/s` ✓ (`08140f7`、実機確認)
5. `docs/cpp-migration-plan.md` の段階実装行 / 検証戦略行に Phase 13 が反映される (CLAUDE.md Phase completion 規約) ✓ (本 commit)

## リスク・残課題

| ID | 内容 | 対応 |
|---|---|---|
| R1 | `kRollSinLow = 0.15` (sin 8.6°) がチューニング前の暫定値。実機シーンによっては「歩行 swing 中の primary も時々 leak する」可能性 | M2 stats で歩行中の各 tracker `leakage_pct` を継続観察、5% 超なら 0.10 への引き下げを検討 |
| R2 | hysteresis 未実装。`sin θ` が `kRollSinLow` 付近で行き来する条件で valid ↔ invalid フリッカが起きる可能性 | 実機評価で oscillation 確認できれば後追いで実装 (Schmitt trigger: invalid から復帰には sin > kRollSinHigh 要求) |
| R3 | foot の `state=leakage 100%` 誤表示。`kFootSmoothingWeight=0.3` は intentional throttle で degeneracy leakage ではないが、WebUI table が区別できていない | cosmetic、Phase 14 で is_throttled flag を追加して state 表示を分離 |
| R4 | M3 (max_freeze lifecycle) を不採用としたが、長時間 occlusion で SlimeVR Server 側の挙動 (avatar が固まる / 暴れる / disconnect マーク) が変な場合は再起動候補 | 実運用で 5 秒以上のオクルージョンを試して挙動評価。問題あれば `docs/phase13-full-ik.md` の Tier A 構想と合わせて Phase 14 で扱う |
| R5 | `docs/phase13-full-ik.md` の Tier A/C はまだ概念段階。Phase 14 で着手するなら、Bullet の Jetson Orin Nano Super での実測ベンチを Tier C 着手前に必ず挟む | Tier C 着手時の検証ステップに明記済 |

## 関連ドキュメント

- [`phase13-full-ik.md`](phase13-full-ik.md) — backstop プラン (Tier A swing-twist + ROM / Tier C Bullet ragdoll)。本 Phase 13 で取り込まなかったが Phase 14 候補
- [`phase12-slimevr-bridge-relay.md`](phase12-slimevr-bridge-relay.md) — Phase 12 M1 (confidence-modulated smoothing) の前提設計。Phase 13 修正 2 (上腕 1-stage 化) で同 doc の Upper Arm セクションを更新
- [`phase11-slimevr-integration.md`](phase11-slimevr-integration.md) — Phase 11 Firmware UDP 経路 (本 Phase 13 が乗る本流)
- [`cpp-migration-plan.md`](../cpp-migration-plan.md) — 全体ロードマップと検証戦略 (Phase 13 行追加済)

## 参照リソース

- `cpp/src/slimevr/tracker_extract.{cpp,hpp}` — extract_trackers + apply_quat_smoothing
- `cpp/src/slimevr/slime_tracker_bus.{cpp,hpp}` — Phase 13 M1 新規、tracker snapshot atomic store + JSON fragment 生成
- `cpp/src/slimevr/tracker_extractor.{cpp,hpp}` — Phase 13 M1 新規、Skeleton3DBus → SlimeTrackerBus の producer thread + stats 計算
- `cpp/src/slimevr/native_publisher.{cpp,hpp}` — Phase 13 M1 で SlimeTrackerBus consume 化に refactor
- `cpp/src/web/crow_server.{cpp,hpp}` — Phase 13 M1 で `set_tracker_bus()` + bundle JSON への trackers fragment 注入
- `cpp/src/pipeline/snapshot.{cpp,hpp}` — `Skeleton3DBus::make_bundle_json` に `extra_fields_json` 引数を追加
- `web/dual_rtmpose/app.js` — per-tracker AxesHelper × 10 + stats table 更新
- `cpp/tools/test_tracker_extract.cpp` — Phase 13 で T-pose の膝 10 cm 前 / 手首 15 cm 下 / 上腕テストの sin θ 値更新
