# vr-output: 自動・半継続 HMD キャリブレーション

(着手日 2026-05-29 / 関連トラック: vr-output, pose-3d /
前提: [archive/phase15-vmt-hmd-auto-align.md](../archive/phase15-vmt-hmd-auto-align.md))

## 背景 / 動機

VMT alignment(カメラ世界座標 → SteamVR/VMT 世界座標の整合)は Phase 15 で
HMD pose 駆動の自動算出を入れたが、**単発操作**だった:

- `solve_tpose`(瞬時)/ `solve_motion`(3 秒歩行、cv::SVD 2D Procrustes、xz のみ)を
  Web UI ボタンで叩く。`cpp/src/vmt/auto_alignment.{hpp,cpp}`。
- 対応点は **chest tracker**(`SlimeTrackerBus`)と HMD。chest は HMD と物理位置が
  レバーアーム分ずれており、剛体 2D Procrustes では吸収しきれず residual に乗る。
- SLAM ドリフト・部屋の再センタリングで時間とともにズレるが、ユーザーが都度
  キャリブし直す必要があった。

**目的**: 起動時から HMD と「信頼性高く報告された頭部位置」を継続サンプリングし、
代表値を蓄積して alignment を**自動・半継続的に**収束・追従させる。完了基準は
「手動 T ポーズ無しで直立 → 軽く動くだけで yaw+xz が自動収束し、ドリフトにも
緩く追従し、ジャンプが出ない」こと。

確定方針(ユーザー判断):常時バックグラウンド / 頭部優先・chest フォールバック /
Y は手動 slider 維持 / 首ボーン垂直性スコアはキャリブのサンプル品質のみに適用。

## 検討した案

- **案A: 既存 3 秒モードを定期再トリガするだけ** — 実装は最小だが、固定窓・固定
  しきい値で品質の悪いフレームも一様に混ざる。直線軌道だと degeneracy で no-op。
  ドリフト追従も窓の切り替わりで不連続。→ **没**(品質ゲートと連続性が無い)。
- **案B: 全身 IK ベースで毎フレーム外部姿勢を最適化** — 高精度だが重く、SLAM/IK
  双方の誤差が結合して発散しやすい。HMD–頭部の 2D 対応で足りる要求に対し過剰。
  → **没**(コスト過大・要求過剰)。
- **案C(採用): 常時バックグラウンドの品質ゲート付き reservoir + 既存 solve_motion
  + clamped EMA**。対応点を chest 固定から「頭部優先・chest フォールバック」に変更し、
  HMD と同一物理点に近づけてレバーアーム残差を削減。サンプル品質の主要因に
  **首/脊椎ボーンの垂直性**(直立フレームほど頭部–HMD 対応がクリーン)を採用。
- 対応点について **chest 固定のまま継続化** も検討したが、レバーアーム残差が
  恒常的に残るため頭部優先に。頭部が不安定(背向き・遮蔽)な間のみ chest に
  フォールバックする二段構えとした。
- 垂直性スコアの適用先を **tracker の roll_confidence にも波及** させる案もあったが、
  出力姿勢全体への影響が読みづらく、今回はキャリブのサンプル選別に限定(残課題)。
- **Y(高さ)自動推定**(頭頂 vs HMD)は個人差が大きく、Phase 15 同様 0 固定 +
  手動 slider のまま据え置き(残課題)。

## 採用設計

新規バックグラウンドコンポーネント `ContinuousAligner`(`cpp/src/vmt/continuous_aligner.{hpp,cpp}`、
`fitra_vmt`)。既存プリミティブを read-only consumer として束ねる。

### データフロー(背景スレッド、`sample_hz` 既定 15Hz)

1. `HmdPoseBus::snapshot(stale_ms)` → `valid && !stale` でなければ skip。
2. `Skeleton3DBus::snapshot()` の先頭 person から head_top(17)/neck(18)/hip_center(19)
   を読む(`pipeline/snapshot.hpp`)。**trackers ではなく生 skeleton を参照**
   する理由は、HMD と同一点である頭部が tracker 層では省かれているため
   (`tracker_extract.hpp`: HEAD は HMD 提供なので意図的に非抽出)。
3. **対応点選択**(`make_sample`):
   - `head_top.valid && head_top.score ≥ head_conf_thresh(0.5)` → 頭部採用
   - else neck/hip_center 有効 → chest 中点 `(neck+hip_center)/2`
   - neck か hip_center が欠ける → `CorrSource::None`(垂直性基準も作れないので破棄)
   - body 点は `world_pos_to_vmt` で VMT frame へ。**world Z-up → VMT y(高さ)** なので
     Procrustes が走る地平面は world(x,y)→VMT(x,z)。
4. **サンプル品質** = `conf · vert · vel`(全て [0,1]):
   - `vert` = **脊椎/首ボーン(neck−hip_center)の垂直性**。`verticality_score` =
     `ramp(tilt_deg, vert_zero_deg(40), vert_full_deg(15))`、`tilt = acos(|unit.z|)`。
     直立(tilt 小)ほど 1。これがユーザー要望「首ボーンが地面に垂直なほど高得点」。
   - `conf` = 頭部採用時 head_top.score / chest 時 min(neck,hip score)。
   - `vel` = `ramp(hmd_speed, vel_zero(1.5), vel_full(0.3))`。準静止ほど 1
     (高速移動中のフレーム積分ブレを soft に減衰。ハード reject はしない)。
   - `quality < quality_thresh(0.25)` は admit しない。
5. **空間 reservoir**(`SampleReservoir`): HMD.xz を `cell_size_m(0.3)` 格子にバケット化、
   各セルで最高品質サンプル 1 個を保持。`kRefreshFactor(0.9)` 以上の近品質・新着は
   セルを上書きして position/time を更新 → 緩いドリフトに追従。`prune` で
   `sample_ttl_s(60)` 超過セルを失効、`max_cells(64)` 超過分は低品質から evict。
   これで Procrustes 入力が空間的に well spread(非直線)かつ有界に保たれる。
6. **定期 re-solve**(`resolve_period_s` 既定 2s): 占有セル ≥ `min_cells(8)` なら
   `solve_motion(samples, min_cells)`(既存)。degeneracy(σ1/σ0)ガードも既存を再利用。
7. `status==Ok && residual_m ≤ residual_max_m(0.15)` のとき **clamped EMA**
   (`blend_alignment`)で live alignment へ反映:
   - `VmtPublisher::set_alignment` 経由(手動 UI / 単発 solver と同じチャネル、last-write-wins)。
   - **Y は current 値を保持**(手動 slider 不変)。yaw は最短弧で blend。
   - 1 更新あたり並進 `max_pos_step_m(0.05)`・`|Δyaw| max_yaw_step_deg(2)` にクランプ
     → 飛び(ジャンプ)防止。

### 純粋ヘルパ(スレッド/クロック非依存)

`ramp` / `verticality_score` / `make_sample` / `SampleReservoir` / `blend_alignment` は
明示入力 + タイムスタンプを取り、`ContinuousAligner` がポーリングループで束ねる。
これにより ctest がスレッドやクロック無しで全分岐を検証できる。

### 設定 / 配線

- CLI/YAML(`config/main_config.*`): `--vmt-continuous-align`(既定 ON)/
  `--no-vmt-continuous-align` / `--vmt-continuous-sample-hz` / `--vmt-continuous-resolve-s` /
  `--vmt-continuous-blend`。YAML `vmt.continuous_*`。閾値類は当面 `ContinuousAlignerConfig`
  既定値のまま(必要なら後で昇格)。
- `main.cpp`: `vmt_out && bus3d && hmd_listen_enabled && vmt_continuous_align` のとき構築・
  start。`SlimeStop` で publisher より先に stop(buses を読むため)。
- `crow_server`: `/api/vmt/alignment/auto/continuous/{start,stop,status}` で runtime toggle、
  `/stats3d` に `continuous_align` ブロック。Web UI(`web/dual_rtmpose`)に状態行 +
  「自動追従」トグル。
- **単発 tpose/motion との関係**: 同じ `set_alignment` チャネルを last-write-wins で共有。
  continuous ON 中に単発ボタンを押すと一旦上書きされるが、次の resolve で EMA が
  再び寄せていく(= one-shot override 扱い)。

## Milestone

- **M1**: 純粋ヘルパ + `SampleReservoir` + `make_sample`(配線なし、ctest 緑)。
- **M2**: `ContinuousAligner` 背景スレッド + clamped EMA + `set_alignment` 配線 +
  CLI/YAML + 既定 ON + `main.cpp`/`SlimeStop`。
- **M3**: Web API(start/stop/status)+ `/stats3d` ブロック + Web UI 状態行・トグル。

(M1–M3 は本コミットで一括投入。可ビルド単位として分割不要な規模。)

## 検証

- **ctest**: `test_continuous_aligner`(`ctest -R 'continuous_aligner|auto_alignment|vmt'`)
  - `ramp` 端点・降順境界、`verticality_score`(垂直 1 / 水平 0 / tilt27.5°→0.5 / 符号不変 / 退化 0)
  - `make_sample`(頭部↔chest 選択、品質 = conf·vert·vel、neck/hip 欠落→None、高速→0)
  - `SampleReservoir`(同一セル best 保持、近品質上書き、TTL 失効、max_cells evict)
  - `blend_alignment`(Y 保持、並進/Δyaw クランプ、収束、最短弧)
  - reservoir→`solve_motion` ラウンドトリップ(既知 yaw/並進の復元、residual<1e-3)
- **実機**(Jetson + Quest + VMT Manager + VRChat FBT):
  1. 起動後、手動 T ポーズ無しで直立 → 軽く歩く → yaw/xz が自動収束し追従。
  2. ジャンプが出ない(EMA + step clamp)。
  3. カメラに背を向け head score が落ちる場面で chest フォールバックに切替わり収束維持
     (Web UI の `cont_cells head=/chest=` 内訳で確認)。
  4. 時間経過(SLAM ドリフト)後も reservoir 失効で緩く再収束。

## 残課題

- **Y(高さ)自動推定**: 頭部採用時は頭頂 vs HMD で Y も推定可能。直立サンプル限定なら
  誤差小。別 milestone で評価。
- **垂直性スコアの tracker 反映**: chest/waist の roll_confidence に垂直性を乗算する拡張
  (出力姿勢全体に効くため要慎重評価)。
- **重み付き Procrustes**: 現状はセル毎ベスト選択で暗黙重み付け。`solve_motion` に
  明示 weight overload を足す案。
- **再センタリング検知**: VMT Room Matrix 変更で HMD 座標系が飛ぶ → reservoir flush の
  ハンドリング(backlog)。
