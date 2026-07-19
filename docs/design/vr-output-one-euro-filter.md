# vr-output: One Euro フィルタによる動静適応スムージング

(着手日 2026-06-03 / 旧見送り記録: [`archive/phase14-vmt-steamvr.md`](../archive/phase14-vmt-steamvr.md) の「One-Euro filter を採用しない理由」)

## 背景 / 動機

椅子に座って静止しているときでもトラッカー(位置・回転)が細かく揺れる。原因は
`TrackerExtractor` のスムージングが**固定 α の EMA**であること。固定 α は 1 つの
壁時計時定数しか持たないため、「静止で滑らか」と「動作で遅延なく追従」を構造的に両立
できない:

- α=0.5 @60Hz の実効カットオフは `fc = 1/(2π·τ)`、`τ = te/(1-α)·... ` 換算で **≈ 9.5 Hz** と
  高く、静止時の三角測量ノイズ(高周波)がほぼ素通りする。
- α を下げると静止は滑らかになるが、動作時に明確な遅延(ぬるっと遅れて付いてくる)が出る。

一般的なカメラ姿勢推定でも「自然な動き=低周波 / ジッタ=高周波」という前提から**速度適応
ローパス**が定番。リアルタイム用途では **One Euro Filter (Casiez et al. 2012)** が事実上の標準
(MediaPipe ほか)。非リアルタイム系では学習ベースの時系列リファインメント(SmoothNet 等)も
あるが、前後フレーム参照・重量級でエッジ・リアルタイムには不適。

本リポジトリは Phase 14 (2026-05-25) で One Euro を「3 パラメータのチューニングコスト + 実装
コストが EMA より高い」として一度見送り、「EMA が物足りなければ Phase 15 候補」と明記して
いた。今回の静止ジッタ要求がまさにその昇格条件に該当する。

**完了条件**: 固定 α EMA を One Euro(速度適応カットオフ)に置き換え、静止時はカットオフを
下げて強くスムージング(無ジッタ)、動作時はカットオフを上げて遅延なく追従する。位置・回転
両方に適用。既定 ON。

## 検討した案

- **固定 α を上げ下げ(却下)**: 1 つの時定数では静止と動作を両立不能(上記)。
- **EMA の後段に One Euro をカスケード(却下)**: 二重ローパスで位相遅れが増え、かつ
  チューニングパラメータが二重化する。One Euro は本質的に「α が速度依存になった EMA」なので、
  既存 EMA を**置き換える**のが正しく、重ねる必要はない。
- **明示デッドバンド / ハード静止ロック(却下)**: 「動・静を区切る」要求にハードに応える案だが、
  速度しきい値の境界でカクつき、ヒステリシス設計コストもかかる。ユーザー判断で不採用、
  One Euro の連続(ソフト)な適応カットオフのみを採用。
- **クォータニオン per-component One Euro(却下)**: 4 成分を独立フィルタ→再正規化は、符号反転
  (二重被覆)と成分間非独立性で破綻する。回転は**測地角速度→単一適応 α** を既存 slerp に
  乗せる方が幾何的に正しい。
- **学習ベース(SmoothNet 等)(却下)**: 高精度だがオフライン・重量級。本プロジェクトの
  エッジ・リアルタイム制約に合わない。
- **採用**: 固定 α を、位置=per-axis / 回転=測地角速度ベースの **One Euro 適応 α** に置換。
  既存の swing/twist 分離・parent-yaw transport・hip-relative hold・外れ値ゲートは**温存**。

## 採用設計

データフロー・所有権は不変(`TrackerExtractor` が単一 producer、consumer は平滑済みのみ参照)。
既存の 2 関数の**骨格を再利用し、「base α の作り方」だけを差し替える**。

### One Euro 核 (`tracker_extract.{hpp,cpp}`)

```
te      = dt (実ステップ秒)
dx      = (x − x_prev_filtered) / te
dx_hat  = lowpass(dx, alpha(dcutoff, te))      // 速度を平滑
cutoff  = mincutoff + beta·|dx_hat|            // 速度適応カットオフ
x_hat   = lowpass(x, alpha(cutoff, te))        // 出力
```

- `one_euro_alpha(cutoff, dt)` = `dt/(dt+τ)`, `τ = 1/(2π·cutoff)`。dt を直接使うため
  **フレームレート非依存**(既存 `rate_adjust_alpha` と同じ役割を内包)。
- `mincutoff`(Hz)=静止時の滑らかさ(低いほど滑らか/遅延大)、`beta`=動作追従(高いほど
  遅延小)、`dcutoff`(Hz)=速度推定の平滑(既定 1.0)。**`beta=0` で固定カットオフ EMA に縮退**
  (回帰・フォールバック)。

### 位置 (`apply_pos_smoothing` の `OneEuroParams` 新オーバーロード)

- トラッカー×軸(x,y,z)で独立 One Euro。`alpha_rate`(固定)を per-axis `one_euro_alpha` に置換。
- **既存の外れ値ゲート(8–16 m/s で freeze)は残す**。役割が別物(三角測量グリッチ除去)で、
  One Euro 単体だと大スパイクでカットオフが開いてグリッチを追ってしまうため。
  最終 α = `one_euro_alpha(cutoff, te) · (1 − smoothstep(v_mps, 8, 16))`。
- 速度推定 `te` は `dt·(1 + invalid_ticks)`(dropout ギャップを正しく反映)。
- 速度状態 `pos_dx_hat[i]`(per-axis)を `PosSmoothingContext` に追加。初フレームは snap
  (prev←curr, dx_hat←0)で原点収束の過渡を排除。hip-relative hold は無改変。

### 回転 (`apply_quat_smoothing` の `QuatSmoothingContext`/`OneEuroParams` 新オーバーロード)

- swing/twist + parent-yaw transport 本体を `apply_quat_smoothing_impl(... per-tracker alpha ...)`
  に抽出。固定 α 版は全要素同値の alpha 配列で呼ぶ(**bit-identical**、既存 ctest 温存)。
- One Euro 版は、prev(平滑値)と raw の**測地角速度** `2·acos(|p·q|)/te` を低域通過 → cutoff →
  `one_euro_alpha` で per-tracker α を算出し、impl に渡す。算出 α は既存の
  `sa = α·swing_confidence` / `ta = α·roll_confidence` の base になるので、信頼度ゲート・
  transport・held-roll を**全て無改変で再利用**。
- 角速度状態 `ang_vel_hat[i]` と初期化フラグ `initialized[i]` を `QuatSmoothingContext` に保持
  (`TrackerExtractor` 所有)。初フレームは snap(prev←raw)。

### 配線

- `TrackerExtractorOptions`: `bool one_euro=true` + `OneEuroParams pos_one_euro{0.8,0.4,1.0}` /
  `quat_one_euro{1.0,0.3,1.0}`。`run_loop` は `one_euro` で新旧オーバーロードを分岐(既定 One Euro)。
- `MainConfig`(`three_d` セクション、`vr_extract_event_driven` の隣)に
  `vr_one_euro` / `vr_quat_smooth` / `vr_{pos,quat}_{mincutoff,beta,dcutoff}` を追加。YAML パース + CLI
  (`--vr-no-one-euro` / `--vr-quat-smooth` / `--vr-{pos,quat}-{mincutoff,beta,dcutoff}`) + validation(dcutoff>0 等)。
  One Euro を無効化したときは `vr_quat_smooth` (既定 0.5) と `vmt.pos_smooth` が固定 EMA の
  回転・位置 alpha になる。両者は VMT 出力の有無にかかわらず WebUI tracker stream も使うため、
  3D 有効時に検証する。

### しきい値の根拠

初期値(本 doc 執筆時)は位置 `mincutoff 0.8Hz / beta 0.4`、回転 `mincutoff 1.0Hz / beta 0.3`。
旧 α=0.5 ≈ カットオフ 9.5Hz に対し静止カットオフを 1 桁下げる方向(静止ジッタを強く抑え、
`beta·速度` で動作時のみカットオフを開く)考え方は不変。

> **現行既定値**: M3 実機チューニングの結果、既定値は位置 `mincutoff 1.0Hz / beta 4.0`、
> 回転 `mincutoff 1.5Hz / beta 1.5` に更新済み(`MainConfig` / `main.cpp --help` が真の source。
> 上の初期値は採用判断時の値であって現行設定ではない)。詳細は
> [`docs/tracks/vr-output.md`](../tracks/vr-output.md) の changelog を参照。

## Milestone

- **M1**: One Euro 純関数 `one_euro_alpha` + 位置/回転の新オーバーロード(`impl` 抽出含む) +
  単体テスト。
- **M2**: `run_loop` 分岐、`TrackerExtractorOptions` / `MainConfig` / CLI / YAML / validation 配線、
  `test_main_config` 追記。
- **M3**: 実機チューニング(座位静止→歩行/ジェスチャ遷移で `mincutoff/beta` 調整)、本 doc 確定、
  changelog 記載。

## 検証

- **ctest** `tracker_extract` / `tracker_extract_pos` / `main_config`(全 11 件パス):
  - `one_euro_alpha` の単調性・端点。
  - 初フレーム snap(位置・回転)。
  - **静止小ジッタの応答が固定 α=0.5 EMA の半分未満**(位置・回転)。
  - 持続運動でカットオフが開き追従割合が静止時を上回る(位置)。
  - `beta=0` で step 量に依らない固定カットオフに縮退(= `one_euro_alpha(mincutoff)`)。
  - 外れ値スパイク(>16 m/s)が One Euro 下でも freeze する。
  - 既存の bit-identical 系テスト(旧オーバーロード)が無傷でパス。
- **実機**: 2 カメラ起動 → WebUI `/ws3d` の per-tracker stats(`angular_velocity_rad_s_p50/p95`、
  `leakage_pct`)で座位静止時の角速度低下 / 歩行時の追従維持を確認。VMT/SlimeVR 出力で
  「座って静止→揺れない、動くと遅延なく付いてくる」を体感確認。`--vr-no-one-euro` で旧 EMA と A/B。

## 残課題

- M3 実機チューニング(`mincutoff/beta` の最終値)。足(`swing/roll_confidence` 低)と
  剛体ボーンで最適 `beta` が異なる可能性 → per-role パラメータ化は将来 backlog。
- WebUI からの runtime スライダ調整(現状は CLI/YAML のみ)。
