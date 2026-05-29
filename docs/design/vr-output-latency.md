# vr-output: 出力レイテンシ — frame-rate 非依存 smoothing (キーストーン)

(着手日 2026-05-29 / 派生元: [`core-pipeline-e2e-latency.md`](core-pipeline-e2e-latency.md) の
「VR ペーシング」検討、その M4 でイベント駆動 extractor を opt-in 導入したが smoothing の
dt 非依存問題を未解決のまま残していた点を解消する)

## 背景 / 動機

E2E (photon → VR 出力) の支配項は、GPU フロントエンド (core-pipeline M1–M4) でパイプラインを
`cap→pub 11.7ms` まで詰めた後は **VR 出力経路の 2 ホップ** に移った:

```
... → central RTMPose → SlimeTrackerBus
   → [hop1] TrackerExtractor (60Hz fixed) → [hop2] Publisher (60Hz fixed) → SteamVR/SlimeVR
```

各 60Hz ホップは平均 +8.3ms / 最悪 +16.7ms を足し、2 ホップで **avg +16.7ms / worst ~33ms** —
パイプライン本体 (11.7ms) を上回る最大の遅延源。

e2e-latency M4 は hop1 を**イベント駆動** (`--vr-extract-event-driven`、opt-in) にして固定 60Hz の
ホップ遅延を除いたが、**smoothing は dt 非依存の固定 alpha のまま**だった。これは罠で:

- `apply_quat_smoothing` / `apply_pos_smoothing` の `base_alpha` は「1 フレームあたりの重み」。
  固定 60Hz では妥当だが、**イベント駆動はソースレート (三角測量 ~30–90Hz) に同期**するため、
  90fps では 30fps の 3 倍の頻度で同じ alpha の平滑がかかり **wall-time あたり過平滑** (= 余計な遅延)。
  レートが上がるほど平滑が強まり、せっかくのレイテンシ削減を相殺する。
- これは e2e-latency M4 がイベント駆動を入れた時点で混入した**潜在バグ**でもある (固定 60Hz では
  顕在化しない)。

## 検討した案

### A. レート引き上げ (extractor/publisher を 120Hz に) → 単独では没
ホップ遅延は半減するが、dt 非依存 alpha のままだとレート倍で平滑が倍効き相殺する (e2e-latency doc が
指摘した「罠」)。frame-rate 非依存 smoothing が前提条件。

### B. frame-rate 非依存 (time-constant) smoothing → 採用 (本 doc)
per-step alpha を `alpha_eff = 1 - (1-base_alpha)^(dt/nominal_dt)` に一般化。wall-clock の時定数が
レート非依存になり、イベント駆動でも固定レートでも同じ平滑感。**これがレート引き上げ・イベント駆動
両方を安全にするキーストーン**。`base_alpha` の意味は「nominal cadence (= `extract_rate_hz`) での
1 ステップ重み」に再定義 (後方互換: `dt==nominal` で従来と完全一致)。

### C. publisher も event-driven (hop2 除去) → 保留
SlimeTrackerBus 更新で送信すれば hop2 の固定遅延も消えるが、SteamVR/SlimeVR の定レート期待 +
送信レートがソース可変になる挙動は実機 (被写体 + SteamVR) 検証が要る。本 doc では扱わず、
frame-rate 非依存 smoothing が入った後の follow-up とする。

## 採用設計 (Milestone)

- **M1 ✅ (2026-05-29)**: **frame-rate 非依存 smoothing**。
  - `tracker_extract.cpp` に `rate_adjust_alpha(base_alpha, dt_s, nominal_dt_s) = 1 - (1-base_alpha)^(dt/nominal)`
    を追加。`apply_quat_smoothing` / `apply_pos_smoothing` に `dt_s` / `nominal_dt_s` 引数を追加
    (default 0 → `base_alpha` のまま = 既存呼出し/テスト後方互換)。quat は `alpha_rate · roll_confidence`、
    pos は `alpha_rate` を使用。
  - `tracker_extractor.cpp::run_loop` が既に算出している `dt_s` (event: 実測 clamp [1e-3,0.5]s /
    fixed: nominal) と `nominal_dt_s` (=1/`extract_rate_hz`) を smoothing に渡す。
  - **不変条件**: 固定レート (`dt==nominal`) では `alpha_rate==base_alpha` で**従来挙動と完全一致**
    (既定経路ゼロリスク)。イベント駆動では高 fps で per-step alpha が小さくなり過平滑を解消。
    長い idle 後 (`dt≫nominal`) は `alpha→1` で stale prev に固執せず snap。
  - **検証 (被写体不要)**: `test_tracker_extract_pos` に rate-independence テスト追加 —
    ① dt=nominal/2 の 2 ステップ == dt=nominal の 1 ステップ (EMA の厳密性質、float 一致)、
    ② dt==nominal == legacy default、③ 高レート単発ステップは full ステップより under-shoot。ctest 9/9。

- **M2 (follow-up, 被写体要)**: イベント駆動 extractor を既定化 or 推奨化 + 必要なら publisher レート
  引き上げ/event-driven (hop2)。`e2e_capture_to_send_ms` で実 photon→send を計測し judder を確認。
  M1 で smoothing がレート非依存になったので、ここでレートを上げても過平滑にならない。

## 検証

- `test_tracker_extract_pos` の rate-independence テスト (上記、被写体不要、ctest 常時)。
- 既存の `test_tracker_extract` / `test_tracker_extract_pos` 既存ケースが不変 (固定レート後方互換)。
- 実機 judder / `e2e_capture_to_send_ms` の数値検証は被写体 (`ik_locked`) + SteamVR が要るため M2 送り。

## 残課題 / リスク

- **被写体依存の検証**: VR 挙動 (judder, 体感遅延) は被写体が要る。M1 は smoothing の数学的性質に閉じた
  ので unit test で担保し、挙動確認は M2 へ。
- イベント駆動の既定化はソースレート可変 (三角測量の bursty さ) に挙動が左右されうる。timeout fallback で
  stale クリアは維持しているが、判断は実機検証 (M2) で。
