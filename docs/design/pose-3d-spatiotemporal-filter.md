# pose-3d: 時空フィルタ (spatiotemporal-filter) — tracker 段の regime 適応平滑

(着手日 2026-07-03 / 関連: issue #48、トラック pose-3d / vr-output、
[pose-3d-spatial-filtering](pose-3d-spatial-filtering.md) の **M-C を再定義**、
既存 [vr-output-one-euro-filter](vr-output-one-euro-filter.md) / [pose-3d-locomotion-stability](pose-3d-locomotion-stability.md))

## 背景 / 動機

空間フィルタ track (spatial-filtering M-A/M-B) の実測で **当初前提が覆った**:

- 剛体フィットの効きは segment の **per-joint 再投影誤差 (独立ノイズ量) に比例**。骨盤 (reproj 7–10px) は
  −10〜20% 取れたが、**目に見えるプルプルの本体＝肩帯 (reproj 2–4px, jitter 14–19mm) は剛体では消せない**
  (コヒーレントな剛体ポーズ動揺)。骨盤の −1mm は実機で「わからん」(目視分解能以下)。
- → **見える残ジッタ (肩帯・四肢) は空間でなく時間側の仕事**。M-B の負の所見が「時間フィルタが担う」を裏付けた。

そこで **M-C を再定義**する。当初 M-C は「空間で取ったので時間フィルタを弱める」だったが、これは誤り
(時間フィルタこそが肩帯ジッタを抑えている → 一律弱化は最悪部位を悪化させる)。新 M-C は
**時間フィルタを弱めるのでなく、regime 適応の「時空フィルタ」に置き換える**。

**症状と狙い**: (1) 静止プルプル (肩帯・四肢) を消す (2) 動作の追従性を保つ (lag を増やさない)。
本質は「静止 jitter ↔ 動作 lag」トレードオフからの脱出で、これを **非線形 (距離 × 速度の 2 軸 regime)** で狙う。

**完了条件**: 肩帯・四肢の静止ジッタを目視で明確に低減、かつ動作 (歩行・腰曲げ・キック) の追従・忠実さを
非悪化。既存 ctest 回帰なし。既定 OFF で本番挙動不変。

## 検討した案

### 案1: 当初 M-C「時間フィルタを一律弱化」— 没
空間で構造を取った前提で Kalman/One-Euro を弱める案。M-B の実測で前提が崩れた
(見える肩帯ジッタは空間で取れず、時間フィルタが唯一の抑え手) → 一律弱化は最悪部位を悪化。**降格・撤回**。

### 案2: 素の One-Euro を再チューニング — 没
`vr_one_euro` の cutoff/beta を詰めるだけ。線形速度適応の内側を動くだけで、ユーザーの主訴
(静止↔lag トレードオフからの脱出) に応えられない。

### 案3: ハード位置デッドゾーン (完全凍結) — 没
`d < D` で出力完全凍結。静止ジッタは 0 になるが (a) 境界で「止まる→カクッと飛ぶ」不連続 + チャタリング
(b) 凍結点に張り付き最大 D の恒久オフセット。→ **soft デッドバンド + 強フィルタ再センタリング**を採用
(凍結でなく小 α で、微揺れは残すが真の中心へ戻る。恒久オフセット無し・snap 無し)。

### 案4: keypoint 段 (Kalman 前) で filtering — 没
IK + tracker 再導出を後段に挟むので二重平滑 & 間接的。→ **tracker 段** (最終・単一 producer・VMT 直結) に置く。

### 案5: chain Kalman 完全撤去 — 保留 (次手)
regime フィルタ一本にする案。locomotion-stability 一式が chain Kalman (hip 相対 hold / 欠損補完) 前提のため
退行リスク大。→ **Kalman は残すが弱める** (予測/hold 専任・平滑ほぼゼロ)。弱 Kalman でも lag が見える時の次手。

### 案6: 角度空間 (ボーン間相対角速度) で一発記述 — 有力な代替 / 測って判断
骨長固定 (IK) なら姿勢は「ルート位置 + 各ボーンの親相対姿勢」で完全記述され、位置ジッタは (ルート除き) 全部
**ボーン角ジッタ由来**。角度空間で角ジッタを潰せば FK 経由で位置も落ち、木構造にネイティブ (Q4 の腰相対パッチが
タダ) で、roll も同じ枠に入る — **より統一的**。ただし: (a) 角誤差が葉で増幅 (肩 1° → 手先 ~1cm)、VMT が送るのは
葉位置なので**悪化し得る**、(b) roll 観測性 (伸展肢で不定) は角度空間でも不変、(c) 位置を「フィルタ後角 + ルート
から FK 再構成」する作り替えが要る、(d) points→角変換に特異点。→ **座標系の優劣は実クリップで決まる経験的問題**。
オフラインハーネスに **per-bone 相対角速度 dump** を足して仮説検証し、データが支持すれば将来寄せる
(SlimeVR 主出力化するなら角度空間がネイティブ)。今は葉位置を直接制御できる**位置ドメインを採用**。

### 採用: tracker 段の距離 × 速度 2 軸 regime フィルタ (位置 + 推測 roll)
下記「採用設計」。

## 採用設計

### データフロー
```
tri →(M-A時)骨盤剛体→ Kalman(弱化: 予測/hold専任) → IK → tracker_extract[ 時空フィルタ ] → VMT/WebUI
                                                                     │
   位置(全10 tracker, 四肢は腰相対フレーム) ── d,v 2軸 regime ───────┤
   推測 roll(upper_arm/thigh/縮退shin の twist) ── 同 regime を twist_alpha へ ┘
```
- **置き場**: `slimevr/tracker_extract.cpp`。位置は現行 **One-Euro を置換** (優先度 st_filter > one_euro > 固定EMA)、
  roll は既存 `apply_quat_smoothing` の **twist_alpha を regime 駆動**。swing・剛体 roll ボーンは無変更。
- **Kalman は残すが弱める** (process noise↑ = 測定追従 = lag ほぼ足さず、hip 相対 hold / 欠損補完のみ担う)。
  IK (空間拘束・lag 無し) は不変。

### 2 軸モデル (時空)
- **d = |生の目標 − 保持出力|** (空間距離): 追従強度・凍結・ラグキャップを決める。
- **v = |生curr − 生prev| ÷ dt** (生速度, m/s): 外れ値棄却を決める (既存 `kPosVelGate*_mps` を部位別に一般化)。
- **「高速だが本物 (追従)」と「速すぎ = 誤検出 (棄却)」を分けるのは v のみ**。追従の強さは d のみ。

### 位置フィルタ (per-tracker, 四肢は腰相対フレーム)
```
# 四肢: target/held/last_raw を腰相対 (target - waist_world) で扱い、腰は world で別途フィルタ。
d    = |target_rel - held_rel|
v    = |target_rel - last_raw_rel| / dt
gate = 1 - smoothstep(v, part_high, part_reject)        # 1=信頼, 0=棄却
alpha_d = (d < D_core)          ? alpha_rest                                   # 強フィルタ + 再センタリング
        : (d < D_full)          ? smoothstep(d, D_core, D_full)·(a_norm-a_rest)+a_rest
        :                         alpha_normal
alpha = rate_adjust_alpha(alpha_d, dt, nominal_dt) · gate                      # dt 補正は既存流用
out_rel = held_rel + alpha·(target_rel - held_rel)
if gate > eps and |target_rel - out_rel| > cap:                               # 信頼時のみラグキャップ
    out_rel = target_rel - cap · unit(target_rel - held_rel)                  # target の cap 以内へ
out = out_rel + waist_world_filtered (四肢) / out_rel (腰)
held := out ; last_raw := target
```

### 回転 (推測 roll のみ)
`apply_quat_smoothing` の twist 成分に同じ曲線: `d_roll`=保持rollからの角距離, `v_roll`=roll角速度、
`twist_alpha = alpha_d(d_roll) · roll_confidence · gate(v_roll)`。**roll_confidence ゲートは温存** (縮退→hold)。
swing と剛体 roll (chest/waist/shin-anatomical, conf=1 pin) は無変更 (スコープ「腕・脚の曲がりからの推測 roll のみ」)。

### 不変条件
- **既定 OFF (`three_d.st_filter` / `--st-filter`) で byte 不変**。ON で位置=regime・twist=デッドバンド。
- **d は空間距離・v は m/s・cap/D は meter** → **fps 非依存** (距離ベースの自然な利点)。alpha は既存 `rate_adjust_alpha` で
  wall-clock 一貫。
- **状態リセット**: idle/standby 復帰で per-tracker 状態 (held/last_raw/held_roll) を reset (既存 `kalman.reset()` と同型。
  idle 中は更新が走らないので stale から補間させない)。
- **hold/FK との順序**: 時空フィルタは hip 相対 hold / 足 FK fallback の**後** (最終 tracker pos) に走る。tracker が
  invalid→valid に遷移した瞬間の hold ジャンプを「高速動作」と誤認しないよう、valid 遷移でその tracker の状態を
  reset (last_raw を新規 anchor)。
- **腰は world 系の基準**、四肢 (chest/arm/leg/foot) は腰相対で分類 & 平滑 (全身並進は腰フィルタのみを通り四肢が lag しない)。
- SlimeVR (回転のみ出力) 併用時は**位置フィルタは無意味**なので当面スコープ外。

### 部位別パラメータ
- **6 群 (L/R 対称)**: `{waist}`(world基準・遅い) `{chest}` `{upper_arm}` `{upper_leg}` `{lower_leg}` `{foot}`(最速・既存の強damping)。
- **seed (初期値)** はコード既定 (`StFilterParams`) にクリップから proxy 算出で焼く:
  - デッドバンド `D_core/D_full` ← 静止クリップの proxy 関節ジッタ std (`D_core ~2–3σ`)
  - 外れ値 `part_high/part_reject` ← 動作クリップの腰相対 proxy 速度実測 max (`part_reject ~1.5–2×`)
  - `alpha_rest` ← 残留が目視外 (~3mm) になる値 (残留 ≈ raw·√(α/(2-α)))、`cap` ← 初期 10cm (足は大きめ検討)
- YAML には**当面トグルのみ**。オフラインハーネスに主要ノブの CLI 上書きを付けリビルド無しでスイープ→勝った値をコード既定へ。

## Milestone

- **M-C1** ✅ 済 (2026-07-03): **オフライン tracker 計測ハーネス**。`dump_keypoints_3d --dump-trackers` (Halpe26 限定)
  が最終 skeleton から `extract_trackers` を走らせ、JSONL 各行に **`trackers` 配列** (per-tracker の pos + 姿勢 quat +
  valid + roll_confidence) を追記。production 配置デフォルト (foot=Ankle / spine frac 0.65,0.15) + job ごと `ExtractContext`
  (FK fallback) でライブ幾何に一致。`analyze_3d_jitter_lag.py trackers` が per-tracker の **pos ジッタ(mm) / 推測 roll(twist)
  ジッタ(deg) / 全姿勢スキャッタ(deg) / 親相対角速度(deg/s)** を出し、2 ファイルで ON/OFF delta 表。親ツリーは
  transport (腕→chest / 脚→waist / distal は自肢遡上) に一致。→ **今回は RAW = 決定的 OFF ベースライン**を取得。時空フィルタ
  自体を通した ON 側は **M-C2/M-C3 後にハーネスへ差し込んで**取得 (案6 の角度空間検証データは相対角速度列で既に出る)。
- **M-C2** ✅ 済 (2026-07-03): **時空フィルタ core** を `slimevr/st_filter.{hpp,cpp}` に新規実装 (fitra_slimevr)。
  純粋・自己完結・単体テスト済の regime プリミティブ: `StRegime`/`StPosParams`/部位別 6 群 `StFilterConfig` +
  seeded `default_st_config()`、スカラ core `st_alpha_d` (デッドバンド→ramp→normal) / `st_vel_gate` (外れ値) /
  `st_rate_adjust_alpha`、位置ステップ `st_pos_step` (2 軸 regime + ラグキャップ)、twist の
  `st_twist_angle` (+Z 回りの符号付き twist) / `st_twist_alpha` (`alpha_d(d_roll)·roll_confidence·gate(v_roll)`)。
  ctest `test_st_filter`: 凍結コア再センタリング (完全凍結でない・恒久オフセット無し)・ランプ連続/単調・ラグキャップ上限・
  外れ値保持・dt 補正の fps 非依存・twist 符号/swing 除去・default config 健全性。**まだ配線しない** (M-C3)。
  full build + ctest **33/33** パス。
- **M-C3** ✅ 済 (2026-07-03): **`tracker_extract` へ配線**。`TrackerExtractor::run_loop` に第3の平滑モード
  (優先度 st_filter > one_euro > 固定EMA) を追加。位置は `apply_pos_st_filter` (waist を world 先行フィルタ→四肢を
  waist 相対で `st_pos_step`、**腰相対が hip-relative hold を内包**＝invalid 四肢は相対 offset hold で waist に追従、
  復帰は snap)。推測 roll は `fill_st_twist_overrides` が has_roll 群のみ twist_alpha を regime 計算し、
  `apply_quat_smoothing` の新 optional `twist_alpha_override` 経由で注入 (swing・pin は不変。**transport は厳密には
  不変でない**: carry = 1 − ta/sa が override 後の ta から再計算されるため、roll_conf=1 でも静止時 carry ≈ 0.26 の
  親 yaw 結合が残る — transport_t = carry·alpha_rate で有界・定常オーバーシュートなし。コードの carry コメント
  CAVEAT 参照、腕の対トルソ逆回転が結合して見えたら再訪)。
  **chain Kalman 弱化**: `ThreeDConfig.st_filter` ON で `SkeletonKalman` の q_pos/q_vel/offset を ×100 (seed) して
  測定追従化 (M-C4 調整)。フラグ `three_d.st_filter` / `--st-filter` (既定 OFF) を `output_builder`(tracker) と
  `threed_builder`(Kalman) の両方へ配線。**既定 OFF で byte 不変** (override=nullptr で apply_quat_smoothing 完全一致・
  Kalman default・config 非emit) を ctest で固定。ctest `test_st_filter` に全身並進 no-lag / 静止抑制 / invalid-hold
  drag / recovery snap / twist scope / nullptr byte 一致を追加。full build + ctest **33/33** パス。
  **次**: M-C4 (ハーネスに st_filter 差し込み→6 群 param スイープ)、M-C5 (実機 A/B)。
- **M-C4** ✅ 計測済 (2026-07-04、実 59fps クリップ + 敵対的検証): ハーネスに tracker 段平滑を差し込み
  (`--tracker-smoothing {raw,one_euro,st}` + `--st-kalman-weaken F`) 、still/walk_around/forward_bend で
  raw/one_euro/st を A/B。**検証済の結論** (workflow `mc4-motion-study`、8 agent):
  - **lag: st は one_euro に追加 lag ゼロ、むしろ速い** (walk_around 全4 tracker が信頼域 sigma>500mm・corr>0.999 で
    st の lag は正 causal かつ one_euro 未満: l_upper_arm −19.8ms / l_lower_leg −9.9ms / feet −3.6〜−4.9ms)。→ roll 改善は **lag コストゼロ**。
  - **roll: 真の regime 効果は arm/feet 限定・控えめ** (l_upper_arm −12.6% vs one_euro / −34% vs raw)。**脚の派手な数字
    (upper_leg −36〜−41%, lower_leg −25〜−36%) は metric artifact** — 脚は roll_conf=0 で **twist regime は INACTIVE**、
    roll_rms の動きは `2*atan2(d.z,d.w)` 投影に swing/parent-yaw wander が漏れたもの (roll delta が ori delta に 1:1 追随)。
    完全 confident な r_upper_arm(conf=1.0) は 0%。**可視プルプルの proxy は roll_rms でなく rel_dps** (角速度) を採るべき
    — 脚で乖離し rel_dps は st legs=calm (5dps vs raw 12–42=slow drift, not shimmer)。
  - **position: pos_rms は wash かつ推論ノイズ内** (raw/one_euro/st は別々の推論 run で skeleton は byte 不一致・
    diff max 78mm=FP16 非決定性は想定より桁違いに大。当初「~0.02–0.05mm」は過小評価)。ただし **position path を削るな** —
    脚/足の swing を安定させる orientation 効果は position regime 由来。「position は low-value」は誤り、正しくは
    「pos_rms は ~6mm 上流フロア律速だが filtering は leg/foot orientation を駆動」。
  - **×100 Kalman 弱化は棄却確定** (rest で rel_dps 2–3×・held-leg roll +300–1400% vs raw)。**weaken=1** に既定変更済
    (skeleton が raw と一致)。**st は rest+lag の全 metric で one_euro 以上**。→ **roll regime を weaken=1・default-OFF で ship、
    最終 ON は M-C5 実機で判定**。
- **M-C5** (offline 部分 ✅ 2026-07-04 kick/dash + 敵対的検証 / 実機部分は残): **fast-motion offline 判定**:
  - **fast lag: st は one_euro に lag を足さない** (全12 moving tracker で st≤one_euro、integer-frame +2→+1 で robust。One-Euro が
    強い low-pass ゆえ st が速いのは物理的に期待通り)。**例外: kicking foot は spatial lag_cap(150mm)係合で burst-window ~14ms の
    bounded lag** (whole-clip xcorr が隠す。st worst-case trail 150mm < one_euro 238mm)。velocity-gate freeze は人体速度で不発。
    **ただし offline lag は directional-only** — reliability gate(σ>200mm & corr>0.999)を kick/dash 全 tracker で未達(per-axis 83-217mm)。
  - **kick で脚 roll は genuinely observable (roll_conf 0.95-0.99)＝twist regime ACTIVE。それでも st は脚 roll を改善しない**
    (rel_dps が st で悪化、roll_rms tied)。→ **twist regime が legs を助けないことは active ケースでも確定、roll win は arm 限定**。
    (kick leg roll_rms ~65deg は swing-leakage、twist の証拠にしない。)
  - **default-ON は offline だけでは不可** — st は mode-switch filter(roll_conf flicker で twist DOF lock/unlock)、whole-clip metric は
    threshold-snap/chatter/動き出し onset を原理的に見られない。→ **実機 A/B が唯一の gate**。
  - **実機 A/B ✅ favorable (2026-07-04)**: WebUI 3D viewer で st ON vs OFF(one_euro) を体感比較。**st ≥ OFF・新規 dealbreaker なし**。
    A(腕 roll 静止)・C(chatter)・D(onset)・E(kick 足) は「大体 OK」。**B(閾値越え snap): st も OFF もどちらも出る** ＝ これは st 固有でなく
    **roll_confidence ゲート(肘 sin 8.6°/17.5°)に内在する既存アーティファクト**(OFF=One-Euro も同じゲートを使う)、**st の方が体感まし**(緩和)。
    → offline で足りなかった体感 gate が好転。**default-ON が正当化可能**(ただし upside は arm twist 限定で narrow)。
  - **✅ ship 確定 (2026-07-04): arm 限定 scope + default-ON**。has_roll を **upper_arm のみ**に縮小(legs は twist regime から除外＝
    rel_dps を足すだけで roll gain ゼロ、position regime は legs/feet にも残す)。`three_d.st_filter` を**既定 ON**(`--no-st-filter` で
    One-Euro へ戻す)。Kalman weaken=1(no-op)。full build + ctest 33/33。
  - **残**: >200mm 大振幅 clip で lag を registered bar 正式判定(offline directional-only の穴埋め)。閾値 snap は
    「伸展で真っ直ぐ snap + roll を最終観測/足先/コントローラー twist から供給」(下記 残課題)で根本対策。
- **将来**: (a) 案6 角度空間の検証結果次第で角度ドメインへ寄せる。(b) **スリム化** — st_filter を唯一の平滑にし
  One-Euro / 固定 EMA と `vr_one_euro`/`vr_pos_*` 系を撤去 (ユーザー意向: 最終的に既存を壊してスリム化)。

## 検証

- **ctest**: `StFilter` core を合成信号で固定 (M-C2)。既定 OFF での byte 不変 (M-C3)。
- **主検証 = 決定的オフライン再生** (M-C1 ハーネス): 静止クリップ → per-tracker 位置 RMS を比較、
  動作クリップ → `trackers-lag` (tracker 位置軌跡の相互相関) で lag を比較。
  **⚠ 交絡注意 (M-C4 で判明)**: raw/one_euro/st を別々の dump run で取ると **skeleton が byte 不一致** (別々に
  YOLOX+RTMPose 推論するため。実測 diff max 78mm、FP16 非決定性は当初想定 ~0.02–0.05mm より桁違いに大)。
  sub-mm の位置 delta は run 間ノイズ内なので過信禁物。**将来の A/B は 2D 検出/triangulated skeleton を 1 回
  cache し同一信号で filter variant を replay** して noise floor を与えるべき。lag は高 motion tracker
  (sigma>~200mm・peak_corr>0.999) のみ信頼。**可視ジッタ proxy は roll_rms でなく rel_dps** (roll_rms は held DOF で
  swing wander を twist に誤帰属する)。
- **実機**: WebUI 3D viewer で静止プルプル低減 / 歩行・腰曲げ・キックの追従と忠実さ / snap・チャタリング無しを目視。

## 残課題

- ✅ **arm 限定 scope + default-ON 済 (2026-07-04)**。残: >200mm 大振幅 clip で lag を registered bar 正式判定。
- ✅ **伸展 roll snap → roll gate-raise hysteresis 実装済 (2026-07-04、既定 OFF)**。設計 workflow の敵対的検証で**当初の
  「canonical-slew(last_good up 再供給)」案は棄却**: 閾値跨ぎ snap は既に smoothstep で masking 済(ta→0 で恣意 roll は捨てられる)、
  真の snap 源は**中間帯(sinθ 0.15–0.30)の測定 roll ノイズ追随**(roll azimuth ノイズ ∝ 1/sinθ で 3–7× 増幅)。ユーザーの「last_good
  保持」は既存機構(swing parallel-transport + parent-yaw transport)が既に供給済。→ **採用: acquire/release hysteresis で実効 trust
  gate を引き上げ**(`kRollAcquireSin=0.42`/`kRollReleaseSin=0.26`、arm/thigh の inferred-roll 限定)。lock は clearly-bent 時のみ、
  伸展でノイズ帯を追わず last-confident roll を held(conf→0)。`extract_trackers(...,roll_hysteresis)` + `ExtractContext.roll_locked`、
  flag `three_d.roll_hysteresis`(既定 OFF・`--roll-hysteresis`)、`dump_keypoints_3d --roll-hysteresis` で offline A/B。ctest
  `test_tracker_extract_roll`(byte 不変/arm・thigh hysteresis 列/NaN/reset)、**35/35**。**経路非対称**: 恩恵は arm=st+One-Euro+EMA /
  leg=One-Euro+EMA(st has_roll は arm-only)。**残**: `acquire/release=0.42/0.26` は seed、伸展 snap を訴えた本人の動作クリップで
  raw 比 roll_rms/rel_dps を A/B して tuning + 実機体感。**将来案(未採用)**: 腕はコントローラー twist を roll source に(plumbing 重・実機必須)。
- **held-twist (roll_conf≈0) pass-through — 候補 (要否は M-C5 待ち)**: 脚等の held 軸に st/one_euro とも spurious twist を
  注入 (roll_rms +300–800% vs raw)。ただし M-C4 検証では **rel_dps 上 st legs=calm で可視プルプルではない** =
  roll_rms metric の過大評価の可能性が高く、「解くべき問題があるか」自体が未確定。実機 M-C5 で脚 roll が気になるなら実装。
- **A/B インフラ**: 2D 検出/triangulated skeleton を 1 回 cache → 同一信号で filter variant を replay (sub-mm position
  delta に noise floor)。現状は別々推論で skeleton diff max 78mm、位置 delta が交絡。
- **arm twist の alpha_rest チューニング** — genuine な roll 効きどころ (arm/feet)。
- **案6 角度空間の一発化**: M-C1 の相対角速度 dump で仮説を検証。葉位置の増幅を測って角度 vs 位置ドメインを判断。
- **スリム化**: st_filter 確定後に One-Euro/固定 EMA と関連 config を撤去 (end-state)。**注意**: position path は
  「pos_rms は floor-limited だが leg/foot orientation を駆動」なので rotation-only 化で安易に削らない。
- **部位別 param の YAML 露出**: 実機で効くと分かったノブのみ後で露出 (当面コード既定)。
- **hold/FK・valid 遷移**との相互作用の実機確認 (dropout 頻発シーンで誤「高速」判定が出ないか)。
- **SlimeVR 主出力**に切り替える場合は角度空間がネイティブ → 案6 と統合検討。
