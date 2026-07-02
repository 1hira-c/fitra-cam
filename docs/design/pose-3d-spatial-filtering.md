# pose-3d: 空間ベース・フィルタ層の導入 (spatial-filtering)

(着手日 2026-06-29 / 関連: issue #48, トラック pose-3d / vr-output, 既存設計 [pose-3d-locomotion-stability](pose-3d-locomotion-stability.md), [vr-output-one-euro-filter](vr-output-one-euro-filter.md))

## 背景 / 動機

**症状**: (1) 静止時のプルプル (jitter) (2) 全体の遅延 (latency) (3) 特定部位(足元)。

**現状のジッタ抑制は時間ベースのみ**:
- **Kalman** (`cpp/src/lift/kalman.cpp`) = kinematic-tree 時間平滑
- **One-Euro** (`cpp/src/slimevr/tracker_extract.cpp`) = 速度適応 low-pass

時間平滑は本質的に **「静止 jitter を消す ↔ 動作 lag」のトレードオフ**から逃げられない。

**着想**: jitter を時間ではなく**空間(多視点幾何 + 骨格構造 + 床)**で潰せば、時間平滑を弱められ lag が減る。

**ジッタの性質 (重要・前提)**: グリルの結論として、本リグの jitter は
**「片視点だけが外れる (a)」ではなく「各視点が独立に微動して 3D 点が累積的にずれる (b)」** が支配的
(ユーザー実感)。これが設計全体を決める:
- **(b) では 2-view 関節の再投影誤差は ≈0 のまま 3D 点だけ揺れる** (DLT が 2 視点をほぼ完全に満たすため)
  → 再投影誤差はジッタに対して**盲目**。3-view でも残差は「外れ値」でなく「全体ノイズ量」を反映するだけ。
- 一方 **view_count は (b) でも有効** (独立ノイズは視点数で分散 1/N)。
- (b) で止まっているのは**剛体構造** → 構造への射影が本命レバー。

**いま空間情報は揃っているのに下流で使われていない**:
- 三角測量は per-joint の `reproj_error_px[]` / `view_count[]` を算出済み (`triangulator.cpp:155-156`) だが Kalman は全 valid 関節を等価扱い。
- IK (`ik.cpp`) は既に骨長射影・対称性・ヒンジ角を毎フレーム・ラグ無しで適用 (= 空間フィルタの前例)。
  ただし `enforce_lengths` は親固定で子を**現在の(揺れた)方向**に沿って固定距離へ動かすだけ
  (`ik.cpp:206`) → **半径方向ジッタは消すが、接線方向ジッタとルートのジッタは素通り**。

**完了条件**: コア 6 関節の静止ジッタを有意に低減し、時間平滑を弱めた後も lag が非悪化(または改善)。
腰曲げ等の実動作を殺さない。既存 ctest 回帰なし。

## 検討した案

### 案1: robust 三角測量 (IRLS) を主役にする — 格下げ
当初は IRLS / robust loss で「片視点の暴れキーポイント」を源で除去する案。**(b) 前提では無力**:
2-view で reproj が盲目、3-view でも外れ値が存在しない。既存の三角測量精度は実用上十分という
実感もあり、新規 IRLS 投資の対効果は低い。→ **外れ値保険 (遮蔽・誤検出) の最小限のみに格下げ**、
ジッタ対策の看板からは外す。

### 案2: 時間フィルタを強化する — 没
Kalman / One-Euro のチューニングを詰める方向。これはトレードオフの内側を動くだけで、
ユーザーの主訴(トレードオフからの脱出)に応えられない。M4 として「空間で削った後の残差を軽く取る」
位置づけに**降格**して残す。

### 案3: IK の関節角 limit を pose-manifold 射影として拡張 — 見送り
SlimeVR `Constraint.kt` 相当の swing/twist 角制約で姿勢多様体へ射影する案。有効域内のジッタは
減らせず、(b) の接線ジッタには効きが薄い。コア剛体化(採用案)で大半を取れる見込みなので見送り。

### 案4: コア剛体フィット (採用・主役)
各視点が独立微動しても**骨盤・肩帯という剛体構造は止まっている**。骨盤(hip_center, l_hip, r_hip)と
肩帯(neck, l_shoulder, r_shoulder)をそれぞれ 1 枚の剛体として**重み付き Kabsch (Procrustes)** で
毎フレーム位置合わせ → 独立な per-joint ジッタがセグメント内で平均化され、特にルート(コア)が安定。
コアが安定すれば四肢も連鎖的に落ち着く。ラグ無しの純空間処理。SlimeVR の waist/chest 剛体分割と同発想。
**剛体テンプレートは既存の被験者プロファイルの距離だけで完全に決まる** (校正 UI 変更不要):
- 骨盤三角 = `bone_lengths_m[11]`, `[12]`, `hip_width_m`
- 肩帯三角 = `bone_lengths_m[5]`, `[6]`, `shoulder_width_m`
- 脊椎 hip_center→neck = `bone_lengths_m[18]`

### 順序: 空間 vs 時間 (案A 採用 / 案B 没 / 案C 保留)
- **案A (採用)**: `tri → 空間(剛体フィット+enforce_lengths+hinges) → 軽い時間平滑`。
  空間が構造を壊すジッタを即除去 → 残差だけ軽く時間平滑。**lag↔jitter を実際に動かせる唯一の順序**。
- **案B (没)**: `tri → Kalman → IK` 維持。時間平滑を先に強くかけると lag がそこで確定し、後段の
  空間整形では取り戻せない。
- **案C (保留)**: kinematic-tree Kalman に剛体を畳み込む (親相対オフセットをテンプレへ極小 Q で固定)。
  統一的だが複雑。将来検討。

### 脊椎カップリング (soft 採用 / hard・none 不採用)
- **hard 固定**: 深い前屈で弦長(直線距離)が縮むのに固定するので neck を遠くに置きすぎる → 腰曲げと相性悪い。
- **none (連結なし)**: 両端が剛体フィットで既にデノイズ済みなので成立はするが、フィット暴走時の乖離ガードが無い。
- **soft (採用)**: neck を脊椎軸方向に ±一定% (初期当たり ±12%、実機で詰める) まで自由、超えたら戻すバネ。
  腰曲げ忠実 + 片セグメント暴走時のガード。

### 信頼度の置き場 (コア/四肢で分担)
当初 M2「適応 Kalman R」は (b) で reproj が弱いため再設計:
- **コア**: 重み付き Kabsch (score × view_count を重みに) で信頼度を**空間融合**として吸収。
- **四肢・頭部** (剛体でない): 軽い Kalman の **per-joint 適応 R** (view_count・score 主体、reproj は補助)。
  → 二重計上を避け「コア=空間 / 四肢=時間」に分担。

## 採用設計

データフロー (案A):
```
triangulate → [calib tap: 生角度] → 空間ステージ → 軽 Kalman → (床アンカ: 後日) → tracker extract(One-Euro 軽め)
                                       │
              ┌────────────────────────┴────────────────────────┐
              │ 1. 骨盤 {19,11,12} 重み付き Kabsch (valid 3 のみ)  │
              │ 2. 肩帯 {18,5,6}   重み付き Kabsch (valid 3 のみ)  │
              │ 3. 脊椎 soft 連結 (neck を骨盤へ ±%でピン)          │
              │ 4. 四肢 enforce_lengths + enforce_hinges (既存)    │
              │ 5. 頭部 enforce_lengths (既存)                     │
              └─────────────────────────────────────────────────┘
```

**不変条件**:
- 剛体フィットは **valid 3 点そろったセグメントのみ**。足りなければそのフレームだけ**既存
  enforce_lengths + 軽 Kalman へ状態レス・フォールバック** (今日の挙動より悪化させない)。
- 剛体ステージは**状態を持たない** (テンプレは静的、フィットは毎フレーム独立)。idle/standby で
  リセットすべき状態は軽 Kalman / 床アンカ側のみ (既存パターン)。
- 肩帯は自分のフィット向きを保ち、neck を脊椎 soft 拘束で骨盤へ連結 = **neck 基部ボールジョイント**
  (脊椎の捻り/反りは肩帯フィットに任せ、骨盤とは長さだけで連結)。
- 校正タップ (角度認識用の skeleton, `multi_pipeline.cpp:442-462`) は**三角測量直後**へ移動。
  構造射影前の素の角度を測れる (案A の副次効果)。
- 軽 Kalman は**既存 `SkeletonKalman` を構造そのまま流用**。M4 で process noise を上げて弱める。
  四肢の `correct()` に per-joint 適応 R を差す (`update` に任意引数追加、未指定で従来挙動 = 回帰ゼロ)。
  locomotion-freeze 対策は温存。

**主要しきい (初期当たり、実機で詰める)**:
- 脊椎 soft 拘束: 軸方向 ±12%。
- Kabsch 重み: `w_j = score_j · g(view_count_j)` (view 2 < view 3 を底上げ程度)。
- 床アンカ (M-D): 接地速度 ~2.4 m/s / 床距離 0.025–0.075 m (SlimeVR LegTweaks 準拠)。

## Milestone

一度に一つだけ変えて切り分け計測する。

- **M0** (このコミット): 設計 doc。
- **M-infra**: 検証ハーネス。`cpp/tools/dump_keypoints_3d.cpp` を **3-cam 対応 + ステージ個別トグル +
  per-joint 3D 位置ダンプ**へ拡張。**未使用ツールのため、まずライブ結果と突き合わせてスモークテスト**
  し、ゲートに使えるか判定 (壊れて修正コスト高ならライブ WebUI テレメトリ + 目視へフォールバック)。
  指標スクリプト (per-joint 標準偏差 = jitter / step・相互相関 = lag)、静止・動作(腰曲げ含む)クリップ録画。
- **M-A**: 骨盤剛体フィットのみ。重み付き Kabsch、valid-3 フォールバック、**空間-first へ並べ替え**。
  **Kalman の強さは据え置き** (変更は一度に一つ)。コアジッタ低減を計測。
- **M-B**: 肩帯剛体 + 脊椎 soft 連結。
- **M-C**: 時間フィルタを弱める (M4) + 四肢適応 R。軽 Kalman 化 + One-Euro 再調整。
  lag 低減・jitter 非悪化を計測。
- **M-D** (保留): 床アンカ。最小ソフトクランプ、脚チェーン再解なし、既定 OFF。design-doc-first。
- **M-E** (任意): ライブテレメトリ (per-joint reproj / 接地割合を WS3D stats に追加)。

## 実装記録

### M-infra (2026-06-30) — 検証ハーネスのソフト側

オフライン再生ハーネスの**ソフト側**を実装 (録画 + ライブ突き合わせは実機作業として後述)。

- **`tools/dump_keypoints_3d` を N カメラ (2 or 3) 化**: 2 視点固定 (`cam0`/`cam1` ハードコード)
  だった解析ツールを、`--video` の本数 (2 or 3) でカメラ数を決める一般形へ。calib は
  `expected_camera_ids(n)` (= `cam0..cam{n-1}`) で trim (`select_calib_cameras` /
  `require_camera_ids`、`make_threed` / `threed_builder` と同じ正規化) するので、3 カメラ
  extrinsics を 2 or 3 視点どちらでも食える。`VideoPair::videos` を `vector` 化、`pose_session`
  の clips 上限 2 を撤廃 (全 pose で同一カメラ数を要求)、`caps`/`frames`/`overlays`/
  `persons_by_cam` のループ・コンテナを `n_cams` 化。同期オフセットは `--cam2-frame-offset`
  を追加 (既存 `--cam1-frame-offset` は不変、負値は共有参照 cam0 をスキップ、二重指定は max)。
  triangulator/Kalman/IK のコアは無改変。
- **per-joint ダンプ拡充**: 各フレームの `stats` に `joint_reproj_px[]` を追加
  (`joint_view_counts[]` は既出)。解析側が per-joint の reproj / view_count を読める。
- **指標スクリプト `tools/analyze_3d_jitter_lag.py`** (numpy のみ依存、ハーネスと co-locate):
  - `jitter <file...>`: 静止クリップの per-joint 3D 位置 RMS (mm)。コア 6 関節
    {18,5,6,19,11,12} を強調、複数ファイル渡しで ON/OFF 差分表。
  - `lag <baseline> <candidate>`: 同一動作クリップの指定関節軌道を相互相関し、candidate の
    baseline に対する遅延 (frames → ms、サブフレーム放物線補間) を算出。静止クリップ誤用は
    motion sigma しきいで警告。
  - 合成クリップでスモーク済 (jitter ON/OFF −58%、lag 注入遅延 +4f を corr 1.0 で復元)。
- **3 カメラ同期 raw recorder `tools/record_3cam`**: 静止 / 動作(腰曲げ)クリップの録画手段。
  既存 2 カメラ Python recorder (`record_dual_rtmpose_overlay.py`) は 2 視点のみなので新規。
  **config 駆動** (`--config session.yaml` = ライブ `run` と同一) で `camera_builder` と同じ
  per-camera `V4l2Options` (解像度 / cap override / pixel format / 露出) を組む。**幾何 (解像度 /
  crop / 露出) はライブと一致**するが、**デコードは非一致** — 本ツールは常に CPU `cv::imdecode`、
  ライブの `pixel_format: nvjpeg` は HW NVJPEG デコードなので画素が僅かに異なりうる (dump は
  ライブを *近似* 再現・内部整合で jitter 数値は有効、bit 一致ではない。dedup 共有化は残課題)。
  YUYV は `cvtColor`、downscaling は INTER_AREA で出力解像度へ (FrameSource と同形)。
  **全カメラに fresh フレームが揃ったときだけ各 `raw_cam{i}.mp4` に 1 枚ずつ書く** index 同期
  ペーシング (最遅カメラが律速・重複フレーム無し・等長クリップ → dump の index ペアリングに整合)。
  ウォームアップで最遅 recv_fps を測って出力 fps を決め、`meta.json` + 次手 (dump コマンド) を案内。
  TRT 非実行 (link で `fitra_infer` を引くが未使用)。
- **未 (実機 = M-infra 残)**: (1) **ライブ突き合わせスモークテスト** — このツール群は長らく未使用
  なので、まず `record_3cam` で録画 → `dump_keypoints_3d` のオフライン結果を WebUI 3D ライブと
  突き合わせ、ゲートに使えるか判定 (壊れて修正コスト高ならライブテレメトリ + 目視へフォールバック)。
  recorder 自体の初回実機実行がこのスモークテストを兼ねる。(2) **初回ベースライン採取**
  (静止・腰曲げクリップ) → jitter/lag 指標の現状値を取り、受け入れ基準の数値を確定。

### M-infra ベースライン (2026-07-01) — 実データで確定

実機で 6 クリップ録画 (`record_3cam`、静止3: `still`/`still_t`/`still_crouch`、動作3:
`forward_bend`/`bending_and_stretching`/`walk_around`、各 1280×960×3cam)。ハーネス全体
(`record_3cam` → `dump_keypoints_3d` 3cam → `analyze_3d_jitter_lag`) が **実 3 カメラで
end-to-end 動作**することを確認 = **M-infra スモークテスト合格 (オフラインをゲートに採用可)**。
median reproj ~3.8px、26 関節 valid。

**静止 jitter ベースライン (コア6関節 median 3D RMS, raw 三角測量)**:
| clip | core median | 最悪関節 | pelvis {19,11,12} |
|---|---|---|---|
| still (立位) | 6.4mm | r_shoulder 9.0 | 4.4–6.3mm |
| still_t (T字) | 9.2mm | l_shoulder **19.0** | 6.4–9.3mm |
| still_crouch (中腰) | 12.2mm | 肩帯 17–19 | 7.1–7.2mm |

**最重要の観測**: **現状の Kalman+IK は静止コアジッタをほとんど落とさない** (delta ±0–7%、
hip_center のみ −15〜−20%)。= (b) ジッタ (各視点独立微動 → 3D 点が累積スキャッタ) は時間平滑に
盲目、という設計の前提を実データが裏付けた。時間フィルタでは頭打ち → **空間 (剛体フィット) が
本命**という方針が正当化された。**肩帯 {18,5,6} が最悪 (中腰・T字で 17–19mm)・骨盤 {19,11,12}
は最良 (~5–9mm)** → M-B (肩帯剛体) の伸びしろが最大、M-A (骨盤剛体) は既に安定な骨盤をさらに固める。

**lag** (forward_bend、filtered vs raw): hip_center/neck とも filtered が raw に対し **~66ms 遅延**
(corr 0.99)。ただし後述のとおり 8.5fps 録画で 117ms/frame と粗く、サブフレーム補間頼み。

**暫定受け入れ基準** (M-A 初回計測でさらに詰める):
- **M-A/M-B**: 肩帯 {18,5,6} の静止 RMS を raw 比 **≥30% 低減** (T字/中腰クリップ、伸びしろ最大)、
  骨盤 {19,11,12} を非悪化。コア6 median を有意に低減。
- **M-C**: 時間フィルタ弱化後も lag 非悪化 (raw 比増加なし)。
- 数値は絶対でなく **同一クリップの raw 比**で見る (ポーズで 6–12mm と変動するため)。

**recorder の既知の制約 (要 follow-up)**: `record_3cam` は 1280×960×3cam で **~8.6fps** 律速
(内訳: mp4v encode 30ms/frame + imdecode 23ms/frame をメインスレッド直列 = 3cam で ~158ms/set。
**encode が支配的**)。かつ MP4 ヘッダ fps が公称 (~59) で真値と乖離。**暫定運用**: `dump --fps` /
`analyze lag --fps` に `meta.json` の `fps_written` を渡す (Kalman dt を実時間に矯正)。静止 jitter は
fps 非依存なので M-A/M-B のドライバ計測には影響なし。**lag を精密化する M-C 前に**、per-camera
encode スレッド化 (+ FrameSource で HW nvjpeg デコード) で ~25–40fps へ引き上げる recorder v2 が要る。

### M-A (2026-07-01) — 骨盤剛体フィット

- **`lift/rigid_fit.{hpp,cpp}`** (fitra_lift, 汎用・M-B 肩帯でも再利用): `RigidTemplate::from_distances`
  が 3 辺距離から剛体テンプレ (canonical 3 点) を構築 (三角不等式 + 「apex の垂線高 < 5mm =
  ほぼ共線 → 3D 向き不定」で退化 reject)。`fit_rigid_triangle` が**重み付き Kabsch** (rotation +
  translation, no scale, det 符号で反射禁止) でテンプレを測定 3 点にフィットし、剛体化 (denoise)
  した位置を返す。ステートレス・毎フレーム独立。ctest `test_rigid_fit` (形状保存・既知 R,t 復元・
  denoise・重み・退化/ゼロ重み reject)。
- **`dump_keypoints_3d` 配線**: `--subject-profile PATH` (IK ロック + 剛体テンプレ供給、ライブと同源)
  と `--rigid-pelvis` を追加。ON で **案A 空間-first に並べ替え** (`tri → rigid_pelvis → IK →
  Kalman`)、骨盤 {hip_center 19, l_hip 11, r_hip 12} を **valid-3 gate** + 重み `w=score·view_count`
  でフィット。OFF は現状 (`tri → Kalman → IK`) をビット不変で維持。テンプレは profile の
  `bone_lengths_m[11]/[12]` + `hip_width_m` から (身長 anthropometry は hip_center を股関節中点に
  置き**共線退化**するため不可 = 実データ由来必須)。COCO17 は hip_center 不在で reject。
- **計測 (baseline=Kalman+IK+profile vs M-A=rigid+IK+Kalman、同一クリップ、fps=meta.fps_written)**:
  | clip | hip_center 19 | l_hip 11 | r_hip 12 | 肩帯 {18,5,6} |
  |---|---|---|---|---|
  | still (立位) | **−20.7%** | −11.9% | −15.0% | ±1%(不変) |
  | still_t (T字) | −12.0% | −11.7% | −10.5% | ±1%(不変) |
  | still_crouch (中腰) | −1.7% | +1.6% | +0.7% | ±1%(不変) |
  - **骨盤剛体フィットは立位/T字で骨盤ジッタを −10〜−21% 低減**。肩帯は設計どおり不変 (M-B の担当)。
    中腰は横ばい (骨盤が既に raw 近傍 ~7mm + 骨盤三角が浅く hip_center が軸から ~2.5cm しか外れず
    向き拘束が弱いため伸びしろ小)。**下流破壊なし** (肩帯・四肢は不変)。
  - M-A スコープ (骨盤のみ) は達成。**コア全体の本命 = 肩帯 {18,5,6} (14–16mm と最悪) は M-B**。
- **測定の妥当性 (TRT 非決定性)**: パイプライン (TensorRT FP16 推論) は**非決定的** — 同一コマンド
  2 回で 173/175 フレームが末尾桁レベルで相違する。ただし集計 jitter (per-joint stddev) の
  **run-to-run ばらつきは ~0.02–0.05mm** に収まり、M-A 効果 ~0.6–1.1mm の **20–50 分の 1**。よって
  **≥0.5mm 程度の効果は信頼できる** (byte 一致は測れないので stddev の run 間安定性で担保)。

### M-A ライブ配線 (2026-07-01)

計測で骨盤剛体フィットの効果を確認したのでライブ `multi_pipeline` に配線。**既定 OFF のフラグ**
`three_d.rigid_pelvis` (CLI `--rigid-pelvis`) でゲートし、実機 WebUI 3D viewer で A/B 目視できる。

- 共有化: `apply_segment_rigid_fit` / `apply_spine_coupling` を `lift/rigid_fit` へ移し、harness と
  ライブが**同一コードでデノイズ** (オフライン計測がライブを忠実に再現)。
- `MainOptions.rigid_pelvis_3d` (YAML `three_d.rigid_pelvis` / CLI `--rigid-pelvis`) →
  `ThreeDConfig.rigid_pelvis` → `MultiCameraDriver`。ドライバは構築時に subject profile 距離から
  骨盤テンプレを組み、**Halpe26 + profile ロード済 + 非退化**のときだけ `rigid_pelvis_active_`。
- `maybe_update_3d`: active のとき**案A 空間-first** (`tri → 骨盤剛体 → IK → Kalman`) に分岐、
  それ以外は従来 (`tri → Kalman → IK`) を**byte 不変**で維持。calib-subject は profile 不在ゆえ
  active にならず、測定角 tap のセマンティクスは無変更。M-B 肩帯はライブ未配線 (効果小のため保留)。
- 検証: full build + ctest 32/32 パス (`test_main_config` の新キー含む)。既定 OFF で本番挙動不変。
  実機での rigid ON/OFF 目視 (静止プルプル・腰曲げ・追従) はユーザー作業。

### M-B (2026-07-01) — 肩帯剛体 + 脊椎 soft 連結（＋重要な負の所見）

- **実装**: `apply_rigid_segment` を汎用化 (M-A 骨盤と共用) し肩帯 {neck 18, l_sh 5, r_sh 6} に適用。
  `apply_spine_coupling` = neck の hip_center 距離を脊椎長 ±`--spine-tol` (既定 12%) にクランプ、
  超えたら肩帯 {18,5,6} を剛体平行移動 (向き保持 = neck 基部ボールジョイント)。`--rigid-shoulders`
  で有効、順序 骨盤→肩帯→脊椎連結→IK→Kalman。テンプレは profile の `bone_lengths_m[5]/[6]` +
  `shoulder_width_m`、脊椎長 = `bone_lengths_m[18]`。
- **計測 (baseline vs M-B, 静止, コア6)**: 肩帯は **立位 neck −6%/肩 −3〜5%・T字 ±1%・中腰 −2〜3%**
  と**わずか**。骨盤は M-A どおり −10〜20%。Kalman OFF の純空間でも肩帯は **±0〜+5% (改善せず)**。
- **重要な負の所見 = 剛体フィットの効きは segment の再投影誤差 (独立ノイズ量) に比例する**:
  骨盤の per-joint reproj は 7–10px (視点間不一致大 = 独立ノイズ大 → 剛体平均で −10〜18% 取れる)。
  肩帯は reproj 2–4px と**低い** (視点間で一致 = 形状破壊ノイズが少ない) → 剛体フィットで消せる成分が
  そもそも少ない。肩帯の 10–19mm ジッタは**コヒーレントな剛体ポーズ動揺** (girdle 全体が揺れる) で、
  剛体フィットは 6-DoF ポーズ動揺を保存するため消せない。→ **設計の当初前提「肩帯が最大の伸びしろ」は
  実データで否定**。肩帯ジッタは空間(剛体)ではなく**時間フィルタ (M-C) か 2D 改善**の領分。
  最悪は l_shoulder (RMS 19.5mm・間欠 2-view 85%・reproj 4.2px) の構造的誤差。
- **脊椎 soft 連結は IK の hard enforce_lengths と冗長**: Halpe26 では neck の親 = hip_center なので
  IK が hip→neck を毎フレーム脊椎長へ hard 固定 (soft ±12% より厳しい)。よって soft 連結は実質不活性
  (IK OFF 時 / 大excursion のガードとして残す。害はない)。
- **腰曲げは保持** (forward_bend: M-B の neck z-range 781mm ≥ raw 735mm)。むしろ raw は脊椎を
  0.515m と短く三角測量するのを M-B が校正長 0.56m へ復元。**下流破壊なし** (骨盤不変・bend 保持)。
- **結論**: M-B は無害だが静止ジッタへの寄与は小。**コアの残ジッタ (肩帯) は M-C (時間フィルタ調整) と
  2D 側**へ引き継ぐ。剛体フィットは骨盤 (高 reproj) が主戦場という理解に更新。

## 検証

**主検証 = 決定的オフライン再生** (M-infra のハーネスが使える前提):
- 静止クリップ → コア 6 関節の per-joint 位置標準偏差を各ステージ ON/OFF で比較。
- 動作(腰曲げ含む)クリップ → step / 相互相関で lag を計測、M-C 前後で比較。
- 受け入れ基準: 「コア静止ジッタを有意に低減」かつ「M-C 後に lag 非悪化」。**数値目標 (例: 静止ジッタ
  ○mm 以下) は初回ベースライン取得後に確定**。

**ctest**: `test_triangulator` / `test_kalman_chain` を拡張 (Kalman は適応 R 未指定で従来一致 = 回帰ゼロ)。
剛体フィット (Kabsch・脊椎 soft・valid-3 フォールバック) の新規ユニットテスト。

**実機**: WebUI 3D viewer で静止プルプルの低減 / 腰曲げ忠実 / 全体追従感を目視。

## 残課題

- **M-infra のオフライン再生が動かない場合**のフォールバック (ライブテレメトリ + 目視)。3-cam 化と
  同時にスモークテスト必須。
- **脊椎 soft 許容幅・Kabsch 重み関数・床アンカ閾値**の実機チューニング。
- **案C (Kalman へ剛体畳み込み)**: M-A/B で剛体ステージと軽 Kalman の二段が冗長と判明したら統合検討。
- **四肢の接線ジッタ**: 剛体でないため空間では消せず、軽 Kalman + One-Euro の時間平滑が責務。
  伸展肢で残る場合は別途検討。
- **床アンカの脚チェーン再解**: 最小ソフトクランプで沈み込み/skating が残る場合のみ着手。
