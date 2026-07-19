# Track: pose-3d

2D keypoint から **3D pose / bone tracker** を起こす経路。lift / IK / Kalman / roll 品質 /
subject calibration。vr-output トラックの上流 (= tracker の単一 producer) を担う。

## 現状 (2026-06-12)

`SlimeTrackerBus` + `TrackerExtractor` が tracker snapshot の **単一 producer**。
Firmware UDP / VMT publisher / WebUI viz が同じ smoothing 履歴を共有する。Kalman は
**kinematic-tree (root = hip_center, children = parent-relative offset)** で動く。

**プロセスは排他 RunMode (`run` / `calib-subject` / `calib-extrinsic` /
`calib-extrinsic-floor` / `calib-intrinsic`) で動く**
([design/pose-3d-calib-mode-separation.md](../design/pose-3d-calib-mode-separation.md) M1–M4
実装済み)。mode は既存フラグから導出 (`--calibrate` → calib-subject、`--extrinsic-calib` or
`--excal-replay` → calib-extrinsic、`--floor-calib` or `--floor-replay` → calib-extrinsic-floor)。
**extrinsic 校正は 2 方式**: controller-marker hand-eye (案C、VR コントローラ固定マーカー) と
floor-apriltag PnP (案D、床に既知配置したタグへ各カメラを個別 localize・VR 不要、
[design/pose-3d-floor-apriltag-extrinsic.md](../design/pose-3d-floor-apriltag-extrinsic.md))。
WebUI `/extrinsic-calib` の方式トグル (= flow-switch) で選択でき、案D は出力 `T_cw` を
fitra Z-up で無変換書出。calib↔runtime の契約は **YAML 成果物のみ**
(CalibrationSet / SubjectProfile) + プロセス再起動 — ライブ再注入 (triangulator ホットスワップ /
IK ホットリロード / tap mux) はコンパイルレベルで存在しない。構築は `cpp/src/app/` の
builder + モード runner (main.cpp は dispatch のみ)。calib-extrinsic は TRT 非依存の
decode-only で、`--excal-replay <dir>` により `tools/excal_record` セッションから実機レスで
solve まで再現できる (live↔replay 等価性は `test_excal_replay` で固定)。

**主経路は flow daemon** ([design/pose-3d-flow-daemon.md](../design/pose-3d-flow-daemon.md))。
`./main --daemon --config session.yaml` がモードモジュール (同一バイナリ + モードフラグ) を
1 つずつ fork/exec し、exit code (80/81/82) で連鎖する: excal solve → calib-subject →
approve → run の自動連鎖、`POST /api/flow/switch` による任意切替、クラッシュ時は run へ
自動復帰 (3 連続で give-up)。daemon 自身はソケット/CUDA/TRT に触れない。モジュールは
`--flow-managed` でのみ flow 挙動が有効になり、手動起動は従来どおり (exit 0 + 再起動案内)。
web は `/flow.js` が `/api/state` を追従し、タブ 1 枚で 3 段が完結する。
運用手順は [runbook-pose-3d-calibration.md](../runbook-pose-3d-calibration.md)。

### 設計原則 / live な制約

- **伸展補正は tracker 専用・製品既定 ON**: `limb_extension_snap` は元の Skeleton3D を変更せず、
  `extract_trackers` の private copy 上だけで腕/脚を直線化する。`extended_leg_toe_direction` は伸展脚の
  thigh/shin twist を観測済み `ankle→big_toe` から作り、欠損時は既存 held roll へ戻る。両機能は
  YAML の個別 `false` または `--no-limb-extension-snap` / `--no-extended-leg-toe-direction` で停止可能。
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

### 2026-07-15 — 四肢伸展スナップ / 足先方向推論を製品既定 ON 化

`MainOptions` と直接構築時の `TrackerExtractorOptions` で `limb_extension_snap` /
`extended_leg_toe_direction` を両方 ON に昇格した。既存 YAML の明示 `false` はそのまま尊重し、CLI
から即時 A/B・切り戻しできる `--no-limb-extension-snap` / `--no-extended-leg-toe-direction` を追加。
低レベル `extract_trackers()` の引数既定は旧出力との厳密比較用に OFF のまま維持し、製品経路との
違いをテストで固定した。

### 2026-07-13 — 四肢伸展スナップ + 伸展脚の足先方向推論（既定 OFF）

ほぼ伸び切った腕・脚を tracker 専用の private skeleton copy 上で一直線へ射影し、VMT の位置と
VMT/SlimeVR の回転を同じ幾何へ揃える `limb_extension_snap` を追加。伸ばす途中は flexion 20°で
enter、曲げる途中は 12°で exit する逆向き per-limb hysteresis とし、移動量 + 連続 2 frame の
方向確認で単純な閾値交換による再吸着と 1-frame spike を防ぐ。伸展脚では `ankle→big_toe` を脚軸へ
直交射影し、thigh / shin の共通 twist 基準にできる `extended_leg_toe_direction` も追加した。
toe 欠損・退化時は既存 held-roll + parent-yaw transport へ戻り、world-axis roll は作らない。
両機能は個別・既定 OFF、閾値も YAML/CLI で A/B 調整可能。設計と検証手順は
[design/pose-3d-limb-extension-snap.md](../design/pose-3d-limb-extension-snap.md)。

### 2026-06-27 — calib-latest 解決の堅牢化（レビュー指摘対応・コードレビュー / bot 指摘） (バグ修正)
latest 解決 PR に対する自動/手動レビューの指摘をまとめて対応。
**(1) 明示 Run の missing calib クラッシュループ**: `--enable-3d` + extrinsics 未生成のとき、daemon の
crash-fallback / `--daemon-initial run` / WebUI run 切替 / standalone `./main --enable-3d` のいずれも
`make_threed`→`load_calibration` で fatal し、daemon が3連敗 give-up していた。`run_mode_run` を try/catch
し**校正不在は 2D へ degrade**（precheck の「Run は missing calib を 2D 許容」契約と一致）。
**(2) build_excal_session の load 非対称**: floor 版が try/catch で nullptr 返すのに controller 版は素通しで
throw が `fatal:` に漏れていた → 対称化。
**(3) `clear_calib_latest` の例外**: ヘッダの "Does not throw" 契約に反し `fs::is_symlink(p)` が OS エラーで
throw しうる → `fs::is_symlink(p, ec)` に。
**(4) calib のカメラ順序非対称**: `make_threed` / `dump_keypoints_3d` の trim を**無条件 `select_calib_cameras`**
へ一般化（`size>n_cams` 限定を撤廃、`[cam1,cam0]` 等の順序ズレも吸収。`CalibrationSet` 全フィールド保持を確認）。
**(5) 破損 subject profile の present 誤判定**: `subject_profile_compatible` を schema peek から**全 load+validate**へ
（schema 一致でも subject_id 欠落 / bone 不足 / quality=fail の部分破損は「未校正」扱いで CalibSubject へ routing）。
`test_flow_daemon` の chain fixture を valid profile へ更新。
**(6)** subject 校正の再起動ヒント文字列が raw `opts.calib`（空）を埋め込み `--calib `(空) を案内 → `effective_extrinsics_path`
解決へ。**(7)** setup の latest クリアに `floor_out` 追加、daemon 起動時 `profile_now()` 二重 open 解消、
stale コメント / 例 config (`live_2cam_3d`) の焼かれた読み既定を撤去。ctest 31/31 パス。

### 2026-06-27 — dump_keypoints_3d も3カメラ extrinsics を2視点に trim（解析クラッシュ修正）+ 共有ヘルパー化 (バグ修正)
subject 校正のライブ計測は `make_threed` の trim で2視点化していたが、**録画後の解析**で使う
`dump_keypoints_3d`（2カメラ専用ツール）が3カメラ calib を読み `require_camera_ids({cam0,cam1})`
で同じ camera-id 不一致 fatal を起こしていた（`expected [cam0,cam1], got [cam0,cam1,cam2]`）。
trim ロジックを `lift::select_calib_cameras(calib, ids)` に共通化し、`dump_keypoints_3d` と
`make_threed` の両方から使用（require_camera_ids の本番呼び出しは triangulator.hpp 除き 2 箇所、
両方カバー）。ctest `test_calib_io` に `select_calib_cameras`（順序追従・欠損 id スキップ）を追加。

### 2026-06-27 — subject 校正の "calibration YAML not found"（最後の未解決 calib パス）を修正 (バグ修正)
calib-latest-resolution で `three_d.calib`(読み) は config 既定空・実行時 `effective_extrinsics_path`
解決にしたが、`mode_calib_subject.cpp` の preflight に渡す `calib_yaml` だけ **raw `opts.calib`(空)**
のままで、preflight (`calibration_session.cpp:191`) が `"calibration YAML not found: "`(空パス) を
出し subject 校正が起動できなかった。`calib_defaults.calib_yaml = config::effective_extrinsics_path(opts)`
に変更（他の読み口 threed_builder / precheck と同じ解決経路へ）。監査の結果これが最後の未解決 raw
読み口（残る `opts.calib` の raw は daemon の警告ログのみ・空時ガード付き）。

### 2026-06-27 — subject 校正(2視点)が3カメラ extrinsics で camera-id 不一致クラッシュする問題を修正 (バグ修正)
subject 校正は cam0+cam1 の2視点だけ使う設計だが、`make_threed` が3カメラの extrinsics 全体で
triangulator を作り `require_camera_ids([cam0,cam1])` が**完全一致**を要求 → `expected [cam0,cam1],
got [cam0,cam1,cam2]` で fatal（3カメラリグで subject 校正が必ず落ちる）。`make_threed` で calib の
カメラ数が `n_cams` より多いとき、`expected_camera_ids(n_cams)`(= cam0..cam{n-1}) を順序どおり
選んで calib を trim してから triangulator を構築（余分なカメラの calib entry は未使用＝コメントの
設計意図を実装）。run（n_cams=calib 数）は trim 無しで不変。これで profile-compat が
「非互換 profile → CalibSubject へ routing」した先で実際に再校正が通る。

### 2026-06-27 — run は非互換/欠損 subject profile を fatal にせず警告して継続 (バグ修正)
subject profile は IK 精度のための**任意入力**なのに、`threed_builder` の `load_subject_profile`
が schema 不一致（v1/COCO17 を halpe26 で読む等）で **fatal throw** → run がクラッシュしていた。
profile-compat は daemon の通常チェーンを「非互換 → CalibSubject へ routing」にしたが、**crash-
fallback to run**（例: CalibSubject が別件でクラッシュ → run へ fallback）や手動 run はその routing を
通らず、run が v1 profile を読んで fatal → 3連敗 give-up のループになっていた。`threed_builder` の
profile ロードを try/catch で囲み、失敗時は **警告して profile 無しで継続**（height prior + 既定 bone
長で 3D は動く）。全経路（通常/fallback/手動）をカバー。

### 2026-06-27 — setup で intrinsic 校正の レンズモデル(pinhole/fisheye) を選択可能化 (バグ修正)
setup ウィザードに intrinsic 校正の `model` 選択が無く、`/api/config` 往復も `intrinsic_calib`
は `enabled`/`out` しか運ばなかったため、**model は常に既定 `pinhole` 固定**。ELP AR0234 等の
魚眼レンズで pinhole intrinsic solve が走り、高 RMS で受け入れゲート落ち + 収束遅延（「内部校正が
失敗する・solve が遅い」の主因）。`merge_config`/`draft_to_json` に `intrinsic_calib.model` を追加、
`ConfigIntrinsicCalib` 型に `model`、SetupPage の「内部校正で生成」ON 時に pinhole/fisheye セレクタ
を表示。YAML 層 (`intrinsic_calib.model`) は既対応。注: ChArUco 盤面寸法 (squares_x/y 等) はまだ
setup 非露出で既定 5×7 のまま（実物盤面が 7×5 なら config で別途設定が必要 = degenerate 回避）。

### 2026-06-27 — 校正成果物を run-time latest 解決 + setup で latest クリア
校正ファイル（intrinsics/extrinsics）は**生成物**なのに、`--enable-3d requires --calib PATH`
が生成物のパスをユーザーに要求し、daemon に生 config を渡すと calib 空で即死していた。原則
「生成物は **明示 > latest > 校正へ routing** で解決し、無いと作る（hard-error しない）」へ。
**書き込み先**(excal_out/floor_out/intrinsic_out) の既定を `calibrations/<kind>/latest.yaml`
にし `write_calibration_versioned` で `<ts>.yaml` + latest symlink を生成。**読み**は config に
既定値を焼かず（`three_d.calib` 空のまま）、`config::effective_extrinsics_path`(calib ▸ excal_out)
/ `effective_intrinsics_path`(explicit ▸ intrinsic_out が在れば ▸ extrinsics) で解決。daemon/setup
の存在判定・threed_builder・extrinsic/floor の intrinsics 入力・precheck を解決経由に変え、
`--enable-3d requires --calib` を撤廃（未校正→校正へ routing）。**手動 setup 開始で
`lift::clear_calib_latest` が latest ポインタ symlink のみ削除**（timestamp 履歴は残す）＝
「入ったら作り直す」。明示パス（既存 `extrinsics_1.yaml` 運用）は最優先で無変更。実機: 生 config
（enable_3d+calib空）で `initial mode: calib-intrinsic` に routing し hard-error 解消を確認。
ctest: `test_calib_io`(versioned write/clear=symlinkのみ) + `test_main_config`(enable_3d は calib
非必須へ更新)。31/31 パス。**UI(M2)**: `SetupPage` から校正ファイルパス欄を撤去（生成物なので
手入力不要 = 手動入力最小限）、`normalizeCalibPaths` を恒等化して `three_d.calib` を config に
焼かない（空＝解決）。明示固定は YAML 直編集に倒す。`floor_map` は外部入力なので残置。**サンプル
config**(`configs/*.yaml.example`) も latest スキームへ更新（`setup_first` は明示 calib パス撤去、
`intrinsic_calib` の out は latest、`live_2cam_3d`/`medium_3d_floor_calib` の stale な
`measure_session/cam_params.yaml` を解消）。前回没にした「latest を setup 既定値に焼く」案との差は
design doc の「検討した案」参照。
→ [design/pose-3d-calib-latest-resolution.md](../design/pose-3d-calib-latest-resolution.md)

### 2026-06-27 — subject profile の presence 判定を schema-aware 化 (非互換は再校正へ) (バグ修正)
flow daemon / setup の profile 存在判定が `std::filesystem::exists()` だけで、keypoint
topology の不一致を見ていなかった。例: config が `halpe26` なのに `subject01` の既存
プロファイルが `v1`(coco17 時代) だと、「ファイルがある＝present」と誤判定して run に進み、
run の `validate_subject_profile` が schema 不一致で fatal → daemon が crash-loop して
3 連敗で giving up していた (再校正にも飛ばない)。`lift::subject_profile_compatible(path)`
を追加 (profile の `schema` だけ読み `subject_profile_schema(active_keypoint_format())` と
照合、欠損/不正/不一致は false・throw しない) し、`daemon.cpp::profile_now()` と
`mode_setup.cpp::derive_next_mode()` を `exists` から compatible 判定へ差し替え。非互換/古い
プロファイルは「未校正」として **自動で CalibSubject へ routing** されるので、halpe26 に
切り替えても subject を選び直さずに再校正で復帰できる。実機 (subject01 v1 + halpe26 config)
で `initial mode: run`→`calib-subject` に変わり crash-loop が解消することを確認。ctest:
`test_flow_daemon` の chain fixture を schema 付き実プロファイルへ更新。

### 2026-06-17 — intrinsics 解像度コンバータ (高解像度で校正→低解像度で実行)
マーカー/ChArUco 検出は高解像度が要るが、ランタイムは低解像度で fps を稼ぎたい。triangulator は
K をスケールしないので、校正(1280×960)のまま 640×480 で回すと K が2倍ズレて三角測量が崩れる。
`lift::scale_intrinsics(Intrinsics, w, h)`(fx,fy と主点を画素中心 −0.5 規約でスケール、歪み係数は
正規化座標で定義されスケール不変なので不変、アスペクト変化は例外)+ `tools/scale_intrinsics`
(CalibrationSet を読み intrinsics を目標解像度へ、extrinsics は解像度非依存でそのまま通す)を追加。
案D の `floor_out_intrinsics` 設計と整合。ctest: `test_calib_io` に scale ケース。実機: 1280×960
校正 → 640×480 実行で 3D fps 回復・スケール一致を確認。

### 2026-06-17 — fisheye の solve_tag_pose 再投影クラッシュ修正
`solve_tag_pose` の fisheye 分岐の再投影で `cv::fisheye::projectPoints` に Point2f 出力を渡しており、
(double の object 点から出力点型 Point2d を要求するため) OpenCV の create() 型アサートで abort。
実機の fisheye intrinsics で floor-calib を回した初回に発火 (合成テストが fisheye 経路を踏んで
いなかった)。出力を Point2d に分離して修正、`test_apriltag_marker` に fisheye=true 回帰テスト追加。

### 2026-06-17 — intrinsic 校正に受け入れゲート (退化解の書き出し防止)
盤面寸法の転置 (squares_x/y) や square/marker/dict の取り違えは intrinsic solve が「通る」のに
rms 数百 px・異方 K の退化解になり、書き出すと extrinsic/triangulation を静かに壊す (実例:
ChArUco 5×7↔7×5 転置で rms 203px↔0.72px、リグの `intrinsics.yaml` も同転置で 137px 退化)。
`IntrinsicCalibSession::solve_and_write` に **受け入れゲート**を追加: `rms_px > max_rms_px`
(既定 1.5) または `|fx-fy|/max(fx,fy) > max_fxfy_aniso` (既定 0.25) なら**そのカメラを失敗扱い**に
して書き出さず、理由 (盤面転置の可能性を含む) を `state_json`/stdout に出す。CLI
`--intrinsic-max-rms` / YAML `intrinsic_calib.max_rms_px` で調整可。`configs/intrinsic_calib.yaml`
の盤面を実物に合わせ 7×5 に修正 + 向きの注意コメント。ctest: `test_intrinsic_calib_session`
(rms ゲートで clean solve も閾値次第で失敗することを固定) / `test_main_config`。
設計 = [design/pose-3d-intrinsic-calibration.md](../design/pose-3d-intrinsic-calibration.md)。

### 2026-06-17 — スマホ動画から床 AprilTag マップを SfM 生成 (案D mode (b))
案D の `FloorTagMap` を**巻尺実測なしで動画から自動生成**する mode (b) を実装
(floor-apriltag-extrinsic doc が予告した拡張点。コア `floor_extrinsic_solver` は無改変)。
(1) **pose-graph コア** `lift/floor_map_sfm` (純幾何): フレーム毎の共可視タグ相対姿勢を
蓄積 → 各エッジ MAD トリム平均 → アンカー BFS で配置 → pose 平均緩和 → 床平面再ゲージ
(FitraWorld z-up, 床=z=0)。スケールは各タグ実寸 (114.5mm) の PnP が固定。(2) オフライン
ツール 2 本: `charuco_intrinsic_video` (ChArUco 動画 → スマホ intrinsics、`IntrinsicCalibSession`
無改変流用) と `sfm_floor_map` (マーカー動画 + intrinsics → `floor_tag_map.yaml` + holdout 再投影
検証)。C++ 4.8 の既存検出/PnP/IO を再利用 (Python cv2 は 4.5.4 legacy のため不採用)。設計 =
[design/pose-3d-smartphone-sfm-marker-map.md](../design/pose-3d-smartphone-sfm-marker-map.md)。
ctest: `test_floor_map_sfm` (連結復元 < 1e-3deg・床フィット・スケール保存・ノイズ・分割報告・
`solve_floor_extrinsics` 往復)。実サンプル (iPhone 2160×1214): intrinsic RMS 0.83px、8/8 タグ
連結マップ (plane_rms 6.7mm)、3+ タグ holdout 再投影 median 5.3px。**注意**: ChArUco 盤面は実物
`7×5` で `configs/intrinsic_calib.yaml` の `5×7` は転置 — リグ intrinsic 退化の疑い (要再校正確認)。

### 2026-06-16 — C++ 内部パラメータ (intrinsic) 校正 + 歪みモデル明示
extrinsic の前提工程だった intrinsic 校正を C++/WebUI に取り込み、setup の step0 に
据えた。(1) **歪みモデル基盤**: intrinsics YAML に `distortion_model` (pinhole|fisheye)
を追加し、consumer (triangulator / apriltag PnP / floor solver) を係数数でなくモデルで
分岐。案D の `floor_fisheye` が“裏付けのないフラグ”でなくなり魚眼 intrinsics を正しく
食える。(2) **ChArUco 検出** (`lift/charuco_board`) + **収集 session**
(`pipeline/intrinsic_calib_session`、多様性ゲートで per-camera ビュー収集、pinhole=
cv::calibrateCamera / fisheye=cv::fisheye::calibrate)。(3) **RunMode::CalibIntrinsic** +
`--calib-intrinsic`/`--intrinsic-replay`/`--charuco-*` + mode runner (live Crow / replay
無人) + **WebUI** `/intrinsic-calib` (`/api/incal/*`、per-camera views/被覆/rms)。
(4) **flow 統合**: `kExitFlowToCalibIntrinsic(84)`、daemon `initial_mode` が
intrinsic_calib.enabled かつ出力不在で step0 に入り intrinsic→extrinsic→subject→run と
連鎖。**切替前 precheck** (`precheck_mode_switch`) も追加し、設定不備のモード切替を
respawn 前に WebUI へ理由表示 (静かな run フォールバックを解消)。設計 =
[design/pose-3d-intrinsic-calibration.md](../design/pose-3d-intrinsic-calibration.md)。
ctest: test_calib_io / test_charuco_board / test_intrinsic_calib_session /
test_main_config (precheck + intrinsic ケース)。実機 rms は ChArUco 撮影後に確定。

### 2026-06-15 — 床 AprilTag 既知配置 PnP による extrinsic 校正 (案D) コア実装
VR を extrinsic チェーンから外す 2 つ目の extrinsic 方式。床に既知配置した AprilTag マップへ
各カメラを多タグ PnP で個別 localize し、`T_cam←world` を **fitra Z-up で無変換書出** (案C の
VmtWorld→FitraWorld 基底変換が無い)。新規: `lift/floor_tag_map` (FileStorage マップ I/O +
grid)、`lift/floor_extrinsic_solver` (集約 solvePnP + 再投影 + 平面縮退検出、案a/b 共有の
localize コア)、`pipeline/floor_calib_session` (静的前提のコーナー算術平均)、
`RunMode::CalibExtrinsicFloor` + `app/mode_calib_extrinsic_floor` (`--floor-calib` live /
`--floor-replay` 無人) + `floor_calib_runner`/`floor_live_input`。AprilTag 検出に CLAHE
オプション追加 (案C/D 共通、検出律速の局所コントラスト不足対策)。WebUI は flow-switch で
案C/案D を選択式に (`/extrinsic-calib` 方式トグル + Crow floor ルート、`PAGE_FOR_MODE` 追加、
redirect を target ページ比較化)。intrinsics は校正解像度で PnP、出力 YAML はランタイム解像度
(`T_cw` 解像度非依存)。設計 = [design/pose-3d-floor-apriltag-extrinsic.md](../design/pose-3d-floor-apriltag-extrinsic.md)
(research [floor-apriltag-sfm-map.md](../research/floor-apriltag-sfm-map.md) から昇格、(b) スマホ
SfM は research 残置)。ctest: test_floor_tag_map / test_floor_extrinsic_solver /
test_floor_calib_session / test_floor_calib_replay / test_main_config (floor ケース追加)。
実機の再投影 RMS / 平面縮退実値は高解像度 intrinsics 取得後に確定 (前提工程)。

### 2026-06-14 — flow daemon PR29 レビュー対応 (堅牢化 + 特殊 ID / subject 省略時の遷移修正)
PR #29 のレビュー指摘 (Gemini + self-review) への後追い対応。(1) `daemon.cpp`: シグナル
ハンドラを `SignalGuard` RAII 化し復帰時に旧 disposition を戻す (dangling `&stop` 防止)、
fork 前に argv を構築 + `execv`→`execvp` (PATH 探索 / async-signal-safe)、
`std::filesystem::exists` を `error_code` 版へ (常駐中の throw 防止)。(2) `subject_id` を
wizard と同じ `CalibrationSession::sanitize_id` で正規化 (`alice.v1`→`alicev1` 等で run が
profile を読めない不具合)、`sanitize_id` を public 化。(3) subject 未設定 daemon では
extrinsic solve 後に calib-subject を飛ばし run へ直接遷移 (子の `--calibrate requires
--calib-subject-id` クラッシュ回避)。(4) `/api/flow/switch` の `mode` 非文字列で 500 を
返さない型ガード。(5) `flow.js`: `/api/state` 404 (Python fallback) を再起動扱いせず
watcher 停止。ctest 全 21 件 pass。design doc なし (changelog のみ)。
→ [design/pose-3d-flow-daemon.md](../design/pose-3d-flow-daemon.md)

### 2026-06-13 — pose relay punch (calib で controller pose が来ない問題の修正)
カスタム VMT driver は受信パケットの送信元 IP を学習して pose を返す構成
(`refs/VirtualMotionTracker` CommunicationManager.cpp Phase 15.5)。VMT publisher を
持たない calib-extrinsic は Jetson から一切送信せず IP が学習されないため、controller/
HMD pose relay が一切来なかった。対処: relay receiver (`TrackedPoseReceiver`) の bind
ソケットから `vmt.host:vmt.port` へ定期 OSC punch (`/fitra/punch`) を送り、VMT に IP を
学習させる。全 relay 経路 (calib-extrinsic / run / hmd-listen) で有効。ローカル UDP で
punch 送信 (src=受信ポート 39571・OSC 20B・1s 間隔) を実証。将来は broadcast/multicast
での自動ディスカバリ (PC IP 設定不要化) が残課題。
→ [design/pose-3d-flow-daemon.md](../design/pose-3d-flow-daemon.md)

### 2026-06-12 — flow daemon: main の常駐 daemon 化とモードのモジュール起動 (M1–M4 完了)
`./main --daemon --config session.yaml` で main が常駐 daemon になり、モードモジュール
(同一バイナリ + モードフラグ) を fork/exec して exit code (80/81/82) で連鎖する。
M1 = flow 基盤 (`app/flow.hpp` FlowControl、`POST /api/flow/switch`、`/api/state.managed`、
managed 時の calib 自動連鎖、`--flow-managed` / `--no-vmt-out` / `--no-slimevr-out`、
approve 応答 next_step)。M2 = daemon 本体 (`app/daemon.{hpp,cpp}`: argv 合成 / exit 判定 /
initial auto 判定の純関数 + spawn/wait ループ、`--daemon` / `--daemon-initial`、SIGTERM 転送、
crash→run fallback + 3 連続 give-up、新 ctest `test_flow_daemon` は stub スクリプトで実
spawn 連鎖まで固定)。M3 = web (`/flow.js` の /api/state 追従: calib ページは次モードへ自動
遷移、ビューワはバナー + managed 時の再キャリブ切替ボタン)。M4 = 本ドキュメント群。
検討した代替 (外部 supervisor スクリプト / self-exec / reverse-proxy / 制御別ポート) の
棄却理由と union YAML 運用規約は設計 doc 参照。実機 3 段通しはユーザー後日。
M2 後の検証で SIGINT/SIGTERM 停止の不具合 (SIGINT が子に非転送で waitpid block /
SIGTERM が stop を立てず crash respawn) を発見し、両シグナルを「stop 設定 + 子へ
SIGINT 転送」の共通ハンドラに統一して修正 (`test_flow_daemon` に SIGTERM→rc0 を追加)。
→ [design/pose-3d-flow-daemon.md](../design/pose-3d-flow-daemon.md)

### 2026-06-11 — 専念モード化のドキュメント整備 (M5、M1–M5 完了)
track doc 現状節をモード分離後アーキへ更新、`cpp-migration-plan.md` のレイアウト節に
`app/` composition root の注記を追加、設計 doc に実装記録 (doc 未記載だった実装判断 +
意図的挙動変更 + 残検証) を追記。3 段フロー (excal → subject → run) と
`excal_record` / `--excal-replay` の運用手順を
[runbook-pose-3d-calibration.md](../runbook-pose-3d-calibration.md) として新設。
**残**: 実機での 3 段フロー通し確認と実録 fixture からの solve 再現。
→ [design/pose-3d-calib-mode-separation.md](../design/pose-3d-calib-mode-separation.md)

### 2026-06-11 — calib-extrinsic オフライン replay (--excal-replay) + live↔replay 等価性 ctest (専念モード化 M4)
`tools/excal_record` セッション (JPEG 連番 + ペア済み frames.jsonl) を `ExcalInputSource` の
replay 実装 (`pipeline/excal_replay_input`) として再生し、`--excal-replay <dir>` 単独で
calib-extrinsic を無人実行 (collect→solve→YAML、カメラ・SteamVR・web 不要、solve 失敗は
EXIT_FAILURE)。決定性の要: frames.jsonl の**行順逐次投入** (velocity 推定の prev 状態が
セッション全体で 1 本のため per-cam 分割や ts 再ソートは等価性を壊す) と、記録時確定の
`running_ok` を再判定しないこと。等価性 ctest (`test_excal_replay`) は合成 AprilTag フレームを
recorder フォーマットで一時 dir に書き、同一 JPEG バイト列を on_frame 直叩きと replay 経路の
両方に通して sample 列の bit-exact 一致を固定 (`samples_snapshot()` を比較用に追加)。
parser (`parse_excal_frame_line`) の strict reject も固定。実録 fixture での solve 再現確認は
実機作業として残 (M5 runbook に手順)。
→ [design/pose-3d-calib-mode-separation.md](../design/pose-3d-calib-mode-separation.md)

### 2026-06-11 — composition root 抽出 + Crow ルートのモード別モジュール化 (専念モード化 M3)
main.cpp (~1030 行) の構築シーケンスを `cpp/src/app/` の builder
(trt_stack / camera_builder / threed_builder / pose_relay_builder / output_builder /
server_builder / stats_loop) + モード runner (mode_run / mode_calib_subject /
mode_calib_extrinsic) へ抽出。main.cpp は config parse → validate → RunMode dispatch のみ
(~260 行、ほぼ help テキスト)。Crow は calib/excal ルート群を `web/crow_routes_setup.cpp` に
分離し (deps 構造体渡し、session 未 attach なら静的ページ含め未登録)、共有 JSON helper を
`web/crow_util.hpp` へ。`GET /api/state` を常設し mode ラベルを返す — viewer の calib 導線は
mode に応じて表示。挙動変更は web 表面のみ: run モードで `/subject-calib`・`/extrinsic-calib`
静的ページも 404 に、calib-subject での hmd-listen 受信は廃止 (消費者が存在しなかった)。
→ [design/pose-3d-calib-mode-separation.md](../design/pose-3d-calib-mode-separation.md)

### 2026-06-11 — ライブ再注入の物理削除 + calib-extrinsic 軽量ループ化 (専念モード化 M2)
calib↔runtime のプロセス内再注入経路をコンパイルレベルで削除:
`MultiCameraDriver::set_triangulator` (triangulator ホットスワップ)、`IkSolver::reload_from_profile`
(approve 後の IK ホットリロード)、`CrowServer::set_extrinsic_calib_solved_callback`、frame tap の
excal/calib 状態分岐 mux。calib-extrinsic は `ExcalInputSource` 抽象 (新 `pipeline/excal_input_source.hpp`)
越しの軽量 capture ループ (`app/excal_live_input` + `app/excal_runner`、新 static lib `fitra_app`) に
載せ替え、TRT 初期化・MultiCameraDriver・publisher を一切構築しない (decode-only。
`--det-engine`/`--pose-engine` も不要に)。solve 成功は `ExtrinsicCalibSession::set_on_solved` 経由で
auto-exit し、`/api/excal/solve` 応答と web UI に再起動コマンド (`next_step`) を提示。
approve 後の wizard も run モード再起動のガイダンスログに置換。replay (M4) は同じ
`ExcalInputSource` 経路に載る。
→ [design/pose-3d-calib-mode-separation.md](../design/pose-3d-calib-mode-separation.md)

### 2026-06-11 — RunMode 導入 + モード別構築ゲーティング (専念モード化 M1)
`run_mode(MainOptions)` で排他 RunMode (`run` / `calib-subject` / `calib-extrinsic`) を導出し
(既存フラグから導出、invocation 互換)、main.cpp の構築をモードでゲート: CalibrationSession +
`calib_recording_flag` 配布は calib-subject 限定、ExtrinsicCalibSession は calib-extrinsic 限定、
SlimeVR/VMT publisher は run 限定 (`--slimevr-out`/`--vmt-out` × `--extrinsic-calib` を validate
で排他化)。run モードの `/api/calib/*` は 503 スタブ廃止で未登録 (GET 404 / POST 405)。
挙動変更: calib 中の tracker 出力停止、run での wizard API 消滅。ライブ再注入の削除は M2。
→ [design/pose-3d-calib-mode-separation.md](../design/pose-3d-calib-mode-separation.md)

### 2026-06-10 — キャリブレーション専念モード化の設計 doc (M0、実装は別途)
「初期設定・キャリブ中に他モジュールは動かなくてよい」という前提合意を受け、subject wizard /
controller-marker extrinsic と live パイプラインの同居 + ライブ再注入 (IK ホットリロード・
Triangulator ホットスワップ・frame tap 多重化) を解消する設計を記録。1 バイナリのまま排他
RunMode (`run` / `calib-subject` / `calib-extrinsic`) に分け、モード間の受け渡しを YAML 成果物
(CalibrationSet / SubjectProfile) + プロセス再起動のみへ縮退。excal→subject の同一プロセス続行
UX は再起動ガイダンスへ置換 (案C' 同一プロセス再構築は teardown リスクで没、案B 別バイナリ化は
build graph 手術が先で将来含み)。到達目標としてオフライン replay を追加: calib-extrinsic を
サンプル動画 + 記録済み VR 座標 (JSONL) だけで solve まで再現可能にし、live↔replay 等価性を
ctest で固定する (M4)。記録側の単体ツール `tools/excal_record` (MJPEG パススルー JPEG 連番 +
frame↔pose ペア済み frames.jsonl、main 非依存・TRT 実行不要) は本エントリで先行実装済み —
実機 2 カメラ + 擬似 OSC pose でスモーク確認 (30fps×2 維持・ペアリング/単調 ts 検証)。
モード分離本体の M1〜M5 は別ブランチ。
→ [design/pose-3d-calib-mode-separation.md](../design/pose-3d-calib-mode-separation.md)

### 2026-06-10 — 座標フレームを型レベルで区別 (split-brain 再発を型で防止)
3D 数学が素の `cv::Matx44d` / `cv::Vec3d` で、フレーム意味論が変数名とコメントだけに宿っていた問題に対し、
SE(3) レイヤ (extrinsic solver / triangulation / calib I/O / calib session) へ phantom-typed
`geom::Transform<To,From>` 代数を導入。中間フレーム不一致の合成と world 種別 (fitra Z-up / VMT Y-up) の
取り違えをコンパイルエラー化する。3 箇所に散在していた Z-up↔Y-up 変換を単一の `geom::fitra_to_vmt_basis()` /
`vmt_to_fitra_basis()` に集約 (`extrinsic_calib_session` の `kVmtWorldToFitra` を置換、wire 変換は実装据置で
test クロスチェック)。leaf (`Joint3D`/kalman/IK/wire) は現状維持し「常に fitra Z-up」を不変条件で固定。
2026-06-09 の split-brain リグレッションの根因 (型で防げない frame 混同) を構造的に塞ぐ。挙動・数値は不変。
→ [design/pose-3d-typed-coordinate-frames.md](../design/pose-3d-typed-coordinate-frames.md)

### 2026-06-10 — subject calib の drift ゲートを post-IK 値へ戻す (pose 判定不能を修正)
2026-06-09 の pre-IK skeleton 化で、calibration tap に渡す `bone_drift_pct` まで pre-IK の
measured 値に変えてしまい、`PoseRecognizer` の `max_bone_drift_pct` (~10%) ゲートを毎フレーム超過
→ ポーズが永久に検出されず IK 較正に入れない回帰を生んでいた (身長入力済みでも不可)。post-IK
skeleton は bone 長が model にクランプされ drift ≈ 0 (=従来動いていた緩いゲート) なのに対し、生の
pre-IK 三角測量は容易に 10% を超えるのが原因。**角度の算出元は measured (pre-IK) skeleton のまま**
(hinge clamp バイアス回避は維持) で、**tap に渡す drift だけ post-IK 値へ戻す**よう
`MultiCameraDriver::maybe_update_3d` を修正。measured skeleton のコピーは tap がある時だけ取得し
live path のコストは増やさない。`test_pose_recognizer` に「有効な T-pose は低 drift で in_band、
高 drift では `bone_drift` 軸で reject」を固定。

### 2026-06-09 — extrinsic (研究) — 床 AprilTag SfM マップ方式を検討
共視不要の静的アンカー代替を整理。床 + 可搬スタンド (壁不要) に大判 AprilTag を配置し、スマホ全景
撮影で SfM マップを自動復元 → 各カメラを共通マップに localize する。案A/案B の却下理由 (同時共視不能
/ BA 共視経路) をマップ構築と localize の時間分離で迂回し、VR を外すことで案C 律速の Quest SLAM
drift 項を誤差バジェットから消せる。案C を潰さず初期設定で選べる方式の 1 つとして位置づけ、案C の
drift 実測リファレンスにも使える。未実装・設計フェーズ前。
→ [research/floor-apriltag-sfm-map.md](../research/floor-apriltag-sfm-map.md)
(設計doc 案D に相互リンク: [design/pose-3d-controller-marker-extrinsic.md](../design/pose-3d-controller-marker-extrinsic.md))

### 2026-06-09 — hand-eye extrinsics を fitra Z-up で書き出し (検証シーンは VMT Y-up 維持)
controller-marker hand-eye の解 `T_cam←world` は controller pose と同じ VMT/SteamVR **Y-up** frame
で出るため、Z-up 前提の downstream (triangulation・IK・メイン ws3d viewer・SlimeVR・solve 後の live
hot-swap) で床が垂直に表示され (カメラ中心 z が負) 誤った frame に乗っていた。`solve_and_write` で
**永続化 YAML を作るときだけ** 各 `T_cw` に基底変換 (`world_pos_to_vmt` の回転 Rx(−90°) の逆 = 世界軸
の付け替え) を右から掛け fitra Z-up へ再表現し、`coordinate_system` ラベルも z-up へ上書き。一方
`/extrinsic-calib` の 3D 検証シーン (`scene.js`) は `/api/excal/extrinsics` のカメラと
`/api/excal/poses` の live HMD/コントローラ姿勢を同一 frame に重ねる道具で、live 姿勢が VMT Y-up・床
Y=0 のため、`extrinsics_json` (=`solution_`) は **無変換 (VMT Y-up) のまま** 維持する。世界軸の回転
なので相対 extrinsic・基線長は不変、永続化側の絶対姿勢だけが Z-up に揃う。`test_extrinsic_calib_session`
に「書き出し `T_cw` が ground truth×basis change と一致 (回転 ~0°)・未変換とは ~90° 異なる・ラベルが
z-up」と「`extrinsics_json` の cam 中心が VMT frame のまま」を固定。詳細は
`docs/design/pose-3d-controller-marker-extrinsic.md`。

> 注: 当初 `extrinsics_json` も Z-up に変換したが、検証シーンでカメラだけ Z-up・live 姿勢が Y-up と
> なりカメラが別位置に出る回帰を生んだため、変換を永続化 YAML 限定へ修正 (同日)。

### 2026-06-09 — subject calibration の角度判定を pre-IK skeleton 化
subject calib の `PoseRecognizer` が live publish と同じ post-IK skeleton を見ていたため、身長 prior
や hinge/length clamp が作った補正後の関節角で hold 判定していた。腕を伸ばしていても肘 flex が
人工的に増える可能性があるため、`MultiCameraDriver` の calibration tap を Kalman 後・IK 前の
measured skeleton に移動。`bone_drift_pct` は同じ measured skeleton 対象で渡し、公開 `/ws3d` /
tracker 出力は従来どおり post-IK skeleton を維持。`test_pose_recognizer` で伸展腕の raw 角度と
post-IK hinge clamp バイアスを固定。

### 2026-06-09 — extrinsic solve 後に subject calibration へ続行
`/` のヘッダには常に `/subject-calib` リンクが出る一方、Crow 側は `CalibrationSession`
attach 時だけ `/subject-calib` 静的 route を登録していたため、`--enable-3d` なし / 2 カメラ以外 /
`--extrinsic-calib` 中など subject wizard 無効条件ではリンク先が 404 になっていた。静的 UI 配信を
session 有無から切り離し、`/api/calib/*` は session 未 attach 時に JSON 503 (`state=unavailable`)
を返す形へ変更。`test_crow_excal` に未 attach 時の `/subject-calib` 静的配信 smoke を追加。
さらに `--extrinsic-calib` 中でも `CalibrationSession` を attach し、frame tap は extrinsic
collecting/solving 中だけ AprilTag collector へ、それ以外は subject recorder へ振り分ける。
`/api/excal/solve` 成功時は書き出した `--excal-out` を読み直して live `Triangulator` を
hot-swap し、WebUI に `Subject calib` 導線を出す。これにより同一プロセスで extrinsic solve →
subject preflight/capture へ進める。終了時は solve 済み extrinsics を再 solve しない。

### 2026-05-29 — Triangulator のスクラッチバッファ再利用 (挙動不変リファクタ)
`Triangulator::triangulate()` / `triangulate_joint()` が per-keypoint・per-view で
確保していた `std::vector` (`views` / `undistortPoints` の入出力 1 要素ベクタ /
`indices` / `kept`) を関数内 `static thread_local` スクラッチへ移し再利用。Halpe26 × 2cam で
1 三角測量フレームあたり ~100 個の小ヒープ確保を除去。`mutable` メンバ案は const の
スレッド安全契約を壊す (将来並列化時のデータ競合 / コピー・ムーブ肥大化、PR #22 Gemini 指摘)
ため不採用とし、`thread_local` でスレッドごとに独立したスクラッチを持たせて const 契約を維持。
出力はビット同一 (`test_triangulator` golden 通過)。微最適化のため design doc なし (changelog のみ)。

### 2026-05-29 — locomotion-stability PR の AI レビュー対応
PR #21 の Gemini / Copilot レビュー指摘を反映。(1) parent-yaw transport の
**オーバーシュート修正** — transport delta は親の全変化 (prev smoothed → curr raw)
だが参照 tracker 自身は `alpha_rate` ずつしか収束しないため、毎フレーム全量を乗せると
持続旋回で held 肢が `Θ/alpha` まで回り続ける (alpha=0.5 で 2× オーバーシュート)。
transport を `alpha_rate` でスケールし親と同速で収束させる (alpha=1 では no-op = M5
テストはビット同一)。(2) `PosSmoothingContext::prev_hip_valid` を毎 tick 反映 —
hip dropout を跨いだ stale な `prev_hip_pos` での re-anchor を防止。(3) `SkeletonKalman`
の防御的バウンズチェック (`i` をループ先頭で検査 / `parent` の範囲検査 / `ensure_topology`
の `kMaxKeypoints` 超過で throw)。回帰テスト 2 件追加
(`test_pelvis_yaw_transport_no_overshoot` / `test_hip_dropout_clears_prev_hip_valid`)。
ctest 全 10 スイート通過。
→ [design/pose-3d-locomotion-stability.md](../design/pose-3d-locomotion-stability.md) M5

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

### 2026-05-27 — コントローラ固定 AprilTag extrinsic: M1 transport + M2 ソルバ/検出コア
案C (コントローラ固定マーカー + VR world 連結) のソフト側コアを着手・実装。実機を要する
M0 (PnP 残差 / SLAM ドリフト実測) と live 収集 UI (M2 後半 / M4) は手戻りリスクを承知で後回し、
ハードに依存しない 3 モジュールを単体テスト付きで先行実装した:
- **M1 transport**: `vmt/controller_pose_receiver.{hpp,cpp}` — `/fitra/controller_pose`
  (`,iiffffffff` = HMD の `,iffffffff` に `eTrackingResult` を 1 個追加) を HMD と並列の
  latest-wins bus で受信。`running_ok()` ゲート (`bPoseIsValid && Running_OK`)。OSC decode
  helper を `vmt/osc_decode.hpp` に抽出し HMD 経路と共有。
- **M2 ソルバ**: `lift/extrinsic_solver.{hpp,cpp}` — `A_i·X = Z·B_i` を
  `cv::calibrateRobotWorldHandEye` (AX=ZB) に写像 (A=T_cam←marker / B=T_world←controller /
  Z=T_cam←world / X=T_marker←controller=Y_face⁻¹)。(camera,face) グループ毎に解き、面間で
  T_cam←world を集約 + spread を品質指標化、自己残差も算出。
- **M2 検出**: `lift/apriltag_marker.{hpp,cpp}` — `DICT_APRILTAG_36h11` 検出 + 面 ID→サイズ
  設定 + `SOLVEPNP_IPPE_SQUARE` で T_cam←face。`objdetect` を OpenCV link に追加。
- **収集ループ骨格 + 配線**: `pipeline/extrinsic_calib_session.{hpp,cpp}` — frame tap → 検出 →
  controller pose ペアリング → モーションゲート (線速/角速しきい) → (cam,face) 毎バースト平均 →
  `ExtrinsicSample` 蓄積 → 終了時 solve + `CalibrationSet` 書き出し。`calib_io::write_calibration`
  新設。`main`/`main_config` に `--extrinsic-calib` / `extrinsic_calib:` セクションと
  `ControllerPoseReceiver` 起動を配線 (subject wizard と frame tap 排他)。
- **Crow 配線 + WebUI**: `crow_server` に `set_extrinsic_calib_session` +
  `/api/excal/{state,start,stop,solve,extrinsics}` (subject wizard の `/api/calib/*` と同型) +
  `/extrinsic-calib` 静的配信。フロントエンド `web/extrinsic_calibration/` — Start/Stop/Solve、
  理由付き Capture gate (GOOD/MOVING/NO_TAG/NO_POSE)、per-camera ライブ検出 (face·reproj·age)、
  被覆マトリクス (cam×face, min_samples でセル色)、Solve 後 per-camera 結果。`state_json` に
  `num_cams`/`min_samples`/`faces`/`gate_reason`/`detections` 追加。
- **3D 検証シーン**: `scene.html`+`scene.js` (three.js, vendor 流用) が `/api/excal/extrinsics`
  (intrinsics + `T_cam_world` + center) を読み、各カメラ 6DoF を共通 world frame に frustum 配置。
  session は `on_frame` で per-camera 最新検出を保持 + `gate_reason_`/`extrinsics_json` を公開。
- 検証: `ctest -R 'extrinsic_solver|apriltag_marker|controller_pose_receiver|extrinsic_calib_session|main_config|crow_excal'`。
  合成データで厳密復元 (1e-9 m / 1e-6°)・相対 extrinsic 恒等・ノイズ <1cm/<1°、tag
  render→detect→PnP 往復、収集ループの gate/burst + end-to-end solve→write→reload、
  レンダ tag→on_frame の検出/gate_reason、`extrinsics_json` 内容、ループバック実 HTTP で
  `/api/excal/*` + 静的配信 (collect/scene)。
- **未** (実機 or 大掛かりで保留): Windows sender の controller 送出、per-camera ライブ**映像**
  オーバーレイ (検出ステータスはテキスト表示済、映像配信は別途)、M4 live solver 重畳、M3 BA
  (閉形式が合成厳密復元 + 動機が M0 実測待ちのため保留)、M0 実測。収集制御は headless 既定 +
  WebUI/API 手動制御。
→ [design/pose-3d-controller-marker-extrinsic.md](../design/pose-3d-controller-marker-extrinsic.md)

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
