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
