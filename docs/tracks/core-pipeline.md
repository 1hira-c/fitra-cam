# Track: core-pipeline

カメラ取り込み → TensorRT 推論 (YOLOX + RTMPose) → Web 配信の **基盤パイプライン**と性能。
C++ 移行 (旧 Phase 0–6) で確立し、現在は安定。pose-3d / vr-output トラックの土台。

## 現状

3 USB カメラ並列、aggregate **170 fps** (旧 Phase 3 baseline 比 5.5×) を達成済み。
keypoint topology は COCO17 / Halpe26 を `--keypoint-format` で切替。

### アーキ仕様の source of truth

アーキ図 (capture thread × N / 単一 TRT context / 単一 CUDA stream / SPSC queue size 1 drop-old /
Crow WS 30Hz)、リポジトリレイアウト、依存表 (FetchContent header-only) は
[`cpp-migration-plan.md`](../cpp-migration-plan.md) が今も live な仕様。CLAUDE.md「Architecture」も参照。

### live な制約 (要点)

- **単一 TRT context・単一 CUDA stream**: Python 版の per-camera セッション分離とは逆。
  context 切替コストをゼロにするのが移行の肝。
- **latest-frame-wins capture**: SPSC queue size 1 で drop-old。リアルタイム鮮度 > 全フレーム処理。
- **FP16 RTMPose drift**: 低スコア keypoint (score<0.5) が Y で 100–200px ずれることがある。
  FP32 なら max kpt L2 ≈ 1.15px。INT8/FP16 を扱うときは Phase 1 と同じ動画で再現テスト必須。
- **YOLOX end2end + TRT 10.3 INT8 不可**: mmdeploy end2end ONNX の NMS/TopK/Where chain が
  calibration `slot::decode` assert に当たる。calibrator infra は RTMPose INT8 計画で salvage 予定。
- **keypoint topology**: `kMaxKeypoints=26` で配列統一、`kp_count` で論理点数を伝搬。
  subject profile schema (v1/v2) はマイグレーションしない (pose-3d トラック参照)。

### 検証

`tools/correctness_check` で Python ORT と bbox IoU>0.99 / kpt L2<1px。
ベンチ・合格基準は [`cpp-migration-plan.md`](../cpp-migration-plan.md) の検証戦略表 + 各 Phase 着地メモ。

## Changelog (新しい順)

### 2026-06-20 — WebUI 主導セットアップ (RunMode::Setup daemon モジュール)
「初回セットアップから実推論まで」をほぼ全部ブラウザから回せるようにした。残っていた gap
(カメラ選択 UI 無し / portless daemon の bootstrap 鶏卵 / 名前付き config 無し /
calib UI が vanilla-JS と React に分裂) を、flow daemon の「モード=モジュール」パターンを
再利用して解消。**新 `RunMode::Setup`** = daemon が最初に spawn する軽量モジュール
(Crow + V4L2 列挙/preview のみ、TRT/CUDA 不使用)。cameras 未設定の config なら `initial_mode`
が Setup に着地し、ブラウザで cameras/engines/出力先を合成 → union config を書き出し →
exit code (`kExitFlowToSetup=85`) で intrinsic→extrinsic→subject→run へ連鎖
(子が config 再ロード)。reverse-proxy/別バイナリ不要 (旧 launcher 設計を supersede)。
新規: `emit/save_main_config` (loader の逆、往復 ctest)、`SetupConfigStore`、
`v4l2_enumerate` + `setup_camera_manager` (単発 JPEG preview)、`/api/{cameras,config,setup}*`、
React の Setup/Intrinsic/Extrinsic ページ + WizardSteps、Crow の SPA history-fallback。
calib 静的配信を `web-ui/dist` へ向け直し legacy `web/{extrinsic,intrinsic}_calibration` を retire。
validate は `!opts.daemon` で run-form を緩め daemon 親が空 config で起動できるようにした。
ctest 全 29 pass。実機 smoke: 3 カメラ列挙 + preview + proceed 連鎖を確認。
設計: [`core-pipeline-setup-mode.md`](../design/core-pipeline-setup-mode.md)。

### 2026-06-19 — per-camera 露出/gain 制御 (手動固定 + 簡易ソフトAE)
cam1 の「ガタつき + ブラー + 妙に明るい」を追及 → **カメラ純正の自動露出が原因**と確定
(Windows でも MJPEG/YUYV 両方でもガタつく → ホスト/圧縮非依存。`exposure_time_absolute`
default=15.6ms が 60fps 予算 16.67ms の 94% で、暗いと AE が予算超へ伸ばし pacing 不均一 +
動体ブラー)。**AE bias では解けない** (当該カメラは bias control 非公開 + AE は浮動)。対策:
per-camera 露出制御を `V4l2Capture` に追加。**設計の肝 = exposure(高コスト: ブラー+fps超過)は
短く固定上限、明るさは gain(低コスト)で取る**。3 モード: `auto`(既定・無改変) / `manual`(固定) /
`assist`(常時・遅いデッドバンド ソフトAE; gain 優先・exposure は fps 安全上限で頭打ち)。
start() で `auto_exposure=manual`+exposure+gain+`focus_auto=off` を適用、gain range は
QUERYCTRL。assist は `FrameSource` が平均輝度を ae_interval(30f≈0.5s) 毎にデッドバンド判定し
gain/exposure を遅く追従。config `cam{N}_exposure_mode/exposure/gain/ae_target`。既定 auto なので
既存リグ無影響。ブラー低減は 2D 精度の底上げにも効く見込み。設計:
`docs/design/core-pipeline-camera-exposure-control.md`。

### 2026-06-19 — cam1 capture 頭打ち修正 (毎フレーム mmap 撤廃) + 電源モード是正
cam1 (USB3.0 1280×960) が 53fps 頭打ち/ジッタだった件を追及。**カメラは無実** — 純 v4l2-ctl で
YUYV 1280×960 は 60fps (要求 120fps でも 119fps) を steady に出すと実証。真因は 2 点:
(1) **毎フレーム 2.46MB ヒープ確保**: `V4l2Capture::worker_loop` が毎フレーム新規 `Frame` の
空 vector に `assign` で 2.46MB(YUYV) をコピーしていた。glibc malloc は >128KB を mmap/munmap で
扱うため毎フレーム mmap+ページフォルト(ゼロ埋め)+munmap が走り capture を ~53 に拘束。
キャプチャスレッド専有の `spare_data_` を `latest_->data` とスワップして capacity を再利用
(確保はウォームアップ後ゼロ)。消費側も `FrameSource::decode_loop` の `Frame raw` をループ外に
出し `raw = *latest_` の copy-assign で storage 再利用 (同じ mmap 罠が decode スレッドにもあった)。
(2) **電源モード**: nvpmodel が 25W + schedutil で cpu4/5 が 0.73GHz の省エネコアに落ち、
そこにスレッドが載るとジッタ。**真の最大は MAXN_SUPER (mode 2)** — このデバイスは mode 番号が
振り直されており `0`=15W(最弱)、`1`=25W、`2`=MAXN_SUPER。CLAUDE.md の `nvpmodel -m 0` は
Orin Nano Super では誤り (15W 固定になる)。**実測 (MAXN_SUPER + jetson_clocks, 3カメラ)**:
cam0/cam2 solid 60、cam1 55-60 で pending 7 安定、cap→pub 30→**5.6ms**。cam1 は HW NVJPEG 競合と
MJPEG ソースジッタを避ける **YUYV 推奨** (stage_ms 6.7→1.8ms)。**残**: cam1 単一では steady 60 だが
3カメラ全負荷で 55-60 に揺れる (1280×960 capture が 640 機より重く 6 コア競合でスリップ) — 深掘り中。
設計: `docs/design/core-pipeline-3cam-60fps-smoothing.md` (cam1 capture 節)。

### 2026-06-19 — 3カメラ 60fps 安定化 (GPU 直列化解消 + 検出スケジュール + 位相ずらし)
3カメラ 60fps のガタつきを解消。実測 e2e breakdown で**当初診断 (GPU null ストリーム直列化が
天井) は主因でなかった**ことが判明 — 中央 RTMPose は `bake->pose=1.1ms` で律速でなく、真の律速は
worker 側 YOLOX (`dec->det=18ms`) だった。3 修正: (1) **per-handle 非ブロッキング CUDA
ストリーム** — RTMPose 前処理カーネルを NULL ストリームから専用ストリームへ (`det->bake` →
0.00ms; 全 GPU バリア解消だが単独では不十分)。(2) **検出スケジュール修正** — `||
cached_bboxes_.empty()` を削除。無人/未検出時に毎フレーム YOLOX(18ms×3) が暴走し ~48fps に
張り付く逆転挙動を停止 (`dec->det` 18→2.6ms、cap→pub 30.6→7.9ms)。(3) **検出位相ずらし**
(`det_phase = i*freq/N`) — 3カメラの YOLOX バースト重なりを分散しジッタ解消。**実測 (無人)**:
cam0/cam2 recent_pose ~48→**59-61 安定**。残課題: cam1 は recv 50-60 で振れる (カメラ/UVC 配信
特性、帯域起因でない; YUYV 安定 53 推奨) → 別タスク。設計:
`docs/design/core-pipeline-3cam-60fps-smoothing.md`。

### 2026-06-19 — HW NVJPEG 破損フレーム guard + per-camera pixel_format + VIC スケール decode
3カメラリグ (cam0/cam2=ELP, cam1=USB3.0 1280→640) を立ち上げる過程で 3 点を追加。
(1) **破損フレーム guard**: 多カメラの USB 帯域飽和で truncated/garbage MJPEG が出ると
HW NVJPEG が segfault する。`frame_source` の `looks_like_jpeg()` が SOI/EOI+長さを検査して
HW decoder に渡す前に drop (CPU `cv::imdecode` 経路は元々耐性あり)。nvjpeg 運用全般の
クラッシュ耐性が上がる。(2) **per-camera `pixel_format`** (`cam{N}_pixel_format`): 混在
decode 経路を可能に。(3) **VIC スケール decode**: `fitra_nvjpeg_decode_to_device` に
`target_w/h` を追加し VIC の YUV→RGBA で 1280→640 を同時実行、縮小カメラも all-GPU front-end
に残す。設計: `docs/design/core-pipeline-per-camera-capture-downscale.md` (M4-M6 + 残課題)。
> この時点の「3カメラ 60fps の天井 = GPU null ストリーム直列化」という診断は**後日 (同 2026-06-19)
> 実測で否定**された (主因は worker 側 YOLOX の毎フレーム検出)。次エントリ参照。

### 2026-06-18 — per-camera capture 解像度 + ソフト downscale
増設した USB3.0 個体 (`Global Shutter Camera` serial `2601240001`) は、既存 2 機と違い
**640×480 だけセンサ中央 center-crop** に切替わり狭画角になることを実機キャプチャで確認。
カメラ単位で V4L2 キャプチャ解像度を上書きでき (`cam{N}_capture_width/height` /
`--camN-capture WxH`)、`FrameSource::decode_loop` が decode 直後に共通出力解像度へ `cv::resize`
(INTER_AREA) する経路を追加。`V4l2Options` を出力 (`width/height`) と capture (`cap_width/height`,
0=無し) の二層に分離 — ダウンストリームは `options().width/height` を読むので無改造。縮小カメラは
nvjpeg .so が VIC スケールを露出しないため all-GPU front-end を切り BGR scratch + CPU prebake へ
降ろす (HW JPEG decode は維持)。新カメラは 1280×960 撮影→640 縮小で既存機と画角・座標系が一致。
intrinsic は 1280 校正→`scale_intrinsics` 640。設計: `docs/design/core-pipeline-per-camera-capture-downscale.md`。

### 2026-06-18 — WebUI/VMT 未接続時の待機 (idle) モード仕様起票 (仕様のみ)
消費者 (WebUI の WS ビューア / VR 側 VMT) が誰も繋がっていない時に重い GPU 推論を自動で止めて
省電力にする待機モードを設計。新 `RunMode::Idle` (flow daemon 再起動方式) は復帰に数秒かかるため没とし、
`RunMode::Run` プロセスを生かしたまま既存スレッド内でゲートする **in-process throttle** を採用
(既存 `calib_recording_flag` と同型の `shared_ptr<atomic<bool>>` を流用)。消費者ゼロが `enter_after_s`
継続したら `FrameSource::decode_loop` の YOLOX と `MultiCameraDriver::loop` の RTMPose/3D をスキップ、
ループを `idle.tick_hz` (既定 2Hz) へスロットル。**待機深度は推論スキップのみ** (capture/decode/TRT は
温存) で復帰は atomic 反転の <100ms、**既定 ON** (`--no-idle` で無効、calib モードは対象外)。
WS は `clients2d/3d.conns`、VR は `HmdPoseBus` の freshness で検出。VMT-out かつ HMD-listen 無し
(戻り信号無し) は安全側で idle に入れない。**実装は未着手** — M1 状態/計数/config、M2/M3 ゲート、
M4 復帰ジャンプ対策、M5 安全既定。
→ [design/core-pipeline-idle-standby.md](../design/core-pipeline-idle-standby.md)

### 2026-05-29 — `maybe_update_3d` の冗長 bone_drift_pct 計算を除去 (挙動不変)
`MultiCameraDriver::maybe_update_3d` で IK 前に `ik_.bone_drift_pct(skel)` を計算していたが、
`ik_enabled` 時は直後に IK 後の値で無条件上書きされ pre-IK 値は常に破棄されていた。
`bone_drift_pct()` は `ik_` の mutex を取り全ボーンを走査するため、公開する skeleton
上で 1 回だけ計算するよう分岐を整理。4 ケース (ik_enabled × locked) すべてで最終 drift は
従来と同一。微最適化のため design doc なし (changelog のみ)。ctest 全 10 通過。

### 2026-05-29 — 全 GPU フロントエンド M4: アーキ整合 + multi-cam 集約スループット実測
per-cam CPU 前処理の GPU 化は M2/M3 で完了済のため M4 は整合確認 + 実測。各 `FrameSource` は独立に
`hw_decoder_` (NvJPEGDecoder + EGL register + `DeviceChwPool`) を所有し各 worker が共有 primary context に
bind、カメラ間で EGL/CUDA リソース共有なし (multi-cam 独立性は M3 2cam 実機で実証)。**2cam 90fps@VGA で
recv 88.3×2 = 176fps 集約を CPU 1.01 cores で維持** (recent_pose ≈ recv、定常 drop なし)、central RTMPose
は `rtm=4.84ms/iter` で GPU 推論律速 → 旧 170fps 目標を 2cam で超過しつつ大幅 CPU 余力。**M5 (SimCC
argmax GPU 化) は実測より便益小と判明し見送り**: SimCC の D2H (~61KB/person) + CPU argmax は sub-ms で
残 1 コアの律速 (capture + TRT enqueue + V4L2) ではない。将来 SimCC decode が律速化したら再評価。
→ [design/core-pipeline-gpu-frontend.md](../design/core-pipeline-gpu-frontend.md)

### 2026-05-29 — 全 GPU フロントエンド M3 Step B: frame_source 統合 + cvtColor 撤去
device 経路を `decode_to_device` (host map / RGBA→BGR cvtColor なし) に切替え、検出フレームで
`Yolox::infer_device`、calib/retain_bgr 時のみ `decode_keep_device`。**残っていた最後の per-frame
CPU フルパス (full-frame cvtColor) を撤去**。フレーム寸法は decode 戻り値で追跡、device decode 失敗時は
BGR+CPU フォールバック (BGR なし時は pose スキップで空 Mat deref 回避)。**2cam 90fps@VGA: CPU
1.28→0.98 cores (mjpeg 1.83 比 −46%)、cap→pub 12.8→11.7ms** で 1 コアを切った。SIGINT rc=0/0.32s、
ctest 9/9、CHW/keypoint/bbox correctness 維持。残 CPU は後段推論 + capture → M4/M5。
→ [design/core-pipeline-gpu-frontend.md](../design/core-pipeline-gpu-frontend.md)

### 2026-05-29 — 全 GPU フロントエンド M3 Step A: YOLOX 前処理 CUDA カーネル
YOLOX letterbox 前処理 (resize+HWC→CHW, 正規化なし, 114 pad) を GPU カーネル化。`cv::resize` の
half-pixel convention + edge clamp を再現。`Yolox::infer_device(fill)` で engine 入力 device バッファを
カーネル直書き (静的 shape, H2D なし, kernel と enqueue を同一 stream で順序付け)。YOLOX は per-camera
worker の TRT context なので cross-thread なし。loader に `decode_to_device`/`preprocess_yolox_into`/
`yolox_device_capable`。`gpu_preprocess_check` に bbox モード追加 (host `infer` vs device、IoU マッチ +
corner L2)。実機 **bbox corner L2 = 0.0px (8/8 matched)** — letterbox 微差を FP16 が量子化吸収
(device-first で stale false-pass 排除)。ctest 9/9。Step B で frame_source 統合 + full-frame cvtColor 撤去。
→ [design/core-pipeline-gpu-frontend.md](../design/core-pipeline-gpu-frontend.md)

### 2026-05-29 — 全 GPU フロントエンド M2 Step B: device CHW を TRT 入力直結
前処理カーネル出力を host を介さず TRT 入力に渡す配線。`TrtEngine::copy_input_region_from_device`
(D2D, offset 対応) + `RtmPose::PrebakedRequest.chw_dev` で `run_one_prebaked` を device バッチ経路化
(H2D 消滅)。逆アフィンのみ `RtmPose::compute_m_inv` で CPU 算出。.so に `fitra_nvjpeg_preprocess_from_last`
/ `decode_to_device`、loader に `device_capable`/`decode_keep_device`/`preprocess_into`。レース対策に
per-camera `DeviceChwPool` (`shared_ptr<DeviceChwBuf>` deleter がプールを生かし、worker は consumer 保持中の
バッファを触らない = host copy-on-pop の device 版)、取得失敗時は CPU prebake フォールバック。検証:
`gpu_preprocess_check` の keypoint モードで host `infer` vs device `infer_prebaked` を照合、confident
keypoint L2 **avg 0.34px / worst 1.18px** (低スコアは FP16 既知の argmax 不安定で除外)。実機 (単一カメラ
nvjpeg fake-bbox): **det→bake 4.1→1.1ms / cap→pub ~20→~15ms**、30fps 維持、SIGINT 0.52s、ctest 9/9。
**2cam 90fps@VGA A/B** (`FITRA_DISABLE_GPU_PREPROCESS=1` で CPU prebake 強制比較): GPU フロントエンドが
**nvjpeg の ~1.8 コア床を初めて割った** — CPU 1.77→**1.28 cores (−28%)**、cap→pub 16.4→**12.8ms**。
「高 fps では色変換が床」とした nvjpeg doc 結論に対し per-person warp/normalize の GPU 移行が効くと実証。
BGR は YOLOX/calib 用に残置 (full-frame cvtColor 除去は M3/M4)。
→ [design/core-pipeline-gpu-frontend.md](../design/core-pipeline-gpu-frontend.md)

### 2026-05-29 — 全 GPU フロントエンド M2 Step A: RTMPose 前処理 CUDA カーネル
前処理 (crop+resize+normalize+HWC→CHW) を GPU 化する `.cu` カーネルを隔離 .so 内に追加
(`enable_language(CUDA)` を Jetson 分岐で有効化、Orin sm_87)。幾何は CPU が算出する `M_inv` を
カーネルに渡し `cv::warpAffine` と完全一致 (`getAffineTransform` device 再実装不要)。RGBA→正規化 BGR を
CHW=[B,G,R] に出力、per-neighbor ゼロ境界。correctness ツール `tools/gpu_preprocess_check` で録画動画
raw_cam0.mp4 を CPU `preprocess_to_blob` と CHW 比較: **mean abs 0.0028 / L2 0.0046 / worst max 0.058**
(OpenCV 固定小数補間 1/32 量子化の床 ≈0.07 以内、カーネルの方が高精度)。ctest 9/9。Step B で TRT 入力
device 直結 + prebake 配線 + H2D 消滅。
→ [design/core-pipeline-gpu-frontend.md](../design/core-pipeline-gpu-frontend.md)

### 2026-05-29 — 全 GPU フロントエンド M1: EGL→CUDA ブリッジ常設化
スパイクで実証した EGL→CUDA 経路を隔離 .so (`libfitra_nvjpeg.so`) に常設化。`ensure_egl`
(`NvBufSurfaceMapEglImage`→`cuGraphicsEGLRegisterImage`→`GetMappedEglFrame`) で RGBA decode 出力を
**確保時 1 回だけ** CUDA device ptr に register・キャッシュ (`cudaFree(0)` で primary context を
デコードスレッドに bind、解像度変更時のみ teardown)。新 C API `fitra_nvjpeg_decode_cuda` を追加
(`decode_rgba` 本番経路は不変)。loader は `FITRA_NVJPEG_EGL=1` で opt-in し 300 フレームごとに
device↔CPU map の R-mean を回帰ログ。.so に `CUDA::cudart`/`CUDA::cuda_driver` をリンク。実機
(単一カメラ 640×480@30, nvjpeg): **device→host R-mean が CPU map と完全一致 (diff=0)**、device ptr 安定、
30fps 維持、SIGINT 0.42s クリーン終了、既定 mjpeg 経路無影響、ctest 9/9。M1 は足場 (BGR は host map から)、
M2 で device ptr 直結。
→ [design/core-pipeline-gpu-frontend.md](../design/core-pipeline-gpu-frontend.md)

### 2026-05-29 — (設計) 全 GPU フロントエンドを起票 + EGL→CUDA ブリッジ実証
nvjpeg の「高 fps では CPU 色変換が床」知見を受け、decode→前処理→TRT を host 経由なしで回す設計を
起票。make-or-break の **EGL→CUDA interop をスパイクで実証** (NvBufSurfaceMapEglImage→
cuGraphicsEGLRegisterImage→CUeglFrame の pitch-linear device ptr、cudaMemcpy2D で CPU マップと画素
一致)。VIC の CUDA メモリ直出力は非対応と確認し EGL 経路採用。RTMPose 前処理は回転なし(crop+scale)で
CUDA bilinear 吸収可。実装は未着手 (M1〜)。
→ [design/core-pipeline-gpu-frontend.md](../design/core-pipeline-gpu-frontend.md)

### 2026-05-29 — MJPEG HW デコード (`--pixel-format nvjpeg`) M1 実装
Jetson HW NVJPEG ブロックで MJPEG をデコード。libnvjpeg の無バージョン jpeg_* が OpenCV の
libjpeg-turbo と衝突するため、`NvJPEGDecoder` + `NvBufSurfTransform` を**独立 .so
(`libfitra_nvjpeg.so`)** に隔離し本体から `dlopen(RTLD_DEEPBIND|RTLD_LOCAL)`。既定 mjpeg(CPU) 経路は
非リンクで無影響。実測 (単一カメラ 640×480@30): cap→pub mjpeg 22.5 / yuyv 17.1 / **nvjpeg 20.7ms**、
色は CPU decode と meanAbsDiff<1。真価は CPU を entropy decode から解放する点 (multi-cam 向け)。
→ [design/core-pipeline-nvjpeg-decode.md](../design/core-pipeline-nvjpeg-decode.md)

### 2026-05-29 — (設計) MJPEG GPU デコード (HW NVJPEG) を新目標化
E2E レイテンシ調査中に「MJPEG の CPU decode (~6.7ms) を GPU に逃がす」目標が派生。実機調査で
CUDA `nvjpeg.h` は未搭載・**Jetson MMAPI `NvJPEGDecoder` が唯一の HW 経路**と確定（出力は YUV、
MMAPI common-class ソースの取り込みが要）。`--pixel-format nvjpeg` 追加で段階導入する設計を起票。
実装は未着手 (別コミット予定)。
→ [design/core-pipeline-nvjpeg-decode.md](../design/core-pipeline-nvjpeg-decode.md)

### 2026-05-29 — E2E レイテンシ計測基盤 + YUYV 切替 + sleep 除去 + VR イベント駆動
ステージ別レイテンシ TS を `DecodedFrame`/`Skeleton3DSnapshot` に inline 化し、central loop の
3 秒 breakdown を `cap→dec/dec→det/det→bake/bake→pose/pose→pub` 拡張、VR 側に
`e2e_capture_to_send_ms` を追加。`--pixel-format {mjpeg,yuyv}` / `--n-buffers` を CLI/YAML 化
(`Frame.jpeg`→`data`、YUYV は `cv::cvtColor` 経路)。capture/decode/central の 2ms poll sleep を
condition_variable に置換 (単一カメラ)。`--vr-extract-event-driven` で TrackerExtractor を
三角測量フレーム駆動に (opt-in, default off)。実機計測: 単一カメラ 640×480@30 で
**YUYV は MJPEG 比 cap→pub −5.5ms** (cap→dec 6.7→0.95ms、decode 消滅) かつ 30fps 維持。
M4 VR e2e は被写体 (`ik_locked`) 要のため残課題。
→ [design/core-pipeline-e2e-latency.md](../design/core-pipeline-e2e-latency.md)

### 2026-05-20 — COCO17 → Halpe26 keypoint 移行
`--keypoint-format {coco17,halpe26}` で CLI 切替。`SkeletonDef` active format singleton
(`cpp/src/lift/keypoint_format.hpp`)。subject profile v1/v2 を厳格分離。
→ [archive/phase9-halpe26-migration.md](../archive/phase9-halpe26-migration.md)

### 2026-05-15 — Phase 0–6: C++ 移行 + 性能 (aggregate 170 fps 達成)
Python 退避 + C++ skeleton (P0) → TRT engine ラッパ + correctness (P1) → 1cam e2e (P2) →
3cam + Crow Web (P3) → FP16/INT8/pinned-memory (P4) → per-cam YOLOX (P5) → 90fps push 170fps (P6)。
詳細・着地メモ・ベンチ表は [`cpp-migration-plan.md`](../cpp-migration-plan.md) に保存。

> Phase 10 (3 カメラ + C++ ライブキャリブ) はスキップ。設計メモのみ
> [archive/phase10-cpp-live-calib.md](../archive/phase10-cpp-live-calib.md) に残置。
