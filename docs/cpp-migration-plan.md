# fitra-cam C++ 移行計画 (3 カメラ・最大性能)

> **【凍結・歴史記録】** C++ 移行 (Phase 0–6) はゴール達成済み (aggregate 170 fps)。
> 2026-05-27 に phase 番号制は廃止し、継続開発は**ドメイン別トラック制** ([`docs/tracks/`](tracks/)) に移行した。
> 本 doc は **移行の歴史記録 + core-pipeline のアーキ仕様 (アーキ図・レイアウト・依存表・検証戦略表)** として残す。
> 新しい作業は該当トラック doc の changelog に追記すること。phase 詳細設計 doc は [`docs/archive/`](archive/) に退避済。
>
> | 旧 Phase | 移行先トラック |
> |---|---|
> | 0–6, 9 (capture / 推論 / Web / 性能 / keypoint) | [core-pipeline](tracks/core-pipeline.md) |
> | 7, 8, 12-M1, 13 (lift / IK / roll 品質 / calibration) | [pose-3d](tracks/pose-3d.md) |
> | 11, 12-bridge, 14, 15, 15.5 (SlimeVR / VMT / SteamVR) | [vr-output](tracks/vr-output.md) |

## Context

Python 版で並列 2 カメラ × RTMPose を動かしたところ、`recent_pose_fps` が 18fps で頭打ちになった。マイクロベンチでは pose 単独 50fps 出るのに、二並列で詰まる原因は CUDA セッションのコンテキスト切替 + Python GIL に張り付いた前後処理。

最終目標は **Jetson Orin Nano Super で 3 USB カメラ同時、リアルタイム** に高速・高精度の 2D 姿勢推定 (YOLOX + RTMPose) を回すこと。これは Python では到達不能と判断し、**TensorRT C++ API 直 + Jetson Multimedia API zero-copy** にフル移行する。

実機調査で確認済みのツール:

- `libnvinfer 10.3.0.30` (`/usr/include/aarch64-linux-gnu/NvInfer.h`)
- CUDA 12.6 / cuDNN 9.3
- Jetson Multimedia API `/usr/src/jetson_multimedia_api/` (NvBuffer / libnvjpeg / Argus)
- OpenCV 4.x C++ dev headers
- g++ 11.4, CMake 3.22

既存 Python 実装 (`scripts/`, `web/dual_rtmpose/`, `README.md`, `requirements-jetson.txt`, `scripts/setup_jetson_env.sh`) は **`python/` 配下に退避** して参照用に残す。新 C++ 実装は **`cpp/`** に作る。フロントエンド静的ファイルは `web/dual_rtmpose/` のまま、Python / C++ どちらからも serve できる位置に置く。

## アーキテクチャ (3 カメラ・1 プロセス・GIL なし)

```
USB cam 0 ┐
USB cam 1 ┤── V4L2 mmap ring (4 buf/cam) ──┐
USB cam 2 ┘                                 │
                                            ▼
                  [Capture thread × 3]  → NVJPEG decode (GPU) → NvBuffer
                                            │
                                            ▼ per-cam SPSC queue (size 1, drop-old)
                  [Inference thread]
                    1. detect frame == det_frequency? → YOLOX (B=3 batch) on shared TRT context
                    2. crop+normalize on GPU → RTMPose (B≤3 batch) on shared TRT context
                    3. SimCC argmax decode + inverse affine on GPU/CPU
                    4. write PoseSnapshot atomically
                                            │
                                            ▼
                  [Publisher thread]  Crow HTTP+WS, broadcasts snapshot @ 30Hz
                                       static files /web/dual_rtmpose/*

                  [Recorder] (optional, ondemand)  cv::VideoWriter or libav
```

設計の肝:

- **CUDA stream 単一・TensorRT context 単一**: セッション切替コストをゼロに。複数 context は無し
- **NVJPEG batched decode**: 3 枚同時 JPEG decode を 1 回の API call で
- **NvBuffer zero-copy**: V4L2 dequeue → NVJPEG output → TensorRT 入力までホスト経由しない
- **RTMPose dynamic batch (1..3)**: TRT engine を `min=1 / opt=3 / max=3` で optimization profile 構成
- **YOLOX 静的 batch=1 を 3 回 enqueue**: mmdeploy NMS-in-graph のため batch 化が難しい。CUDA stream 上で背中合わせ実行すれば実用上は B=3 とほぼ同等

> **2026-05-29 更新 (E2E レイテンシ作業)**: 上図の capture / SPSC は実装後に一部進化している。
> (1) ピクセル形式は MJPEG 固定ではなく `--pixel-format {mjpeg,yuyv,nvjpeg}` で切替可 (YUYV は decode を
> `cv::cvtColor` に分岐、nvjpeg は Jetson HW NVJPEG)。(2) per-cam SPSC slot (size 1, drop-old は不変) の
> ハンドオフは 2ms poll sleep から condition_variable 通知 (単一カメラ) に変更。(3) ステージ別レイテンシ
> 計測 + VR `e2e_capture_to_send_ms` を追加。(4) TrackerExtractor をイベント駆動にする opt-in
> (`--vr-extract-event-driven`)。詳細・実機数値は
> [`design/core-pipeline-e2e-latency.md`](design/core-pipeline-e2e-latency.md)。
>
> **続き (nvjpeg + 全 GPU フロントエンド)**: MJPEG HW デコード (`--pixel-format nvjpeg`、独立 .so に
> 隔離し `dlopen(RTLD_DEEPBIND)`) と、その後 decode→前処理→TRT を host を介さず GPU で回す**全 GPU
> フロントエンド**を実装済 (EGL→CUDA ブリッジ + RTMPose/YOLOX 前処理 CUDA カーネル + TRT 入力 device
> 直結)。2cam 90fps@VGA で CPU 1.83→0.98 cores。詳細は
> [`design/core-pipeline-nvjpeg-decode.md`](design/core-pipeline-nvjpeg-decode.md) /
> [`design/core-pipeline-gpu-frontend.md`](design/core-pipeline-gpu-frontend.md)。
> SimCC argmax の GPU 化 (M5) は残課題。
>
> **2026-06-19 更新 (3 カメラリグ実機 60fps 安定化)**: 検証基準 (IoU/kpt L2/170fps) は不変だが、
> capture front-end に以下を追加。(1) **per-camera capture 解像度 + ソフト downscale**: カメラ単位で
> 高解像度キャプチャ→共通ランタイム解像度へ縮小 (center-crop 個体対策; nvjpeg は VIC スケール
> decode で device 上縮小)。(2) **per-handle 非ブロッキング CUDA ストリーム**: per-camera RTMPose 前処理
> カーネルを NULL ストリームから専用ストリームへ (全 GPU バリア解消)。(3) **YOLOX 検出スケジュール**:
> 空検出時の毎フレーム再検出を廃止 + カメラ間で検出位相をずらし GPU バーストを分散。(4) **per-camera
> 露出/gain 制御** (`auto`/`manual`/`assist`): 純正 AE のブラー+fps 予算超過を回避。(5) capture の
> 毎フレーム mmap 撤廃 (`spare_data_` 再利用)。実機: 3 カメラ 640@60fps steady、`cap→pub ~8.7ms`。
> 詳細は [`design/core-pipeline-per-camera-capture-downscale.md`](design/core-pipeline-per-camera-capture-downscale.md) /
> [`design/core-pipeline-3cam-60fps-smoothing.md`](design/core-pipeline-3cam-60fps-smoothing.md) /
> [`design/core-pipeline-camera-exposure-control.md`](design/core-pipeline-camera-exposure-control.md)。
>
> **2026-06-22 更新 (idle/standby、issue #37)**: 検証基準 (IoU/kpt L2/170fps) は不変。消費者
> (WS ビューア / ライブ VR ピア) がゼロのまま継続したら重い GPU 推論を止める**待機 (idle) モード**を
> 既定 ON で追加。`calib_recording_flag` と同型の共有 atomic フラグ (素の `const std::atomic<bool>*`)
> を `FrameSource::decode_loop` (YOLOX + RTMPose pre-bake をスキップ) と `MultiCameraDriver::loop`
> (RTMPose バッチ + 3D をスキップし `idle_tick_hz` へスロットル) に通す **in-process throttle**。
> capture / decode / HW NVJPEG / EGL / TRT context は温存し、復帰は atomic 反転の次フレーム (<100ms)。
> プレゼンスは `IdleState` + ~10Hz `IdleEvaluator` (WS 計数 + HMD pose freshness、非対称ヒステリシス)。
> 復帰時は Kalman / One Euro を明示リセットして pose lurch を防ぐ。Run mode のみ (calib は対象外)。
> 詳細は [`design/core-pipeline-idle-standby.md`](design/core-pipeline-idle-standby.md)。

## リポジトリレイアウト

> **2026-06-11 更新 (pose-3d トラック)**: 下のツリーは移行計画時点のもの。その後
> `cpp/src/` には `lift/` (3D lifting / IK / calibration I/O)、`slimevr/`・`vmt/`
> (VR 出力・pose 受信)、`config/` (YAML/CLI loader)、そして **`app/` (composition root)**
> が増えた。main.cpp は config parse → validate → 排他 RunMode
> (`run` / `calib-subject` / `calib-extrinsic`) の dispatch のみで、構築シーケンスは
> `app/` の builder + モード runner にある。calib↔runtime の契約は YAML ファイルのみ
> (ライブ再注入なし)。→ [`design/pose-3d-calib-mode-separation.md`](design/pose-3d-calib-mode-separation.md)
>
> **2026-06-12 追記**: `app/daemon.{hpp,cpp}` (flow daemon)。`./main --daemon` は
> モードモジュールを fork/exec して exit code で連鎖する常駐ループで、CUDA/TRT/
> ソケットに触れない。→ [`design/pose-3d-flow-daemon.md`](design/pose-3d-flow-daemon.md)

```
fitra-cam/
├── README.md                      (C++ 用に書き直し)
├── .gitignore                     (cpp/build, *.engine 等を追加)
├── cpp/
│   ├── CMakeLists.txt
│   ├── cmake/
│   │   └── FindTensorRT.cmake     # 自前 Find module
│   ├── src/
│   │   ├── main.cpp
│   │   ├── camera/
│   │   │   ├── v4l2_capture.{hpp,cpp}    # ioctl + mmap ring
│   │   │   └── nvjpeg_decoder.{hpp,cpp}  # libnvjpeg batched
│   │   ├── infer/
│   │   │   ├── trt_engine.{hpp,cpp}      # IBuilder/IRuntime/IExecutionContext 抽象
│   │   │   ├── yolox.{hpp,cpp}           # letterbox (GPU) + post NMS scale-back
│   │   │   └── rtmpose.{hpp,cpp}         # affine warp (NPP) + SimCC decode
│   │   ├── pipeline/
│   │   │   ├── pose_pipeline.{hpp,cpp}
│   │   │   └── snapshot_bus.{hpp,cpp}    # MPMC, per-camera latest slot
│   │   ├── web/
│   │   │   └── server.{hpp,cpp}          # Crow app
│   │   └── util/
│   │       ├── cuda_check.hpp            # CUDA_CHECK/TRT_CHECK マクロ
│   │       ├── ring_buffer.hpp           # NvBuffer pool
│   │       └── logging.hpp               # spdlog ラッパ
│   ├── third_party/                     # 全て header-only / FetchContent
│   └── tools/
│       ├── build_engines.cpp             # ONNX → .engine プリビルド CLI
│       ├── pose_bench.cpp                # オフライン推論ベンチ
│       └── correctness_check.cpp         # Python 出力との数値一致確認
├── python/                              (退避された旧実装、原本そのまま)
│   ├── scripts/
│   ├── requirements-jetson.txt
│   └── setup_jetson_env.sh
├── web/dual_rtmpose/                    (既存フロントエンドそのまま流用)
├── models/                              (ONNX/engine cache。.gitignore で engine は除外)
└── outputs/                             (git ignore)
```

## 依存 (header-only / CMake FetchContent 中心)

| 用途             | ライブラリ              | 入手             |
|----------------|---------------------|----------------|
| 推論             | TensorRT 10.3       | apt (済)         |
| CUDA            | CUDA 12.6           | apt (済)         |
| 取り込み         | Jetson Multimedia API | `/usr/src/jetson_multimedia_api/` |
| JPEG decode      | libnvjpeg           | MM API 同梱       |
| アフィン         | OpenCV 4.x C++      | apt (済)         |
| HTTP + WS        | **Crow** v1.x       | FetchContent (header-only) |
| ログ             | spdlog              | FetchContent (header-only) |
| JSON             | nlohmann/json       | FetchContent (header-only) |
| CLI              | CLI11               | FetchContent (header-only) |
| 並行キュー       | moodycamel/readerwriterqueue | FetchContent (header-only) |

第三者ライブラリは全部 header-only + FetchContent。binary 依存追加無し。

## 段階実装

### Phase 0 — Python 退避 + C++ skeleton

- `scripts/`, `requirements-jetson.txt`, `setup_jetson_env.sh` を `python/` 配下に `git mv`
- `README.md` を C++ 中心に書き換え (python/ への参照を残す)
- `cpp/CMakeLists.txt` 雛形、`cmake/FindTensorRT.cmake` を整備
- "Hello TensorRT": `cudaGetDeviceCount()` と `nvinfer1::createInferRuntime()` が呼べて main がリンクすることを確認

### Phase 1 — TRT 推論エンジンラッパ + correctness

- `infer/trt_engine.{hpp,cpp}`: ONNX → engine build (CLI tool `tools/build_engines.cpp`) と engine deserialize、bindings の動的取得、CUDA stream 駆動
- `infer/yolox.{hpp,cpp}` (B=1 入力 [1,3,416,416] 静的)
- `infer/rtmpose.{hpp,cpp}` (dynamic batch 1..3 の optimization profile)
- `tools/correctness_check.cpp`: 既存 Python `pose_pipeline.py` と同じ ONNX を使い、固定テスト画像で keypoint 座標差 < 1px / bbox IoU > 0.99 を assert

### Phase 2 — 1 カメラ end-to-end

- `camera/v4l2_capture`: 1 カメラの mmap ring (4 buf) を回し MJPEG バッファを取得
- `camera/nvjpeg_decoder`: 1 枚モードで MJPEG → CUDA buffer (BGR uint8) decode
- `pipeline/pose_pipeline`: 1 カメラ inference のサンプル
- ベンチ: `--max-frames 200 --save-every 50` 相当を C++ ツール側に作って recent_pose_fps を出す。Python 単独 1 カメラより速くなることを確認

### Phase 3 — 3 カメラ並列 + Crow Web

- `camera/v4l2_capture` を N カメラ対応に
- `camera/nvjpeg_decoder` を batched API に拡張
- `pipeline/pose_pipeline`: 3 カメラ inference + per-camera tracker
- `web/server.cpp`: Crow で `/ws` WebSocket + `/` static (`web/dual_rtmpose/`)
- snapshot JSON のスキーマは Python 版と一致させる → 既存 `web/dual_rtmpose/app.js` がそのまま動く

### Phase 4 — TensorRT 最適化 + ベンチ

- FP16 engine 作成 (`--fp16` フラグ)
- INT8 PTQ: 100 frame ぐらいで calibration table 作って RTMPose を INT8 化
- pinned memory (`cudaHostAllocMapped`) で host↔device 転送高速化
- `nsys profile` でタイムライン取って残りボトルネック特定
- ベンチで 3 カメラ × 30fps (90fps aggregate) を超えるか確認

### Phase 5 — 録画オーバーレイ (任意)

- `cpp/tools/record_overlay.cpp`: raw mp4 を 3 本録 → 推論 overlay → side-by-side
- `cv::VideoWriter` (mp4v) で十分。libav 直は不要

### Phase 9 — COCO17 → Halpe26 キーポイント移行 (2026-05-20)

- `--keypoint-format {coco17,halpe26}` で CLI 切替。既定は `coco17`、回帰検証後に `halpe26` を昇格予定
- `kMaxKeypoints=26` で配列サイズを統一し、`Person::kp_count`/`Skeleton3D::kp_count` で論理点数を伝搬
- 新規ヘッダ `cpp/src/lift/keypoint_format.hpp` に SkeletonDef + active format singleton
- `fitra_subject_profile_v1`(COCO17) / `v2`(Halpe26) を厳格に分離。マイグレーション無しで再キャリブを要求
- 完了条件 = halpe26 でフル機能 (検出→IK→Phase 8 calibration)
- 詳細は [`phase9-halpe26-migration.md`](archive/phase9-halpe26-migration.md)

### Phase 11 — SlimeVR ネイティブ Firmware UDP 連携 (2026-05-21 改訂)

- Phase 10 (3 カメラ + C++ ライブキャリブ) はスキップ。Phase 9 完了の Halpe26 を前提に直接 Phase 11 に着手
- 初版は **VMC over OSC** で 8 trackers を実装したが、SlimeVR Server 上で連番表示にしかならず body-part assign が非実用的だったため、**SlimeVR ネイティブ Firmware UDP プロトコル (port 6969) へ移行** (2026-05-21)
- 10 trackers に拡張: `LEFT_UPPER_ARM` / `RIGHT_UPPER_ARM` / `CHEST` / `HIP` / `LEFT_UPPER_LEG` / `RIGHT_UPPER_LEG` / `LEFT_LOWER_LEG` / `RIGHT_LOWER_LEG` / `LEFT_FOOT` / `RIGHT_FOOT` の SlimeVR `TrackerPosition` enum に完全一致 (骨盤は当初 `WAIST(5)` を指定していたが Server で auto-assign が走らなかったため `HIP(6)` に変更)
- 起動シーケンス: Handshake (tag 3) → SensorInfo × 10 (tag 15、`trackerPosition` 指定で named display) → 60 Hz の RotationData (tag 17) + 1 Hz Heartbeat
- recv ループで Ping (tag 10) を反射、SlimeVR 側の "disconnected" マークを回避
- 位置は wire に乗らない (Firmware UDP は回転のみ)。位置は SlimeVR の IK が骨格 + HMD から再構築。カメラ由来の絶対位置を VR 側で活用したい場合は別途リレーが必要 → [`backlog-slimevr-bridge-relay.md`](backlog-slimevr-bridge-relay.md)
- MAC は `gethostname()` → SHA-1 先頭 6 byte (locally-administered + unicast)。同じ Jetson 再起動後も同 MAC → SlimeVR 側の trackerPosition 設定が persistence される
- 依存追加なし: `cpp/src/slimevr/firmware_protocol.{hpp,cpp}` で wire-format シリアライザを自前実装、`native_publisher.{hpp,cpp}` で UDP + threading を実装
- `Skeleton3DBus::snapshot()` getter を流用 (M1)。publisher スレッドは値コピーのみで pose pipeline と非干渉
- `/stats3d` に `"slimevr":{sent_handshakes,sent_sensor_info,sent_rotations,sent_heartbeats,skipped_invalid,ping_count,last_send_ms}` を露出
- 完了条件 = 10 trackers が SlimeVR Server GUI に **名前付き** で自動表示 (連番ならない / 手動 assign 不要)、live skeleton で avatar が動くこと
- 詳細は [`phase11-slimevr-integration.md`](archive/phase11-slimevr-integration.md)

### Phase 12 — roll 品質改善 (M1 のみ完了) + Bridge relay 経路は没 (2026-05-22 起票 / 2026-05-23 縮退)

- **M1 完了** (PR #12 で Develop マージ済): `tracker_extract.cpp` の 二の腕 / 大腿 / 足の up を多段選択 + confidence-modulated smoothing に書き換え。Phase 11 Firmware UDP 経路 (10 本構成) で即時有効。実機評価で二の腕ひねり症状の解消 + 腕完全伸展時の roll twist 振動収束を確認
- **Bridge relay 経路 (M2-M7) は没**: Jetson `bridge_publisher` → Windows `slimevr-bridge-relay.exe` → Named pipe → SlimeVR Server の構成は採用しない。理由:
  - **座標系の問題**: `world_pos_to_slime` と `world_quat_to_bridge_slime` を同一 Rx(-90°) basis change に揃えた後も、SlimeVR avatar 上で位置と回転の整合が運用上安定しない
  - **SteamVR と Named Pipe の排他**: SteamVR 起動中は `\\.\pipe\SlimeVRInput` が排他占有され、relay の接続要求が受け付けられない (少なくとも 2026-05-23 時点の SlimeVR Server 実装)。FBT 運用は SteamVR + SlimeVR 同時起動前提なので不可
- 凍結保存: Bridge 関連実装一式 (Jetson 側 + Windows .NET 8 relay) は `archive/botsu-phase12-bridge-relay` に Y 字 merge で残置
- **位置情報を VR に流す経路は Phase 13 以降に持ち越し**
- 詳細は [`phase12-slimevr-bridge-relay.md`](archive/phase12-slimevr-bridge-relay.md) 冒頭の 没 セクション

### Phase 13 — roll 品質詰め + WebUI tracker 可視化 + per-tracker stats (2026-05-24 着手 / 2026-05-25 締め)

- Phase 12 M1 の confidence-modulated smoothing でも残っていた「立位で脚を伸ばし切ると大腿 / 脛 / 上腕が一気に 90° roll する」症状を、**観察基盤を先に作って** → **データで仮説確定** → **構造修正** の順で解消
- **M1** (`ba9ec2c`): `SlimeTrackerBus` + `TrackerExtractor` 新規。`NativePublisher` は smoothed tracker snapshot を bus から consume するだけに refactor し、`prev_quat_` の所有を移譲。WebUI viz と Firmware UDP 送信が同じ smoothing 履歴を共有。WebUI に per-tracker `THREE.AxesHelper` × 10 を常設、`/ws3d` bundle JSON に `trackers[]` フィールド embed、`show trackers` toggle 追加
- **M2** (`71899c8`): per-tracker rolling stats (`SlimeTrackerStats`) を TrackerExtractor 内で計算。120 frame ring buffer で `angular_velocity_rad_s` p50/p95、`roll_confidence_avg`、`leakage_pct` (smoothstep 中間域フレーム比)、`freeze_pct` / `freeze_current_ms` / `freeze_max_ms` / `dropout_count` を `/ws3d` + `/stats3d` JSON に露出。WebUI に `#trackers-table` を追加、state 色分け (active / leakage 黄 / frozen 赤)
- **修正 1** (`18ef73e`): `quat_from_forward_up` の degeneracy 判定を `norm(cross) < 1e-6` (絶対しきい) → `sin θ < kRollSinLow` (相対しきい) に変更。`kRollSinLow` 0.05 → **0.15** (sin 8.6°)、`kRollSinHigh` 0.20 → **0.30** (sin 17.5°) に引き上げ。pick_up_multistage を経由しない rigid tracker (shin / chest / waist / foot) も degeneracy 保護下に
- **修正 2** (`08140f7`): `upper_arm` を `upper_leg` と同じ 1-stage 構造に揃え、secondary lateral pin (neck - shoulder) / world-Z tertiary を `Vec3f{0,0,0}` sentinel に変更。水平腕で primary degenerate → freeze に倒れる経路を確立 (Phase 12 で大腿から撤去した lateral pin と同型の anti-pattern 解消)
- **backstop** (`3613ade`): フル IK 設計メモ [`phase13-full-ik.md`](archive/phase13-full-ik.md) を起票済として保存 (Tier A swing-twist + ROM clamp + 角速度 clamp + constrained Kalman / Tier C Bullet ragdoll)。本 Phase 13 で degeneracy gate 系統が実用品質に達したため、Tier A M1 の Phase 13 内取り込みは保留。Phase 14 候補
- **M3 (max_freeze lifecycle) は不採用**: `valid=false` 時の publisher skip + SlimeVR Server 側の前周期保持で実用上問題なかったため見送り
- 詳細は [`phase13-quality-refinement.md`](archive/phase13-quality-refinement.md)

### Phase 14 — Virtual Motion Tracker (VMT) 経由 SteamVR 直結 (2026-05-25 着手)

- Phase 12 Bridge relay が没になった「位置情報を VR に流す経路」を、VMT (Virtual Motion Tracker) 経由で **SlimeVR Server を完全に飛ばして SteamVR Driver に直結** することで復活させる
- VMT は SteamVR Driver として直接登録される (`vrpathreg --install` + SteamVR 再起動)。Phase 12 のブロッカーだった SlimeVR `\\.\pipe\SlimeVRInput` 排他 + 座標系整合の不安定 を構造的に回避
- 10 trackers (Phase 11 と同じ TrackerRole 順) を `/VMT/Room/Driver` OSC で 60 Hz 送信。既定は `vmt_index = 10 + static_cast<int>(TrackerRole)` で VMT_10..VMT_19。VRChat の 8-point IK + 拡張で活用
- Phase 13 の `TrackerExtractor` (= 単一 producer) を流用、`VmtPublisher` は read-only consumer として並列接続。Firmware UDP (Phase 11/13) と VMT は **同時 enable 可能**
- Room Matrix calibration は VMT Manager GUI 任せ (CLI から OSC で打つ機能は Phase 14 では入れない)
- 位置 smoothing は `TrackerExtractor` に位置 EMA (`apply_pos_smoothing`) を追加 (回転と同じ場所で state を持つ Phase 13 原則準拠)。degeneracy 時は default で `enable=1` + 前周期 pos/quat 保持
- 座標変換 (`world_pos_to_vmt` / `world_quat_to_vmt`) は archive Bridge と完全同型 (Bridge も SteamVR `TrackerYaw.kt` の Y-up RH frame を target にしていた) なので関数本体を `cpp/src/vmt/vmt_protocol.cpp` にコピー
- OSC 1.0 wire writer は Phase 11 commit `a64becf` (撤去は `14ec5d4`) から復元し namespace `fitra::vmt` に再配置
- M1: VMT wire (`osc_writer` 復元 + `vmt_protocol`) + 8 ctest, M2: VMT publisher + CLI 配線 + main 統合, M3: 位置 EMA を TrackerExtractor に追加 + 4 ctest, M4: `/stats3d` に `vmt` ブロック splice, M5: 本 doc + 検証戦略表行 + 実機 E2E
- 詳細は [`phase14-vmt-steamvr.md`](archive/phase14-vmt-steamvr.md)

### Phase 15 — SteamVR HMD pose 駆動の自動 VMT alignment (2026-05-26 着手)

- Phase 14 の手動 alignment UI (yaw + xyz offset) では yaw 数度の誤差で足元が大きくずれ、被験者ごとに毎回数十秒の調整が必要だった。Phase 15 で **SteamVR 側 HMD の現在 pose を取り込み、chest tracker との対応から VmtAlignment を 2D Procrustes で自動算出** する経路を確立する
- **Windows 側は独立 overlay app `vmt_hmd_pose_sender` を新設** (`windows/vmt_hmd_pose_sender/`)。VMT 本体は触らない。OpenVR の `VRApplication_Background` + `TrackingUniverseStanding` で HMD pose を取り、`/fitra/hmd_pose` を 60 Hz UDP で Jetson に送る
- Jetson 側 `HmdPoseReceiver` (UDP listen + 手書き OSC parser) → `HmdPoseBus` (latest-wins + stale 判定) → `AutoAlignmentSolver` (`cv::SVD` ベース 2D Procrustes, Eigen 引かない)
- 操作は (a) T ポーズで瞬時キャリブ (`solve_tpose` n=1) と (b) 3 秒間歩行で精度モード (`solve_motion` 2D Procrustes, default n=90) の 2 通り。両方とも結果は `VmtAlignment` (手動 UI と同 channel) に流し込み、`writeVmtAlignmentForm` で UI にも反映 → 自動→手動微調整の連続フロー
- HMD = 頭頂 / chest = 胴体中心の Y 差 (個人差 0.35–0.55m) は自動では触らない (`alignment.y` 据え置き)。Y は手動 slider で被験者ごとに合わせる運用
- Room Matrix 自動化、alignment YAML 永続化、Y 軸自動補正、複数 reference 選択は Phase 16 候補
- M1: Windows overlay app, M2: HmdPoseBus + HmdPoseReceiver + CLI 配線 + 4 ctest, M3: AutoAlignment solver (`solve_tpose` + 2D Procrustes) + 8 ctest, M4: `/api/vmt/alignment/auto/*` 4 ルート + Web UI 新 form + `/stats3d.hmd` splice, M5: 本 doc + 検証戦略表行
- 詳細は [`phase15-vmt-hmd-auto-align.md`](archive/phase15-vmt-hmd-auto-align.md)

### Phase 15.5 — VMT 登録ゲートによるコントローラ奪取回避 + sender の Manager 統合 (2026-05-27 着手)

- **VMT が SteamVR のコントローラを奪う** 問題 (Quest 接続より先に VMT デバイスが登録され入力フォーカスが張り付く) を解決。切り分けで Priority / 互換モード / Driver コード改変いずれも無効、**fitra-cam を Quest 接続後に起動すれば奪われない** = 純粋な登録タイミングレースと確定
- **案A (Driver 側ハードゲート)** を採用: Driver の `Config.WaitForHmd=true` 時、`RegisterToVRSystem` を arm まで保留。Manager が `IVRSystem` で HMD valid && L/R controller connected を検知して `/VMT/Set/RegistrationEnable 1` を送る → Quest が揃ってから登録され奪取が起きない
- **`vmt_hmd_pose_sender` (Phase 15) を廃止して `vmt_manager` に吸収**: Manager は既に OpenVR client (`IVRSystem` + `OSC.cs` + 100ms timer) なので、HMD pose 中継 + 登録 arm + auto-launch を担える。常駐の手間ゼロ
- **Jetson IP は自動学習**: Driver が OSC 受信の `remoteEndpoint` (`CommunicationManager.cpp:184`) から fitra-cam の IP を取得し `/VMT/Report/JetsonAddr` で Manager に中継。Manager は `<学習IP>:39571` に `/fitra/hmd_pose` を送る。Jetson IP の手動設定が全経路から消える
- **fitra-cam は無改修**: Phase 15 の `HmdPoseReceiver` がそのまま `/fitra/hmd_pose` を受ける (送信元が sender → Manager に変わるだけ、スキーマ不変)。**実装は VMT フォーク側に存在し、fitra-cam には本 doc (設計の source of truth) のみ**
- ビルド/検証は Windows のみ (`vmt_driver.sln` MSVC / `vmt_manager.sln` C#)。ctest 対象外、Windows 手動スモークで検証
- M1: Driver ゲート + `/VMT/Set/RegistrationEnable` + `WaitForHmd` Config + 送信元 IP 通報, M2: Manager presence poll + arm + HMD pose 中継 + Jetson IP 受信, M3: `.vrmanifest` + auto-launch 登録, M4: 本 doc + 検証戦略表行 + Windows E2E
- 詳細は [`phase15.5-vmt-registration-gate.md`](archive/phase15.5-vmt-registration-gate.md)

## 検証戦略

| Phase | 検証コマンド                                                  | 合格基準                                                                 |
|-------|-----------------------------------------------------------|----------------------------------------------------------------------|
| 0     | `cmake --build cpp/build && ./cpp/build/main --help`        | リンク成功、TRT runtime が初期化できる                                         |
| 1     | `./cpp/build/tools/correctness_check --image test.jpg`     | YOLOX bbox IoU > 0.99 / RTMPose 各 keypoint 距離 < 1.0 px (vs Python ORT) |
| 2     | `./cpp/build/main --cam0 ... --max-frames 200 --bench`     | recent_pose_fps が Python 同条件比 1.5× 以上                                |
| 3     | `./cpp/build/main --cam0 ... --cam1 ... --cam2 ... --port 8000` + ブラウザ目視 | 3 ペイン分の skeleton が滑らかに描画され WS bundle 30Hz で届く |
| 4     | `./cpp/build/main --device tensorrt --fp16 --bench`           | aggregate pose ≥ 90 fps / GPU 利用率 80% 超                                |
| 5     | `./cpp/build/record_overlay --seconds 30`                    | 5 本の mp4 出力、メタデータ fps が実測通り                                       |
| 9     | `./cpp/build/main --keypoint-format=halpe26 --pose-engine <halpe26.engine> ...` | 起動ログに `kp_format=halpe26 (26 keypoints)` / `/ws` JSON に `kp_format` フィールド / `grep -rn kNumKeypoints cpp/` = 0 件 |
| 11    | `./cpp/build/main --enable-3d --keypoint-format=halpe26 --slimevr-out --slimevr-host=<windows-ip>` + Windows 側 SlimeVR Server GUI 目視 | 10 trackers (LeftUpperArm/RightUpperArm/Chest/Hip/LeftUpperLeg/RightUpperLeg/LeftLowerLeg/RightLowerLeg/LeftFoot/RightFoot) が GUI に **名前付き** で自動表示、live skeleton で avatar が破綻なく動く、`/stats3d` に `slimevr` フィールド (sent_rotations が 60×10≈600/s で増加 / ping_count > 0)、`ctest` で `test_firmware_protocol` + `test_tracker_extract` pass |
| 12    | `ctest --test-dir cpp/build --output-on-failure -R 'tracker_extract\|firmware_protocol'` + Phase 11 Firmware UDP 経路 (`--slimevr-out --slimevr-host=<windows-ip>`) で目視 | M1 のみ完了: `test_tracker_extract` の 9 pose golden + 4 confidence ケース pass、Phase 11 経路の実機で二の腕ひねり解消 / 大腿 roll が SlimeVR GUI で追従 / 完全伸展時の roll twist 振動が収束。Bridge relay 経路 (M2-M7) は SteamVR と Named Pipe 排他 + 座標系問題で **没** にしたため archive ブランチに凍結 (詳細 [`phase12-slimevr-bridge-relay.md`](archive/phase12-slimevr-bridge-relay.md)) |
| 13    | `./cpp/build/main --enable-3d --keypoint-format=halpe26 --port 8000 ...` 起動 → ブラウザで `http://<jetson>:8000/` + `curl http://<jetson>:8000/stats3d` + Phase 11 経路で目視 | 3D viewer に 10 個の AxesHelper が表示・追従、`#trackers-table` が 30 Hz で更新、立位伸展時に脚 4 軸 + 上腕 2 軸が **state=frozen** + `ang_vel p95 < 1 rad/s`、歩行/しゃがみで `state=active` + `ang_vel p95 < 2 rad/s`、しゃがみ↔立位の遷移で snap なし。`ctest -R 'tracker_extract\|firmware_protocol'` で全 21 ケース pass。実機 (Phase 11 UDP 経路 SlimeVR Server GUI) で水平腕の上腕が体の捻りに rigid 共有されないこと |
| 14    | `./cpp/build/main --enable-3d --keypoint-format=halpe26 --vmt-out --vmt-host=<windows-ip>` + Windows 側 SteamVR + VMT Manager v0.15 + VRChat 目視 | `vmt_10..vmt_19` が SteamVR Manage Trackers に出現、role 手動割当 (Waist/Chest/両足/両膝/両肘) 後 VRChat FBT で 10-point IK 追従、`/stats3d.vmt.sent_bundles` が ~60/s で増加、`disabled_count` 定常 0、`ctest -R 'vmt\|tracker_extract\|firmware_protocol'` で新 12 + 既存全 pass、`--slimevr-out` 併用で extractor state 競合なし、`nc -u -l 39570 \| xxd` で `#bundle\0` + 10 × `/VMT/Room/Driver` が 60 Hz 観測可 |
| 15    | Windows で `vmt_hmd_pose_sender.exe --jetson <jetson-ip>` 起動 → Jetson で `./cpp/build/main --enable-3d --keypoint-format=halpe26 --vmt-out --hmd-listen-enabled ...` → `http://<jetson>:8000/` をブラウザで開く | `/stats3d.hmd` が `valid=true / age_ms < 50` で更新、Web UI の HMD status バッジが "tracking"、「Tポーズで合わせる」押下で yaw/tx/tz が手動 form に反映され SteamVR avatar が HMD と一致、「移動キャリブ開始 (3s)」で歩行後 `residual_m < 0.02m`、`ctest --output-on-failure -R 'hmd_pose\|auto_alignment\|vmt'` で新 12 + 既存全 pass |
| 15.5  | (VMT フォーク, Windows 手動スモーク) `WaitForHmd=true` でビルド → SteamVR 起動 → `vmt_manager` auto-launch → HMD/コントローラ未接続のまま fitra-cam で `--vmt-out` 送信 → Quest 装着で arm | HMD 未接続の間は VMT デバイスが Manage Trackers に出ない、Quest 装着 + 両コントローラ起動で初めて登録され **VRChat FBT でコントローラが Quest に割り当たる (奪取なし)**、`vmt_hmd_pose_sender` 不起動で `curl http://<jetson>:8000/stats3d \| jq .hmd` が `valid=true` (Jetson IP 手動設定なし)、fitra-cam 先行起動 → SteamVR 後起動の順序でも奪取なし。実装は VMT フォーク側、fitra-cam は [`phase15.5-vmt-registration-gate.md`](archive/phase15.5-vmt-registration-gate.md) のみ |
| 番外 #9 | `./cpp/build/tools/det_bench --frame outputs/recorded_rtmpose/20260515_064342/raw_cam0_frame0.jpg --iters 300 --warmup 50 --engine outputs/tensorrt_engines/yolox_tiny.fp32.engine --engine outputs/tensorrt_engines/yolox_s.fp16.engine` | yolox-s FP16 の median latency が yolox-tiny FP32 の +10% 以内に収まる (Orin Nano Super, 22→24ms 帯)。`Yolox` ロード時のログに `input_size auto-set from engine: 640` が出る。 詳細は [`research/yolox-detector-eval-result.md`](research/yolox-detector-eval-result.md) |
| vr-output One Euro | `ctest --test-dir cpp/build --output-on-failure -R 'tracker_extract\|main_config'` + 実機 `./cpp/build/main --enable-3d --keypoint-format=halpe26 --vmt-out ...`(必要なら `--vr-no-one-euro` で A/B) | `tracker_extract` / `tracker_extract_pos` / `main_config` 全 pass(One Euro alpha 単調性・初フレーム snap・静止ジッタが固定 α=0.5 EMA の半分未満・持続運動でカットオフ開・`beta=0` 固定カットオフ縮退・外れ値ゲートが位置と速度状態の両方を freeze・旧 EMA の bit-identical 系無傷)。実機は座位静止で揺れない / 動作で遅延なく追従、`--vr-no-one-euro` で旧 EMA と差が出る。設計: [`design/vr-output-one-euro-filter.md`](design/vr-output-one-euro-filter.md) |
| pose-3d intrinsic (ChArUco) | `ctest --test-dir cpp/build --output-on-failure -R 'calib_io\|charuco\|intrinsic_calib_session\|main_config'` + 実機 `./cpp/build/main --calib-intrinsic --cam0 ... --intrinsic-model fisheye --charuco-squares-x 5 --charuco-squares-y 7 ...` (WebUI /intrinsic-calib で被覆を埋めて Solve) | `test_calib_io` (distortion_model round-trip + fisheye=4/pinhole>=4 validate)、`test_charuco_board` (生成盤の検出 + matchImagePoints)、`test_intrinsic_calib_session` (合成多姿勢ビュー→solve、pinhole fx 1%内/rms<0.5・fisheye fx ~3%内/rms<1.0)、`test_main_config` (calib-intrinsic mode/precedence/validate + precheck) 全 pass。実機: ChArUco を各カメラに見せ被覆 1.0 近く → Solve で rms_px < 0.5 目安、distortion_model 付き intrinsics.yaml 出力 → 案C/案D extrinsic + 三角測量で破綻なし (fisheye は端の歪み改善)。daemon は intrinsic_calib.enabled + 出力不在で intrinsic→extrinsic→subject→run と自動連鎖。設計: [`design/pose-3d-intrinsic-calibration.md`](design/pose-3d-intrinsic-calibration.md) |
| pose-3d floor-apriltag (案D) | `ctest --test-dir cpp/build --output-on-failure -R 'floor\|main_config\|apriltag'` + 実機 (高解像度 intrinsics 取得後) `./cpp/build/main --floor-replay <excal_record dir> --floor-map configs/floor_tag_map.yaml.example --floor-intrinsics <hires.yaml> --floor-out /tmp/extr.yaml` | `test_floor_tag_map` / `test_floor_extrinsic_solver` / `test_floor_calib_session` / `test_floor_calib_replay` / `test_main_config` (floor ケース) 全 pass。solver round-trip がノイズ無で復元 < 1e-3°/float 精度、床のみ配置で `planar_degenerate=true`・面外タグ追加で false、live↔replay が蓄積 bit-exact。実機 replay の per-camera 再投影 RMS < 3px (目標 < 1px)・出力 `T_cw` (fitra Z-up 無変換) を `Triangulator` に渡して三角測量成立。WebUI: daemon で `/extrinsic-calib` 方式トグル → 案D 再 spawn → solve → run 自動遷移、案C ⇄ 案D 往復。設計: [`design/pose-3d-floor-apriltag-extrinsic.md`](design/pose-3d-floor-apriltag-extrinsic.md) |
| pose-3d floor-contact stability | `ctest --test-dir cpp/build --output-on-failure -R 'floor_contact_stabilizer\|main_config\|tracker_extract\|kalman_chain'` + `dump_keypoints_3d --fps <measured>` を同一静止録画へ既定 ON / `--no-floor-contact-stability` で A/B + 実機 WebUI / VMT / SlimeVR | 左右独立・剛体平行移動・XY jitter 80%以上減衰・孤立 sole 外れ値除外・8cm 超貫通の有界 fail-safe・単発離地候補を2回+50ms graceで吸収・継続離地と補正単調減衰・grace/実適用/fresh 分離・8fps latch・FootAnchor 補正前脚長・COCO17 no-op が pass。接地率は current evidence を分母にして ≤1。静止 A/B は `ankle_xy_rms_m_pooled` が OFF 比40%以上低下、ON の `sole_below_floor_fraction == 0`、補正ノルム p95 ≤ 0.089m。実機は接地リングと `/stats3d` が一致し、sync miss で air 点滅せず、静止で足が滑らず、歩行/足踏みで離地遅延・snap がない。問題時は `--no-floor-contact-stability` で従来経路へ戻る。設計: [`design/pose-3d-floor-contact-stability.md`](design/pose-3d-floor-contact-stability.md) |

## リスク・未確定事項

- **mmdeploy 版 YOLOX ONNX の TRT 化**: NMS plugin (`EfficientNMS_TRT`) が TRT 10.3 で動くか未確認。動かなければ NMS なし版に export し直すか、CPU で NMS する暫定モードを置く
  - **Phase 1 で確認済み**: TRT 10.3.0 で問題なくビルドできる。ただし NMS 出力は data-dependent shape のため `IOutputAllocator` 経由で読む必要がある (`cpp/src/infer/trt_engine.cpp::BindingOutputAllocator`)
- **RTMPose dynamic batch profile**: optimization profile の min/opt/max を `1/2/3` に。1 人で済むケースを opt にしておかないと latency が悪化する可能性
- **NVJPEG batch 制約**: バッチ内で JPEG ヘッダの色空間 / サイズが一致している必要。USB UVC カメラの MJPG はすべて同 640x480 YUV420 なので OK のはず
- **Jetson Orin Nano Super の電力**: 3 カメラ + GPU 全力で MAXN 必須。`/etc/nvpmodel.conf` 確認
- **engine cache invalidate**: TRT バージョン / FP16 設定 / GPU SM が変わったら .engine 無効。`models/` の `.engine` は git 管理しない
- **FP16 RTMPose drift (再確認)**: Phase 1 の correctness で観測 — RTMPose を FP16 engine で回すと、低スコア keypoint (score < 0.5 帯) が input frame の Y で 100-200px ずれることがある。FP32 engine なら max kpt L2 ≈ 1.15px / p95 ≈ 0.57px に収まる。Phase 4 で INT8/FP16 を扱うときは Phase 1 と同じ動画 (`outputs/recorded_rtmpose/20260515_064342/raw_cam0.mp4`) で再現テストすること

## Phase 6 着地メモ (2026-05-15)  branch `cpp-90fps-push`

ELP AR0234 が MJPG VGA 90 fps を出せる (`v4l2-ctl --list-formats-ext`
で確認済み) ので、Phase 5 までの camera-bound 60 fps を超えにいく
フェーズ。最終 aggregate **170 fps**、Phase 3 baseline 比 **5.5×**。

### 6a — 測定支援 + preprocess の小手先最適化

- `--bench-fake-bbox`: YOLOX が空でも synthetic 中央 bbox を流して
  RTMPose を毎フレーム走らせる、ベンチ専用モード。subject 不在でも
  pipeline 上限を計測できる。
- main.cpp の stats 行に `recv_fps` と `pending` を追加。
- multi_pipeline::loop に per-pass timing (poll / rtm / snap) を追加して
  3 秒ごとに `iter_ms breakdown` を吐く。
- `RtmPose::preprocess_one`:
  - cv::Mat::convertTo の独立パスを normalize ループに fuse
  - warp 用 scratch を class shared (`warp_`, `warp_f_`) から function
    local cv::Mat に変更し、thread-safe 化
- `RtmPose::run_one_batch`: n ≥ 2 のとき std::thread で並列 preprocess。
  item 0 は caller スレッドで実行して thread spawn 数を 1 つ節約。

### 6b — preprocess を FrameSource worker thread に押し出す

main thread の RTMPose stage 14 ms 中 ~7 ms が preprocess、~5.5 ms が
GPU。preprocess を per-cam に移して main を GPU dispatch だけに減らす。

- `RtmPose::preprocess_to_blob(opts, frame, bbox, dst_chw, M_inv)` を
  static helper として公開 (scratch なし、完全 thread-safe)。
- `RtmPose::PrebakedRequest { chw, M_inv, bbox }` + `infer_prebaked()`:
  preprocess 済み入力を直接食う高速経路。
- `camera::DecodedFrame` に `chw_concat` + `M_invs` を追加。FrameSource
  decode_loop は decode → YOLOX → preprocess まで一気にやって publish。
- `MultiCameraDriver::loop` は (chw, M_inv, bbox) を集めて
  `rtmpose.infer_prebaked(reqs)` を 1 回呼ぶだけ。

### ベンチ表 (live 2 cam, `--fps 90 --det-frequency 30 --bench-fake-bbox`, RTMPose-M FP16, YOLOX FP32)

| 段階 | recv/cam | recent_pose/cam | aggregate | iter_ms | rtm_ms |
|---|---|---|---|---|---|
| Phase 5 baseline (--fps 30 cam-bound) | 30 | 30.0 | 60 | (n/a) | (n/a) |
| 6a fake-bbox 開始 (preprocess on main) | 88 | 52-56 | 107 | 17.7 | 17.7 |
| 6a + preprocess fuse | 88 | 51-56 | 107 | 17.6 | 17.6 |
| 6a + parallel preprocess (B=2) | 88 | 65-71 | **138** | 13.7 | 13.7 |
| **6b preprocess on per-cam worker** | 88 | **85-87** | **170** | **4.5** | **4.4** |

iter_ms 17.7 → 4.5、rtm step 17.7 → 4.4。recent_pose 85 fps × 2 cam =
aggregate 170 fps で camera-saturated (88 fps 受信ぎりぎり)。main thread
は 220 iter/s の余力あり、3 カメラ目を足せば aggregate ~260 fps が射程。

### Correctness 回帰 (raw_cam0.mp4 30 frames, RTMPose-S FP32)

dump_keypoints は内部 preprocess 経路 (infer_batch) で max kpt L2 = 1.15 px。
Phase 1 / 4 / 5 と完全一致。リファクタは数値に影響なし。

Phase 6b の本番経路に近い `dump_keypoints --prebaked` でも Python CPU
参照比は bbox IoU min = 0.9932、max kpt L2 = 1.1490 px、p95 = 0.5797 px。
内部 preprocess 経路と同等。ただし `compare_keypoints.py` の既定閾値
`--kpt-threshold 1.0` は max 値だけ超えるので、判定は既存経路との同等性
確認として扱う。

### 残課題 (Phase 7 候補)

- 3 カメラ目の実機接続で aggregate ≥ 250 fps を確認
- GPU 前処理 (CUDA カーネルで warpAffine + 正規化 + HWC→CHW): per-cam
  CPU を完全に解放して decode + YOLOX に専念させる
- INT8 PTQ で RTMPose-M / YOLOX を更に高速化
- NEON SIMD による normalize ループ最適化 (現状 ~7 ms → <1 ms 期待)
- USB 3.0 ハブで camera あたり 120+ fps を狙う

## Phase 5 着地メモ (2026-05-15)  branch `cpp-phase5`

Phase 4 で残った最大支配 (推論スレッド側の逐次 YOLOX = ~26 ms/call)
を per-camera 並列化することで解消。

### 5a — TrtEngine の共有可能化
`TrtEngine` は元々 `unique_ptr<ICudaEngine>` を抱える持ち主だったが、
TRT execution context が thread-safe ではないので per-cam 並列推論には
N 個の context が要る。

- `engine_` を `shared_ptr<ICudaEngine>` に変更。
- 新規 API:
  - `TrtEngine::load_shared(runtime, path) → shared_ptr<ICudaEngine>`
  - `TrtEngine::from_shared(shared_ptr) → unique_ptr<TrtEngine>`
- 1 個の deserialized ICudaEngine を main で読み、各カメラで
  `from_shared()` を呼ぶと per-cam な IExecutionContext を持つ
  `TrtEngine` が出来上がる。device buffer / stream / bindings は
  TrtEngine ごとに独立。

### 5b — per-camera YOLOX を FrameSource に同居
`camera::FrameSource` を「decode 専用」から「decode + YOLOX」に拡張:
- ctor で `std::unique_ptr<infer::Yolox>` を受け取る (nullptr 可)。
- decode worker で `decoder_.decode(...)` の直後に
  det-frequency / single-person フィルタ込みで `yolox->infer()`。
- 公開する `DecodedFrame` に `bboxes` を入れて、 main 側が
  そのまま RTMPose batched に渡せるようにした。
- `MultiCameraDriver` から YOLOX 参照と det_frequency state が
  消え、3 パスのうちパス 1 が単純な「latest slot ポーリング」だけになる。

main.cpp の繋ぎ込みは:
```
  yolox_shared = TrtEngine::load_shared(runtime, det_engine_path);
  rtmpose_eng  = TrtEngine::from_file(...);           // 共有不要 (バッチ専用)
  for each cam:
    yolox_eng = TrtEngine::from_shared(yolox_shared); // per-cam context
    yolox     = Yolox(*yolox_eng, opts);
    source    = FrameSource(capture, move(yolox), src_opts);
```

### ベンチ (live 2 cam, det-frequency=10, 持続値)

| 構成 | per-cam recent_pose | aggregate | vs Phase 3 |
|---|---|---|---|
| Phase 3 baseline (B=1, serial decode, FP32) | 15.7 fps | 31.4 fps | 1.00× |
| Phase 4a+b cross-cam batched RTMPose | 15.4 fps | 30.8 fps | 0.98× |
| Phase 4d parallel CPU decode | 19.3 fps | 38.6 fps | 1.23× |
| Phase 4c FP16 YOLOX (緩 correctness) | 26.7 fps | 53.5 fps | 1.70× |
| **Phase 5 per-cam YOLOX + RTMPose-M FP16** | **30.0 fps** | **60.0 fps** | **1.91×** |

Phase 5 構成は **2 カメラ × 30fps が camera-bound に張り付く** 状態 ——
推論側に余裕があり、3 カメラ目を足せば aggregate ≥ 90 fps 目標を
満たす見込み (要 USB 帯域の確認)。stage_ms も 60-90ms → 25-34ms に半減。

### Correctness (raw_cam0.mp4 30 frames, Phase 5 build)

| pose engine | dump_keypoints fps | max kpt L2 vs Python CPU | 判定 |
|---|---|---|---|
| RTMPose-S FP32 | 21.33 | 1.15 px | ✓ Phase 1 と同一 |
| RTMPose-M FP16 | 19.63 | 2.51 px | ✓ Phase 4c と同一 |

(dump_keypoints は単一カメラ経路なので Phase 5 並列化と無関係に通る。
リファクタが既存パスを壊していないことの回帰確認。)

### 残課題 (Phase 6 候補)

- 3 カメラ目を物理接続して aggregate ≥ 90 fps を確認
- GPU 前処理 (warpAffine + 正規化 + HWC→CHW を CUDA カーネル化) で
  RTMPose 周りの CPU 時間を削る
- Jetson MMAPI NvJpegDecoder + NvBuffer zero-copy
- INT8 PTQ (RTMPose-M / YOLOX、calibration set 選定)
- pinned memory (`cudaHostAllocMapped`) で H2D/D2H 短縮

## Phase 4 着地メモ (2026-05-15)  branch `cpp-phase4`

最終目標 (3 cam × 30fps aggregate ≥ 90fps, Python 比 4×) **未達**。
得られたもの:

### 4a — RtmPose の真のバッチ詰め (commit 7035166)
- `infer_batch(reqs)` で複数 (frame, bbox) を 1 回の enqueue にまとめる。
- engine profile `opt=1 → opt=3` (build_engines preset)。TRT は opt 形状で
  カーネルを選ぶので 3 カメラ前提なら opt=3 が正しい。
- trtexec で確認したカーネル単体性能 (FP32, sm_87):
  - B=1: 2.63 ms / call → 380 persons/s
  - B=3: 4.20 ms / call → 712 persons/s (1.87× / person)

### 4b — multi cam 跨ぎのバッチ詰め (commit 7035166)
- multi_pipeline を 3 パス構造 (collect / batch infer / distribute) に変更。
- RTMPose enqueue 回数が N → ceil(N/3) に。

### 4d — 並列 CPU JPEG decode (commit 909d8d1)
- `cpp/src/camera/frame_source.{hpp,cpp}` 新設。V4l2Capture を専用 decode
  スレッドでラップ。各カメラの cv::imdecode が並列実行される。
- CUDA `nvjpeg.h` は JetPack に未搭載。Jetson MMAPI `NvJpegDecoder` は
  NvBuffer/DMABUF 前提で侵襲が大きく Phase 5 候補に延期。

### 4c — FP16 evaluation
- **RTMPose-S FP16**: 壊滅。max kpt L2 = 237 px (raw_cam0.mp4 30 frames)。
  低スコア帯だけでなく全体的に座標がランダムに飛ぶ。
- **RTMPose-M FP16**: 健全。max kpt L2 = 2.51 px / p99 = 0.84 px、
  かつ GPU compute は S FP32 と同等 (B=1 2.78 vs 2.64 ms, B=3 4.69 vs 4.20 ms)。
  → **S FP16 の異常は容量起因と推測**。狭いチャネル幅で FP16 の動的レンジに
  乗らない活性化が一部レイヤで発生し argmax が壊れる。M は同じ手順で問題なし。
  推奨は **RTMPose-M FP16** へシフト (S FP32 から速度を落とさず精度を上げる)。
- **YOLOX FP16**: 推論は 1.6× 高速 (5.87 → 3.67 ms GPU)、
  しかし bbox 微差 (IoU 0.93) が RTMPose の crop に伝播し
  最終 keypoint で max 4.6 px の差。プロダクション許容範囲だが
  Phase 1 の correctness 基準 (IoU > 0.99) は満たさない。
  → デフォルトは FP32 のまま、`--det-engine models/yolox_tiny.fp16.engine`
  を選択肢として提供。

### 4c 補足 — 全 (model, precision) ベンチ表 (raw_cam0.mp4 30 frames vs Python ORT CPU)

| pose engine | GPU B=1 | GPU B=3 | max kpt L2 | mean | p99 | bbox IoU min | 判定 |
|---|---|---|---|---|---|---|---|
| RTMPose-S FP32 | 2.64 ms | 4.20 ms | 1.15 px | 0.10 | 0.75 | 0.993 | ✓ 現 default |
| **RTMPose-M FP16** | **2.78 ms** | **4.69 ms** | **2.51 px** | **0.16** | **0.84** | 0.993 | **✓ 推奨** |
| RTMPose-M FP32 | 4.63 ms | 8.87 ms | 1.66 px | 0.07 | 0.63 | 0.993 | ✓ (遅) |
| RTMPose-S FP16 | 2.30 ms | 3.40 ms | **237 px** | 63.0 | 201 | 0.993 | ✗ 壊滅 |

(YOLOX は 全行 FP32 engine 固定で比較)

### 4c 補足 — RTMPose-S FP16 壊れ方の per-keypoint パターン

raw_cam0.mp4 30 frames で、S FP16 と Python ORT CPU の keypoint diff を kp 別に集計:

| keypoint        | max (px) | mean (px) |
|---|---|---|
| nose, l_eye, r_eye | 189–194 | 110–172 |
| l_ear, r_ear        | 193–203 | 105–169 |
| **l_shoulder, r_shoulder** | **0.74–1.23** | **0.37–0.59** |
| l_elbow / r_elbow / wrist / hip / knee / ankle | 60–237 | 8–176 |

スコア帯別の mean drift:

| ref score 帯 | S FP16 mean | S FP16 max | M FP16 mean | M FP16 max |
|---|---|---|---|---|
| ≥ 0.7  | 71.9 px | 210.7 | 0.11 px | 0.92 |
| 0.3–0.7 | 47.9 px | 237.1 | 0.29 px | 2.51 |
| < 0.3  | 41.6 px | 143.1 | 0.82 px | 1.20 |

**観察**: スコアと drift が無相関で、**両肩だけ無事 / それ以外は全滅** という配置依存パターン。
SimCC head の argmax が FP16 のビン比較反転で別位置に飛んでいると説明可能。M はこの不安定領域が消える。
要 follow-up (将来): trtexec の `--exportTimes` / TRT API で intermediate tensor (simcc_x/y 自体) を dump して、ピーク bin の値・隣接 bin との比、を S/M で比較すれば原因確定する。

### ベンチ表 (live 2 cam, det-frequency=10, 持続値)

| 構成 | per-cam recent_pose | aggregate | vs Phase 3 |
|---|---|---|---|
| Phase 3 baseline (B=1, serial decode, FP32) | 15.7 fps | 31.4 fps | 1.00× |
| + cross-cam batched RTMPose (4a+4b)         | 15.4 fps | 30.8 fps | 0.98× (empty scene) |
| + parallel CPU decode (4d)                  | 19.3 fps | 38.6 fps | 1.23× |
| + FP16 YOLOX (correctness 緩, 4c)            | **26.7 fps** | **53.5 fps** | **1.70×** |

(注: 上記は live test 中シーン無被写体の "YOLOX every frame" 最悪値.
被写体ありで det_frequency=10 が効くシーンでは aggregate 60-70 fps 域に届く見込み)

### Python 比 (raw_cam0.mp4, 200 frames, det-frequency=1)

| 構成 | fps | vs Python ORT-CUDA |
|---|---|---|
| Python ORT-CUDA           | 15.57 | 1.00× |
| C++ TRT FP32 (current best) | 23.34 | 1.50× |
| C++ TRT FP16 YOLOX + FP32 pose | 26.84 | 1.72× |

### 残課題 (将来の Phase 5+)

- **per-camera YOLOX context**: 現状 YOLOX は推論スレッド側で逐次実行で
  最大コスト (FP32 で 26ms/call)。N 個の TRT execution context を per-cam
  スレッドに置けば aggregate が大きく動く見込み。
- **GPU 前処理**: warpAffine + 正規化 + HWC→CHW を CUDA カーネル化。
  CPU 側を inference スレッドから完全に剥がす。
- **NVJPEG GPU decode**: Jetson MMAPI NvJpegDecoder + NvBufferTransform で
  zero-copy 経路を作る。CUDA stream 上で完結し CPU 開放。
- **INT8 PTQ**: 100 frame calibration で RTMPose / YOLOX を INT8 化。
  drift 観察と calibration set の選定が必要。
- **pinned memory**: 入力 H2D / 出力 D2H に `cudaHostAllocMapped`。

## Phase 3 完了メモ (2026-05-15)

- 構成:
  - `cpp/src/pipeline/snapshot.{hpp,cpp}` — 全カメラ最新スナップショットを mutex 保護で保持。`make_bundle_json()` で Python 互換スキーマの JSON を生成
  - `cpp/src/pipeline/multi_pipeline.{hpp,cpp}` — N カメラ駆動。N capture スレッド + 1 共有推論スレッド + 共有 Yolox/RtmPose (single TRT context)。round-robin で順番に処理し、SnapshotBus に書く
  - `cpp/src/web/crow_server.{hpp,cpp}` — Crow ベース。`/` index.html、`/<path>` static、`/stats` JSON、`/ws` WebSocket。Crow デフォルトの SIGINT ハンドラを `signal_clear()` で外し、自前ハンドラから driver/server を順に閉じる
  - `cpp/src/main.cpp` — Phase 0 の probe-only main を置き換え。`--cam0/1/2` で N カメラ起動、`--no-web` でドライバ単独動作
  - `web/dual_rtmpose/index.html` + `app.js` — bundle.cameras 配列から動的にペインを生成 (2/3 cam どちらでも動く)
- 動作確認 (cam0 + cam1, FP32 engine, det-frequency=10):
  - WebSocket `/ws` で 30Hz 配信、JSON schema は Python `dual_rtmpose_web.py` 互換
  - 各カメラ recv=30 fps / recent_pose=15.7 fps (USB 2.0 共有で 1 cam あたり 15fps 推論)
  - pending が増え続ける (バックログ蓄積) → Phase 4 で batched RTMPose + GPU 前処理で吸収
- 既知の課題 (Phase 4 行き):
  - **pending 増大**: 30 fps × 2 受信 vs 15 fps × 2 処理 → 毎秒 30 フレーム遅れる (latest-frame-wins でドロップされ続けるので破綻はしない)
  - **RTMPose B=1 ループ**: 複数人 / 複数カメラの bbox を 1 リクエストずつ enqueue している。engine は dynamic batch 1..3 でビルド済みなので、バッチ詰めで 2-3× 改善見込み
  - **CPU JPEG decode**: Phase 2 baseline のまま。NVJPEG に置換で stage_ms 短縮 + CPU 解放
- SIGINT で driver と Crow を順番に閉じて exit code 0。停止には Crow run() のドレインで数秒かかる
- ベンチコマンド (live):
  ```
  ./cpp/build/main --cam0 /dev/v4l/by-path/...:2.3:1.0-video-index0 \
                   --cam1 /dev/v4l/by-path/...:2.4:1.0-video-index0 \
                   --det-engine models/yolox_tiny.fp32.engine \
                   --pose-engine models/rtmpose_s.fp32.engine
  # ブラウザで http://JETSON_IP:8000/
  ```

## Phase 2 完了メモ (2026-05-15)

- 構成:
  - `cpp/src/camera/v4l2_capture.{hpp,cpp}` — V4L2 MJPEG 直叩き (ioctl + mmap)、4 buffer ring、latest-frame-wins
  - `cpp/src/camera/jpeg_decoder.{hpp,cpp}` — Phase 2 は `cv::imdecode` (CPU)。Phase 4 で Jetson MM API libnvjpeg に差し替え
  - `cpp/src/pipeline/pose_pipeline.{hpp,cpp}` — 1 カメラの capture → decode → YOLOX → RTMPose
  - `cpp/tools/pose_bench` — ライブカメラベンチ
- ライブカメラ動作 (cam0 単独, `--det-frequency 10`, FP32 engine):
  - recv=30.04 fps / avg_pose=28.92 / recent_pose=29.97 / stage_ms=32 / pending≈3
  - カメラの 30fps 上限に張り付く (パイプライン側に余裕あり)
- ベンチ (raw_cam0.mp4, 200 フレーム, `--det-frequency 1`):
  - Python ORT-CUDA: 15.57 fps
  - C++ TRT FP32:    23.49 fps  (**1.51× vs Python**, Phase 2 目標達成)
  - C++ TRT FP16:    31.19 fps  (Phase 4 でドリフト解決後の上限値、参考)
- 残課題 (Phase 4 で対応):
  - JPEG decode が CPU 経路 (`cv::imdecode`)。GPU NVJPEG にすると stage_ms 短縮 + CPU 開放
  - 1 カメラだと recv=30fps が天井。3 カメラ aggregate ≥ 90fps が Phase 4 ゴール
  - FP16 RTMPose drift (Phase 1 既知) を INT8 PTQ / 入力 cast で吸収する

## Phase 1 完了メモ (2026-05-15)

- engine 構築: `models/{yolox_tiny,rtmpose_s}.fp32.engine`
- correctness:
  - 入力: `outputs/recorded_rtmpose/20260515_064342/raw_cam0.mp4` の最初 30 フレーム
  - 基準: `python/scripts/dump_reference_keypoints.py --device cpu`
  - 候補: `cpp/build/tools/dump_keypoints` (YOLOX/RTMPose とも FP32)
  - 結果: bbox IoU min 0.993 / kpt L2 max 1.15px (99% < 0.75px) / score diff max 0.016
  - 合格基準は計画値の **< 1.0px から ~1.5px に緩和** が現実的 (TRT 10.3 vs ORT 1.23 のカーネル差で説明できる微差)
- 関連ファイル:
  - `cpp/src/infer/trt_engine.{hpp,cpp}` — `IOutputAllocator` で data-dependent shape 対応 (YOLOX NMS, RTMPose dynamic batch)
  - `cpp/src/infer/{yolox,rtmpose}.{hpp,cpp}` — Python と同じ前後処理 (cv::warpAffine, BGR mean/std, SimCC argmax + 逆 affine)
  - `cpp/tools/{build_engines,dump_keypoints}.cpp` — engine ビルド CLI と correctness 用ダンプ
  - `python/scripts/{dump_reference_keypoints,compare_keypoints}.py` — Python リファレンスと差分集計

## 完了の定義

- `cpp/build/main` が 3 USB カメラ + WebSocket + 録画オプションを一気通貫で提供
- recent_pose_fps が aggregate で 80fps 以上、Python 比 4× 以上
- 既存 `web/dual_rtmpose/` 静的ファイルがそのまま使え、ブラウザで 3 ペイン skeleton が見える
- `python/` 配下に旧実装が残り、README から退避場所が辿れる
- engine prebuild → cold start ≤ 3 秒

## 番外編: YOLOX 検出器のモデルサイズ × 量子化精度の再評価 (2026-05-24)

Issue [#9](https://github.com/1hira-c/fitra-cam/issues/9) — 横向き / 部分遮蔽で YOLOX-tiny humanart が崩れる運用上の不満を起点に、より大きい YOLOX-{S, M, X} を yolox-tiny **FP32** の e2e latency 予算で再評価。

- `det_bench` (新規 `cpp/tools/det_bench.cpp`) で recorded `raw_cam0.mp4` の 1 フレームに対し `Yolox::infer()` の host 側 wall-clock (preprocessing + H2D + enqueue + sync + D2H) を 300 iter × 3 round 測定
- mmdeploy 配布の YOLOX-S/M/X end2end ONNX は TopK K=5000 で TRT 10.3 の `ITopKLayer` 上限 (3840) を越えるため、`outputs/onnx/<slug>.topk3000.onnx` に書き換えてから build。手順は [`research/yolox-detector-eval-result.md`](research/yolox-detector-eval-result.md)
- `Yolox::Options::input_size` は engine binding の dims から自動で 416/640 を拾う作りに変更 (`cpp/src/infer/yolox.cpp`)。`--det-engine` 差し替えだけで切り替わる

結果 (Orin Nano Super, raw_cam0 1 frame, median ms / 300 iter):

| engine | 入力 | median (ms) | p90 (ms) | 判定 |
|---|---|---|---|---|
| yolox-tiny FP32 | 416 | **22.3** (budget) | 22.5 | baseline |
| yolox-tiny FP16 | 416 | 19.1 | 19.5 | 現運用 |
| **yolox-s FP16** | **640** | **23.9** | **24.1** | **採用 (budget +7%)** |
| yolox-m FP16 | 640 | 27.9 | 30.4 | 予算 +25% |
| yolox-x FP16 | 640 | 52.3 | 52.6 | 予算 +134% |

- 既定切替: `docker-compose.yml` の runtime + `build-engines-yolox` を `yolox_s.fp16.engine` に変更 (旧 tiny は `build-engines-yolox-tiny` で並存)
- Python 側 (`python/scripts/pose_pipeline.py::DEFAULT_DET_MODEL`) は **据え置き**。Python ORT YOLOX-tiny は Phase 1 の回帰 baseline として固定 (`docs/backlog-yolox-detector-upgrade.md` の評価データセット節)。`--det-model outputs/onnx/yolox_s_8xb8-300e_humanart-3ef259a7.onnx` で個別に試せる
- 視覚再評価 (横向き / 部分遮蔽) は本番運用で都度確認する方針に変更。失敗フレームの定量比較は backlog のままで、INT8 PTQ も RTMPose 側と合流するタイミング (`docs/research/rtmpose-int8-eval-plan.md`) で扱う

## Phase 番外編: Docker 化 (2026-05-19)

別 Jetson へ持ち運ぶたびに `docs/build-environment.md` の apt 手順を踏む負担を減らすため、C++ 実装を Docker コンテナで完結させる。

- ベースイメージ: `nvcr.io/nvidia/l4t-jetpack:r36.4.0` (CUDA 12.6 + TensorRT 10.3 同梱)
- `Dockerfile` / `docker-compose.yml` / `.dockerignore` / `scripts/install_docker.sh` を追加
- engine cache は `./outputs/tensorrt_engines/` をホスト bind mount でコンテナと共有
- 起動コマンド (sudo 不要、 docker グループのユーザーで実行):

  ```bash
  docker compose --profile tools run --rm build-engines-yolox       # 初回のみ
  docker compose --profile tools run --rm build-engines-rtmpose     # 初回のみ
  docker compose up
  ```

詳細は **[docs/docker-setup.md](./docker-setup.md)** を参照。
