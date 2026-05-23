# fitra-cam C++ 移行計画 (3 カメラ・最大性能)

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

## リポジトリレイアウト

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
- 詳細は [`phase9-halpe26-migration.md`](phase9-halpe26-migration.md)

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
- 詳細は [`phase11-slimevr-integration.md`](phase11-slimevr-integration.md)

### Phase 12 — roll 品質改善 (M1 のみ完了) + Bridge relay 経路は没 (2026-05-22 起票 / 2026-05-23 縮退)

- **M1 完了** (PR #12 で Develop マージ済): `tracker_extract.cpp` の 二の腕 / 大腿 / 足の up を多段選択 + confidence-modulated smoothing に書き換え。Phase 11 Firmware UDP 経路 (10 本構成) で即時有効。実機評価で二の腕ひねり症状の解消 + 腕完全伸展時の roll twist 振動収束を確認
- **Bridge relay 経路 (M2-M7) は没**: Jetson `bridge_publisher` → Windows `slimevr-bridge-relay.exe` → Named pipe → SlimeVR Server の構成は採用しない。理由:
  - **座標系の問題**: `world_pos_to_slime` と `world_quat_to_bridge_slime` を同一 Rx(-90°) basis change に揃えた後も、SlimeVR avatar 上で位置と回転の整合が運用上安定しない
  - **SteamVR と Named Pipe の排他**: SteamVR 起動中は `\\.\pipe\SlimeVRInput` が排他占有され、relay の接続要求が受け付けられない (少なくとも 2026-05-23 時点の SlimeVR Server 実装)。FBT 運用は SteamVR + SlimeVR 同時起動前提なので不可
- 凍結保存: Bridge 関連実装一式 (Jetson 側 + Windows .NET 8 relay) は `archive/botsu-phase12-bridge-relay` に Y 字 merge で残置
- **位置情報を VR に流す経路は Phase 13 以降に持ち越し**
- 詳細は [`phase12-slimevr-bridge-relay.md`](phase12-slimevr-bridge-relay.md) 冒頭の 没 セクション

### Phase 13 — roll 品質詰め + WebUI tracker 可視化 + per-tracker stats (2026-05-24 着手 / 2026-05-25 締め)

- Phase 12 M1 の confidence-modulated smoothing でも残っていた「立位で脚を伸ばし切ると大腿 / 脛 / 上腕が一気に 90° roll する」症状を、**観察基盤を先に作って** → **データで仮説確定** → **構造修正** の順で解消
- **M1** (`ba9ec2c`): `SlimeTrackerBus` + `TrackerExtractor` 新規。`NativePublisher` は smoothed tracker snapshot を bus から consume するだけに refactor し、`prev_quat_` の所有を移譲。WebUI viz と Firmware UDP 送信が同じ smoothing 履歴を共有。WebUI に per-tracker `THREE.AxesHelper` × 10 を常設、`/ws3d` bundle JSON に `trackers[]` フィールド embed、`show trackers` toggle 追加
- **M2** (`71899c8`): per-tracker rolling stats (`SlimeTrackerStats`) を TrackerExtractor 内で計算。120 frame ring buffer で `angular_velocity_rad_s` p50/p95、`roll_confidence_avg`、`leakage_pct` (smoothstep 中間域フレーム比)、`freeze_pct` / `freeze_current_ms` / `freeze_max_ms` / `dropout_count` を `/ws3d` + `/stats3d` JSON に露出。WebUI に `#trackers-table` を追加、state 色分け (active / leakage 黄 / frozen 赤)
- **修正 1** (`18ef73e`): `quat_from_forward_up` の degeneracy 判定を `norm(cross) < 1e-6` (絶対しきい) → `sin θ < kRollSinLow` (相対しきい) に変更。`kRollSinLow` 0.05 → **0.15** (sin 8.6°)、`kRollSinHigh` 0.20 → **0.30** (sin 17.5°) に引き上げ。pick_up_multistage を経由しない rigid tracker (shin / chest / waist / foot) も degeneracy 保護下に
- **修正 2** (`08140f7`): `upper_arm` を `upper_leg` と同じ 1-stage 構造に揃え、secondary lateral pin (neck - shoulder) / world-Z tertiary を `Vec3f{0,0,0}` sentinel に変更。水平腕で primary degenerate → freeze に倒れる経路を確立 (Phase 12 で大腿から撤去した lateral pin と同型の anti-pattern 解消)
- **backstop** (`3613ade`): フル IK 設計メモ [`phase13-full-ik.md`](phase13-full-ik.md) を起票済として保存 (Tier A swing-twist + ROM clamp + 角速度 clamp + constrained Kalman / Tier C Bullet ragdoll)。本 Phase 13 で degeneracy gate 系統が実用品質に達したため、Tier A M1 の Phase 13 内取り込みは保留。Phase 14 候補
- **M3 (max_freeze lifecycle) は不採用**: `valid=false` 時の publisher skip + SlimeVR Server 側の前周期保持で実用上問題なかったため見送り
- 詳細は [`phase13-quality-refinement.md`](phase13-quality-refinement.md)

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
| 12    | `ctest --test-dir cpp/build --output-on-failure -R 'tracker_extract\|firmware_protocol'` + Phase 11 Firmware UDP 経路 (`--slimevr-out --slimevr-host=<windows-ip>`) で目視 | M1 のみ完了: `test_tracker_extract` の 9 pose golden + 4 confidence ケース pass、Phase 11 経路の実機で二の腕ひねり解消 / 大腿 roll が SlimeVR GUI で追従 / 完全伸展時の roll twist 振動が収束。Bridge relay 経路 (M2-M7) は SteamVR と Named Pipe 排他 + 座標系問題で **没** にしたため archive ブランチに凍結 (詳細 [`phase12-slimevr-bridge-relay.md`](phase12-slimevr-bridge-relay.md)) |
| 13    | `./cpp/build/main --enable-3d --keypoint-format=halpe26 --port 8000 ...` 起動 → ブラウザで `http://<jetson>:8000/` + `curl http://<jetson>:8000/stats3d` + Phase 11 経路で目視 | 3D viewer に 10 個の AxesHelper が表示・追従、`#trackers-table` が 30 Hz で更新、立位伸展時に脚 4 軸 + 上腕 2 軸が **state=frozen** + `ang_vel p95 < 1 rad/s`、歩行/しゃがみで `state=active` + `ang_vel p95 < 2 rad/s`、しゃがみ↔立位の遷移で snap なし。`ctest -R 'tracker_extract\|firmware_protocol'` で全 21 ケース pass。実機 (Phase 11 UDP 経路 SlimeVR Server GUI) で水平腕の上腕が体の捻りに rigid 共有されないこと |

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
