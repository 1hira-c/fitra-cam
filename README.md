# fitra-cam

> **「妥協じゃない」フルボディカメラトラッキング** —
> Jetson Orin Nano Super 上で複数 USB カメラから 2D 姿勢 → 多視点三角測量 → IK + Kalman で
> 3D skeleton を止めずに回し、SlimeVR / SteamVR に full-body tracker として流す。
> 速度 (per-camera TRT + cross-camera batched RTMPose + 全 GPU フロントエンド)、
> 精度 (被験者ごとの骨長 IK lock)、立体性 (床原点 PnP) を全部譲らない。

実装は C++/TensorRT 直叩き + Jetson Multimedia API。旧 Python 実装 (ONNX Runtime) は
数値リファレンス兼フォールバックとして `python/` に残置。

---

## 今 Develop で動くもの

- **2D 姿勢推定**: YOLOX (検出) + RTMPose を **per-camera × per-TRT-context** で並列。検出器は
  per-camera context、RTMPose は cross-camera batched (単一 context)。aggregate 170 fps
  (旧ベースライン比 5.5×)。keypoint topology は `--keypoint-format {coco17,halpe26}` で切替
  (既定 `coco17`)。
- **全 GPU フロントエンド** (Jetson): `--pixel-format nvjpeg` で MJPEG HW デコード → EGL→CUDA →
  YOLOX/RTMPose 前処理まで host を経由せず GPU で回す。2cam 90fps@VGA で CPU 1 コア以下、
  photon→VR-send e2e は CPU 経路比 −8ms。`mjpeg` (既定 CPU) / `yuyv` (非圧縮) も選択可。
- **3D リフト**: `--enable-3d --calib <cam_params.yaml>` で多視点三角測量 → kinematic-tree 6D
  Kalman → 骨長 IK → `/ws3d` で配信。立位伸展・横移動でも足 tracker が世界座標に置き去りに
  ならない locomotion 安定化済み。
- **VR 出力** (要 `--enable-3d --keypoint-format halpe26`、2 経路は同時 enable 可):
  - **SlimeVR Firmware UDP** (`--slimevr-out`, port 6969) — 回転のみ。位置は SlimeVR 側 IK が
    骨格 + HMD から再構築。10 tracker を named display。
  - **VMT (Virtual Motion Tracker) → SteamVR 直結** (`--vmt-out`) — 位置 + 回転。SlimeVR Server を
    飛ばし SteamVR Driver に直結。HMD pose との 2D Procrustes で yaw+xyz を自動 alignment。
- **被験者キャリブ wizard**: `--calibrate --calib-subject-id <ID> --calib-subject-height-m <m>` で
  4 ポーズ自動キャプチャ → `calibrations/subjects/<ID>/latest_profile.yaml` を出力。
  Web UI: `web/subject_calibration/`。
- **YAML config**: 長い CLI を `--config configs/<name>.yaml` に集約 (雛形は `configs/*.example`)。
  優先順位は code 既定 < YAML < 同一実行の CLI フラグ。
- **Web UI**: `/` skeleton viewer (`web/dual_rtmpose/`)、`/calibration/` wizard
  (`web/subject_calibration/`)。WebSocket + Canvas、MJPEG ストリームは出していない。
- **Python フォールバック**: `python/` に旧実装 (ONNX Runtime) を残置。新機能は追加しない。
  詳細は [`python/README.md`](python/README.md)。

---

## 開発トラックと進行中の作業

phase 番号制 (Phase 0–15.5) は 2026-05-27 に廃止し、**ドメイン別トラック制**へ移行した。
継続開発は 3 トラックの changelog として一本の流れで読める。**現状とロードマップの source of
truth は各トラック doc** ([`docs/tracks/README.md`](docs/tracks/README.md) が入口)。

| トラック | 範囲 | 状態 |
|---|---|---|
| [core-pipeline](docs/tracks/core-pipeline.md) | capture / TRT 推論 / Web / 性能 / keypoint topology / GPU フロントエンド | 安定 |
| [pose-3d](docs/tracks/pose-3d.md) | 3D lift / IK / Kalman / roll 品質 / locomotion 安定化 / subject calibration | 継続改善 |
| [vr-output](docs/tracks/vr-output.md) | SlimeVR Firmware UDP / VMT / SteamVR alignment / 出力レイテンシ | 最もアクティブ |

未着手の構想 (backlog):

- [`backlog-yolox-detector-upgrade.md`](docs/backlog-yolox-detector-upgrade.md) — 検出器の選択肢拡張 / INT8
- [`backlog-pose-backend-abstraction.md`](docs/backlog-pose-backend-abstraction.md) — 推論バックエンド抽象化
- [`backlog-main-yaml-config.md`](docs/backlog-main-yaml-config.md) — 残りの CLI を YAML config に寄せる
- [`backlog-slimevr-body-proportions.md`](docs/backlog-slimevr-body-proportions.md) — 被験者プロファイル → SlimeVR body-config

> ドキュメント構成: トラック doc = 現状 + 逆時系列 changelog、実装済み/進行中の設計は
> [`docs/design/`](docs/design/)、過去 phase の詳細設計は [`docs/archive/`](docs/archive/) に凍結、
> C++ 移行のアーキ仕様は [`docs/cpp-migration-plan.md`](docs/cpp-migration-plan.md) (凍結) が今も live。

---

## アーキテクチャ

```
USB cam × N → V4L2 mmap (4 buf/cam) → FrameSource (decode + YOLOX, per-cam thread)
                                          │  Jetson: nvjpeg HW decode + EGL→CUDA + GPU 前処理
                                          ▼ SPSC queue (size 1, drop-old)
                MultiCameraDriver → RTMPose batched (B≤3, 単一 TRT context)
                                          │
                                          ▼
                Skeleton3DBus (triangulate + 6D kinematic-tree Kalman + IK bone lock)
                                          │
                                          ▼
                TrackerExtractor (単一 producer, swing/twist smoothing)
              ┌───────────────┬───────────────┬──────────────────────┐
              ▼               ▼               ▼                      ▼
        Crow /ws (2D)   Crow /ws3d (3D)  SlimeVR Firmware UDP    VMT → SteamVR
```

設計の肝:

- **per-cam YOLOX context / cross-camera batched RTMPose**: TRT execution context は thread-safe で
  ないので `ICudaEngine` を `shared_ptr` 共有 + per-camera context。RTMPose は 3 カメラ分の bbox を
  `B≤3` の dynamic profile で 1 回 enqueue。
- **latest-frame-wins capture**: SPSC queue size 1 で drop-old。フレーム取りこぼしより**鮮度**を優先。
- **TrackerExtractor は tracker snapshot の単一 producer**: Firmware UDP / VMT / WebUI が同じ
  smoothing 履歴を共有。位置は hip 相対 hold、向きは swing/twist 分離で伸展肢の roll を保持。

詳細は [`docs/cpp-migration-plan.md`](docs/cpp-migration-plan.md) と各トラック doc。

---

## クイックスタート — Docker (推奨)

Docker Engine 導入 (初回のみ、sudo 必要): `sudo bash scripts/install_docker.sh`
ONNX 配置 (リポジトリ非同梱): `outputs/onnx/yolox_tiny_*.onnx` と `outputs/onnx/rtmpose-*.onnx`

```bash
docker compose build
docker compose --profile tools run --rm build-engines-yolox            # 初回のみ
docker compose --profile tools run --rm build-engines-rtmpose-halpe26  # 初回のみ
docker compose up
```

ブラウザで `http://JETSON_IP:8000/`。engine は `outputs/tensorrt_engines/` にホスト bind mount で
永続化 (TRT/FP16/GPU 変更で invalidate)。詳細は [`docs/docker-setup.md`](docs/docker-setup.md)。

---

## ベアメタル C++ ビルド

```bash
sudo apt install -y cmake g++ libnvinfer-dev libnvinfer-plugin-dev nvidia-jetpack libopencv-dev libv4l-dev
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
```

CMake 3.22+ / g++ 11 / TensorRT 10.3 (apt) / CUDA 12.6。header-only deps (Crow, spdlog,
nlohmann_json, CLI11, readerwriterqueue, yaml-cpp) は `FetchContent` で取得 (初回 configure は要 internet)。

初回 engine ビルド (ONNX → `.engine`, FP16 約 1〜7 分):

```bash
./cpp/build/tools/build_engines --model yolox   --onnx outputs/onnx/yolox_tiny_*.onnx --out models/yolox_tiny.fp16.engine  --fp16
./cpp/build/tools/build_engines --model rtmpose --onnx outputs/onnx/rtmpose-m_*.onnx  --out models/rtmpose_m.fp16.engine   --fp16
```

2 カメラ + 3D + VR 出力の起動例 (`--config` で同じ内容を YAML に寄せられる):

```bash
./cpp/build/main \
  --cam0 /dev/v4l/by-path/...:2.3:1.0-video-index0 \
  --cam1 /dev/v4l/by-path/...:2.4:1.0-video-index0 \
  --det-engine  models/yolox_tiny.fp16.engine \
  --pose-engine models/rtmpose_m_halpe26.fp16.engine \
  --keypoint-format halpe26 \
  --enable-3d --calib calibrations/measure_session/cam_params.yaml \
  --subject-id $USER \
  --pixel-format nvjpeg \
  --vmt-out --vmt-host 192.168.1.20
```

完全なフラグ一覧は `./cpp/build/main --help`。

### 代表 CLI フラグ抜粋

| 用途 | フラグ |
|---|---|
| 必須 | `--cam0` (+ `--cam1` / `--cam2`), `--det-engine`, `--pose-engine` |
| config | `--config <yaml>` (CLI が上書き)、雛形 `configs/*.example` |
| 取り込み | `--width 640 --height 480 --fps 30`, `--pixel-format {mjpeg,yuyv,nvjpeg}`, `--n-buffers 4`, `--det-frequency 10` |
| 推論 | `--keypoint-format {coco17,halpe26}`, `--multi-person`, `--det-score 0.5` |
| 3D | `--enable-3d --calib <yaml>`, `--subject-id <ID>`, `--no-3d-kalman`, `--no-3d-ik`, `--sync-window-ms 15` |
| SlimeVR | `--slimevr-out --slimevr-host <IP>` (要 halpe26) |
| VMT/SteamVR | `--vmt-out --vmt-host <IP>`, `--vmt-degeneracy-mode {hold,disable,skip}` (要 halpe26) |
| キャリブ wizard | `--calibrate --calib-subject-id <ID> --calib-subject-height-m <m>` |
| Web | `--port 8000 --host 0.0.0.0`, `--no-web` (driver only) |
| 診断 | `--probe` (CUDA / TRT runtime の sanity check のみ) |

---

## Web フロントエンド

| パス | 役割 |
|---|---|
| [`web/dual_rtmpose/`](web/dual_rtmpose/) | skeleton viewer。`/ws` (2D) と `/ws3d` (3D) を購読し Canvas 描画。per-tracker AxesHelper / state 表 / stats を可視化 |
| [`web/subject_calibration/`](web/subject_calibration/) | 被験者キャリブ wizard: preflight → 4 ポーズキャプチャ → review → approve → `latest_profile.yaml` |
| [`web/calibration/`](web/calibration/) | ChArUco 内部キャリブ表示 |

JSON スキーマは Python 版と互換なので、Python の `dual_rtmpose_web.py` を起動しても同じ frontend が使える。

---

## VR 出力 (Windows 側)

VMT (Virtual Motion Tracker) → SteamVR 経路の Windows 側コンポーネント (Driver ゲート / Manager /
HMD pose 中継) は **VMT フォーク** (`vmt_driver.sln` / `vmt_manager.sln`) 側に存在し、fitra-cam は
スキーマ送信のみで無改修。HMD pose は `/fitra/hmd_pose` (OSC/UDP) で受信し auto alignment に使う
(`cpp/src/vmt/hmd_pose_receiver.cpp`)。

> `windows/vmt_hmd_pose_sender/` は独立 overlay app 時代の名残で、現在は `vmt_manager` に
> 吸収済み (vr-output トラック 2026-05-27 参照)。wire format の記録としてのみ残置。

SlimeVR Firmware UDP 経路は Windows 側の SlimeVR Server がそのまま受ける (10 tracker を named display)。

---

## Python 版を使う (フォールバック)

```bash
./python/scripts/setup_jetson_env.sh
. python/.venv/bin/activate
python python/scripts/dual_rtmpose_web.py --device auto --host 0.0.0.0 --port 8000
```

詳細は [`python/README.md`](python/README.md)。新機能は追加せず、C++ 移行後のフォールバック扱い。

---

## Jetson 共通の制約

- **OpenCV / TensorRT は apt 版**を使う。`pip install opencv-python` で上書きしない (Jetson AArch64 wheel と ABI が違う)。
- **`.engine` はデバイス固有**。TRT / FP16 / GPU SM が変わると invalidate。`models/` 以下 `.engine` は git 管理しない。
- **カメラは `/dev/v4l/by-path/...`** で固定する。`/dev/video*` は再起動・抜き差しで順序が変わる。
- **ベンチ前に `sudo nvpmodel -m 0 && sudo jetson_clocks`**。MAXN + クロック固定でないと複数カメラ + 推論を捌けない。
- **FP16 RTMPose drift**: 低スコア keypoint が Y で 100–200px ずれることがある。INT8/FP16 を扱うときは録画動画で再現テスト必須。

---

## License

fitra-cam 本体は **MIT License** ([`LICENSE`](LICENSE))。

第三者ソフトウェア (Crow / asio / OpenCV / NVIDIA TensorRT・CUDA / 学習済みモデル
YOLOX・RTMPose 等) のライセンスと著作権表示は [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)
を参照。NVIDIA SDK と学習済みモデルは本リポジトリには再配布せず、ユーザーが
各配布元から導入する運用です。

---

## ディレクトリ

```
fitra-cam/
├── cpp/                # C++ / TensorRT 実装 (主軸)
│   ├── src/{camera,infer,pipeline,lift,slimevr,vmt,config,web,util}/
│   ├── tools/          # build_engines, dump_keypoints(_3d), pose_bench, det_bench,
│   │                   #   gpu_preprocess_check, check_calibration, test_* (ctest)
│   └── CMakeLists.txt
├── python/             # 旧 ONNX Runtime 実装 (フォールバック / 数値リファレンス)
├── web/                # 静的フロントエンド (dual_rtmpose, subject_calibration, calibration)
├── configs/            # main の YAML config (*.example が雛形、実体は gitignore)
├── windows/            # VMT HMD pose sender (名残、wire format 記録)
├── models/             # .onnx / .engine (engine は .gitignore)
├── calibrations/       # cam_params.yaml, subjects/<ID>/, measure_session/
├── docs/
│   ├── tracks/         # ドメイン別トラック (現状 + changelog; 入口は README.md)
│   ├── design/         # 実装済み/進行中の設計 doc
│   ├── archive/        # 旧 phaseN 詳細設計 (凍結)
│   ├── research/       # 未実装アイデアの調査メモ
│   ├── backlog-*.md    # 未着手構想
│   └── cpp-migration-plan.md   # C++ 移行記録 + アーキ仕様 (凍結, 今も live)
└── docker-compose.yml  # fitra-cam / build-engines-{yolox,yolox-tiny,rtmpose,rtmpose-halpe26}
```
