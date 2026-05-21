# fitra-cam

> **「妥協じゃない」フルボディカメラトラッキング** —
> Jetson Orin Nano Super 上で 3 USB カメラから 2D 姿勢 → 多視点三角測量 → IK + Kalman で 3D skeleton を止めずに回す。
> 速度 (per-camera TRT + cross-camera batched RTMPose)、精度 (被験者ごとの骨長 IK lock)、立体性 (床原点 PnP) を全部譲らない。

実装は C++/TensorRT 直叩き + Jetson Multimedia API。旧 Python 実装 (ONNX Runtime) はフォールバックとして `python/` に残置。

---

## 今 Develop で動くもの

- **2D 姿勢推定**: YOLOX (検出) + RTMPose (Halpe26 26 keypoints 切替可) を **per-camera × per-TRT-context** + 並列 JPEG decode で 2 カメラ aggregate 170fps ([Phase 6b](docs/cpp-migration-plan.md))。
- **3D リフト**: `--enable-3d --calib calibrations/cam_params.yaml` で多視点三角測量 → 6D Kalman → 骨長 IK → `/ws3d` で 30Hz 配信 ([Phase 7](docs/phase7-3d-ik-kalman.md))。
- **被験者キャリブ wizard**: `--calibrate --calib-subject-id <ID> --calib-subject-height-m <m>` で 4 ポーズ自動キャプチャ → `calibrations/subjects/<ID>/latest_profile.yaml` を吐く ([Phase 8](docs/phase8-ik-pose-calib.md))。Web UI: `web/subject_calibration/`。
- **キーポイントトポロジ切替**: `--keypoint-format {coco17,halpe26}` ([Phase 9](docs/phase9-halpe26-migration.md))。既定は `coco17`。halpe26 は対応 engine + 被験者プロファイル `v2` 必須 (v1 ↔ v2 マイグレーションなし、再キャリブ要)。
- **Web UI**: `/` skeleton viewer (`web/dual_rtmpose/`)、`/calibration/` wizard (`web/subject_calibration/`)、いずれも WebSocket + Canvas。MJPEG ストリームは出していない。
- **Python フォールバック**: `python/` に旧実装 (ONNX Runtime) を残置。新機能は追加しない。詳細は [`python/README.md`](python/README.md)。

---

## 進行中 (Roadmap)

| ブランチ | 内容 | 詳細 |
|---|---|---|
| `cpp-phase10` | 3 カメラ全部 C++ で内部 (ChArUco) + 外部 (床点 PnP) キャリブ。ライブ Canvas 表示 (MJPEG 廃止) | [`docs/phase10-cpp-live-calib.md`](docs/phase10-cpp-live-calib.md) |
| `cpp-phase11` | SlimeVR (VMC over OSC) 連携。8 tracker 構成で SteamVR / VRChat の FBT に流す | [`docs/phase11-slimevr-integration.md`](docs/phase11-slimevr-integration.md) |

backlog (未着手の構想):

- [`backlog-yolox-detector-upgrade.md`](docs/backlog-yolox-detector-upgrade.md) — 検出器の選択肢拡張 / INT8
- [`backlog-pose-backend-abstraction.md`](docs/backlog-pose-backend-abstraction.md) — 推論バックエンド抽象化
- [`backlog-main-yaml-config.md`](docs/backlog-main-yaml-config.md) — 長い CLI を YAML config に置換
- [`backlog-slimevr-body-proportions.md`](docs/backlog-slimevr-body-proportions.md) — fitra-cam の被験者プロファイル → SlimeVR body-config

「妥協じゃない」の到達点は Phase 11 まで完了した状態: 3 カメラ × 30fps、3D skeleton aggregate ≥ 90fps、SlimeVR から FBT デバイスとして認識。

---

## アーキテクチャ

```
USB cam × N → V4L2 mmap (4 buf/cam) → FrameSource (decode + YOLOX, per-cam thread)
                                          │
                                          ▼ SPSC queue (size 1, drop-old)
                MultiCameraDriver → RTMPose batched (B≤3, 単一 TRT context)
                                          │
                                          ▼
                Skeleton3DBus (triangulate + 6D Kalman + IK with bone lock)
                                          │
                                          ▼
                CrowServer  /ws (2D skeleton)  /ws3d (3D skeleton)  /<static>
```

設計の肝:

- **per-cam YOLOX context**: TRT execution context は thread-safe でないので `ICudaEngine` を `shared_ptr` で共有 + per-camera context。検出スレッドが互いに待たない。
- **cross-camera batched RTMPose**: bbox を 3 カメラ分集めて `B≤3` の dynamic profile で 1 回 enqueue。`opt=3` でカーネルを選ばせる。
- **latest-frame-wins**: capture worker は単一スロットを上書き。フレーム取りこぼしより**鮮度**を優先 (リアルタイム性)。

詳細は [`docs/cpp-migration-plan.md`](docs/cpp-migration-plan.md)。

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

ブラウザで `http://JETSON_IP:8000/`。engine は `outputs/tensorrt_engines/` にホスト bind mount で永続化 (TRT/FP16/GPU 変更で invalidate)。詳細は [`docs/docker-setup.md`](docs/docker-setup.md)。

---

## ベアメタル C++ ビルド

```bash
sudo apt install -y cmake g++ libnvinfer-dev libnvinfer-plugin-dev nvidia-jetpack libopencv-dev libv4l-dev
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
```

初回 engine ビルド (ONNX → `.engine`, FP16 約 1〜7 分):

```bash
./cpp/build/tools/build_engines --model yolox  --onnx outputs/onnx/yolox_tiny_*.onnx     --out models/yolox_tiny.fp16.engine     --fp16
./cpp/build/tools/build_engines --model rtmpose --onnx outputs/onnx/rtmpose-m_*.onnx    --out models/rtmpose_m.fp16.engine      --fp16
```

3 カメラ + 3D 起動:

```bash
./cpp/build/main \
  --cam0 /dev/v4l/by-path/...:2.3:1.0-video-index0 \
  --cam1 /dev/v4l/by-path/...:2.4:1.0-video-index0 \
  --cam2 /dev/v4l/by-path/...:2.5:1.0-video-index0 \
  --det-engine  models/yolox_tiny.fp16.engine \
  --pose-engine models/rtmpose_m.fp16.engine \
  --enable-3d --calib calibrations/cam_params.yaml \
  --subject-id $USER
```

完全なフラグ一覧は `./cpp/build/main --help`。

### 代表 CLI フラグ抜粋

| 用途 | フラグ |
|---|---|
| 必須 | `--cam0` / `--cam1` / `--cam2`, `--det-engine`, `--pose-engine` |
| 取り込み | `--width 640 --height 480 --fps 30`, `--det-frequency 10` |
| 推論 | `--keypoint-format {coco17,halpe26}`, `--multi-person`, `--det-score 0.5` |
| 3D | `--enable-3d --calib <yaml>`, `--subject-id <ID>`, `--no-3d-kalman`, `--no-3d-ik` |
| キャリブ wizard | `--calibrate --calib-subject-id <ID> --calib-subject-height-m <m>` |
| Web | `--port 8000 --host 0.0.0.0`, `--no-web` (driver only) |
| 診断 | `--probe` (CUDA / TRT runtime の sanity check のみ) |

---

## Web フロントエンド

| パス | 役割 |
|---|---|
| [`web/dual_rtmpose/`](web/dual_rtmpose/) | skeleton viewer。`/ws` (2D) と `/ws3d` (3D) を 30Hz で購読し Canvas 描画 |
| [`web/subject_calibration/`](web/subject_calibration/) | Phase 8 wizard: preflight → 4 ポーズキャプチャ → review → approve → `latest_profile.yaml` |
| [`web/calibration/`](web/calibration/) | ChArUco 内部キャリブ表示 (Phase 10 で本格運用予定) |

JSON スキーマは Python 版と互換なので、Python の `dual_rtmpose_web.py` を起動しても同じ frontend が使える。

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
- **ベンチ前に `sudo nvpmodel -m 0 && sudo jetson_clocks`**。MAXN + クロック固定でないと 3 カメラ + 推論を捌けない。

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
│   ├── src/{camera,infer,pipeline,lift,web,util}/
│   ├── tools/          # build_engines, dump_keypoints, dump_keypoints_3d, pose_bench, check_calibration
│   └── CMakeLists.txt
├── python/             # 旧 ONNX Runtime 実装 (フォールバック)
├── web/                # 静的フロントエンド (dual_rtmpose, subject_calibration, calibration)
├── models/             # .onnx / .engine (engine は .gitignore)
├── calibrations/       # cam_params.yaml, subjects/<ID>/, measure_session/
├── docs/               # cpp-migration-plan.md, phase*-*.md, backlog-*.md, docker-setup.md
├── outputs/            # 録画・engine cache (.gitignore)
└── docker-compose.yml  # fitra-cam / build-engines-{yolox,rtmpose,rtmpose-halpe26}
```
