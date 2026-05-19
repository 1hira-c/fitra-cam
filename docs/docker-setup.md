# Docker 経由でビルド・起動する手順

`cpp/` 実装を Jetson Orin Nano Super の Docker コンテナ内でビルド・起動し、WebUI まで動かすまでの手順。`docs/build-environment.md` は **ホスト直接ビルド** の手順、本書は **Docker 経由ビルド** の手順。両方が同じ TensorRT 10.3 を共有するため `outputs/tensorrt_engines/*.engine` はホスト/コンテナ間で再利用できる。

対象環境 (このリポジトリで動作確認した実機):

- L4T R36.5.0 / JetPack 6.2.2+b24
- CUDA 12.6.68 / TensorRT 10.3.0.30
- `nvidia-container-toolkit` v1.16.2 (既導入)
- USB UVC カメラ 2 台 (`/dev/video0..3`)

## ⚠ sudo が必要な操作

以下の 2 つだけが sudo を要する。それ以外 (`docker compose ...`) は docker グループのユーザーで実行できる。

1. `sudo bash scripts/install_docker.sh` — Docker Engine の apt 導入と docker グループ追加。**ユーザー側で実行してください。**
2. `sudo nvpmodel -m 0 && sudo jetson_clocks` — 性能測定前のみ。ベンチしないなら省略可。**ユーザー側で実行してください。**

スクリプト/手順書では sudo が出てくる場所を明示する。

## 0. 前提確認 (sudo 不要)

```bash
cat /etc/nv_tegra_release           # R36.5.0 系であること
dpkg -l | grep nvidia-container     # nvidia-container-toolkit があること
ls /dev/v4l/by-path/                # USB カメラの by-path symlink が見えること
which docker || echo "docker 未導入 → 次節へ"
```

`nvidia-container-toolkit` が未導入なら先に `sudo apt install -y nvidia-container-toolkit` を実行する (本書では既導入を前提)。

## 1. Docker Engine の導入 (sudo 必要 ← ユーザー実行)

リポジトリ直下で以下を 1 回だけ実行する。

```bash
sudo bash scripts/install_docker.sh
```

スクリプトの内容 (`scripts/install_docker.sh` 参照):

- Docker 公式 apt リポジトリを `/etc/apt/sources.list.d/docker.list` に登録
- `docker-ce` `docker-ce-cli` `containerd.io` `docker-buildx-plugin` `docker-compose-plugin` を apt 導入
- `nvidia-ctk runtime configure --runtime=docker` で `/etc/docker/daemon.json` に nvidia runtime を登録
- 実行ユーザー (`$SUDO_USER`) を `docker` グループに追加
- `systemctl enable --now docker`

完了後、**いったんログアウト/ログイン** して docker グループの所属を反映する (`newgrp docker` でも可だがログアウトの方が確実)。

動作確認:

```bash
docker run --rm hello-world
docker info | grep -i runtime                 # "nvidia" が出ること
docker run --rm --runtime=nvidia nvcr.io/nvidia/l4t-base:r36.2.0 \
    bash -c 'ls /usr/local/cuda && echo OK'   # CUDA が見えれば runtime OK
```

## 2. モデル ONNX の配置 (sudo 不要)

`outputs/onnx/` ディレクトリに以下 2 ファイルを置く (リポジトリには同梱しない)。

```text
outputs/onnx/yolox_tiny_8xb8-300e_humanart-6f3252f9.onnx
outputs/onnx/rtmpose-s_simcc-body7_pt-body7_420e-256x192-acd4a1ef_20230504.onnx
```

これらは `~/.cache/rtmlib/hub/checkpoints/` などにあれば `cp` で持って来ても良い。なければ `python python/scripts/dual_rtmpose_cameras.py` を一度ホストで実行すると `rtmlib` が自動 DL してくれる (apt OpenCV 経由)。

```bash
mkdir -p outputs/onnx outputs/tensorrt_engines
cp ~/.cache/rtmlib/hub/checkpoints/yolox_tiny_*.onnx       outputs/onnx/
cp ~/.cache/rtmlib/hub/checkpoints/rtmpose-s_*.onnx        outputs/onnx/
```

## 3. イメージビルド (sudo 不要)

```bash
docker compose build
```

初回は以下が走るため 10〜15 分ほどかかる:

- ベースイメージ `nvcr.io/nvidia/l4t-jetpack:r36.4.0` の pull (約 9〜12 GB)
- `apt install build-essential cmake libopencv-dev libv4l-dev v4l-utils`
- FetchContent で `asio` と `Crow` を git clone
- `cmake --build cpp/build -j` で C++ バイナリ生成

成功すれば `cpp/build/main` と `cpp/build/tools/build_engines` 等がイメージ内 `/workspace/cpp/build/` に生成される。

## 4. TensorRT engine の生成 (sudo 不要、初回のみ ~7 分)

ONNX を engine に変換する。`outputs/tensorrt_engines/` に書き出され、ホスト bind mount 経由で永続化される。

```bash
docker compose --profile tools run --rm build-engines-yolox
docker compose --profile tools run --rm build-engines-rtmpose
```

生成物:

```text
outputs/tensorrt_engines/yolox_tiny.fp16.engine
outputs/tensorrt_engines/rtmpose_s.fp16.engine
```

これらは GPU/TRT バージョン依存。別 Jetson に持って行く場合は再生成。

## 5. 本起動 (sudo 不要)

```bash
docker compose up
```

ログに `Crow/v1.2.1.2 Crow Server is running` 等が出れば起動成功。別端末から:

```bash
curl -s http://localhost:8000/ | head        # HTML が返ること
```

ブラウザで `http://<jetson-ip>:8000/` を開いて WebUI を確認する。カメラ画像とスケルトンが描画されれば成功。

停止は `Ctrl-C` または別端末で `docker compose down`。

## 6. 動作確認チェックリスト

| 項目 | 確認方法 | 期待値 |
|---|---|---|
| イメージビルド | `docker compose build` | エラーなし。最後に `TensorRT version = 10.3.x` が出る |
| GPU 認識 | `docker compose run --rm fitra-cam --probe` | `CUDA device count = 1` が出る |
| engine 共有 | 2 回目以降の `docker compose up` 起動時間 | 数秒で listen まで到達 |
| WebUI | ブラウザで `http://<jetson-ip>:8000/` | 2 カメラのスケルトンが描画される |
| fps | コンテナログ `cam0: recv=... avg_pose=...` | ホスト native と同等 (0.9x 以上) |

## 7. トラブルシュート

### `docker: permission denied while trying to connect ...`

docker グループの反映前。**ログアウト/ログイン** する。

### `Error response from daemon: ... unknown runtime: nvidia`

`/etc/docker/daemon.json` が未設定。**sudo で** 以下を実行:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### `Could not open device /dev/video0: Permission denied`

コンテナ内ユーザーが `video` グループでない、または `devices:` 列に `/dev/video*` が漏れている。`docker-compose.yml` の `devices:` ブロックを再確認する。

### `failed to find TensorRT` (ビルド時)

ベースイメージのタグが古いか、TRT 配置が違う。`Dockerfile` の `FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0` のタグを上げるか、`l4t-tensorrt:r10.3-devel` ベースに切り替えて、`libopencv-dev` `libv4l-dev` を追加 apt する。

### `outputs/tensorrt_engines/*.engine` がコンテナから書き出されない

ホスト側ディレクトリの所有者・パーミッションを確認。コンテナは root で動くため、`outputs/` がユーザー所有でも書き込みは可能だが、生成された engine ファイルの所有者は root になる。気になる場合は `sudo chown -R $USER:$USER outputs/`。

### 性能が出ない (ホスト native と比較して 0.7x など)

- `sudo nvpmodel -m 0 && sudo jetson_clocks` を実行 (ベンチ前の最大性能化、**ユーザー実行**)
- `tegrastats --interval 1000` で GPU/EMC clock が頭打ちでないか確認
- 2 つの USB カメラが同じ USB 2.0 バスに乗っている場合は ~15 fps × 2 が物理上限

## 関連ドキュメント

- `docs/build-environment.md` — ホスト直接ビルドの手順 (Docker を使わない場合)
- `docs/cpp-migration-plan.md` — 全体フェーズ計画
- `python/README.md` — Python リファレンス実装
