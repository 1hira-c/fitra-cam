# ビルド環境整備手順

Jetson Orin Nano Super 上で `fitra-cam` の C++/TensorRT 実装をビルドし、必要に応じて Python 退避版も動かすための手順。

対象は Ubuntu 22.04 / L4T R36.x / JetPack 6.2.x 系。C++ 側が主軸で、Python 側は数値リファレンスとフォールバックとして使う。

## 1. 現状確認

まず OS、Jetson BSP、コンパイラ、CUDA/TensorRT/OpenCV、Python venv を確認する。

```bash
uname -m
cat /etc/nv_tegra_release
g++ --version
cmake --version
nvcc --version
pkg-config --modversion opencv4
python3 --version
python3 -m ensurepip --version
```

Jetson では `nvidia-smi` ではなく `tegrastats` を使う。

```bash
tegrastats --interval 1000
```

apt パッケージの導入状況をまとめて見る場合:

```bash
dpkg-query -W -f='${binary:Package} ${Version}\n' \
  build-essential cmake git pkg-config ca-certificates \
  nvidia-jetpack cuda-toolkit-12-6 \
  libnvinfer-dev libnvinfer-plugin-dev libnvonnxparsers-dev \
  libopencv-dev python3.10-venv python3-pip python3-opencv
```

## 2. sudo が必要な apt 依存

未導入の apt パッケージは sudo で入れる。C++ ビルド、TensorRT engine build、Python 退避版の最小構成をまとめると以下。

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config ca-certificates \
  nvidia-jetpack cuda-toolkit-12-6 \
  libnvinfer-dev libnvinfer-plugin-dev libnvonnxparsers-dev \
  libopencv-dev \
  python3.10-venv python3-pip python3-opencv
```

`nvidia-jetpack` は大きいメタパッケージなので、すでに JetPack/CUDA/cuDNN/TensorRT が揃っている実機では再導入しなくてよい。CMake が見る必須ファイルは次で確認できる。

```bash
ls /usr/local/cuda
ls /usr/include/aarch64-linux-gnu/NvInfer.h
ls /usr/lib/aarch64-linux-gnu/libnvinfer.so
pkg-config --modversion opencv4
```

`nvcc` が見つからない場合は、CUDA の PATH をシェルに追加する。

```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

永続化する場合は同じ内容を `~/.bashrc` へ追記する。

## 3. Python 退避版の venv

apt 導入後、ユーザー権限で `python/.venv` を作る。

```bash
./python/scripts/setup_jetson_env.sh
. python/.venv/bin/activate
```

`ensurepip is not available` で失敗した場合は `python3.10-venv` が不足している。apt 導入後、作成途中の venv を消してから再実行する。

```bash
rm -rf python/.venv
./python/scripts/setup_jetson_env.sh
```

GPU/CUDA/TensorRT provider 付きの ONNX Runtime が必要な場合は、venv 有効化後に Jetson AI Lab の JetPack 6 / CUDA 12.6 向け wheel を入れる。これは sudo 不要だがネットワークが必要。

```bash
. python/.venv/bin/activate
python -m pip uninstall -y onnxruntime onnxruntime-gpu
PYTHONNOUSERSITE=1 python -m pip install --no-deps \
  https://pypi.jetson-ai-lab.io/jp6/cu126/+f/4eb/e6a8902dc7708/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl
```

確認:

```bash
PYTHONNOUSERSITE=1 python - <<'PY'
import cv2
import numpy as np
import onnxruntime as ort

print("cv2:", cv2.__version__, cv2.__file__)
print("numpy:", np.__version__)
print("ort:", ort.__version__)
print("providers:", ort.get_available_providers())
PY
```

`opencv-python` / `opencv-python-headless` は pip で入れない。Jetson では apt の `python3-opencv` を使い、V4L2/GStreamer 付き OpenCV を維持する。

## 4. C++ ビルド

初回 configure は CMake `FetchContent` で `asio` と `Crow` を GitHub から取得するため、ネットワークが必要。sudo は不要。

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j"$(nproc)"
ctest --test-dir cpp/build --output-on-failure
./cpp/build/main --help
```

ビルドで生成される主な実行ファイル:

```text
cpp/build/main
cpp/build/tools/build_engines
cpp/build/tools/dump_keypoints
cpp/build/tools/dump_keypoints_3d
cpp/build/tools/pose_bench
cpp/build/tools/check_calibration
```

ONNX から TensorRT engine を作る処理には `libnvonnxparsers-dev` が必要。モデルファイルや `.engine` は環境依存なのでリポジトリにコミットしない。

## 5. Python 退避版の動作確認

モデル ONNX は既定では `~/.cache/rtmlib/hub/checkpoints/` 配下を読む。未配置の場合は、配布元から次の 2 ファイルを置く。

```text
yolox_tiny_8xb8-300e_humanart-6f3252f9.onnx
rtmpose-s_simcc-body7_pt-body7_420e-256x192-acd4a1ef_20230504.onnx
```

短い確認:

```bash
. python/.venv/bin/activate
PYTHONNOUSERSITE=1 python python/scripts/dual_rtmpose_cameras.py \
  --device auto --max-frames 120 --save-every 30
```

Web UI:

```bash
PYTHONNOUSERSITE=1 python python/scripts/dual_rtmpose_web.py \
  --device auto --host 0.0.0.0 --port 8000
```

## 6. ベンチ前だけ sudo が必要な設定

ビルドには不要だが、性能測定前は最大性能モードにする。

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

温度、クロック、throttle 傾向は別端末で見る。

```bash
tegrastats --interval 1000
```

## 7. この環境での確認結果

2026-05-19 にこのワークスペースで確認した結果。

- `./python/scripts/setup_jetson_env.sh` は成功。
- Python venv は `python/.venv`。
- `cv2` は apt の `/usr/lib/python3/dist-packages/cv2...so` を参照。
- venv 内 `numpy` は `1.26.4`。
- `onnxruntime-gpu` は未導入。GPU/CUDA/TensorRT provider を Python 退避版で使う場合だけ Jetson AI Lab wheel を入れる。
- CMake configure/build は成功。
- `ctest --test-dir cpp/build --output-on-failure` は成功。
- `./cpp/build/main --help` は成功。
- `./cpp/build/main --probe` は CUDA device count と TensorRT runtime 作成まで成功。

この確認時点では `nvcc` は PATH に出ていなかったが、`/usr/local/cuda` は存在し、CMake の `find_package(CUDAToolkit)` は CUDA 12.6 を検出できた。`nvcc` を直接使う作業では次をシェルに追加する。

```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

`./cpp/build/main --probe` は GPU デバイスノードへアクセスするため、制限された sandbox 内では `CUDA error 999` で失敗することがある。通常のログインシェルでは成功する。
