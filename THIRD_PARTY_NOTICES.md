# Third-Party Notices

fitra-cam は MIT License で配布されますが、以下の第三者ソフトウェア / モデル /
ランタイムを利用します。それぞれの著作権表示・ライセンスは各配布元のものを尊重してください。

## ソース取り込み (FetchContent)

### Crow — BSD 3-Clause License
- URL: https://github.com/CrowCpp/Crow
- 取り込み箇所: `cpp/CMakeLists.txt` (`FetchContent_Declare Crow` @ `v1.2.1.2`)
- Copyright (c) 2014-2017 ipkn / 2020-2022 CrowCpp

### asio (standalone) — Boost Software License 1.0
- URL: https://github.com/chriskohlhoff/asio
- 取り込み箇所: `cpp/CMakeLists.txt` (`FetchContent_Declare asio` @ `asio-1-28-2`)
- Copyright (c) 2003-2023 Christopher M. Kohlhoff

## システム / ランタイム依存 (apt / NVIDIA JetPack)

ユーザーが Jetson に各自インストールするため、本リポジトリには再配布しません。

### OpenCV — Apache License 2.0
- apt パッケージ `libopencv-dev` / `python3-opencv`
- URL: https://github.com/opencv/opencv

### CUDA Toolkit / TensorRT / Jetson Multimedia API — NVIDIA Software License Agreement
- apt パッケージ `nvidia-jetpack`, `libnvinfer-dev`, `libnvinfer-plugin-dev`
- URL: https://docs.nvidia.com/cuda/eula/
- 再配布禁止。ユーザーは JetPack をデバイスに導入することでライセンス条件に同意。

## Python 旧実装 (`python/`)

| ライブラリ | ライセンス | 用途 |
|---|---|---|
| numpy | BSD 3-Clause | 数値演算 |
| fastapi | MIT | WebSocket サーバ (旧実装) |
| uvicorn | BSD 3-Clause | ASGI runner |
| onnxruntime-gpu | MIT | ONNX 推論 (Jetson AI Lab wheel) |
| rtmlib (任意) | Apache 2.0 | ONNX モデル取得用ヘルパ (実行時のみ) |

## 学習済みモデル (`outputs/onnx/`, `~/.cache/rtmlib/`)

リポジトリには含めず、ユーザーが各自配布元から取得します。各モデルの利用規約に
従ってください。

### YOLOX (yolox_tiny) — Apache License 2.0
- 配布元: Megvii (https://github.com/Megvii-BaseDetection/YOLOX)
- HumanArt fine-tuned ONNX: rtmlib 配布

### RTMPose (rtmpose-{s,m,l}_simcc-body7) — Apache License 2.0
- 配布元: OpenMMLab MMPose (https://github.com/open-mmlab/mmpose)
- body7 ONNX: rtmlib 配布

## Web フロントエンド (`web/`)

`web/dual_rtmpose/` / `web/subject_calibration/` / `web/calibration/` 配下の HTML /
JavaScript / CSS は本プロジェクト独自のため MIT License で配布します。`vendor/`
配下に第三者 JS を追加する場合は、当該ライブラリのライセンスを `vendor/` 直下に
同梱してください (現状 2026-05-21 時点では空 / プロジェクト独自コードのみ)。

## 互換性についての確認

すべて MIT との互換性に問題はありません:

- BSD-3 / BSL-1.0 / Apache-2.0 / MIT は MIT 化と両立 (Permissive 同士)
- NVIDIA EULA は「ライブラリを利用する側のコードのライセンスを縛らない」
  ランタイム依存のため、fitra-cam を MIT で配布することに制約を加えない
- 上記いずれも GPL/AGPL の伝染条項を含まない
