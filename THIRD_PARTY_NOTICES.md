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

## RoMa v2

空室の静止画を使う校正実験では、RoMa v2.0.1 と Meta の DINOv3 ViT-L/16 を使った。
`scripts/markerless_roma/` に含めるのは実験スクリプト、Jetson 用の差分、ライセンス文書。
実行者は RoMa 公式の複合チェックポイントと固定版 DINOv3 のソースを手元に用意する。
DINOv3 の特徴抽出器の重みもこのチェックポイントから読み込む。C++ の人体校正へ渡す背景 YAML の内容は、
カメラ内参、相対姿勢、画像上の対応点である。

| 対象 | 適用される条件と出典 |
|---|---|
| RoMa v2 作者のコード | MIT、Copyright (c) 2025 Johan Edstedt。`jetson-buffer.patch` の上流部分と、局所相関を行単位に分割した `tiled_correlation.py` に原著作権表示を添付する。[同梱の MIT 全文](scripts/markerless_roma/licenses/RoMaV2-MIT.txt)・[上流 v2.0.1 の説明](https://github.com/Parskatt/RoMaV2/blob/v2.0.1/README.md#license) |
| DINOv3 のコードと重み | Meta の DINOv3 License。特徴抽出器に加え、RoMa の照合器が読み込む `src/romav2/vit/` にも同契約の表示がある。[検証版の契約全文](scripts/markerless_roma/licenses/DINOv3-LICENSE.md)・[Meta の現行原文](https://github.com/facebookresearch/dinov3/blob/main/LICENSE.md)・[照合器内の表示](https://github.com/Parskatt/RoMaV2/blob/v2.0.1/src/romav2/vit/__init__.py#L1-L4) |
| 複合チェックポイント `romav2.0.1.pt` | v2.0.1 の公式配布物をローカルで読む。非 DINO 部分の学習済みパラメータについては、2026-11-27 時点で再配布条件が上流で未回答。[公式リリース](https://github.com/Parskatt/RoMaV2/releases/tag/v2.0.1)・[配布元の Issue #52](https://github.com/Parskatt/RoMaV2/issues/52) |
| torchvision 0.20.1 | `torchvision-optional-ops.patch` 内の上流コードは BSD 3-Clause。VGG19-BN の定義を実験環境で利用する。[同梱の著作権表示・条件・免責](scripts/markerless_roma/licenses/torchvision-BSD-3-Clause.txt)・[上流のライセンス](https://github.com/pytorch/vision/blob/v0.20.1/LICENSE) |

DINOv3 License は利用時にも適用される。§1(b)(i) は DINO Materials とその派生物を第三者に提供する場合に、
同じ契約での提供と契約の写しの添付を求める。§1(b)(ii) は関連する研究成果を投稿する際に DINOv3 の利用を明記するよう定める。
プライバシー、輸出管理、用途などの条件も同節を参照。[DINOv3 License §1](https://github.com/facebookresearch/dinov3/blob/adc254450203739c8149213a7a69d8d905b4fcfa/LICENSE.md#1-license-rights-and-redistribution)

取得・変換したモデルの置き場所は `outputs/` またはリポジトリ外とし、現行の Git・配布イメージには
同梱しない。将来 C++ 版で重み入り ONNX や TensorRT エンジンを配る場合も、DINO 部分と残りの重みの
双方について、配布する版の条件を確認する。

## Web フロントエンド (`web/`)

`web/dual_rtmpose/` / `web/subject_calibration/` / `web/calibration/` 配下の HTML /
JavaScript / CSS は本プロジェクト独自のため MIT License で配布します。`vendor/`
配下に第三者 JS を追加する場合は、当該ライブラリのライセンスを `vendor/` 直下に
同梱してください (現状 2026-05-21 時点では空 / プロジェクト独自コードのみ)。

## 本体のライセンスの適用範囲

ルートの MIT License は fitra-cam 独自のソースに適用する。第三者コードを含む差分には原著作権表示と
該当ライセンスを添付する。外部の SDK、モデルの重み、それらを組み込んだ配布物は各配布元の条件に従う。
