# YOLOX 検出器再評価 — 結果 (2026-05-24)

Issue [#9](https://github.com/1hira-c/fitra-cam/issues/9) / backlog [`backlog-yolox-detector-upgrade.md`](../backlog-yolox-detector-upgrade.md) の対応で実施した、yolox-tiny humanart からの差し替え評価結果。

## 結論

| 採用 | engine | 入力 | precision | median latency (raw_cam0 1 frame, 300 iter × 3 round, Orin Nano Super) |
|---|---|---|---|---|
| 旧 (運用継続可) | `yolox_tiny.fp16.engine` | 416 | FP16 | 19.1 ms |
| **新 (採用)** | **`yolox_s.fp16.engine`** | **640** | **FP16** | **23.9 ms** |
| 不採用 (重) | `yolox_m.fp16.engine` | 640 | FP16 | 27.9 ms |
| 不採用 (重) | `yolox_x.fp16.engine` | 640 | FP16 | 52.3 ms |

予算上限は `yolox_tiny.fp32.engine` = 22.3 ms (median)。yolox-s FP16 は予算 +7% で「同程度」に収まる最大モデル。`--det-frequency 10` での amortize 後は per-loop 影響 2.4 ms 程度で、Phase 4 の aggregate ≥ 90 fps 予算には支障しない。

## 評価マトリクスから減らした選択肢

backlog では FP16 × INT8 の 6 組を全部測る計画だったが、本タスクでは以下の理由で縮退:

- **目視評価は本番運用に持ち込む方針に変更**: 横向き / 部分遮蔽の代表シーンを録り直す前に、まず latency 予算で候補を絞り、実運用で確認する方が早い。
- **INT8 calibrator 実装は本タスクでは見送り**: FP16 単体で予算内に収まり、INT8 にする動機 (速度) が薄いため。INT8 calibrator の実装は RTMPose 側 INT8 計画 ([`rtmpose-int8-eval-plan.md`](rtmpose-int8-eval-plan.md)) と合流させる別タスクで扱う。`cpp/tools/build_engines.cpp` の `--int8` フラグは引き続き calibrator 無し (no-op に近い) のまま。
- **YOLOX-S を選び、L / X は除外**: M の時点で予算超過 (+25%) のため、X (+134%) は明確に対象外。L は mmdeploy 配布版に humanart pretrained ONNX が無いので非対象。

## ベンチ方法

```bash
# 入力フレーム抽出 (raw_cam0.mp4 の 1 frame 目)
python3 -c "import cv2; cap=cv2.VideoCapture('outputs/recorded_rtmpose/20260515_064342/raw_cam0.mp4'); ok,f=cap.read(); cv2.imwrite('outputs/recorded_rtmpose/20260515_064342/raw_cam0_frame0.jpg', f)"

# 全候補 engine をまとめて latency 比較
./cpp/build/tools/det_bench \
  --frame outputs/recorded_rtmpose/20260515_064342/raw_cam0_frame0.jpg \
  --iters 300 --warmup 50 \
  --engine outputs/tensorrt_engines/yolox_tiny.fp32.engine \
  --engine outputs/tensorrt_engines/yolox_tiny.fp16.engine \
  --engine outputs/tensorrt_engines/yolox_s.fp16.engine   \
  --engine outputs/tensorrt_engines/yolox_m.fp16.engine   \
  --engine outputs/tensorrt_engines/yolox_x.fp16.engine
```

- `det_bench` は `Yolox::infer()` 全体 (host 前処理 letterbox + HWC->CHW + H2D + enqueue + sync + D2H) を 1 iter として `std::chrono::steady_clock` で計測する。pure GPU enqueue 時間ではなく **pipeline で実際に支払う wall-clock** を表す。
- `--frame` 省略時は 1280×720 の合成 BGR を流す。recorded `raw_cam0.mp4` の 1 frame 目を使うとレイアウト依存の post-processing コストも反映される。
- 3 round 取って round 間の中央値を採用 (初回は GPU clock が unstable で 25% 程度ぶれることがある)。

## ONNX 入手 + TopK surgery

mmdeploy 配布の yolox humanart end2end ONNX:

| slug | size | input | URL |
|---|---|---|---|
| `yolox_tiny_8xb8-300e_humanart-6f3252f9` | 20 MB | 416 | rtmlib hub (既に同梱) |
| `yolox_s_8xb8-300e_humanart-3ef259a7` | 36 MB | 640 | `https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/<slug>.zip` |
| `yolox_m_8xb8-300e_humanart-c2c7a14a` | 101 MB | 640 | (同上) |
| `yolox_x_8xb8-300e_humanart-a39d44ed` | 396 MB | 640 | (同上) |

zip の中身は `20230928/yolox_onnx/<slug>/end2end.onnx` 。`outputs/onnx/<slug>.onnx` に flat に展開する。

S/M/X の end2end ONNX には NMS pre-filter として TopK K=5000 が埋まっており、TRT 10.3 の `ITopKLayer` 上限 (K ≤ 3840) に抵触してビルドが失敗する:

```
[error] [trt] IBuilder::buildSerializedNetwork: Error Code 4: API Usage Error
        (ITopKLayer TopK_485: K exceeds the maximum value allowed (3840).)
```

K を 3000 に書き換えた `.topk3000.onnx` を作って入力にする (tiny は 416 入力で K が小さく、surgery 不要):

```python
import onnx, numpy as np
from onnx import numpy_helper

NEW_K = 3000
for slug in ["yolox_s_8xb8-300e_humanart-3ef259a7",
             "yolox_m_8xb8-300e_humanart-c2c7a14a",
             "yolox_x_8xb8-300e_humanart-a39d44ed"]:
    m = onnx.load(f"outputs/onnx/{slug}.onnx")
    inits = {ini.name: ini for ini in m.graph.initializer}
    for tk in [n for n in m.graph.node if n.op_type == "TopK"]:
        if len(tk.input) >= 2 and tk.input[1] in inits:
            ini = inits[tk.input[1]]
            if int(numpy_helper.to_array(ini).flatten()[0]) > NEW_K:
                new_arr = np.array([NEW_K], dtype=numpy_helper.to_array(ini).dtype)
                for idx, x in enumerate(m.graph.initializer):
                    if x.name == ini.name:
                        m.graph.initializer[idx].CopyFrom(numpy_helper.from_array(new_arr, name=ini.name))
                        break
    onnx.save(m, f"outputs/onnx/{slug}.topk3000.onnx")
```

実用上は K=3000 でも検出件数余裕 (人物 1 シーンで NMS 前 1〜数十件)。

## engine ビルドコマンド

```bash
./cpp/build/tools/build_engines --preset yolox \
  --onnx   outputs/onnx/yolox_s_8xb8-300e_humanart-3ef259a7.topk3000.onnx \
  --output outputs/tensorrt_engines/yolox_s.fp16.engine \
  --fp16 --workspace-mb 2048

# M / X も同じ要領。X は --workspace-mb 4096 推奨。
```

`build-engines-yolox` Docker 起動例は `docker-compose.yml` に新既定として登録済 (旧 tiny は `build-engines-yolox-tiny` で並存)。

## 実装側の変更点

- `cpp/src/infer/yolox.{hpp,cpp}`
  - `Options::input_size` を engine binding の dims から auto-overwrite (`416 → 640` への切替を呼び出し側コード変更無しで吸収)
  - ヘッダ Spec コメントを tiny / S / M / X 共通の I/O に拡張
- `cpp/tools/det_bench.cpp` 新規 + `cpp/tools/CMakeLists.txt` に `fitra_det_bench` ターゲット追加
- `docker-compose.yml`
  - runtime command の `--det-engine` を `yolox_s.fp16.engine` に変更
  - `build-engines-yolox` を yolox-s 入力に切替、tiny は `build-engines-yolox-tiny` で並存

## 残課題 / 将来タスク

- **目視評価の本格化**: 横向き / 部分遮蔽の代表シーンを録って失敗フレーム数で再評価する作業は backlog のままで残す。本タスクでは latency 予算と engine ビルド経路の整備のみ。
- **YOLOX-S と RTMPose-M halpe26 の組み合わせ動作確認**: aggregate スループット (Phase 4 ≥ 90 fps) は別途長時間ベンチで再検証する。
- **INT8 PTQ**: 2026-05-25 に再着手 (下記 §2026-05-25 追記) → **YOLOX end2end ONNX 経路では TRT 10.3 calibration が assertion で死ぬため見送り**。calibrator 自体は salvage 済み (RTMPose INT8 計画で再利用)。
- **`yolox_tiny.fp32.engine` の扱い**: 本タスクで latency 予算の参照として残置。Phase 4 完了時に FP16 専一にするか別途判断。

## 2026-05-25 追記: YOLOX INT8 PTQ は TRT 10.3 制約で見送り

「INT8 化で yolox-m を予算内に押し込めるか」を確かめるため calibrator を実装したが、mmdeploy 配布の YOLOX end2end ONNX では TRT 10.3 の INT8 calibration が動かないことが判明。**yolox-s FP16 既定** はそのまま維持。

### 実装 (salvage 済み)

- `cpp/src/infer/int8_calibrator.{hpp,cpp}` — `IInt8EntropyCalibrator2` 実装。raw `(N, C, H, W) float32` blob を mmap-less 読みで `getBatch` に流し、`writeCalibrationCache` でキャッシュ書き出し。lifetime は `build_engine()` のスコープで `unique_ptr` 保持。
- `cpp/src/infer/trt_builder.{hpp,cpp}` — `BuildOptions` に `int8_blob_path` / `int8_cache_path` / `int8_batch_size` / `int8_input_name` を追加。`config->setInt8Calibrator()` は TRT 10.3 で deprecated 警告が出るが PTQ の唯一の入口なので `#pragma diagnostic` で局所抑制。
- `cpp/tools/build_engines.cpp` — `--int8-blobs PATH` / `--int8-cache PATH` / `--int8-batch N` / `--int8-input NAME` を追加。help の "no calibrator wired yet" は撤去。
- `python/scripts/dump_yolox_calibration_blobs.py` — `_yolox_letterbox` (BGR raw / 114 pad) を C++ 側と bit-for-bit 同一実装で再現し、`raw_cam{0,1}.mp4` から均等サンプリングで `(N, 3, S, S) float32` を吐く。416 / 640 両対応。

ビルドは clean (`cmake --build cpp/build -j` 通過)、calibrator ログも期待通り (`INT8 calibrator wired: input='input' per_image_bytes=2076672 batch=1 N=200`) まで出る。

### ブロッカー

`build_engines --preset yolox --int8 --int8-blobs ... --onnx <yolox_tiny humanart>.onnx ...` で:

```
[trt] [slot.cpp::decode::44] Error Code 2: Internal Error
      (Assertion index < nbSlots failed. invalid encoded reference to a slot)
[trt] [calibrator.cpp::calibrateEngine::1236] Error Code 2: Internal Error
      (Assertion context->executeV2(bindings.data()) failed.)
```

`--int8 --fp16` mixed precision でも同じ。`Yolox::infer()` 経路 (FP16/FP32 通常 build) では問題なく動いている。

### 原因

mmdeploy YOLOX end2end ONNX には NMS subgraph が baked-in:

- `NonMaxSuppression` × 1, `TopK` × 2, `Where` / `Less` / `Gather` / `Shape` / `Reshape` × 10+
- 出力が `dets (1, -1, 5)` / `labels (1, -1)` で **動的 N**

TRT 10.3 の INT8 calibration は activation range 収集のため QDQ-aware FP32 forward を走らせる段で、この dynamic-output 系列の slot 解決を失敗させる。INT8 calibration 経路特有の TRT 10.3 内部制約 (or バグ)。

### 検討した回避策

- **C1 (per-layer precision constraints)** — `kTOPK` / `kNMS` / `kSELECT` 層に `setPrecision(kFLOAT)` + `kPREFER_PRECISION_CONSTRAINTS` で escape — 30 分実験対象として候補に挙げたが、エラー位置が calibration の executeV2 (INT8 量子化前の FP32 forward 段) なので per-layer 制約では救えない可能性が高く、コストパフォーマンスを取り **未着手で見送り**。
- **C2 (NMS を ONNX から剥がす + C++ NMS)** — backbone+head だけ INT8 calibrate する案。1-2 日工数 + `Yolox::infer()` 数値再検証 (Phase 1 correctness 再回し) を要するため、**INT8 効果が読めない時点では割に合わず却下**。

### 着地: salvage して RTMPose INT8 計画へ繰り越し

- yolox-s FP16 既定はそのまま。本タスクで det 周辺の変更は **無し**。
- 上記 calibrator + dumper のコードは [`rtmpose-int8-eval-plan.md`](rtmpose-int8-eval-plan.md) の Step 3-4 が想定していた成果物そのもの。RTMPose ONNX (NMS 等の dynamic-output ノード無し、純粋な Conv + SimCC) では同じブロッカーは出ない見込みなので、そのまま流用する。
- 視覚評価 (横向き / 遮蔽の失敗フレーム数定量化) は本タスクで着手しないまま [`backlog-yolox-detector-upgrade.md`](../backlog-yolox-detector-upgrade.md) に残置。FP16-only での 3-engine 視覚評価は別タスク。

## 関連

- backlog: [`backlog-yolox-detector-upgrade.md`](../backlog-yolox-detector-upgrade.md)
- 関連 backlog: [`backlog-pose-backend-abstraction.md`](../backlog-pose-backend-abstraction.md)
- INT8 計画: [`rtmpose-int8-eval-plan.md`](rtmpose-int8-eval-plan.md)
- migration plan 行: [`cpp-migration-plan.md`](../cpp-migration-plan.md) の「番外編: YOLOX 検出器のモデルサイズ × 量子化精度の再評価」
