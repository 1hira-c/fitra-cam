# 番外編積みタスク — YOLOX 検出器のモデルサイズ × 量子化精度の再評価

> **状態**: 未着手。
>
> **着手条件**: 「横向き / 部分遮蔽で YOLOX-tiny が人物を見落とす / bbox が崩れる」が再現する代表シーンを `outputs/recorded_rtmpose/` 配下に 1〜2 個追加でき、Python ORT 参照で同様の失敗が観測できる段階に到達したとき。

## 背景

現運用の検出器は mmdeploy 由来の YOLOX-tiny humanart (`cpp/src/infer/yolox.hpp:9` および `python/scripts/pose_pipeline.py:82`) で、416×416 入力・BGR raw・letterbox pad=114 で固定されている。Phase 5 単体ベンチでは det 推論時間が FP16 で 1ms/frame 程度、かつ既定で `--det-frequency 10` のため検出器側の速度予算には大きく余裕がある。

一方で運用中、**横向き (90° に近い側面姿勢) と、椅子・机による下半身の部分遮蔽** で YOLOX-tiny が人物を見落とす / bbox が下流 RTMPose の crop に不適なほど崩れるケースが目に付くようになっている。bbox 不良は RTMPose の simcc 出力をそのまま劣化させるため、骨格全体の見え方が悪くなる。

検出器側にはまだ速度予算があるので、より大きい YOLOX-S / YOLOX-M (humanart pretrained) を候補に、FP16 そのままと INT8 (PTQ) を比較し、運用既定モデル + 精度モードの推奨を出す。

## 目的 / 対象外

| | 範囲 |
|---|---|
| 目的 | (1) `cpp/tools/build_engines.cpp:109` で "no calibrator wired yet" と明記されている INT8 calibrator を `IInt8EntropyCalibrator2` ベースで配線する。(2) YOLOX-tiny / S / M (humanart) の TRT engine を FP16 と INT8 で全 6 組ビルドし、横向き・部分遮蔽を含む eval video で比較する。(3) 「失敗フレーム数」「下流 RTMPose drift」「推論時間」を主軸に評価レポートを出し、運用既定モデルと精度モードの推奨値を提示する。 |
| 対象外 | YOLOX 以外の検出器 (YOLOv8, RTMDet, DETR 系)。検出器の再学習 / fine-tuning。RTMPose 側の差し替え (別 backlog [pose-backend-abstraction](backlog-pose-backend-abstraction.md))。`--det-frequency` の再チューニング。INT8 calibrator の RTMPose 側適用 (別 backlog `docs/research/rtmpose-int8-eval-plan.md` の方で扱う)。multi-person 既定への切替。 |

## 評価マトリクス

| モデル | FP16 | INT8 |
|---|---|---|
| YOLOX-tiny (現運用) | baseline | candidate |
| YOLOX-S | candidate | candidate |
| YOLOX-M | candidate | candidate |

YOLOX-L / X は det-frequency=10 でも per-detection 時間が支配的になるリスクがあるため対象外。S / M で十分な改善が得られない場合は別検出器系統 (別 backlog) に進む判断材料とする。

## 評価データセット

既存:

- `outputs/recorded_rtmpose/20260515_064342/raw_cam{0,1}.mp4` — 既存の正面寄り通常シーン。

追加 (本 backlog 着手時に録画):

- 横向き 30 秒 × 2 カメラ。被験者がカメラに対して 60°〜90° 横を向く動作を含み、距離は 1.5〜3 m。
- 部分遮蔽 30 秒 × 2 カメラ。椅子・机・パーティションで下半身または胴体下部が隠れる状態。

ground truth の方針:

- 真の bbox 正解は持たないため、**モデル間の per-frame disagreement** を主指標にする。
- 補助 ground truth として、Python ORT で YOLOX-M を低 NMS 閾値で回した bbox を「より厳しめの基準」として使い、視覚確認で代表 50 フレームに対し正誤を手付けする。
- 既存 eval video に対しては、Python ORT YOLOX-tiny の出力を回帰用 baseline として固定する (現運用の挙動が崩れていないことの確認用)。

## 評価指標

- 失敗フレーム数: 「人物が画面内にいるが検出 0 件」のフレーム数 (per-scene, per-model)。
- bbox 不安定: 隣接フレーム間の bbox 中心移動と面積変動の標準偏差 (per-track)。
- 下流 RTMPose drift: 同一フレーム・同一 RTMPose 重みで、YOLOX-tiny vs 候補モデルの crop から得た keypoint の L2 差 (px)。
- 推論時間: ms/frame (TRT, FP16/INT8 別)。Jetson Orin Nano Super, `nvpmodel -m 0 && jetson_clocks` 状態。
- INT8 vs FP16 同モデル差: keypoint drift max を Phase 1 の correctness 基準 (L2 < 1 px) と整合させた目線で評価する。

## INT8 calibrator 配線

`cpp/tools/build_engines.cpp` の `--int8` フラグは現状立てるだけで calibrator が無い。本 backlog でここを実体化する。

実装方針:

- 新規 `cpp/src/infer/int8_calibrator.{hpp,cpp}` を追加し、`nvinfer1::IInt8EntropyCalibrator2` を実装する。
- calibration data は eval video から N=500 フレーム等間隔サンプリング (`outputs/recorded_rtmpose/` 既存 + 新規録画)。前処理は YOLOX 本体と同じ letterbox + BGR raw を `Yolox::Options{input_size, ...}` から取り出して共有する。
- calibration cache は `outputs/tensorrt_engines/<model>_int8_calib.cache` に保存し、ONNX / 入力サイズ / TRT バージョンが変わると invalidate する (既存 engine cache の方式に揃える)。
- `build_engines --int8 --preset yolox --onnx ...` で `<model>_int8.engine` が出力される。FP16 engine と並べて使えるようにする。

`Yolox` 側変更:

- `Options::input_size` は既存 (416 既定) のままで対応可能。YOLOX-S/M は通常 640×640 入力で公開されるため、`build_engines --preset yolox` のときは ONNX の入力 dim を読んで `Options::input_size` を engine ごとに自動で 416 か 640 に揃える。

## 実装方針

段階を分け、順に commit する想定:

1. INT8 calibrator 実体化 (`int8_calibrator.{hpp,cpp}` + `build_engines.cpp` 配線、YOLOX-tiny で INT8 engine が出来る所まで)。
2. YOLOX-S / YOLOX-M humanart ONNX 取得 (mmdeploy 公開モデルまたは MMPose hub から ONNX export)。`outputs/onnx_models/` に保存し、`build_engines` で FP16/INT8 6 組をビルド。
3. 評価ツール: `cpp/tools/det_eval.cpp` (新規) または既存 Python 評価スクリプトの拡張。frame-by-frame で各モデルの bbox を出し、上記の評価指標を CSV / JSON に書く。
4. 評価レポートを `docs/research/yolox-detector-eval-result.md` として残し、運用既定モデル + 精度モードの推奨を結論セクションに書く。
5. 結論が「既定差し替え」になった場合は `main.cpp` 既定 (`--det-engine`) と `python/scripts/pose_pipeline.py` の参照モデルパスを別 commit で更新。

## 確認項目

1. `cmake --build cpp/build -j` が calibrator 追加後も通る。
2. `./cpp/build/build_engines --preset yolox --int8 --onnx <yolox_tiny_humanart>.onnx --output models/yolox_tiny.int8.engine` で engine が生成される。
3. INT8 engine の出力が FP16 engine と「同一フレームに対して bbox IoU > 0.95、検出件数差 < 5%」に収まる。外れた場合は calibration data の代表性を疑う。
4. YOLOX-S / YOLOX-M の TRT engine が動き、推論時間が公開ベンチマーク (Orin Nano クラス) から大きく外れない。
5. 追加した横向き / 部分遮蔽シーンで、YOLOX-S / YOLOX-M が YOLOX-tiny より失敗フレーム数が少ない (定量数値をレポートに記録)。
6. 推論時間が aggregate 予算 (Phase 4 完了条件の ≥90 fps 2cam、Phase 6+ の ≥150 fps) を破らない。det-frequency=10 で割って per-loop 影響を確認する。
7. Python ORT YOLOX-tiny baseline に対する既存 eval video の回帰が無い (bbox IoU > 0.99 / 失敗フレーム差 0)。

## リスク / メモ

- humanart pretrained の S/M ONNX が mmdeploy 公開エクスポートに無い場合、MMPose model zoo から ONNX を別途エクスポートする手間が増える。配布物の有無を着手前に確認する。
- INT8 calibration data は eval video のみだとシーン依存 (室内・照明) で過剰最適化になる可能性が高い。最低限、別日・別照明の追加録画でデルタを取って calibration data に混ぜる。
- YOLOX-tiny の弱点が「モデル容量」ではなく「humanart データセットの cover 範囲」だった場合、S/M に上げても改善幅は限定的になる。その時は本 backlog の結論として「データ起因と判定、別検出器系統 (YOLOv8 / RTMDet) を別 backlog で評価」に着地させる。
- INT8 calibrator は本 backlog では YOLOX 用にしか実装しないが、`int8_calibrator.{hpp,cpp}` の signature は RTMPose 側 ([`docs/research/rtmpose-int8-eval-plan.md`](research/rtmpose-int8-eval-plan.md)) からも再利用できる形を意識する。汎用化は別タスクだが、リネームコストが大きくならないだけの抽象度に留める。
- 評価レポートは「数値表 + 推奨」だけで足り、本 backlog 内に分厚い実装メモは持たせない。実装中に出た知見は `docs/research/yolox-detector-eval-result.md` 側に集める。

## 関連

- 既存 backlog [pose-backend-abstraction](backlog-pose-backend-abstraction.md) — 将来 RTMPose 以外の pose model を載せる際に、本 backlog で導入する `int8_calibrator` を pose backend 側でも共有する設計の余地がある。
- 既存研究ノート `docs/research/rtmpose-int8-eval-plan.md` — RTMPose の INT8 PTQ。本 backlog で配線する INT8 calibrator はそちらにも転用可能。
- 既存 phase doc `docs/cpp-migration-plan.md` Phase 4 (FP16/INT8/pinned-memory, aggregate ≥ 90 fps) — 速度予算の参照元。
