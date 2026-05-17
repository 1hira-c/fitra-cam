# メジャー測定 + 床メッシュによるマルチカメラキャリブレーション

## 概要

この手順は、ChArUcoボードがマルチカメラ全体で見えない場合でも、メジャーで測れる床パターンを使って外部パラメータを作るためのもの。

- 単眼 intrinsic は ChArUco で作る。
- ChArUco検出時は、VGAではなくランタイム解像度と同じアスペクト比の最大解像度を使う。
- 得られた高解像度 intrinsic は、ランタイム用の `640x480` などへスケールして同じYAMLに保存する。
- 外部パラメータは、床面 `z=0` のメッシュ交点を画像上で合わせ、PnPで推定する。
- カメラ高さ・カメラ間距離は姿勢を直接決める入力ではなく、推定結果の品質チェックに使う。

床が同一平面なら、個別点を大量にクリックするより、床メッシュとして編集する方が点IDの取り違えを減らせる。フローリングの目地や柄をそのまま使う場合も、対応する交点をメッシュとして扱える。

## 入力JSON

`calibrations/world_points.json` に床メッシュと任意の補助点を書く。単位はm。

```json
{
  "floor_grid": {
    "x_m": [0.0, 0.303, 0.606, 0.909],
    "y_m": [0.0, 0.303, 0.606, 0.909],
    "z_m": 0.0
  },
  "points": [
    {"id": "wall_p01", "x": 0.0, "y": 0.0, "z": 1.0}
  ],
  "camera_heights_m": {
    "cam0": 1.25,
    "cam1": 1.24
  },
  "baselines_m": {
    "cam0-cam1": 1.80
  }
}
```

`floor_grid.x_m` / `floor_grid.y_m` は、床上の原点から測った実測座標の配列。等間隔前提ではないため、フローリング幅にばらつきがある場合は測った値をそのまま並べる。

各交点は `grid_r00_c00` のようなIDへ自動展開される。`points` は床メッシュ外の補助点や、高さ付き点を追加したい場合に使う。

## 1. ChArUco intrinsic を作る

```bash
. python/.venv/bin/activate
PYTHONNOUSERSITE=1 python python/scripts/calibrate_intrinsics_charuco.py \
    --cam cam0=/dev/v4l/by-path/... \
    --cam cam1=/dev/v4l/by-path/... \
    --squares-x 5 --squares-y 7 \
    --square-len 0.04 --marker-len 0.03 \
    --samples 25 \
    --runtime-width 640 --runtime-height 480 \
    --web --host 0.0.0.0 --port 8020 \
    --out calibrations/intrinsics.yaml
```

ブラウザで `http://JETSON_IP:8020/` を開く。

- `Start`: ChArUcoサンプル収集を開始する。
- `Pause`: 収集を止める。
- `Reset`: サンプルを捨ててやり直す。
- `Solve`: `calibrations/intrinsics.yaml` を保存する。

キャリブレーション時は `--runtime-width/--runtime-height` と同じアスペクト比の最大解像度を自動選択する。既定では `640x480` と同じ4:3の最大MJPG解像度を `v4l2-ctl` から探し、失敗した場合は `1920x1440`、`1600x1200`、`1280x960`、`1024x768`、`800x600`、`640x480` を大きい順に試す。

明示したい場合は `--calib-width` / `--calib-height` を指定する。

出力YAMLには2種類のintrinsicが入る。

- `capture_intrinsics`: ChArUco検出に使った高解像度のK/dist。
- `intrinsics`: ランタイム解像度へスケール済みのK/dist。C++側はこのキーを読む。

## 2. 床メッシュで外部パラメータを作る

```bash
PYTHONNOUSERSITE=1 python python/scripts/measure_extrinsics_web.py \
    --intrinsics calibrations/intrinsics.yaml \
    --world-points calibrations/world_points.json \
    --cam cam0=/dev/v4l/by-path/... \
    --cam cam1=/dev/v4l/by-path/... \
    --output-dir calibrations/measure_session \
    --host 0.0.0.0 --port 8010
```

ブラウザで `http://JETSON_IP:8010/` を開く。`floor_grid` がある場合は `Grid` mode が主導線になる。

操作手順:

1. 左の点リストから、画像上で確実に対応できるメッシュ交点を選ぶ。
2. 各カメラ画像で同じ交点をクリックする。まずは4隅など、広く離れた4点以上を置く。
3. `Fit grid` を押す。観測済み交点からhomographyを作り、全メッシュ交点が自動配置される。
4. ずれた交点をドラッグで補正する。
5. 使わない交点は右クリック、または `Disable selected` で無効化する。
6. 高さ付き点やメッシュ外の点を使う場合は `Point` mode で個別にクリックする。
7. `Save` で `annotations.json` を保存する。
8. `Solve` で外部パラメータを推定する。

点の切り替えはキーボードでもできる。`n` / `j` / `↓` / `→` で次の点、`p` / `k` / `↑` / `←` で前の点、`Home` / `End` で先頭/末尾へ移動する。`g` は `Grid` mode、`o` は `Point` mode。

`capture_intrinsics` がある場合、PnPは高解像度Kで解く。最終出力の `intrinsics` はランタイム用にスケール済みのまま維持する。外部パラメータ `T_cw` は解像度非依存。

## 3. C++側の読込確認

```bash
./cpp/build/tools/check_calibration \
    --calib calibrations/measure_session/cam_params.yaml \
    --require-extrinsics
```

`cam0: 640x480 ... center_w=[...]` のように表示されれば、ランタイム用の `intrinsics` と `extrinsics` を読めている。

## 出力

- `calibrations/intrinsics.yaml`: ChArUco単眼結果。`capture_intrinsics` と `intrinsics` を含む。
- `calibrations/measure_session/annotations.json`: ブラウザで編集した画像座標。`grid_observations` には `Fit grid` のアンカー情報も残る。
- `calibrations/measure_session/cam_params.yaml`: ランタイムで使う最終キャリブレーション。
- `calibrations/measure_session/quality.json`: reprojection RMS/p95/max、使用点数、使用grid点数、推定高さ、測定高さとの差、baseline差。
- `calibrations/measure_session/overlays/*_reprojection.jpg`: 観測点、観測メッシュ、再投影メッシュの確認画像。

## 品質目安

初期の合格目安:

- 各カメラの reprojection p95 < 4 px
- 測定baseline差 < 5 cm
- 測定高さ差 < 5 cm
- overlay画像で観測メッシュと再投影メッシュが床目地に沿っている

床面だけで解く場合、床平面に対する姿勢は決まりやすいが、高さ方向の安定性は高さ付き点を含めた場合より弱い。可能なら壁・ドア枠・三脚などに高さ付き点を2〜4点追加するとよい。

## 注意

- 外部パラメータ取得後にカメラを動かしたら、外部キャリブレーションはやり直す。
- `opencv-python*` wheel が混ざるとJetsonのapt OpenCVと衝突しやすいため、実行例では `PYTHONNOUSERSITE=1` を付ける。
- 生成物は設置・デバイス固有なので `calibrations/` 配下に置き、Gitには含めない。
- フローリングの反復模様は点IDを間違えやすい。メッシュの4隅だけでなく、中間点も数点ドラッグ補正して再投影overlayを見る。
