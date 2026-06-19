# core-pipeline: per-camera 露出/gain 制御 (手動固定 + 簡易ソフトAE)

(着手日 2026-06-19 / 関連: core-pipeline-3cam-60fps-smoothing.md の cam1 残課題, メモ project-3cam-rig-layout)

## 背景 / 動機

cam1 (USB3.0 Global Shutter, AR0234) が「ガタつき + ブラー + 妙に明るい」。切り分けで
**カメラ純正の自動露出が原因**と判明:

- **Windows でも・MJPEG/YUYV 両方でもガタつく** → ホスト/圧縮/Jetson USB 競合ではなく、
  カメラ本体の挙動。
- `v4l2-ctl --list-ctrls`: `auto_exposure = 3 (Aperture Priority)`、
  `exposure_time_absolute default=156` (= **15.6ms**)。**60fps の予算は 16.67ms** なので
  デフォルト露光が予算の 94%・余白ゼロ。少し暗いと AE が露光を 16.67ms 超へ伸ばす →
  (a) フレームが枠に収まらず**間隔が不均一 = ガタつき**、(b) 長露光で**動体ブラー**、
  (c) 明るく見える。全症状が「露光時間が長い」一点で説明できる。
- 我々のコードは V4L2 コントロールを一切設定しておらず全カメラ AE 任せだった (Windows 既定と同じ)。

**ブラーは見た目だけでなく RTMPose のキーポイントを滲ませ 2D 精度を落とす** (project-heel-sink-2d-limitation
にも効いている可能性)。モーキャップ用リグは本来全カメラ手動・短露光が正解。

### 自動露出補正 (AE bias) で解けるか → 否
ユーザー検討点。却下理由: (1) このカメラは AE bias コントロールを露出していない
(`brightness` はガンマ/黒レベル系で AE ターゲットを動かせない)、(2) 仮にあっても AE は
フレーム毎に浮動しジッタ/ブラーはゼロにならない、(3) カメラ毎に独立浮動して多視点の
露出/タイミング整合が崩れる。→ **露光時間を短く固定する以外に確実な手はない**。

## 設計の肝: 2軸のコスト差

- **exposure time**: 伸ばすと「ブラー」+「fps 枠超過ジッタ」を両方起こす高コスト軸 →
  **fps 安全上限で頭打ち**にする。
- **gain**: ノイズは増えるがブレ/タイミング劣化なしの低コスト軸 → **明るさ調整はこちら**。

カメラ純正 AE はこの区別をせず露光を伸ばすから駄目だった。我々の AE は exposure を短く保ち
gain で明るさを取る。

## 採用設計

### モード (per-camera, `V4l2Options::ExposureMode`)
- **Auto** (既定): カメラのコントロールを一切触らない (既存リグ無影響)。
- **Manual**: start() で `auto_exposure=Manual` + 固定 exposure + gain + `focus_auto=off`、以後固定。
- **Assist**: Manual の初期設定 + **常時・遅めのソフトAE** (下記)。

### Assist = 遅いデッドバンド・コントローラ (`FrameSource::decode_loop`)
- `ae_interval_` フレーム毎 (既定 30 ≈ 0.5s@60fps) にだけ判定 = **遅い追従** (ユーザー要望)。
- デコード済みフレームの**平均輝度** (`cv::mean` の BGR→luma) を測り、目標 `ae_target` との差が
  **デッドバンド (既定 ±10) 外の時だけ** 1 ノブを 1 ステップ動かす:
  - 暗い → gain↑ (上限まで) → なお暗ければ exposure↑ (**fps 安全上限 `ae_exp_cap_` まで**)
  - 明るい → gain↓ (下限まで) → なお明るければ exposure↓ (`ae_exp_min_` まで)
- `ae_exp_cap_` = フレーム周期 × 0.85 / 100us (60fps → ~141 = 14.1ms)。gain range は
  `VIDIOC_QUERYCTRL(V4L2_CID_GAIN)` で取得。
- 露光は普段短いまま gain だけが動く → ブレ・ジッタを増やさず明るさのみ追従。**普段は無動作**。

### 制御 I/O (`V4l2Capture`)
- start() の format 設定後・STREAMON 前に `apply_exposure_controls()`: gain range 取得 +
  (Auto 以外なら) `V4L2_CID_EXPOSURE_AUTO=MANUAL` / `EXPOSURE_ABSOLUTE` / `GAIN` /
  `FOCUS_AUTO=0`。
- `set_exposure_us100()` / `set_gain()` を public 化し、FrameSource の decode スレッドから
  ライブ適用 (`VIDIOC_S_CTRL` は worker の DQBUF/QBUF と独立でスレッドセーフ)。

### config (`MainOptions` / YAML)
- `cam{N}_exposure_mode`: `""`/`auto` | `manual` | `assist`
- `cam{N}_exposure`: 100us 単位 (manual 値 / assist 初期値; 0=触らない)
- `cam{N}_gain`: `V4L2_CID_GAIN` (<0=触らない)
- `cam{N}_ae_target`: assist 目標 luma (既定 110)
- 既定は全部 Auto = 触らない。CLI は省略 (リグ設定なので run YAML で指定)。

## 検討した代替 (没)

- **カメラ純正 AE + bias**: 上記「否」。
- **全カメラ常時ソフトAE 強制**: 既定を Manual/Assist にするとレンズ/照明前提が違う既存リグを
  壊す。→ 既定 Auto・opt-in に。
- **GPU でフレーム輝度を測る**: 純 all-GPU device 経路は BGR scratch を持たないので、現状
  assist はその経路では非対応 (warn して無効)。推奨の cam1=YUYV は scratch 常時あり問題なし。

## 制約 / 残課題

- Assist は BGR フレームが要る (`cv::mean`)。**純 all-GPU nvjpeg device 経路 (scratch 空) では
  非対応** (warn-once)。cam1 は YUYV 推奨なので実運用は問題なし。nvjpeg 経路で assist が要るなら
  GPU 輝度リダクション (将来)。
- 手動/Assist は照明が安定している前提 (リグは制御下)。
- 適正 exposure/gain 値は照明依存 → `v4l2-ctl -c auto_exposure=1 -c exposure_time_absolute=N
  -c gain=M` の live テストで「ブレ止まる×暗すぎない」点を出して config 初期値に入れる。

## 検証

- ビルド: `cmake --build cpp/build -j`。
- 実機 cam1=YUYV + `cam1_exposure_mode: assist` (or `manual`) で:
  - ブラーが消え、`recv` のペーシング・ジッタが縮むこと。
  - assist が暗所→明所で gain を遅く追従し、exposure が fps 上限を超えないこと。
  - `cam{N}_exposure_mode` 未指定の既存カメラが回帰しないこと (Auto=無改変)。
