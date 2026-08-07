# 番外編積みタスク — `main` ランタイム YAML 設定

> **状態**: 未着手。
>
> **着手条件**: `cpp/build/main` の起動コマンドが長くなり、カメラ・エンジン・3D・被験者キャリブレーションの定番組み合わせを毎回 CLI で指定する運用がつらくなったとき。

## 背景

`cpp/src/main.cpp` は現在、手書きパーサで `--cam0..2`, `--det-engine`, `--pose-engine`, `--enable-3d`, `--calib`, `--subject-*`, `--calibrate*` などを直接ローカル変数へ代入している。Phase 8/9 以降でフラグ数が増え、ライブ起動・ヘッドレスキャリブレーション・プロファイル付き起動のコマンドが長くなっている。

既存 CLI は一時的な実験や上書きには便利なので残しつつ、定番設定を YAML に寄せて次のように起動できるようにする:

```bash
./cpp/build/main --config configs/live_2cam.yaml
./cpp/build/main --config configs/live_2cam.yaml --port 8010 --no-web
```

## 目的 / 対象外

| | 範囲 |
|---|---|
| 目的 | `cpp/build/main` に `--config PATH` を追加し、主要な実行時オプションを YAML から読めるようにする。既存 CLI は維持し、CLI 指定値は YAML より優先する。設定読込後の検証は現行 `main.cpp` の起動前チェックと同じ基準に揃える。 |
| 対象外 | Python 旧 CLI への `--config` 追加。`cpp/tools/*` への共通設定導入。ホットリロード。環境別設定の取り込み / 継承。シークレット管理。既存の calibration YAML / subject profile YAML の形式変更。 |

## 利用者向け CLI

新規フラグ:

```text
--config PATH    実行時設定 YAML。通常の CLI フラグより先に読み込むため、
                 明示した CLI フラグは YAML の値を上書きする。
```

優先順位:

1. コード上の既定値
2. `--config PATH` の値
3. `--config` 以外の CLI フラグ

`--help` には `--config` と優先順位を明記する。`--probe` は設定ファイルなしで従来通り動く。`--config` と `--probe` が同時に指定された場合は、設定読込だけを行わず probe を優先して終了してよい。

## YAML 形式

設定は「CLI フラグ名から `--` を外して snake_case 化したキー」を基本にする。ネストは読みやすさのためだけに使い、内部では既存オプションに展開する。

```yaml
schema: fitra_main_config_v1

cameras:
  cam0: /dev/v4l/by-path/usb-cam0-video-index0
  cam1: /dev/v4l/by-path/usb-cam1-video-index0
  width: 640
  height: 480
  fps: 30

inference:
  det_engine: models/yolox_tiny.fp16.engine
  pose_engine: models/rtmpose_m.fp16.engine
  det_frequency: 10
  det_score: 0.5
  keypoint_format: halpe26
  multi_person: false
  bench_fake_bbox: false

web:
  host: 0.0.0.0
  port: 8000
  static: web/dual_rtmpose
  no_web: false

three_d:
  enable_3d: true
  calib: calibrations/measure_session/cam_params.yaml
  kp_conf_thresh: 0.3
  max_reproj_px: 6.0
  sync_window_ms: 15.0
  bone_calib_frames: 150
  no_3d_kalman: false
  no_3d_ik: false
  no_3d_postprocess: false

subject:
  subjects_dir: calibrations/subjects
  subject_id: subject01
  subject_profile: ""
  subject_height_m: 0.0

calibration:
  calibrate: false
  calib_subject_id: subject01
  calib_subject_height_m: 1.72
  calib_frames_per_cam: 75
  calib_hold_sec: 1.5
  calib_auto_approve: false
  calib_auto_exit: false
  calib_static_dir: ""
  calib_dump_tool: ""

logging:
  log_every_s: 2.0

# コントローラ固定 AprilTag による多カメラ extrinsic キャリブ。
# `enabled: true` のとき `--calibrate` と排他、かつ `intrinsics`
# (または `three_d.calib`) が必須 (validate_options で同じチェック)。
# 詳細は docs/design/pose-3d-controller-marker-extrinsic.md。
extrinsic_calib:
  enabled: false
  intrinsics: ""            # intrinsics-only YAML; 空なら three_d.calib を流用
  out: calibrations/extrinsics.yaml
  faces: "0,1,2"            # AprilTag 36h11 face ID (カンマ区切り)
  tag_size_m: 0.10
  lin_vel_max: 0.03         # モーションゲート m/s
  ang_vel_max: 8.0          # モーションゲート deg/s
  burst_min: 5              # 1 サンプルに平均するフレーム数
  min_samples: 8            # (cam,face) グループあたりの最小サンプル数
  controller_role: right    # VMT pose relay から採用する controller: left|right
  controller_port: 39572    # deprecated: 旧コントローラ pose UDP ポート
  controller_bind: 0.0.0.0
  controller_stale_ms: 200.0
```

> 注: 実装には他に `vmt:` セクション (VMT publisher + `hmd_listen_*`) もあるが、
> この例には未掲載。キー一覧は `cpp/src/config/main_config.cpp` の各 `load_*` を正とする。

ルール:

- 未知のトップレベルセクションや未知キーは、タイプミス検出のため `config: unknown key <path>` を出して失敗する。
- `schema` は必須で、値は `fitra_main_config_v1` に固定する。
- 任意のパス項目で空文字列を指定した場合は、現行 CLI の既定値と同じく「未指定」として扱う。
- 相対パスは現行 CLI 引数と同じく、config ファイルの場所ではなくプロセスの作業ディレクトリ基準で解決する。
- 真偽値は既存の肯定 / 否定フラグに対応させる。たとえば `three_d.no_3d_ik: true` は `--no-3d-ik` と同じ効果を持つ。`three_d.no_3d_postprocess: true` は `--no-3d-postprocess` と同じで、Kalman / IK / floor-contact を一括で bypass する。

## 実装方針

`cpp/CMakeLists.txt` で `yaml-cpp` を `FetchContent` 追加し、`fitra_cam_main` または小さな設定読込ライブラリだけにリンクする。OpenCV FileStorage 形式を運用者に強制せず、普通の YAML を読めるようにするため。手書きパーサも避ける。

`cpp/src/main.cpp` は、設定の重ね合わせが明確になる最小限の範囲で整理する:

- 現在 `main()` のローカル変数として持っている項目と既定値を、そのまま `MainOptions` に移す。
- 1 回目の `argv` 走査では、通常フラグを解釈せずに `--help`, `--probe`, `--config PATH` だけを拾う。
- `--config` があれば、YAML を読んで `MainOptions` に反映する。
- 2 回目の走査で既存 CLI フラグを `MainOptions` に反映する。未知引数と引数不足の挙動は現行と同じにする。
- 既存の起動前チェックを `validate_options(const MainOptions&)` に移す。
- オプション確定後の runtime 構築処理は挙動を変えない。

追加候補ファイル:

- `cpp/src/config/main_config.hpp`
- `cpp/src/config/main_config.cpp`

`main_config` が YAML 解析、型変換、未知キー検出、エラーメッセージを持つ。`main.cpp` 側には `load_main_config(path, options)` 呼び出し以上の YAML node 操作を置かない。

## 利用例

`configs/*.yaml` は `.gitignore` 済 (端末固有のデバイスパス・エンジンパスを含むため)。リポジトリには `configs/live_2cam.yaml.example` / `configs/live_2cam_3d.yaml.example` が手本として置いてあるので、コピーしてから編集する:

```bash
cp configs/live_2cam_3d.yaml.example configs/live_2cam_3d.yaml
# vim configs/live_2cam_3d.yaml で device / engine パスを差し替え
```

2カメラ 3D ライブ起動:

```bash
./cpp/build/main --config configs/live_2cam_3d.yaml
```

同じ設定で一時的に port だけ上書き:

```bash
./cpp/build/main --config configs/live_2cam_3d.yaml --port 8010
```

ヘッドレスの被験者キャリブレーション:

```bash
./cpp/build/main --config configs/calibrate_subject01.yaml --no-web --calib-auto-exit
```

## 確認項目

1. `cmake --build cpp/build -j`
2. `./cpp/build/main --help` に `--config` が出る。
3. `cam0`, `det_engine`, `pose_engine` だけを含む最小設定が、同等の CLI 指定と同じ挙動で起動する。
4. CLI 上書きが効く: 設定側 `port: 8000` と `--port 8010` の組み合わせで 8010 に待ち受ける。
5. 未知キーは stderr にキーパスを出して即失敗する。
6. 型不一致は即失敗する。例: `web.port: "abc"`。
7. `--probe` と `--help` を含む既存の CLI だけのコマンドが従来通り動く。
8. 設定を重ね合わせた後に必須値が足りない場合、既存と同じ help 表示 / 失敗経路に入る。

## リスク / メモ

- `yaml-cpp` はヘッダオンリーではないため、Jetson 上の初回 CMake configure / build では取得とコンパイルの時間が増える。
- `main.cpp` からオプションを切り出す作業は機械的だが、触るフィールド数が多い。挙動維持の小さいコミットに分け、既定値を慎重に確認する。
- config ファイル相対のパス解決は意図的に先送りする。将来、持ち運べる config ディレクトリが必要になったら、v1 の挙動を黙って変えずに v2 スキーマで明示的な `base_dir` ルールを足す。
