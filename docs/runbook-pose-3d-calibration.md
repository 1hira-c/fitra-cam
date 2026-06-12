# Runbook: キャリブレーション 3 段フロー (calib-extrinsic → calib-subject → run)

(2026-06-12 / 前提: [design/pose-3d-calib-mode-separation.md](design/pose-3d-calib-mode-separation.md) +
[design/pose-3d-flow-daemon.md](design/pose-3d-flow-daemon.md) 実装済み。
プロセスは排他 RunMode で動き、段をまたいで渡るのは YAML ファイルのみ)

各段は**別プロセス起動**。前段の出力 YAML が次段の入力になる以外、何も引き継がれない。

```
(1) calib-extrinsic  →  calibrations/extrinsics.yaml   (camera extrinsics)
(2) calib-subject    →  calibrations/subjects/<ID>/latest_profile.yaml
(3) run              →  tracker 出力 (SlimeVR / VMT)
```

## 主経路: flow daemon (推奨)

3 段の起動連鎖は flow daemon に任せる。SSH での再起動は不要になり、ブラウザの
タブ 1 枚で excal solve → subject approve → run 着地まで進む。

```bash
./cpp/build/main --daemon --config calibrations/session.yaml
```

- **設定はすべて union YAML に集約する** (`--config` 以外の CLI override は
  モジュールへ転送されない)。1 つの YAML に 3 モードぶんの設定を同居させる:

```yaml
schema: fitra_main_config_v1
cameras:
  cam0: /dev/v4l/by-path/...index0
  cam1: /dev/v4l/by-path/...index0
  width: 1920
  height: 1200
inference:
  det_engine: models/yolox.engine
  pose_engine: models/rtmpose.engine
  keypoint_format: halpe26
three_d:
  enable_3d: true
  calib: calibrations/extrinsics.yaml     # excal の出力 = run/subject の入力
calibration:                              # calib-subject 段の設定
  calib_subject_id: hiroki
  calib_subject_height_m: 1.70
extrinsic_calib:                          # calib-extrinsic 段の設定
  out: calibrations/extrinsics.yaml       # three_d.calib と同一パスにする
  intrinsics: calibrations/intrinsics.yaml
  faces: "0,1,2"
  tag_size_m: 0.10
vmt:
  vmt_out: true                           # run 段の出力 (calib 段では daemon が打ち消す)
  hmd_listen_enabled: true                # pose relay 受信 (excal / run の alignment 系)
```

- **書いてはいけないキー**: `subject.subject_id` (初回 calib-subject が存在しない
  profile を load して落ちる — run への受け渡しは daemon が
  `calibration.calib_subject_id` から自動付与)、`calibration.calibrate` /
  `extrinsic_calib.enabled` / `extrinsic_calib.replay_dir` (モードフラグは daemon の
  専権。`--daemon` の validate が拒否する)。
- 初期モードは auto: extrinsics YAML がなければ calib-extrinsic、subject profile が
  なければ calib-subject、両方あれば run から始まる (`--daemon-initial MODE` で固定可)。
- ブラウザは `http://<jetson>:8000/` を開いたままでよい。モジュール入れ替えの数秒〜
  数十秒 (TRT ロード) は「restarting…」表示になり、復帰後に該当ページへ誘導/遷移する。
- run 中のビューワには「↺ extrinsic calib / ↺ subject calib」ボタンが出る
  (daemon 配下のみ)。クラッシュしたモジュールは run モードで自動復帰する
  (正常 exit を挟まず 3 連続で daemon ごと停止)。
- 終了は Ctrl-C (モジュール → daemon の順に clean exit)。

以降の各段の説明は **daemon なしの手動運用** (デバッグ / 単段だけ回したい場合)。
モードの中身・web UI・成果物はどちらの経路でも同一。

## 0. 前提

- intrinsics YAML (`--excal-intrinsics` か `--calib` で渡す per-camera 内参) が手元にあること。
- Windows 側 `vmt_hmd_pose_sender.exe --jetson <jetson-ip>` が VMT pose relay
  (HMD + controller) を UDP 39571 へ送っていること (calib-extrinsic と run の alignment 系で使用)。
- AprilTag 36h11 マーカー (face id とサイズは `--excal-faces` / `--excal-tag-size-m` と一致)
  をコントローラに固定。

## 1. calib-extrinsic (カメラ外部パラメータ)

```bash
./cpp/build/main --extrinsic-calib \
  --cam0 /dev/v4l/by-path/...index0 --cam1 /dev/v4l/by-path/...index0 \
  --width 1920 --height 1200 \
  --excal-intrinsics calibrations/intrinsics.yaml \
  --excal-out calibrations/extrinsics.yaml \
  --excal-faces "0,1,2" --excal-tag-size-m 0.10
```

- decode-only モード: TRT エンジン不要 (`--det-engine`/`--pose-engine` を渡す必要なし)。
  tracker 出力・3D・wizard は存在しない。
- `http://<jetson>:8000/extrinsic-calib` で収集状況 (gate 理由 / coverage) を確認し、
  コントローラを静止→移動で各カメラ×faceにサンプルを溜める。
- **solve 成功でプロセスは auto-exit する**。web の solve 応答とコンソールに次段の起動
  コマンド (`next_step`) が出る。Ctrl-C でも終了時に solve+write を試みる
  (solve 失敗は exit code 1)。

## 2. calib-subject (被写体プロファイル)

```bash
./cpp/build/main --calibrate --enable-3d \
  --cam0 ... --cam1 ... \
  --det-engine models/yolox.engine --pose-engine models/rtmpose.engine \
  --keypoint-format halpe26 \
  --calib calibrations/extrinsics.yaml \
  --calib-subject-id <ID> --calib-subject-height-m <H>
```

- (1) の extrinsics YAML が**必須** (2 カメラ限定)。VR publisher は構築されない
  (`--slimevr-out`/`--vmt-out` は validate で拒否)。
- `http://<jetson>:8000/subject-calib` の wizard で pose hold → 録画 → approve。
  `--calib-auto-approve` / `--calib-auto-exit` で無人化可。
- approve 後はプロファイル YAML が書かれ、run モード再起動のガイダンスがログに出る。

## 3. run (トラッカー出力)

```bash
./cpp/build/main --enable-3d \
  --cam0 ... --cam1 ... \
  --det-engine models/yolox.engine --pose-engine models/rtmpose.engine \
  --keypoint-format halpe26 \
  --calib calibrations/extrinsics.yaml \
  --subject-id <ID> \
  --vmt-out --hmd-listen-enabled   # または --slimevr-out
```

- run モードは calibration を一切構築しない。`/api/calib/*` `/api/excal/*` と
  `/subject-calib` `/extrinsic-calib` ページは 404。mode は `GET /api/state` で確認できる。

## オフライン replay (calib-extrinsic の実機レス再現)

### 記録 (実機・main 非依存の単体ツール)

```bash
./cpp/build/tools/excal_record \
  --camera /dev/v4l/by-path/...index0 --camera /dev/v4l/by-path/...index0 \
  --width 1920 --height 1200 --fps 30 \
  --seconds 30 --out outputs/excal_session_$(date +%Y%m%d_%H%M%S)
```

- MJPEG パススルー JPEG 連番 + frame↔pose ペア済み `frames.jsonl` を書く。
  実測 1920x1200 で約 9MB/秒 (2 台) — fixture にするなら短く録る。
- 記録中も VMT pose relay (controller) の受信が必要。

### 再生 (カメラ・SteamVR・web 不要、CI / solver 調整向け)

```bash
./cpp/build/main --excal-replay outputs/excal_session_<ts> \
  --excal-intrinsics calibrations/intrinsics.yaml \
  --excal-out /tmp/replay_extrinsics.yaml
```

- collect→solve→write まで無人実行。solve 失敗は exit code 1。
- **回帰確認**: 実録セッションに対する基準 extrinsics YAML を保持しておき、replay 出力と
  translation/rotation を許容差比較する (solver パラメータ調整の再現環境)。
- live↔replay の経路等価性は ctest `test_excal_replay` が常時固定している
  (合成フレームでの sample 列 bit-exact 比較)。実録データはサイズの都合でコミットしない —
  `outputs/` 配下に保持 (評価動画 `outputs/recorded_rtmpose/` と同じ扱い)。

## トラブルシュート

- `--extrinsic-calib` と `--calibrate` の同時指定、setup 系での `--slimevr-out`/`--vmt-out`
  は validate でエラー (排他モード)。`--daemon` とモードフラグの併用も同様
  (初段は `--daemon-initial` で指定する)。
- ビューワに「↺ … calib」切替ボタンが出ない → daemon 配下でない (手動起動)。
  `GET /api/state` の `managed` が `true` か確認。
- daemon 起動直後に `extrinsic_calib.out != three_d.calib` 警告 → union YAML の
  2 つのパスを揃える (揃っていないと excal の成果物を次段が読まない)。
- daemon が「3 consecutive module failures — giving up」で落ちる → モジュールの
  validate エラー等で run が起動できない。daemon と同じ argv に `--flow-managed` を
  足して手動起動するとエラーが直接見える。
- run モードで calib ページを開きたい → 該当モードで再起動する。導線はトップページが
  `/api/state` の mode を見て出し分ける。
- excal で `gate_reason` が `NO_POSE` のまま → pose relay 未着 (sender / ポート 39571 /
  `--excal-controller-role` を確認)。`MOVING` のまま → 静止待ちの motion gate
  (`--excal-lin-vel-max` / `--excal-ang-vel-max`)。
