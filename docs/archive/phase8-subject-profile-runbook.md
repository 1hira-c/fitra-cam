# Phase 8 Subject Profile 手順書 (C++統合版)

この手順は、`standing` / `t_pose` / `elbow_flex` / `knee_flex` の姿勢シーケンスから個人別 IK profile を作り、ライブ 3D IK を起動直後から lock するためのもの。カメラ校正は既存の ChArUco intrinsics + 床点 PnP extrinsics を使い、Phase 8 では extrinsics を更新しない。

ウィザードは `cpp/build/main` に統合されており、ライブ推論と並行で動く。同じ orchestrator を **WebUI からも CLI からも** 起動できる。

## 1. 前提確認

校正 YAML と TensorRT engine、新規ツール群があることを確認する。

```bash
./cpp/build/tools/check_calibration \
    --calib calibrations/measure_session/cam_params.yaml \
    --require-extrinsics

cmake --build cpp/build -j
./cpp/build/main --help                       # --calibrate* セクションがある
./cpp/build/tools/dump_keypoints_3d --help    # --pose-session, --subject-profile-out, --quality-out
```

### Phase 9 — `--keypoint-format` 注意

- 既定は `coco17`。Halpe26 を使うなら `--keypoint-format=halpe26` を起動コマンド全てに付け、対応する 26 キーポイント版 RTMPose engine を `--pose-engine` で指定する。
- Subject profile schema は `fitra_subject_profile_v1` (COCO17) / `v2` (Halpe26) に分かれる。**自動マイグレーションは行わない**。例えば v1 (COCO17) で作成したプロファイルを `--keypoint-format=halpe26` で読込もうとすると `load_subject_profile()` が「subject profile schema fitra_subject_profile_v1 does not match active --keypoint-format (fitra_subject_profile_v2); re-run the calibration wizard」で起動失敗する。再キャリブが必要。
- ウィザード経由で作った場合は `web/subject_calibration/` の API は無改修。Halpe26 でも angle キー (`l_elbow`/`r_knee` 等) は不変。

## 2. ウィザードの起動 (WebUI)

通常の live と同じコマンドで `main` を起動する。

```bash
./cpp/build/main --enable-3d \
    --calib calibrations/measure_session/cam_params.yaml \
    --cam0 /dev/v4l/by-path/... \
    --cam1 /dev/v4l/by-path/... \
    --det-engine models/yolox_tiny.fp16.engine \
    --pose-engine models/rtmpose_m.fp16.engine \
    --host 0.0.0.0 --port 8000
```

ブラウザで `http://JETSON_IP:8000/subject-calib` を開く。Live UI (`/`) からは右上の「subject calib」リンクで遷移できる。

## 3. Preflight → 姿勢収録 (自動)

1. `Subject ID` と身長 (cm) を入力し、`Preflight` を押す。
2. `Start (auto)` で開始。`standing → t_pose → elbow_flex → knee_flex` の順に**完全自動**で進行する。
   - 目標ポーズを 1.5 秒安定保持で録画開始 (`hold` バー)
   - 規定フレーム数 (既定 75f ≈ 5s @ 15fps) で停止し、自動的に次のポーズへ
   - 失敗時は当該ポーズの `Retake` ボタンで取り直し可能
3. ポーズ判定は 3D 関節角 (左右の肘 flex、肩 abduction、膝 flex、torso tilt) と `bone_drift_pct` の組合せ。
   `IkSolver::apply_subject_height_model()` が AIST/HQL 寸法比で IK を即時 lock するため、初回フレームから view 非依存の関節角が取れる。

ポーズの取り方:
- `standing`: 正面を向き自然に直立。
- `t_pose`: 肩と腕が水平になるよう両腕を横へ伸ばす。
- `elbow_flex`: 肩・肘・手首が見える状態で両肘を約 90° 曲げる (腕は前方〜横へ)。
- `knee_flex`: 膝と足首が見える状態で両膝を約 90° 曲げる (軽い squat)。

## 4. 解析と承認

4 姿勢の録画完了後、自動的に `dump_keypoints_3d` を subprocess で起動して `subject_profile.yaml` と `quality.json` を生成する。

判定:
- `pass`: そのまま `Approve & apply live` で承認。
- `warn`: `force` チェックを入れて承認。
- `fail`: 承認不可。問題となった姿勢を Retake する。

承認すると `latest_profile.yaml` を atomic rename で配置し、**ライブ IK をホットリロード**する (main 再起動不要)。

```text
calibrations/subjects/<subject_id>/
├── latest_profile.yaml                # 承認時に更新
└── sessions/<YYYYmmdd_HHMMSS>/
    ├── pose_session.json
    ├── subject_profile.yaml
    ├── quality.json
    ├── summary.json
    ├── joints3d.jsonl
    ├── raw/{pose}_cam{0,1}.mp4
    └── overlays/
```

## 5. CLI (headless) で実行

WebUI なしで全自動承認まで走らせる:

```bash
./cpp/build/main --enable-3d --calibrate \
    --calib-subject-id subject01 --calib-subject-height-m 1.72 \
    --calib calibrations/measure_session/cam_params.yaml \
    --cam0 /dev/v4l/by-path/... \
    --cam1 /dev/v4l/by-path/... \
    --det-engine models/yolox_tiny.fp16.engine \
    --pose-engine models/rtmpose_m.fp16.engine \
    --no-web --calib-auto-approve --calib-auto-exit
```

進捗は stderr の `[calib] ...` 行に出る (`AWAIT <pose>`, `RECORDING <pose>`, `[analyze] ...`, `quality status=...`)。pass 時は自動承認 → exit 0。warn / fail は WebUI で手動承認するため `--no-web` を外して起動する。

## 6. Live で profile を使う

```bash
./cpp/build/main --enable-3d \
    --cam0 /dev/v4l/by-path/... \
    --cam1 /dev/v4l/by-path/... \
    --det-engine models/yolox_tiny.fp16.engine \
    --pose-engine models/rtmpose_m.fp16.engine \
    --calib calibrations/measure_session/cam_params.yaml \
    --subject-id subject01 \
    --host 0.0.0.0 --port 8000
```

`/stats3d` で `ik_locked=true`, `profile_loaded=true`, `subject_id=subject01`, `quality_status=pass|warn` を確認する。直接指定したい場合は `--subject-profile PATH` (こちらが優先)。

## 7. 解析だけを再実行する

録画済みセッションから profile と quality を作り直す場合:

```bash
./cpp/build/tools/dump_keypoints_3d \
    --pose-session calibrations/subjects/subject01/sessions/<timestamp>/pose_session.json \
    --calib calibrations/measure_session/cam_params.yaml \
    --det-engine models/yolox_tiny.fp16.engine \
    --pose-engine models/rtmpose_m.fp16.engine \
    --out calibrations/subjects/subject01/sessions/<timestamp>/joints3d.jsonl \
    --summary calibrations/subjects/subject01/sessions/<timestamp>/summary.json \
    --overlay-dir calibrations/subjects/subject01/sessions/<timestamp>/overlays \
    --subject-profile-out calibrations/subjects/subject01/sessions/<timestamp>/subject_profile.yaml \
    --quality-out calibrations/subjects/subject01/sessions/<timestamp>/quality.json
```

採用する場合は内容を確認してから `latest_profile.yaml` へコピー (またはウィザードで Retake → Approve)。

## 8. トラブルシュート

- 起動直後に camera error: `/dev/v4l/by-path/...` の指定、USB 接続、他プロセスがカメラを掴んでいないかを確認。
- `hold` バーが進まない: WebUI の `failing` 表示 (`left_elbow_flex`, `right_shoulder_abduction` 等) を見て、対応する関節を目標角度帯に合わせる。`bone_drift_pct` が常に閾値超なら `cpp/src/lift/pose_recognizer.cpp` の `max_bone_drift_pct` を調整する。
- `analyze` が失敗する: stderr の `[calib] [analyze] ...` ログを確認。`dump_keypoints_3d`, `--calib`, `--det-engine`, `--pose-engine` のパスを再確認。
- `fail`: まず該当 session の `overlays/` を見て、隠れ・左右取り違え・床点校正のズレを確認。姿勢が崩れている場合は該当姿勢だけ Retake する。
- Live で `profile_loaded=false`: `--enable-3d` と `--subject-id` / `--subject-profile` の指定、`latest_profile.yaml` の存在を確認。
- Live で `ik_locked=false`: profile が読み込まれていないか、`--no-3d-ik` を指定している可能性がある。
