# Phase 8 Subject Profile 手順書

この手順は、`standing` / `t_pose` / `elbow_flex` / `knee_flex` の姿勢シーケンスから個人別 IK profile を作り、ライブ 3D IK を起動直後から lock するためのもの。カメラ校正は既存の ChArUco intrinsics + 床点 PnP extrinsics を使い、Phase 8 では extrinsics を更新しない。

## 1. 前提確認

先にカメラ校正 YAML と TensorRT engine があることを確認する。

```bash
./cpp/build/tools/check_calibration \
    --calib calibrations/measure_session/cam_params.yaml \
    --require-extrinsics
```

Phase 8 の追加ツールがビルド済みか確認する。

```bash
cmake --build cpp/build -j
python -m py_compile python/scripts/subject_profile_wizard.py
./cpp/build/tools/dump_keypoints_3d --help
./cpp/build/main --help
```

## 2. Subject Profile Wizard を起動

`subject_id` は被験者ごとに固定する。既定では `calibrations/subjects/<subject_id>/` に保存される。

```bash
PYTHONNOUSERSITE=1 python python/scripts/subject_profile_wizard.py \
    --subject-id subject01 \
    --calib calibrations/measure_session/cam_params.yaml \
    --cam0 /dev/v4l/by-path/... \
    --cam1 /dev/v4l/by-path/... \
    --det-engine models/yolox_tiny.fp16.engine \
    --pose-engine models/rtmpose_m.fp16.engine \
    --host 0.0.0.0 --port 8030
```

身長を prior として使う場合だけ `--subject-height-m 1.70` のように追加する。未指定なら姿勢観測から骨長を作る。

ブラウザで `http://JETSON_IP:8030/` を開く。

## 3. 姿勢を収録

画面右側の `Sequence` で、各姿勢の `Record` を押す。既定は各姿勢 `3秒準備 + 5秒録画`。

収録する姿勢:

- `standing`: 正面を向いて自然に直立する。
- `t_pose`: 肩と腕が水平になるように両腕を横へ伸ばす。
- `elbow_flex`: 肩・肘・手首が見える状態で肘を曲げる。
- `knee_flex`: 膝と足首が見える状態で軽く膝を曲げる。

姿勢が崩れた、片方のカメラで隠れた、フレーム数が明らかに少ない場合は、その姿勢だけ `Retake` する。

## 4. 解析と承認

4姿勢すべてが `recorded` になったら `Analyze` を押す。解析が終わると `Quality` に `pass` / `warn` / `fail` とメトリクスが出る。

判定の目安:

- `pass`: そのまま `Approve` してよい。
- `warn`: overlay とメトリクスを見て、明らかな姿勢崩れがなければ承認してよい。
- `fail`: 原則として該当姿勢を撮り直す。

`Approve` すると、次のファイルが live の既定 profile になる。

```text
calibrations/subjects/<subject_id>/latest_profile.yaml
```

各セッションの成果物は以下に残る。

```text
calibrations/subjects/<subject_id>/sessions/<timestamp>/
  pose_session.json
  subject_profile.yaml
  quality.json
  summary.json
  joints3d.jsonl
  raw/<pose>_cam0.mp4
  raw/<pose>_cam1.mp4
  overlays/<pose>_cam0_reproj.mp4
  overlays/<pose>_cam1_reproj.mp4
```

## 5. Live で profile を使う

`--subject-id` を指定すると、`calibrations/subjects/<subject_id>/latest_profile.yaml` を読み込む。

```bash
./cpp/build/main \
    --cam0 /dev/v4l/by-path/... \
    --cam1 /dev/v4l/by-path/... \
    --det-engine models/yolox_tiny.fp16.engine \
    --pose-engine models/rtmpose_m.fp16.engine \
    --enable-3d \
    --calib calibrations/measure_session/cam_params.yaml \
    --subject-id subject01 \
    --host 0.0.0.0 --port 8000
```

直接ファイルを指定したい場合は `--subject-profile PATH` を使う。`--subject-profile` は `--subject-id` より優先される。

ブラウザまたは `/stats3d` で以下を確認する。

- `ik_locked: true`
- `profile_loaded: true`
- `subject_id: subject01`
- `quality_status: pass` または `warn`

## 6. 解析だけを再実行する

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

採用する場合は、内容を確認してから `latest_profile.yaml` へコピーする。

```bash
cp calibrations/subjects/subject01/sessions/<timestamp>/subject_profile.yaml \
   calibrations/subjects/subject01/latest_profile.yaml
```

## 7. トラブルシュート

- Wizard 起動直後に camera error が出る: `/dev/v4l/by-path/...` の指定、USB接続、他プロセスがカメラを掴んでいないかを確認する。
- `Analyze` が失敗する: `dump_keypoints_3d`, `--calib`, `--det-engine`, `--pose-engine` のパスを確認する。
- `fail` になる: まず overlay を見て、隠れ・左右取り違え・床点校正のズレを確認する。姿勢が崩れている場合は該当姿勢だけ Retake する。
- Live で `profile_loaded=false`: `--enable-3d` と `--subject-id` / `--subject-profile` の指定、`latest_profile.yaml` の存在を確認する。
- Live で `ik_locked=false`: profile が読み込まれていないか、`--no-3d-ik` を指定している可能性がある。

