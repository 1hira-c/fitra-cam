# Phase 8 — 姿勢シーケンスによるSubject Profile作成

## Summary

- Phase8の目的は、Tポーズ等の姿勢シーケンスから 個人別IK profile を作り、ライブ3D IKを起動直後から安定させること。
- カメラ校正は現行の ChArUco intrinsics + 床点PnP extrinsics を維持し、Phase8では extrinsics を更新しない。
- 最終UXは Webウィザード。1コマンドで起動し、姿勢収録、解析、品質確認、承認、live適用まで進める。

## Final Workflow

1. ウィザード起動:

   PYTHONNOUSERSITE=1 python python/scripts/subject_profile_wizard.py \
     --subject-id <id> \
     --calib calibrations/measure_session/cam_params.yaml \
     --cam0 /dev/v4l/by-path/... \
     --cam1 /dev/v4l/by-path/... \
     --det-engine models/yolox_tiny.fp16.engine \
     --pose-engine models/rtmpose_m.fp16.engine \
     --host 0.0.0.0 --port 8030

2. ブラウザで http://JETSON_IP:8030/ を開く。
3. Preflightでカメラ、校正YAML、engine、保存先、任意の身長入力を確認する。
4. standing, t_pose, elbow_flex, knee_flex を順に収録する。既定は各 3秒準備 + 5秒録画。
5. 各姿勢は個別clipとして保存し、失敗した姿勢だけ Retake できる。
6. 解析後、qualityを pass / warn / fail で表示し、reprojection overlayと姿勢別メトリクスを確認する。
7. pass は承認で latest_profile.yaml に昇格、warn は明示承認が必要、fail はretakeが既定。
8. live実行:

   ./cpp/build/main --enable-3d \
     --calib calibrations/measure_session/cam_params.yaml \
     --subject-id <id> \
     --cam0 ... --cam1 ... \
     --det-engine ... --pose-engine ...

## Key Interfaces

- 保存先:
    - calibrations/subjects/<subject_id>/sessions/<timestamp>/
    - raw/<pose>_cam0.mp4, raw/<pose>_cam1.mp4
    - pose_session.json, quality.json, subject_profile.yaml, overlays/
    - calibrations/subjects/<subject_id>/latest_profile.yaml
- Runtime profileはOpenCV FileStorage YAMLにする。
    - schema: fitra_subject_profile_v1
    - subject_id, created_at, source_session, subject_height_m
    - bone_lengths_m はCOCO-17 parent配列に対応する1x17 double
    - shoulder_width_m, hip_width_m, quality_status
- C++ CLI追加:
    - dump_keypoints_3d --pose-session pose_session.json --subject-profile-out subject_profile.yaml --quality-out quality.json
    - main --subject-id <id> --subjects-dir calibrations/subjects
    - main --subject-profile PATH は直接指定用で、--subject-id より優先する。
    - --subject-id / --subject-profile は --enable-3d なしならエラーにする。

## Implementation

- 新規 python/scripts/subject_profile_wizard.py と web/subject_profile/ を追加する。
    - FastAPIでMJPEG preview、状態API、Start/Retake/Analyze/Approveを提供する。
    - 解析はC++ dump_keypoints_3d をsubprocess実行し、UIは進捗と成果物リンクを表示する。
- C++に lift/subject_profile.{hpp,cpp} を追加し、profile YAMLの読込・検証を担当させる。
- IkSolver にprofile由来の骨長初期化を追加する。
    - profile読込時は locked=true で開始し、bone_calib_frames を待たない。
    - profile未指定時は現行どおり観測中央値または --subject-height-m を使う。
- 3D statsに profile_loaded, subject_id, quality_status を追加し、Web 3D statsにも表示する。

## Quality Gates

- pass: median reprojection <= 4px、median valid joints >= 13/17、必要bone coverage >= 80%、profile適用後 bone drift <= 5%、左右ペア骨長差 <= 15%。
- warn: pass未満だが median reprojection <= 6px、coverage >= 60%。
- fail: 上記未満、または必須姿勢clip欠落、カメラ片側の録画不足。
- latest_profile.yaml 更新は承認操作後だけ行う。解析生成物はfailでもsession配下に残す。

## Test Plan

- python -m py_compile python/scripts/subject_profile_wizard.py
- cmake --build cpp/build -j
- ./cpp/build/tools/dump_keypoints_3d --help
- ./cpp/build/main --help
- 既存録画または短い実カメラ収録で、4姿勢clip、pose_session.json、subject_profile.yaml、quality.json、overlayが生成されることを確認する。
- --subject-id live起動直後に /stats3d の ik_locked=true と profile_loaded=true を確認する。
