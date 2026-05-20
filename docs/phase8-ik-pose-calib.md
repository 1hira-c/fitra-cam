# Phase 8 — 姿勢シーケンスによるSubject Profile作成 (C++統合版)

## Summary

- Phase 8 の目的は Tポーズ等の姿勢シーケンスから個人別 IK profile を作り、ライブ 3D IK を起動直後から安定させること。
- カメラ校正は ChArUco intrinsics + 床点 PnP extrinsics を維持し、Phase 8 では extrinsics を更新しない。
- Phase 8 ウィザードは **`cpp/build/main` に統合**。ライブ推論を継続したまま、WebUI からも CLI からも同じ orchestrator を起動できる。
- ポーズ認識は **3D 関節角ベースで完全自動**。preflight で身長を入力すると `IkSolver::apply_subject_height_model()` (AIST/HQL 比) が IK を即時 lock し、初回フレームから view 非依存の関節角でポーズを判定する。

## Final Workflow

### 1. WebUI 経由 (推奨)

通常通りライブ起動:

```
./cpp/build/main --enable-3d \
  --calib calibrations/measure_session/cam_params.yaml \
  --cam0 /dev/v4l/by-path/... \
  --cam1 /dev/v4l/by-path/... \
  --det-engine models/yolox_tiny.fp16.engine \
  --pose-engine models/rtmpose_m.fp16.engine
```

ブラウザで `http://JETSON_IP:8000/subject-calib` を開く。

1. **Preflight**: Subject ID と身長 (cm) を入力 → Preflight → Start。
2. **Capture**: 各ポーズ (standing → t_pose → elbow_flex → knee_flex) を取る。
   - 目標ポーズを 1.5 秒安定保持で自動的に録画開始。
   - 規定フレーム数 (既定 75f ≈ 5s @ 15fps) 到達で自動停止、次のポーズへ自動進行。
   - 表示: hold 進捗、cam0/cam1 録画進捗、bone_drift、3D 角度。
3. **Review**: 自動解析後 (`dump_keypoints_3d` を subprocess 実行)、`quality.json` の pass/warn/fail を表示。
4. **Approve**: 「Approve & apply live」で `latest_profile.yaml` を atomic rename → live IK をホットリロード。**main の再起動は不要**。

### 2. CLI (headless) 経由

```
./cpp/build/main --enable-3d --calibrate \
  --calib-subject-id subject01 --calib-subject-height-m 1.72 \
  --calib calibrations/measure_session/cam_params.yaml \
  --cam0 /dev/v4l/by-path/... \
  --cam1 /dev/v4l/by-path/... \
  --det-engine models/yolox_tiny.fp16.engine \
  --pose-engine models/rtmpose_m.fp16.engine \
  --no-web --calib-auto-approve --calib-auto-exit
```

進捗は stderr に `[calib] AWAIT t_pose`、`[calib] RECORDING t_pose`、`[calib] [analyze] ...` の行で出る。pass 時は自動承認 → main が exit 0 で終了。warn/fail の場合は WebUI で手動承認できる (`--no-web` を外す)。

### 3. live で profile を使う

```
./cpp/build/main --enable-3d \
  --calib calibrations/measure_session/cam_params.yaml \
  --subject-id subject01 \
  --cam0 ... --cam1 ... --det-engine ... --pose-engine ...
```

`--subject-id` は `calibrations/subjects/<ID>/latest_profile.yaml` を読み込む。直接指定する場合は `--subject-profile PATH` (`--subject-id` より優先)。

## Key Interfaces

### 保存先

```
calibrations/subjects/<subject_id>/
├── latest_profile.yaml                # 承認時に atomic rename で更新
└── sessions/<YYYYmmdd_HHMMSS>/
    ├── pose_session.json
    ├── raw/{standing,t_pose,elbow_flex,knee_flex}_cam{0,1}.mp4
    ├── subject_profile.yaml           # dump_keypoints_3d が生成
    ├── quality.json                   # dump_keypoints_3d が生成
    ├── joints3d.jsonl
    ├── summary.json
    └── overlays/
```

### Runtime profile

OpenCV FileStorage YAML (schema `fitra_subject_profile_v1`):
- `subject_id`, `created_at`, `source_session`, `subject_height_m`
- `bone_lengths_m` は COCO-17 parent 配列に対応する `1 x 17 double`
- `shoulder_width_m`, `hip_width_m`, `quality_status`

### C++ CLI (Phase 8)

- `./cpp/build/main --calibrate --calib-subject-id <id> --calib-subject-height-m <m>` …
  - `--calib-frames-per-cam N` (既定 75)、`--calib-hold-sec F` (既定 1.5)
  - `--calib-auto-approve`, `--calib-auto-exit`
  - `--calib-static-dir DIR`, `--calib-dump-tool PATH` (override)
  - `--subjects-dir DIR` (既定 `calibrations/subjects`)
  - `--calibrate` は `--enable-3d` 必須。`--calib-subject-id` と `--calib-subject-height-m` は `--calibrate` 必須。
- `./cpp/build/tools/dump_keypoints_3d --pose-session pose_session.json --subject-profile-out PROF.yaml --quality-out Q.json` — 既存仕様、不変。
- `./cpp/build/main --subject-id <id>` / `--subject-profile PATH` — 既存仕様、不変。

### WebUI

- `GET /subject-calib` — フロントエンド (`web/subject_calibration/`)
- `GET /api/calib/state` — JSON で session 状態 (state, target_pose, hold_progress, angles, poses[], quality_status, analyze_log_tail, latest_profile)
- `POST /api/calib/preflight` body=`{subject_id, subject_height_m, required_hold_sec?, recording_frames_per_cam?}`
- `POST /api/calib/start`
- `POST /api/calib/retake` body=`{pose}`
- `POST /api/calib/cancel`
- `POST /api/calib/approve` body=`{force}` — warn は `force=true` 必須、fail は不可
- 通常の `/`, `/ws`, `/ws3d`, `/stats`, `/stats3d` はそのまま並行で稼働する (キャリブ中もライブ推論は止まらない)

## Implementation

- `cpp/src/lift/ik.hpp/cpp`
  - `void reload_from_profile(const SubjectProfile&)` 追加、内部 `std::mutex` で `update()` と保護
- `cpp/src/lift/pose_recognizer.hpp/cpp` (新規)
  - 3D 関節角ベース分類器。kStanding / kTPose / kElbowFlex / kKneeFlex
  - 角度: 左右の肘 flex、肩 abduction、膝 flex、torso tilt
  - `bone_drift_pct > 10%` は無効フレーム扱い
  - `consecutive_ok >= required` (≈ fps × 1.5s) で hold 確立
- `cpp/src/pipeline/multi_pipeline.hpp/cpp`
  - `set_frame_tap` / `set_skeleton3d_tap` 追加。tap は IDLE 時に即 return、RECORDING 時のみ clone してバッファ追加
  - `IkSolver& ik()` accessor を expose
- `cpp/src/pipeline/calibration_session.hpp/cpp` (新規)
  - 状態機械: idle → ready → await_hold → recording → (次のポーズ ↑) → finalizing → analyzing → review → approving → approved | canceled | failed
  - フレーム保持 → 4 ポーズ完了後 `cv::VideoWriter` で MP4 書き出し (実測 fps) + `pose_session.json` 生成
  - `popen("dump_keypoints_3d ... --pose-session ... 2>&1")` で解析を起動、stdout/stderr 行単位で tail に保存
  - approve: `subject_profile.yaml` を読んで `latest_profile.yaml` の tmp に書き出し → atomic rename → `on_approved_(profile)` で `IkSolver::reload_from_profile()` を呼ぶ
- `cpp/src/web/crow_server.hpp/cpp`
  - `set_calibration_session(session*, defaults)` 追加
  - `/subject-calib`, `/subject-calib/<path>`, `/api/calib/*` ルート登録
- `cpp/src/main.cpp`
  - `--calibrate*` フラグ群、`guess_subject_calib_static_dir()` / `guess_dump_tool_path()` ヘルパ
  - boot 時に CalibrationSession を作り、driver にタップ登録、CrowServer に attach、`--calibrate` なら preflight + start を即実行
- `web/subject_calibration/{index.html,app.js,styles.css}` (新規) — preflight フォーム、Capture 進捗、Review、Log
- `web/dual_rtmpose/index.html` — ヘッダに `/subject-calib` リンク

## Quality Gates

`dump_keypoints_3d` の既存閾値を使用:
- pass: median reproj ≤ 4px、median valid joints ≥ 13/17、必要 bone coverage ≥ 80%、profile 適用後 bone drift ≤ 5%、左右ペア骨長差 ≤ 15%
- warn: pass 未満だが median reproj ≤ 6px、coverage ≥ 60%
- fail: 上記未満、必須ポーズ clip 欠落、片側カメラ録画不足

`latest_profile.yaml` 更新は承認操作後だけ行う。解析生成物は fail でも session 配下に残る。

## Test Plan

- `cmake --build cpp/build -j`
- `./cpp/build/main --help` に `--calibrate*` セクションが出る
- `./cpp/build/tools/dump_keypoints_3d --help` は不変 (subprocess で従来通り起動)
- CLI smoke: `--calibrate --no-web --calib-auto-approve --calib-auto-exit` で 4 ポーズ収録 → 解析 → pass → `latest_profile.yaml` 生成 → exit 0
- WebUI smoke: ブラウザで `/subject-calib` を開き、preflight → Start → 4 ポーズ自動 → Approve、`/stats3d` で `profile_loaded=true`, `subject_id`, `ik_locked=true` を確認
- 既存ライブ機能: キャリブ中も `/ws`, `/ws3d` が継続して 30Hz publish

## Notes / Risks

- 身長プライアの bone 長と被験者の実体形差が大きい場合は `bone_drift_pct` が高止まりして開始できないことがある。初期テンプレートは `max_bone_drift_pct = 10%`。閾値は `cpp/src/lift/pose_recognizer.cpp` の `kPoseTemplates` で微調整。
- 録画フレームは memory にバッファ (4 ポーズ × 2 cam × 75f × 640×480×3 ≈ 540MB)。書き出し完了後解放されるので Jetson Orin Nano Super (8GB) で許容。
- ホットリロード中の `IkSolver::update()` 競合は内部 `std::mutex` で保護。
- `popen` 呼び出しの引数は `shell_quote()` で single-quote エスケープ。subject_id は ASCII 英数 + `_-` のみ通すサニタイズ済み。
- 旧 Python wizard (`python/scripts/subject_profile_wizard.py`) と `web/subject_profile/` は本 Phase で削除済み。必要なら git history から復元できる。
