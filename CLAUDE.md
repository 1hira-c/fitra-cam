# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Communication language

**Write user-facing replies, git commit subjects (after the prefix), commit bodies, and PR descriptions in Japanese.** Conventional Commits prefixes (`feat(<track>):`, `fix:`, `docs:`, `refactor:`, `chore:`, ...) stay in English. The `Co-Authored-By` trailer also stays in English.

- Final answers to the user, progress reports, summaries: Japanese.
- Commit subject line: `prefix(scope): <Japanese summary>` — prefix in English, summary in Japanese.
- Commit body: Japanese. Technical tokens (CMake, ctest, function names, file paths) stay in English, mixed inline.
- Code comments / identifiers / exception strings: follow the existing convention of the file (this repo is English-majority).
- `docs/` content: match the existing language of the file (track docs / phase plans are Japanese-majority, build notes are English-majority).

New commits use track scopes: `feat(vr-output): ...`, `fix(pose-3d): ...`, `docs(core-pipeline): ...`. `git log --oneline` up to `cpp-phase15.5` shows the older phase-scoped pattern (`feat(phase8): ...`, `fix(phase9): ...`) — that's historical, don't extend it.

## Branching and commits

**Work is organized by domain track, not by phase number** (phase numbering was retired
2026-05-27 — see `docs/tracks/README.md` for why). The tracks are `core-pipeline`,
`pose-3d`, `vr-output`; their docs live in `docs/tracks/<track>.md`.

- **Branch name**: `<track>/<topic>` (e.g. `vr-output/registration-gate`, `pose-3d/roll-quality`). No numbering. Skip the topic suffix only for trivial one-commit work.
- **Branch base**: cut from `Develop`, or from the prior topic branch on the same track if it hasn't merged yet but has settled (the established "stack on the previous tip" pattern still applies — just keyed to the track, not a phase number).
- **Commit granularity**: one commit per coherent unit of work. Tightly coupled changes (e.g. publisher + CLI + stats wiring) may be bundled into a single commit when separating them would produce un-buildable intermediate states.
- **Commit prefix**: `feat(<track>):` / `fix(<track>):` / `docs(<track>):` where the scope is the track name (e.g. `feat(vr-output): ...`, `fix(pose-3d): ...`). `refactor:`, `chore:`, etc. as before. Bare `docs:` (no track scope) is for repo-wide meta files like CLAUDE.md.
- **Design docs are first-class** (this is what the old `phaseN-*.md` docs were, and they repeatedly proved their worth): work that involves non-trivial design decisions, trade-offs, or multiple milestones **must** get a design doc at `docs/design/<track>-<topic>.md` — background, **options considered + why the rejected ones were rejected**, chosen structure/invariants, milestones, validation (template in `docs/design/README.md`). Trivial single-commit work (a threshold tweak, a bug fix) needs only a changelog line, no design doc.
- **Source of truth**: the track doc (`docs/tracks/<track>.md`) — its scope/design-principles section is current state, its changelog is history (each entry is a summary linking to the design doc). New design docs live in `docs/design/`; the old phase-scheme design docs are frozen under `docs/archive/phaseN-*.md` (don't rewrite them). Forward-looking exploration of *unimplemented* ideas stays in `docs/research/`. `docs/cpp-migration-plan.md` remains the frozen migration record + core-pipeline architecture/validation spec.
- **Completion**: not done until the track doc's changelog has a dated entry, the design doc exists (for non-trivial work), and (when the change touches architecture or validation) `docs/cpp-migration-plan.md` is updated. Shipping code alone is not enough.

> **Historical note (pre-2026-05-27):** earlier work used `cpp-phaseN` branches with
> `feat(phaseN):` commits and one commit per milestone (M1, M2, ...) defined in
> `docs/phaseN-*.md`. `git log --oneline` still shows this pattern up to `cpp-phase15.5`.
> Don't start new phase-numbered branches; map the work onto a track instead.

## Scope and direction

`fitra-cam` runs YOLOX person detection + RTMPose 17-keypoint 2D pose for **multiple USB cameras** on a Jetson Orin Nano Super. The project **migrated from Python (ONNX Runtime) to C++ (TensorRT + Jetson Multimedia API)** to break past the Python parallel-pose ceiling (~18 fps × 2 in the old build); that migration is complete (aggregate 170 fps). Ongoing work is organized by domain track — start at `docs/tracks/README.md`, then the relevant `docs/tracks/<track>.md`. The C++ architecture + migration history is in `docs/cpp-migration-plan.md` (frozen) — read it before non-trivial pipeline work.

Layout:

- `cpp/` — new C++/TensorRT implementation (in progress; primary direction)
- `python/` — preserved Python implementation. Kept working as the **numerical reference** for correctness checks and as a fallback. Don't add features here; only patch to keep it runnable.
- `web-ui/` — Vite + React + TypeScript SPA (viewer at `/`, subject calib at `/subject-calib`). Built to `web-ui/dist/` and served by the C++ Crow app (and the Python FastAPI fallback). Package manager is **pnpm**. `pnpm dev` proxies `/ws`,`/ws3d`,`/api/*` to Crow for HMR. WS/REST JSON schema must stay compatible. See `docs/design/vr-output-webui-vite-react.md`.
- `web/calibration/` — legacy vanilla-JS camera-calibration tools (measure-extrinsics :8010 / ChArUco :8020, Python FastAPI). Not yet migrated to `web-ui/` (pending C++ re-implementation).
- `docs/tracks/` — domain work tracks (`README.md` + `core-pipeline.md` / `pose-3d.md` / `vr-output.md`); current source of truth for ongoing work
- `docs/design/` — design docs for implemented/in-progress work (`<track>-<topic>.md`); successor to the old `phaseN-*.md` docs (template in `docs/design/README.md`)
- `docs/cpp-migration-plan.md` — frozen migration record + core-pipeline architecture, repo layout, validation criteria
- `docs/archive/phaseN-*.md` — historical phase design docs (don't rewrite; referenced from track changelogs)
- `docs/research/*.md` — Japanese-language notes on *unimplemented* decisions; authoritative until promoted to a design doc
- `outputs/recorded_rtmpose/20260515_064342/` — evaluation videos used for C++ correctness checks (`raw_cam{0,1}.mp4` = inputs, `overlay_cam{0,1}.mp4` = Python ORT reference)
- `.github/copilot-instructions.md` — **stale**; pre-dates the python/ relocation and references `rtmlib.PoseTracker` which is no longer in the code. Prefer this file and `python/README.md`.

Jetson-wide constraints (apt OpenCV, NumPy 1.x, `/dev/v4l/by-path`, max-perf power mode, never pip-install opencv/tensorrt) are in `~/CLAUDE.md`. Don't duplicate.

> **Power mode (corrected 2026-06-19):** the max-perf mode on *this* board (Orin Nano Super) is **`sudo nvpmodel -m 2 && sudo jetson_clocks`** — mode IDs are renumbered here: `0`=15W (weakest), `1`=25W, `2`=MAXN_SUPER. The older `nvpmodel -m 0` guidance pins 15W (the *slowest* mode) and was wrong; it measurably throttled multi-camera capture. Verify with `nvpmodel -q`.

## C++ build (current state)

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
./cpp/build/main --help
```

The C++ tree uses CMake 3.22+, g++ 11, TensorRT 10.3 (apt), CUDA 12.6, and pulls header-only deps (Crow, spdlog, nlohmann_json, CLI11, readerwriterqueue) via `FetchContent`. The first cmake configure needs internet for FetchContent; subsequent builds use the populated cache under `cpp/build/_deps/`.

The C++ migration (capture → TRT inference → Crow Web, FP16/INT8, multi-camera perf) is complete and lives on the `core-pipeline` track — its build/run state, architecture, and per-stage validation targets (correctness IoU > 0.99 / kpt L2 < 1px, aggregate 170 fps) are in `docs/tracks/core-pipeline.md` and the frozen `docs/cpp-migration-plan.md` 検証戦略 table. Pose lifting/IK and VMT/SteamVR output are the active tracks — see `docs/tracks/pose-3d.md` and `docs/tracks/vr-output.md`.

### Docker から起動する場合

Jetson 上で `Dockerfile` + `docker-compose.yml` でビルド/起動可能。詳細は `docs/docker-setup.md`。Docker Engine 自体の導入だけ `sudo bash scripts/install_docker.sh` (ユーザー実行) が必要。

## Python build (reference / fallback)

```bash
./python/scripts/setup_jetson_env.sh                         # creates python/.venv
. python/.venv/bin/activate
python python/scripts/dual_rtmpose_web.py --device auto      # FastAPI + WS viewer
python python/scripts/dual_rtmpose_cameras.py --device auto  # CLI snapshot/display
python python/scripts/record_dual_rtmpose_overlay.py --device auto --seconds 30
```

TensorRT first run builds engines under `outputs/tensorrt_engines/` (~7 min); cache invalidates when ONNX / TRT / FP16 / ORT version changes. No tests, no linter, no formal build system for Python.

## Architecture (Python side — still the spec for C++)

**Single shared library, three thin entry scripts.** `python/scripts/pose_pipeline.py` is the spec: V4L2 capture thread, YOLOX ONNX wrapper, RTMPose ONNX wrapper (SimCC argmax + inverse-affine decode, no `rtmlib`), drawer, stats, provider selection, shared argparse. The three entry scripts are different consumer loops on top of these primitives. When porting to C++, treat this file as the contract — preprocessing/postprocessing math must match bit-for-bit (within numerical tolerance) for the correctness check to pass.

**One ORT session per camera, not shared.** `build_engines_for(camera_count, args)` constructs a separate session pair per camera to avoid GIL contention inside ORT's Python wrapper. The C++ side will *invert* this — a **single shared TRT context per model**, single CUDA stream, no per-camera duplication (see `cpp-migration-plan.md` "設計の肝").

**Latest-frame-wins capture.** `CameraReader` overwrites a single `_latest` slot; the worker drops old frames intentionally. The `pending` stat measures inference lag. **Real-time freshness is preferred over processing every frame; preserve this in the C++ rewrite (SPSC queue size 1, drop-old).**

**`CAP_PROP_BUFFERSIZE=2`, not 1.** OpenCV V4L2 with `BUFFERSIZE=1` blocks each `grab()` for a full frame period (~60ms on this UVC), capping throughput at ~15 fps. See `python/scripts/pose_pipeline.py` near `open_v4l2`. The C++ V4L2 path uses `VIDIOC_REQBUFS` with **4 buffers per camera** instead (`cpp-migration-plan.md` arch diagram).

**Detector decimation + single-person default.** `PoseEngine.process` runs YOLOX only every `--det-frequency` frames (default 10), reuses cached bboxes between detections. Default is single-person (largest-area bbox kept); `--multi-person` runs RTMPose on all detections.

**Provider selection is per-model.** Under `--device tensorrt`, only models named by `--trt-models` ({`det` (default), `pose`, `all`}) use TensorRT; the others fall back to CUDA EP. **Default is `det` only because pose-side TensorRT has produced keypoint drift in past observations.** The C++ rewrite must validate pose-side TRT against the Python reference before promoting it.

**Recorder writes measured fps, not requested fps.** Two USB cams on one USB 2.0 bus often deliver ~15 fps × 2 instead of the requested 30. `_record_one` in `python/scripts/record_dual_rtmpose_overlay.py` buffers frames then writes with the *measured* fps so playback timing matches reality.

## Key files

- `python/scripts/pose_pipeline.py` — pipeline spec (capture, YOLOX, RTMPose, drawer, stats, provider selection, argparse)
- `python/scripts/dual_rtmpose_cameras.py` — CLI snapshot/display
- `python/scripts/dual_rtmpose_web.py` — FastAPI + WebSocket viewer
- `python/scripts/record_dual_rtmpose_overlay.py` — 30s raw + overlay MP4 recorder
- `web-ui/` — Vite/React/TS SPA frontend (`src/routes/`, `src/lib/config.ts` = backend-origin seam); JSON schema mirrors `snapshot.cpp` / `dual_rtmpose_web.py`'s `_publisher_loop`
- `cpp/CMakeLists.txt` — top-level CMake for C++ tree (FetchContent, FindTensorRT)
- `cpp/src/` — C++ sources (camera, infer, pipeline, web, util) per `cpp-migration-plan.md` layout
- `docs/tracks/` — domain work tracks; `docs/cpp-migration-plan.md` — frozen migration record + architecture/validation
