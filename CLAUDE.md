# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Communication language

**Write user-facing replies, git commit subjects (after the prefix), commit bodies, and PR descriptions in Japanese.** Conventional Commits prefixes (`feat(phaseN):`, `fix:`, `docs:`, `refactor:`, `chore:`, ...) stay in English. The `Co-Authored-By` trailer also stays in English.

- Final answers to the user, progress reports, summaries: Japanese.
- Commit subject line: `prefix(scope): <Japanese summary>` — prefix in English, summary in Japanese.
- Commit body: Japanese. Technical tokens (CMake, ctest, function names, file paths) stay in English, mixed inline.
- Code comments / identifiers / exception strings: follow the existing convention of the file (this repo is English-majority).
- `docs/` content: match the existing language of the file (phase plans are Japanese-majority, build notes are English-majority).

`git log --oneline` shows the established pattern (`feat(phase8): ...`, `fix(phase9): ...`, `docs(phase11): ...`) — match it.

## Branching and commits

**One branch per phase, one commit per task (milestone)** is the established workflow.

- **Branch name**: `cpp-phaseN` (e.g. `cpp-phase8`, `cpp-phase9`, `cpp-phase11`). Skipping a phase skips the branch number too (Phase 10 was skipped → next branch was `cpp-phase11`, not `cpp-phase10`).
- **Branch base**: cut from the previous phase's tip. The previous phase does not need to be merged into Develop first; once its work has settled, it's a valid base.
- **Commit granularity**: one commit per milestone (M1, M2, ...) in `docs/phaseN-*.md`. Tightly coupled milestones (e.g. publisher + CLI + stats integration in Phase 11) may be bundled into a single commit when separating them would produce un-buildable intermediate states.
- **Commit prefix**: `feat(phaseN):` (new feature), `fix(phaseN):` (bug fix), `docs(phaseN):` (phase-scoped doc update), `refactor:`, `chore:`, etc. Bare `docs:` (no phase scope) is for repo-wide meta files like CLAUDE.md.
- **Design doc**: `docs/phaseN-*.md` is the source of truth for the phase, with milestones (M1–M8 etc.) defined there. Commit bodies should be able to point at "see `docs/phaseN-*.md`" rather than restating the design.
- **Phase completion**: not done until `docs/cpp-migration-plan.md` has a row for the phase in both the "段階実装" section and the "検証戦略" table. Shipping the design doc alone is not enough.

Example (Phase 11):
```
cpp-phase11 (branched from cpp-phase9; Phase 10 was skipped)
  ├─ M1:    feat(phase11): add Skeleton3DBus::snapshot() ...
  ├─ M2:    feat(phase11): hand-rolled OSC 1.0 wire writer ...
  ├─ M3:    feat(phase11): VMC tracker extraction ...
  ├─ M4-6:  feat(phase11): VMC publisher thread + CLI ...
  ├─ M7:    feat(phase11): slimevr_loopback OSC dump tool
  └─ docs:  docs(phase11): execution notes ...
```

## Scope and direction

`fitra-cam` runs YOLOX person detection + RTMPose 17-keypoint 2D pose for **multiple USB cameras** on a Jetson Orin Nano Super. The project is **migrating from Python (ONNX Runtime) to C++ (TensorRT + Jetson Multimedia API)** to break past the Python parallel-pose ceiling (~18 fps × 2 in the old build). The migration plan and architecture are in `docs/cpp-migration-plan.md` — read it before non-trivial work.

Layout:

- `cpp/` — new C++/TensorRT implementation (in progress; primary direction)
- `python/` — preserved Python implementation. Kept working as the **numerical reference** for correctness checks and as a fallback. Don't add features here; only patch to keep it runnable.
- `web/dual_rtmpose/` — Canvas-based static frontend (vanilla HTML/JS). Served by either the Python FastAPI app or (planned) the C++ Crow app. JSON schema must stay compatible.
- `docs/cpp-migration-plan.md` — phase plan, repo layout, validation criteria, completion definition
- `docs/research/*.md` — Japanese-language design notes; authoritative for unimplemented decisions
- `outputs/recorded_rtmpose/20260515_064342/` — evaluation videos used for C++ correctness checks (`raw_cam{0,1}.mp4` = inputs, `overlay_cam{0,1}.mp4` = Python ORT reference)
- `.github/copilot-instructions.md` — **stale**; pre-dates the python/ relocation and references `rtmlib.PoseTracker` which is no longer in the code. Prefer this file and `python/README.md`.

Jetson-wide constraints (apt OpenCV, NumPy 1.x, `/dev/v4l/by-path`, `nvpmodel -m 0 && jetson_clocks`, never pip-install opencv/tensorrt) are in `~/CLAUDE.md`. Don't duplicate.

## C++ build (current state)

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
./cpp/build/main --help
```

The C++ tree uses CMake 3.22+, g++ 11, TensorRT 10.3 (apt), CUDA 12.6, and pulls header-only deps (Crow, spdlog, nlohmann_json, CLI11, readerwriterqueue) via `FetchContent`. The first cmake configure needs internet for FetchContent; subsequent builds use the populated cache under `cpp/build/_deps/`.

Phase status is tracked in `docs/cpp-migration-plan.md` ("段階実装"). Current target per phase:

- Phase 0: link + `nvinfer1::createInferRuntime()` works
- Phase 1: `tools/correctness_check` matches Python ORT within bbox IoU > 0.99 / keypoint L2 < 1 px on the recorded eval videos
- Phase 2: 1-camera end-to-end ≥ 1.5× Python on the same camera
- Phase 3: 3-camera Crow server with skeleton bundles to `web/dual_rtmpose/`
- Phase 4: FP16/INT8/pinned-memory, aggregate ≥ 90 fps

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
- `web/dual_rtmpose/` — static frontend (Canvas); JSON schema defined by `dual_rtmpose_web.py`'s `_publisher_loop`
- `cpp/CMakeLists.txt` — top-level CMake for C++ tree (FetchContent, FindTensorRT)
- `cpp/src/` — C++ sources (camera, infer, pipeline, web, util) per `cpp-migration-plan.md` layout
- `docs/cpp-migration-plan.md` — phase plan, completion criteria
