#!/usr/bin/env python3
"""Phase 8 browser wizard for recording pose clips and building an IK profile."""

from __future__ import annotations

import argparse
import collections
import dataclasses
import datetime as dt
import json
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Iterator

import cv2

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
WEB_DIR = REPO_ROOT / "web" / "subject_profile"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

DEFAULT_SEQUENCE = "standing:5,t_pose:5,elbow_flex:5,knee_flex:5"
DEFAULT_CAM0 = "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.3:1.0-video-index0"
DEFAULT_CAM1 = "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4:1.0-video-index0"


@dataclasses.dataclass
class CameraConfig:
    path: str
    width: int = 640
    height: int = 480
    fps: int = 30
    fourcc: str = "MJPG"


def open_v4l2(cfg: CameraConfig) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(cfg.path, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"failed to open camera: {cfg.path}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*cfg.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
    cap.set(cv2.CAP_PROP_FPS, cfg.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def parse_sequence(value: str) -> list[tuple[str, float]]:
    out: list[tuple[str, float]] = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        if ":" in part:
            name, seconds = part.split(":", 1)
            out.append((name.strip(), float(seconds)))
        else:
            out.append((part, 5.0))
    if not out:
        raise argparse.ArgumentTypeError("pose sequence is empty")
    return out


def rel_to(path: Path, base: Path) -> str:
    try:
        return path.relative_to(base).as_posix()
    except ValueError:
        return path.as_posix()


class CameraWorker:
    def __init__(self, cam_id: int, cfg: CameraConfig):
        self.cam_id = cam_id
        self.cfg = cfg
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._frame = None
        self._seq = 0
        self._ts = 0.0
        self._times: collections.deque[float] = collections.deque(maxlen=60)
        self.error = ""

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name=f"SubjectCam-{self.cam_id}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        try:
            cap = open_v4l2(self.cfg)
        except Exception as exc:  # pragma: no cover - hardware path
            self.error = str(exc)
            return
        try:
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    time.sleep(0.01)
                    continue
                now = time.monotonic()
                with self._lock:
                    self._frame = frame
                    self._seq += 1
                    self._ts = now
                    self._times.append(now)
        finally:
            cap.release()

    def latest(self):
        with self._lock:
            if self._frame is None:
                return self._seq, self._ts, None
            return self._seq, self._ts, self._frame.copy()

    def recv_fps(self) -> float:
        with self._lock:
            if len(self._times) < 2:
                return 0.0
            span = self._times[-1] - self._times[0]
            return (len(self._times) - 1) / span if span > 0 else 0.0


class SubjectProfileSession:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.subject_id = args.subject_id
        self.sequence = parse_sequence(args.pose_sequence)
        self.pose_names = [p[0] for p in self.sequence]
        run_ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.subject_dir = Path(args.output_dir) / self.subject_id
        self.session_dir = self.subject_dir / "sessions" / run_ts
        self.raw_dir = self.session_dir / "raw"
        self.overlay_dir = self.session_dir / "overlays"
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.raw_dir.mkdir(parents=True, exist_ok=True)
        self.workers = [
            CameraWorker(0, CameraConfig(args.cam0, args.width, args.height, args.fps, args.fourcc)),
            CameraWorker(1, CameraConfig(args.cam1, args.width, args.height, args.fps, args.fourcc)),
        ]
        self._lock = threading.Lock()
        self.busy = False
        self.current = ""
        self.message = "ready"
        self.poses: dict[str, dict[str, Any]] = {}
        self.last_analyze: dict[str, Any] | None = None

    def start(self) -> None:
        for worker in self.workers:
            worker.start()
        time.sleep(0.5)
        errors = [w.error for w in self.workers if w.error]
        if errors:
            raise RuntimeError("; ".join(errors))
        self.write_pose_session()

    def stop(self) -> None:
        for worker in self.workers:
            worker.stop()

    def state(self) -> dict[str, Any]:
        with self._lock:
            return {
                "subject_id": self.subject_id,
                "session_dir": str(self.session_dir),
                "pose_session": str(self.session_dir / "pose_session.json"),
                "latest_profile": str(self.subject_dir / "latest_profile.yaml"),
                "busy": self.busy,
                "current": self.current,
                "message": self.message,
                "sequence": [{"name": n, "duration_s": d} for n, d in self.sequence],
                "poses": self.poses,
                "cameras": [
                    {"id": w.cam_id, "path": w.cfg.path, "recv_fps": w.recv_fps(), "error": w.error}
                    for w in self.workers
                ],
                "last_analyze": self.last_analyze,
            }

    def write_pose_session(self) -> None:
        data = {
            "schema": "fitra_pose_session_v1",
            "subject_id": self.subject_id,
            "created_at": dt.datetime.now().isoformat(timespec="seconds"),
            "subject_height_m": float(self.args.subject_height_m),
            "calib": self.args.calib,
            "camera_count": 2,
            "cameras": [
                {"id": "cam0", "path": self.args.cam0},
                {"id": "cam1", "path": self.args.cam1},
            ],
            "poses": [self.poses[name] for name in self.pose_names if name in self.poses],
        }
        path = self.session_dir / "pose_session.json"
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    def record_pose(self, pose: str) -> dict[str, Any]:
        duration = dict(self.sequence).get(pose)
        if duration is None:
            raise ValueError(f"unknown pose: {pose}")
        with self._lock:
            if self.busy:
                raise RuntimeError("session is busy")
            self.busy = True
            self.current = pose
            self.message = f"prepare {pose}"
        try:
            time.sleep(max(0.0, float(self.args.countdown_s)))
            with self._lock:
                self.message = f"recording {pose}"
            frames: list[list[Any]] = [[], []]
            timestamps: list[list[float]] = [[], []]
            last_seq = [-1, -1]
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                for cam, worker in enumerate(self.workers):
                    seq, ts, frame = worker.latest()
                    if frame is not None and seq != last_seq[cam]:
                        last_seq[cam] = seq
                        frames[cam].append(frame)
                        timestamps[cam].append(ts)
                time.sleep(0.002)

            clips: list[str] = []
            counts: list[int] = []
            fps_values: list[float] = []
            for cam in range(2):
                n = len(frames[cam])
                if n < 2:
                    raise RuntimeError(f"recording too short for cam{cam}: {n} frames")
                span = timestamps[cam][-1] - timestamps[cam][0]
                actual_fps = (n - 1) / span if span > 0 else float(self.args.fps)
                out_path = self.raw_dir / f"{pose}_cam{cam}.mp4"
                writer = cv2.VideoWriter(
                    str(out_path),
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    float(actual_fps),
                    (self.args.width, self.args.height),
                )
                if not writer.isOpened():
                    raise RuntimeError(f"failed to open writer: {out_path}")
                try:
                    for frame in frames[cam]:
                        writer.write(frame)
                finally:
                    writer.release()
                clips.append(rel_to(out_path, self.session_dir))
                counts.append(n)
                fps_values.append(actual_fps)

            entry = {
                "name": pose,
                "duration_s": duration,
                "countdown_s": float(self.args.countdown_s),
                "clips": clips,
                "frames": counts,
                "fps": fps_values,
                "status": "recorded",
            }
            with self._lock:
                self.poses[pose] = entry
                self.last_analyze = None
                self.message = f"recorded {pose}"
            self.write_pose_session()
            return entry
        finally:
            with self._lock:
                self.busy = False
                self.current = ""

    def analyze(self) -> dict[str, Any]:
        with self._lock:
            if self.busy:
                raise RuntimeError("session is busy")
            self.busy = True
            self.current = "analyze"
            self.message = "analyzing"
        try:
            missing = [name for name in self.pose_names if name not in self.poses]
            if missing:
                raise RuntimeError("missing poses: " + ", ".join(missing))
            pose_session = self.session_dir / "pose_session.json"
            profile = self.session_dir / "subject_profile.yaml"
            quality = self.session_dir / "quality.json"
            out_jsonl = self.session_dir / "joints3d.jsonl"
            summary = self.session_dir / "summary.json"
            dump_bin = Path(self.args.dump_keypoints_bin)
            if not dump_bin.is_absolute():
                repo_relative = REPO_ROOT / dump_bin
                if repo_relative.exists():
                    dump_bin = repo_relative
            cmd = [
                str(dump_bin),
                "--pose-session", str(pose_session),
                "--calib", self.args.calib,
                "--det-engine", self.args.det_engine,
                "--pose-engine", self.args.pose_engine,
                "--out", str(out_jsonl),
                "--summary", str(summary),
                "--overlay-dir", str(self.overlay_dir),
                "--subject-profile-out", str(profile),
                "--quality-out", str(quality),
                "--det-score", str(self.args.det_score),
                "--kp-conf-thresh", str(self.args.kp_conf_thresh),
                "--max-reproj-px", str(self.args.max_reproj_px),
            ]
            if self.args.subject_height_m > 0:
                cmd += ["--subject-height-m", str(self.args.subject_height_m)]
            proc = subprocess.run(cmd, text=True, capture_output=True, check=False)
            result: dict[str, Any] = {
                "returncode": proc.returncode,
                "cmd": cmd,
                "stdout": proc.stdout[-4000:],
                "stderr": proc.stderr[-4000:],
                "profile": str(profile),
                "quality": str(quality),
                "summary": str(summary),
            }
            if quality.exists():
                result["quality_data"] = json.loads(quality.read_text(encoding="utf-8"))
            with self._lock:
                self.last_analyze = result
                self.message = "analysis done" if proc.returncode == 0 else "analysis failed"
            if proc.returncode != 0:
                raise RuntimeError(proc.stderr[-1000:] or proc.stdout[-1000:] or "analysis failed")
            return result
        finally:
            with self._lock:
                self.busy = False
                self.current = ""

    def approve(self, force: bool = False) -> dict[str, Any]:
        quality_path = self.session_dir / "quality.json"
        profile_path = self.session_dir / "subject_profile.yaml"
        if not quality_path.exists() or not profile_path.exists():
            raise RuntimeError("analysis output is missing")
        quality = json.loads(quality_path.read_text(encoding="utf-8"))
        status = quality.get("status", "fail")
        if status == "fail" and not force:
            raise RuntimeError("quality is fail; retake or approve with force")
        latest = self.subject_dir / "latest_profile.yaml"
        latest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(profile_path, latest)
        return {"latest_profile": str(latest), "quality_status": status}


def build_app(session: SubjectProfileSession):
    from fastapi import FastAPI, HTTPException, Query
    from fastapi.responses import FileResponse, StreamingResponse
    from fastapi.staticfiles import StaticFiles

    app = FastAPI()
    app.mount("/static", StaticFiles(directory=str(WEB_DIR)), name="subject_profile_static")

    @app.get("/")
    def index():
        return FileResponse(WEB_DIR / "index.html")

    @app.get("/api/state")
    def state():
        return session.state()

    @app.post("/api/record/{pose}")
    def record(pose: str):
        try:
            return session.record_pose(pose)
        except Exception as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.post("/api/analyze")
    def analyze():
        try:
            return session.analyze()
        except Exception as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.post("/api/approve")
    def approve(force: bool = Query(False)):
        try:
            return session.approve(force=force)
        except Exception as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.get("/preview/{cam_name}.mjpg")
    def preview(cam_name: str):
        if not cam_name.startswith("cam"):
            raise HTTPException(status_code=404, detail="unknown camera")
        cam = int(cam_name[3:])
        if cam < 0 or cam >= len(session.workers):
            raise HTTPException(status_code=404, detail="unknown camera")

        def gen() -> Iterator[bytes]:
            while True:
                _, _, frame = session.workers[cam].latest()
                if frame is not None:
                    ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                    if ok:
                        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n"
                time.sleep(0.05)

        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/artifacts/{path:path}")
    def artifact(path: str):
        root = session.session_dir.resolve()
        target = (session.session_dir / path).resolve()
        if root not in target.parents and target != root:
            raise HTTPException(status_code=403, detail="forbidden")
        if not target.is_file():
            raise HTTPException(status_code=404, detail="not found")
        return FileResponse(target)

    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Phase 8 subject profile wizard")
    parser.add_argument("--subject-id", required=True)
    parser.add_argument("--calib", required=True)
    parser.add_argument("--cam0", default=DEFAULT_CAM0)
    parser.add_argument("--cam1", default=DEFAULT_CAM1)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--det-engine", required=True)
    parser.add_argument("--pose-engine", required=True)
    parser.add_argument("--dump-keypoints-bin", default="cpp/build/tools/dump_keypoints_3d")
    parser.add_argument("--output-dir", default="calibrations/subjects")
    parser.add_argument("--subject-height-m", type=float, default=0.0)
    parser.add_argument("--pose-sequence", default=DEFAULT_SEQUENCE)
    parser.add_argument("--countdown-s", type=float, default=3.0)
    parser.add_argument("--det-score", type=float, default=0.5)
    parser.add_argument("--kp-conf-thresh", type=float, default=0.3)
    parser.add_argument("--max-reproj-px", type=float, default=6.0)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8030)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.subject_height_m < 0.0 or args.subject_height_m > 2.5:
        raise SystemExit("--subject-height-m must be 0 or a plausible meter value <= 2.5")
    session = SubjectProfileSession(args)
    session.start()
    import uvicorn

    print(f"[ready] http://{args.host}:{args.port}/", file=sys.stderr)
    print(f"[data] session: {session.session_dir}", file=sys.stderr)
    try:
        uvicorn.run(build_app(session), host=args.host, port=args.port, log_level="warning")
    finally:
        session.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
