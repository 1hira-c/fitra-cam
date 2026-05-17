#!/usr/bin/env python3
"""Capture ChArUco observations and write per-camera intrinsics YAML.

The recommended Jetson/headless path is `--web`: it serves MJPEG previews with
ChArUco detection overlays and lets the browser start/pause/solve collection.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Iterator

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
WEB_DIR = REPO_ROOT / "web" / "calibration"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from calibration_io import write_calibration_yaml  # noqa: E402


def parse_cam(value: str, idx: int) -> tuple[str, str]:
    if "=" in value:
        cam_id, path = value.split("=", 1)
        return cam_id.strip(), path.strip()
    return f"cam{idx}", value


def aruco_dictionary(name: str):
    aruco = cv2.aruco
    if not hasattr(aruco, name):
        raise RuntimeError(f"unknown ArUco dictionary: {name}")
    return aruco.getPredefinedDictionary(getattr(aruco, name))


def make_board(args: argparse.Namespace):
    aruco = cv2.aruco
    dictionary = aruco_dictionary(args.dict)
    size = (args.squares_x, args.squares_y)
    if hasattr(aruco, "CharucoBoard"):
        try:
            return aruco.CharucoBoard(size, args.square_len, args.marker_len, dictionary), dictionary
        except TypeError:
            return aruco.CharucoBoard_create(
                args.squares_x, args.squares_y, args.square_len, args.marker_len, dictionary
            ), dictionary
    return aruco.CharucoBoard_create(
        args.squares_x, args.squares_y, args.square_len, args.marker_len, dictionary
    ), dictionary


def same_aspect(w: int, h: int, ref_w: int, ref_h: int) -> bool:
    return w > 0 and h > 0 and w * ref_h == h * ref_w


def list_v4l2_sizes(device: str, fourcc: str) -> list[tuple[int, int]]:
    try:
        res = subprocess.run(
            ["v4l2-ctl", "--device", device, "--list-formats-ext"],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except Exception:
        return []

    sizes: list[tuple[int, int]] = []
    active_format = False
    fmt_re = re.compile(r"\[\d+\]:\s+'([^']+)'")
    size_re = re.compile(r"Size:\s+Discrete\s+(\d+)x(\d+)")
    for line in res.stdout.splitlines():
        m_fmt = fmt_re.search(line)
        if m_fmt:
            active_format = m_fmt.group(1).upper() == fourcc.upper()
            continue
        if active_format:
            m_size = size_re.search(line)
            if m_size:
                sizes.append((int(m_size.group(1)), int(m_size.group(2))))
    return sorted(set(sizes), key=lambda s: s[0] * s[1], reverse=True)


def opencv_accepts_size(device: str, width: int, height: int, fourcc: str, fps: int) -> bool:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return False
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        ok, frame = cap.read()
        if not ok or frame is None:
            return False
        return frame.shape[1] == width and frame.shape[0] == height
    finally:
        cap.release()


def select_calibration_size(device: str, args: argparse.Namespace) -> tuple[int, int, str]:
    if args.calib_width and args.calib_height:
        return args.calib_width, args.calib_height, "explicit"

    ref_w, ref_h = args.runtime_width, args.runtime_height
    v4l2_sizes = list_v4l2_sizes(device, args.fourcc)
    for w, h in v4l2_sizes:
        if same_aspect(w, h, ref_w, ref_h):
            return w, h, "v4l2-ctl"

    fallback = [
        (1920, 1440),
        (1600, 1200),
        (1280, 960),
        (1024, 768),
        (800, 600),
        (640, 480),
    ]
    for w, h in fallback:
        if same_aspect(w, h, ref_w, ref_h) and opencv_accepts_size(device, w, h, args.fourcc, args.calib_fps):
            return w, h, "opencv-probe"

    raise RuntimeError(
        f"no {ref_w}:{ref_h} aspect {args.fourcc} calibration size found for {device}"
    )


def open_camera(path: str, width: int, height: int, fps: int, fourcc: str) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera: {path}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    return cap


def detect_charuco(frame: np.ndarray, board, dictionary) -> tuple[np.ndarray | None, np.ndarray | None, Any]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco = cv2.aruco
    if hasattr(aruco, "CharucoDetector"):
        detector = aruco.CharucoDetector(board)
        corners, ids, marker_corners, _marker_ids = detector.detectBoard(gray)
        return corners, ids, marker_corners
    marker_corners, marker_ids, _ = aruco.detectMarkers(gray, dictionary)
    if marker_ids is None or len(marker_ids) == 0:
        return None, None, marker_corners
    _ok, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board
    )
    return charuco_corners, charuco_ids, marker_corners


def can_use_gui() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def draw_detection_overlay(
    frame: np.ndarray,
    cam_id: str,
    accepted: int,
    target_samples: int,
    corners: np.ndarray | None,
    ids: np.ndarray | None,
    marker_corners,
    *,
    collecting: bool,
    capture_size: tuple[int, int],
    runtime_size: tuple[int, int],
) -> np.ndarray:
    view = frame.copy()
    if marker_corners is not None and len(marker_corners):
        cv2.aruco.drawDetectedMarkers(view, marker_corners)
    if corners is not None and ids is not None:
        cv2.aruco.drawDetectedCornersCharuco(view, corners, ids)
    n = 0 if ids is None else len(ids)
    state = "collect" if collecting else "pause"
    lines = [
        f"{cam_id} {state} {accepted}/{target_samples} corners={n}",
        f"capture={capture_size[0]}x{capture_size[1]} runtime={runtime_size[0]}x{runtime_size[1]}",
    ]
    y = 30
    for text in lines:
        cv2.putText(view, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        y += 32
    return view


def scale_intrinsics_for_runtime(capture: dict[str, Any], runtime_w: int, runtime_h: int) -> dict[str, Any]:
    cap_w = int(capture["width"])
    cap_h = int(capture["height"])
    sx = runtime_w / cap_w
    sy = runtime_h / cap_h
    K = np.asarray(capture["K"], dtype=np.float64).copy()
    K[0, :] *= sx
    K[1, :] *= sy
    return {
        "K": K,
        "dist": np.asarray(capture["dist"], dtype=np.float64).reshape(-1),
        "width": runtime_w,
        "height": runtime_h,
        "source": f"{capture.get('source', 'charuco')}_scaled",
        "rms_px": float(capture.get("rms_px", 0.0)) * ((sx + sy) * 0.5),
        "scale_x": sx,
        "scale_y": sy,
    }


def calibrate_from_samples(
    cam_id: str,
    all_corners: list[np.ndarray],
    all_ids: list[np.ndarray],
    image_size: tuple[int, int],
    board,
) -> dict[str, Any]:
    if len(all_corners) < 3:
        raise RuntimeError(f"{cam_id}: not enough ChArUco samples ({len(all_corners)})")
    ret, K, dist, _rvecs, _tvecs = cv2.aruco.calibrateCameraCharuco(
        all_corners, all_ids, board, image_size, None, None
    )
    print(f"[{cam_id}] intrinsic RMS={ret:.4f}px size={image_size[0]}x{image_size[1]}", file=sys.stderr)
    return {
        "K": K,
        "dist": dist.reshape(-1),
        "width": image_size[0],
        "height": image_size[1],
        "source": "charuco_capture",
        "rms_px": float(ret),
    }


class CharucoCameraWorker:
    def __init__(
        self,
        cam_id: str,
        device: str,
        width: int,
        height: int,
        args: argparse.Namespace,
        board,
        dictionary,
    ):
        self.cam_id = cam_id
        self.device = device
        self.width = width
        self.height = height
        self.args = args
        self.board = board
        self.dictionary = dictionary
        self.collecting = False
        self.stop_evt = threading.Event()
        self.thread: threading.Thread | None = None
        self.lock = threading.Lock()
        self.all_corners: list[np.ndarray] = []
        self.all_ids: list[np.ndarray] = []
        self.last_accept = 0.0
        self.last_frame_at = 0.0
        self.actual_size = (width, height)
        self.latest_jpeg: bytes | None = None
        self.latest_corners = 0
        self.latest_error: str | None = None
        self.recv_count = 0

    def start(self) -> None:
        self.thread = threading.Thread(target=self._loop, name=f"Charuco-{self.cam_id}", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_evt.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)

    def reset(self) -> None:
        with self.lock:
            self.all_corners.clear()
            self.all_ids.clear()
            self.last_accept = 0.0

    def accepted(self) -> int:
        with self.lock:
            return len(self.all_corners)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "id": self.cam_id,
                "device": self.device,
                "capture_width": self.actual_size[0],
                "capture_height": self.actual_size[1],
                "target_samples": self.args.samples,
                "accepted": len(self.all_corners),
                "latest_corners": self.latest_corners,
                "collecting": self.collecting,
                "last_frame_age_ms": max(0.0, (time.monotonic() - self.last_frame_at) * 1000.0),
                "error": self.latest_error,
            }

    def solve(self) -> dict[str, Any]:
        with self.lock:
            corners = [c.copy() for c in self.all_corners]
            ids = [i.copy() for i in self.all_ids]
            image_size = self.actual_size
        return calibrate_from_samples(self.cam_id, corners, ids, image_size, self.board)

    def jpeg_stream(self) -> Iterator[bytes]:
        while not self.stop_evt.is_set():
            with self.lock:
                jpeg = self.latest_jpeg
            if jpeg is None:
                time.sleep(0.05)
                continue
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
            time.sleep(0.08)

    def _loop(self) -> None:
        cap = None
        try:
            cap = open_camera(self.device, self.width, self.height, self.args.calib_fps, self.args.fourcc)
            while not self.stop_evt.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    time.sleep(0.02)
                    continue
                now = time.monotonic()
                self.actual_size = (frame.shape[1], frame.shape[0])
                corners, ids, marker_corners = detect_charuco(frame, self.board, self.dictionary)
                n = 0 if ids is None else len(ids)
                accepted_now = (
                    self.collecting
                    and n >= self.args.min_corners
                    and now - self.last_accept >= self.args.sample_interval
                    and len(self.all_corners) < self.args.samples
                )
                with self.lock:
                    if accepted_now:
                        self.all_corners.append(corners.astype(np.float32))
                        self.all_ids.append(ids.astype(np.int32))
                        self.last_accept = now
                    if len(self.all_corners) >= self.args.samples:
                        self.collecting = False
                    accepted = len(self.all_corners)
                    collecting = self.collecting
                overlay = draw_detection_overlay(
                    frame,
                    self.cam_id,
                    accepted,
                    self.args.samples,
                    corners,
                    ids,
                    marker_corners,
                    collecting=collecting,
                    capture_size=self.actual_size,
                    runtime_size=(self.args.runtime_width, self.args.runtime_height),
                )
                ok_jpeg, buf = cv2.imencode(".jpg", overlay, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                with self.lock:
                    self.recv_count += 1
                    self.last_frame_at = now
                    self.latest_corners = n
                    if ok_jpeg:
                        self.latest_jpeg = buf.tobytes()
                    self.latest_error = None
        except Exception as exc:
            with self.lock:
                self.latest_error = str(exc)
            print(f"[{self.cam_id}] camera worker failed: {exc}", file=sys.stderr)
        finally:
            if cap is not None:
                cap.release()


def calibrate_one_cli(
    cam_id: str,
    device: str,
    args: argparse.Namespace,
    board,
    dictionary,
    width: int,
    height: int,
) -> dict[str, Any]:
    cap = open_camera(device, width, height, args.calib_fps, args.fourcc)
    all_corners: list[np.ndarray] = []
    all_ids: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None
    last_accept = 0.0
    last_preview_save = 0.0
    accepted = 0
    preview = bool(args.preview and can_use_gui())
    preview_dir = Path(args.preview_dir) if args.preview_dir else None
    if args.preview and not preview:
        print(f"[{cam_id}] --preview ignored because no display is available; use --web or --preview-dir", file=sys.stderr)
    if preview_dir is not None:
        preview_dir.mkdir(parents=True, exist_ok=True)

    print(f"[{cam_id}] collecting {args.samples} ChArUco samples from {device} at {width}x{height}", file=sys.stderr)
    try:
        while accepted < args.samples:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.02)
                continue
            image_size = (frame.shape[1], frame.shape[0])
            corners, ids, marker_corners = detect_charuco(frame, board, dictionary)
            n = 0 if ids is None else len(ids)
            now = time.monotonic()
            accepted_now = n >= args.min_corners and now - last_accept >= args.sample_interval
            if accepted_now:
                all_corners.append(corners.astype(np.float32))
                all_ids.append(ids.astype(np.int32))
                accepted += 1
                last_accept = now
                print(f"[{cam_id}] accepted {accepted}/{args.samples} corners={n}", file=sys.stderr)
            save_preview = (
                preview_dir is not None
                and (accepted_now or now - last_preview_save >= args.preview_interval)
            )
            if preview or save_preview:
                view = draw_detection_overlay(
                    frame,
                    cam_id,
                    accepted,
                    args.samples,
                    corners,
                    ids,
                    marker_corners,
                    collecting=True,
                    capture_size=image_size,
                    runtime_size=(args.runtime_width, args.runtime_height),
                )
            if save_preview:
                cv2.imwrite(str(preview_dir / f"{cam_id}_latest.jpg"), view)
                if accepted_now:
                    cv2.imwrite(str(preview_dir / f"{cam_id}_accepted_{accepted:03d}.jpg"), view)
                last_preview_save = now
            if preview:
                cv2.imshow(f"charuco-{cam_id}", view)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    finally:
        cap.release()
        if preview:
            cv2.destroyWindow(f"charuco-{cam_id}")

    if image_size is None:
        raise RuntimeError(f"{cam_id}: no frames captured")
    return calibrate_from_samples(cam_id, all_corners, all_ids, image_size, board)


def write_profiles(
    out_path: Path,
    capture_intrinsics: dict[str, dict[str, Any]],
    args: argparse.Namespace,
    metadata: dict[str, Any],
) -> dict[str, dict[str, Any]]:
    runtime_intrinsics = {
        cam_id: scale_intrinsics_for_runtime(data, args.runtime_width, args.runtime_height)
        for cam_id, data in capture_intrinsics.items()
    }
    quality: dict[str, Any] = {"intrinsic_camera_count": len(capture_intrinsics)}
    for cam_id, capture in capture_intrinsics.items():
        runtime = runtime_intrinsics[cam_id]
        quality[cam_id] = {
            "capture_width": capture["width"],
            "capture_height": capture["height"],
            "runtime_width": runtime["width"],
            "runtime_height": runtime["height"],
            "capture_rms_px": capture["rms_px"],
            "runtime_rms_px": runtime["rms_px"],
            "scale_x": runtime["scale_x"],
            "scale_y": runtime["scale_y"],
        }
    write_calibration_yaml(
        out_path,
        intrinsics=runtime_intrinsics,
        capture_intrinsics=capture_intrinsics,
        quality=quality,
        metadata=metadata,
    )
    return runtime_intrinsics


def build_web_app(workers: list[CharucoCameraWorker], args: argparse.Namespace, out_path: Path):
    from fastapi import FastAPI, HTTPException
    from fastapi.responses import FileResponse, StreamingResponse
    from fastapi.staticfiles import StaticFiles

    app = FastAPI()

    @app.get("/")
    async def index():
        return FileResponse(str(WEB_DIR / "charuco.html"))

    @app.get("/api/charuco/session")
    async def session():
        return {
            "cameras": [w.snapshot() for w in workers],
            "runtime": {"width": args.runtime_width, "height": args.runtime_height},
            "samples": args.samples,
            "min_corners": args.min_corners,
            "output": str(out_path),
        }

    @app.post("/api/charuco/start")
    async def start():
        for worker in workers:
            with worker.lock:
                worker.collecting = True
        return {"ok": True}

    @app.post("/api/charuco/pause")
    async def pause():
        for worker in workers:
            with worker.lock:
                worker.collecting = False
        return {"ok": True}

    @app.post("/api/charuco/reset/{cam_id}")
    async def reset(cam_id: str):
        for worker in workers:
            if worker.cam_id == cam_id:
                worker.reset()
                return {"ok": True}
        raise HTTPException(status_code=404, detail=f"unknown camera: {cam_id}")

    @app.post("/api/charuco/reset")
    async def reset_all():
        for worker in workers:
            worker.reset()
        return {"ok": True}

    @app.post("/api/charuco/solve")
    async def solve():
        for worker in workers:
            with worker.lock:
                worker.collecting = False
        try:
            capture_intrinsics = {worker.cam_id: worker.solve() for worker in workers}
            runtime_intrinsics = write_profiles(
                out_path,
                capture_intrinsics,
                args,
                {
                    "tool": "calibrate_intrinsics_charuco.py",
                    "mode": "web_mjpeg",
                    "fourcc": args.fourcc,
                    "calib_fps": args.calib_fps,
                },
            )
        except Exception as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {
            "ok": True,
            "output": str(out_path),
            "capture_intrinsics": {
                cam_id: {
                    "width": int(data["width"]),
                    "height": int(data["height"]),
                    "rms_px": float(data["rms_px"]),
                }
                for cam_id, data in capture_intrinsics.items()
            },
            "runtime_intrinsics": {
                cam_id: {
                    "width": int(data["width"]),
                    "height": int(data["height"]),
                    "rms_px": float(data["rms_px"]),
                }
                for cam_id, data in runtime_intrinsics.items()
            },
        }

    @app.get("/stream/{cam_id}")
    async def stream(cam_id: str):
        for worker in workers:
            if worker.cam_id == cam_id:
                return StreamingResponse(
                    worker.jpeg_stream(),
                    media_type="multipart/x-mixed-replace; boundary=frame",
                )
        raise HTTPException(status_code=404, detail=f"unknown camera: {cam_id}")

    app.mount("/static", StaticFiles(directory=str(WEB_DIR)), name="calibration_static")
    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate camera intrinsics with a ChArUco board")
    parser.add_argument("--cam", action="append", required=True,
                        help="camera path, or cam_id=/dev/v4l/by-path/...; repeat per camera")
    parser.add_argument("--out", default="calibrations/intrinsics.yaml")
    parser.add_argument("--runtime-width", "--width", dest="runtime_width", type=int, default=640)
    parser.add_argument("--runtime-height", "--height", dest="runtime_height", type=int, default=480)
    parser.add_argument("--calib-width", type=int, default=0)
    parser.add_argument("--calib-height", type=int, default=0)
    parser.add_argument("--calib-fps", "--fps", dest="calib_fps", type=int, default=5)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--squares-x", type=int, required=True)
    parser.add_argument("--squares-y", type=int, required=True)
    parser.add_argument("--square-len", type=float, required=True, help="square length in meters")
    parser.add_argument("--marker-len", type=float, required=True, help="marker length in meters")
    parser.add_argument("--dict", default="DICT_4X4_50")
    parser.add_argument("--samples", type=int, default=25)
    parser.add_argument("--min-corners", type=int, default=8)
    parser.add_argument("--sample-interval", type=float, default=0.35)
    parser.add_argument("--web", action="store_true", help="serve MJPEG browser preview and collect via API")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8020)
    parser.add_argument("--preview", action="store_true",
                        help="show OpenCV GUI preview in CLI mode; ignored when no display is available")
    parser.add_argument("--preview-dir",
                        help="write CLI-mode headless preview images such as cam0_latest.jpg")
    parser.add_argument("--preview-interval", type=float, default=1.0,
                        help="seconds between preview image writes when --preview-dir is set")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is unavailable; install/use apt OpenCV with contrib modules")
    if bool(args.calib_width) != bool(args.calib_height):
        raise RuntimeError("--calib-width and --calib-height must be specified together")
    board, dictionary = make_board(args)
    cams = [parse_cam(cam_arg, idx) for idx, cam_arg in enumerate(args.cam)]
    selected: dict[str, tuple[int, int, str]] = {}
    for cam_id, device in cams:
        selected[cam_id] = select_calibration_size(device, args)
        w, h, source = selected[cam_id]
        print(f"[{cam_id}] calibration capture size {w}x{h} selected by {source}", file=sys.stderr)

    out_path = Path(args.out)
    if args.web:
        workers = [
            CharucoCameraWorker(cam_id, device, selected[cam_id][0], selected[cam_id][1], args, board, dictionary)
            for cam_id, device in cams
        ]
        for worker in workers:
            worker.start()
        try:
            import uvicorn

            print(f"[ready] http://{args.host}:{args.port}/", file=sys.stderr)
            uvicorn.run(build_web_app(workers, args, out_path), host=args.host, port=args.port, log_level="warning")
        finally:
            for worker in workers:
                worker.stop()
        return 0

    capture_intrinsics: dict[str, dict[str, Any]] = {}
    for cam_id, device in cams:
        w, h, _source = selected[cam_id]
        capture_intrinsics[cam_id] = calibrate_one_cli(cam_id, device, args, board, dictionary, w, h)
    write_profiles(
        out_path,
        capture_intrinsics,
        args,
        {
            "tool": "calibrate_intrinsics_charuco.py",
            "mode": "cli",
            "fourcc": args.fourcc,
            "calib_fps": args.calib_fps,
        },
    )
    print(f"[done] wrote {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
