#!/usr/bin/env python3
"""Browser-assisted extrinsic calibration from measured floor points."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
WEB_DIR = REPO_ROOT / "web" / "calibration"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from calibration_io import load_intrinsics_yaml, write_calibration_yaml  # noqa: E402


def parse_kv(value: str, default_id: str) -> tuple[str, str]:
    if "=" in value:
        key, val = value.split("=", 1)
        return key.strip(), val.strip()
    return default_id, value


def _normalize_point(p: dict[str, Any], fallback_id: str) -> dict[str, Any]:
    return {
        "id": str(p.get("id", fallback_id)),
        "x": float(p["x"]),
        "y": float(p["y"]),
        "z": float(p.get("z", 0.0)),
    }


def _expand_floor_grid(grid: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    x_values = [float(v) for v in grid.get("x_m", [])]
    y_values = [float(v) for v in grid.get("y_m", [])]
    if len(x_values) < 2 or len(y_values) < 2:
        raise RuntimeError("floor_grid requires x_m and y_m arrays with at least 2 values each")
    z = float(grid.get("z_m", 0.0))
    point_ids: list[list[str]] = []
    points: list[dict[str, Any]] = []
    for row, y in enumerate(y_values):
        row_ids: list[str] = []
        for col, x in enumerate(x_values):
            pid = f"grid_r{row:02d}_c{col:02d}"
            row_ids.append(pid)
            points.append({"id": pid, "x": x, "y": y, "z": z})
        point_ids.append(row_ids)
    expanded = {
        "x_m": x_values,
        "y_m": y_values,
        "z_m": z,
        "point_ids": point_ids,
    }
    return points, expanded


def load_world_points(path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    raw_points = data.get("points", data if isinstance(data, list) else [])
    if not isinstance(raw_points, list):
        raise RuntimeError("world points JSON points must be a list")
    normalized: list[dict[str, Any]] = []
    metadata = {k: v for k, v in data.items() if k != "points"} if isinstance(data, dict) else {}
    grid = metadata.get("floor_grid", metadata.get("grid"))
    if isinstance(grid, dict):
        grid_points, expanded_grid = _expand_floor_grid(grid)
        normalized.extend(grid_points)
        metadata["floor_grid"] = expanded_grid
    for i, p in enumerate(raw_points):
        normalized.append(_normalize_point(p, f"p{i:02d}"))
    seen: set[str] = set()
    for p in normalized:
        if p["id"] in seen:
            raise RuntimeError(f"duplicate world point id: {p['id']}")
        seen.add(p["id"])
    if len(normalized) < 4:
        raise RuntimeError("world points JSON must contain at least 4 points or floor_grid intersections")
    return normalized, metadata


def open_camera(path: str, args: argparse.Namespace) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera: {path}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    return cap


def capture_still(device: str, args: argparse.Namespace, width: int, height: int) -> np.ndarray:
    cap_args = argparse.Namespace(**vars(args))
    cap_args.width = width
    cap_args.height = height
    cap = open_camera(device, cap_args)
    try:
        frame = None
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            ok, img = cap.read()
            if ok:
                frame = img
            time.sleep(0.03)
        if frame is None:
            raise RuntimeError(f"failed to capture frame from {device}")
        return frame
    finally:
        cap.release()


def load_or_capture_images(
    args: argparse.Namespace,
    out_dir: Path,
    capture_intrinsics: dict[str, dict[str, Any]],
) -> dict[str, Path]:
    images_dir = out_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    images: dict[str, Path] = {}
    for idx, item in enumerate(args.image or []):
        cam_id, path = parse_kv(item, f"cam{idx}")
        src = Path(path)
        if not src.exists():
            raise RuntimeError(f"image not found: {src}")
        dst = images_dir / f"{cam_id}{src.suffix or '.jpg'}"
        img = cv2.imread(str(src), cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"failed to read image: {src}")
        cv2.imwrite(str(dst), img)
        images[cam_id] = dst
    offset = len(images)
    for idx, item in enumerate(args.cam or []):
        cam_id, device = parse_kv(item, f"cam{idx + offset}")
        intr = capture_intrinsics.get(cam_id)
        width = int(intr["width"]) if intr else args.width
        height = int(intr["height"]) if intr else args.height
        frame = capture_still(device, args, width, height)
        dst = images_dir / f"{cam_id}.jpg"
        cv2.imwrite(str(dst), frame)
        images[cam_id] = dst
        print(f"[capture] {cam_id}: {device} -> {dst}", file=sys.stderr)
    if not images:
        raise RuntimeError("provide at least one --image or --cam")
    return images


def load_annotations(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"cameras": {}}
    return json.loads(path.read_text(encoding="utf-8"))


def save_annotations(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def camera_center_from_rt(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(rvec)
    return (-R.T @ tvec.reshape(3, 1)).reshape(3)


def reprojection_errors(
    obj: np.ndarray,
    img: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
) -> np.ndarray:
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    proj2 = proj.reshape(-1, 2)
    return np.linalg.norm(proj2 - img.reshape(-1, 2), axis=1)


def measured_heights(meta: dict[str, Any]) -> dict[str, float]:
    return {
        str(k): float(v)
        for k, v in meta.get("camera_heights_m", meta.get("heights_m", {})).items()
    }


def measured_baselines(meta: dict[str, Any]) -> list[tuple[str, str, float]]:
    raw = meta.get("baselines_m", [])
    out: list[tuple[str, str, float]] = []
    if isinstance(raw, dict):
        for key, val in raw.items():
            if "-" in key:
                a, b = key.split("-", 1)
                out.append((a.strip(), b.strip(), float(val)))
    elif isinstance(raw, list):
        for item in raw:
            out.append((str(item["a"]), str(item["b"]), float(item["distance_m"])))
    return out


def choose_pnp_solution(
    candidates: list[tuple[np.ndarray, np.ndarray]],
    obj: np.ndarray,
    img: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    measured_height: float | None,
) -> tuple[np.ndarray, np.ndarray]:
    scored = []
    for rvec, tvec in candidates:
        center = camera_center_from_rt(rvec, tvec)
        err = reprojection_errors(obj, img, rvec, tvec, K, dist)
        height_penalty = 0.0
        if measured_height is not None:
            height_penalty = abs(float(center[2]) - measured_height) * 25.0
        if center[2] < -0.05:
            height_penalty += 1000.0
        scored.append((float(np.mean(err)) + height_penalty, rvec, tvec))
    scored.sort(key=lambda x: x[0])
    return scored[0][1], scored[0][2]


def solve_camera_pose(
    cam_id: str,
    annotations: dict[str, Any],
    world_points: list[dict[str, Any]],
    intr: dict[str, Any],
    height_hint: float | None,
    grid_ids: set[str] | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    by_id = {p["id"]: p for p in world_points}
    cam_ann = annotations.get("cameras", {}).get(cam_id, {})
    obj_pts = []
    img_pts = []
    used_ids = []
    for pid, xy in cam_ann.items():
        if pid not in by_id or xy is None:
            continue
        if not isinstance(xy, list) or len(xy) < 2:
            continue
        p = by_id[pid]
        obj_pts.append([p["x"], p["y"], p["z"]])
        img_pts.append([float(xy[0]), float(xy[1])])
        used_ids.append(pid)
    if len(obj_pts) < 4:
        raise RuntimeError(f"{cam_id}: need at least 4 annotated points, got {len(obj_pts)}")

    obj = np.asarray(obj_pts, dtype=np.float64).reshape(-1, 3)
    img = np.asarray(img_pts, dtype=np.float64).reshape(-1, 2)
    K = intr["K"]
    dist = intr["dist"].reshape(-1, 1)
    planar = float(np.ptp(obj[:, 2])) < 1e-6
    candidates: list[tuple[np.ndarray, np.ndarray]] = []

    if planar and hasattr(cv2, "SOLVEPNP_IPPE"):
        ok, rvecs, tvecs, _errs = cv2.solvePnPGeneric(
            obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE
        )
        if ok:
            for rvec, tvec in zip(rvecs, tvecs):
                candidates.append((np.asarray(rvec, dtype=np.float64), np.asarray(tvec, dtype=np.float64)))

    if not candidates:
        flag = cv2.SOLVEPNP_EPNP if len(obj) >= 6 else cv2.SOLVEPNP_ITERATIVE
        ok, rvec, tvec, _inliers = cv2.solvePnPRansac(
            obj, img, K, dist, flags=flag, reprojectionError=8.0, iterationsCount=100
        )
        if not ok:
            ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise RuntimeError(f"{cam_id}: solvePnP failed")
        candidates.append((np.asarray(rvec, dtype=np.float64), np.asarray(tvec, dtype=np.float64)))

    rvec, tvec = choose_pnp_solution(candidates, obj, img, K, dist, height_hint)
    if hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)

    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    center = camera_center_from_rt(rvec, tvec)
    errors = reprojection_errors(obj, img, rvec, tvec, K, dist)
    quality = {
        "used_points": len(used_ids),
        "used_grid_points": sum(1 for pid in used_ids if grid_ids and pid in grid_ids),
        "reproj_rms_px": float(math.sqrt(np.mean(np.square(errors)))),
        "reproj_mean_px": float(np.mean(errors)),
        "reproj_p95_px": float(np.percentile(errors, 95)),
        "reproj_max_px": float(np.max(errors)),
        "estimated_height_m": float(center[2]),
    }
    if height_hint is not None:
        quality["measured_height_m"] = float(height_hint)
        quality["height_error_m"] = float(center[2] - height_hint)
    extrinsic = {
        "method": "measured_floor_points_pnp",
        "T_cw": T,
        "camera_center_w": center.reshape(1, 3),
        "rvec": rvec.reshape(3, 1),
        "tvec": tvec.reshape(3, 1),
    }
    return extrinsic, quality


def draw_overlay(
    image_path: Path,
    out_path: Path,
    annotations: dict[str, Any],
    world_points: list[dict[str, Any]],
    intr: dict[str, Any],
    extrinsic: dict[str, Any],
    floor_grid: dict[str, Any] | None = None,
) -> None:
    img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if img is None:
        return
    obj = np.asarray([[p["x"], p["y"], p["z"]] for p in world_points], dtype=np.float64)
    ids = [p["id"] for p in world_points]
    rvec = np.asarray(extrinsic["rvec"], dtype=np.float64).reshape(3, 1)
    tvec = np.asarray(extrinsic["tvec"], dtype=np.float64).reshape(3, 1)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, intr["K"], intr["dist"].reshape(-1, 1))
    proj = proj.reshape(-1, 2)
    proj_by_id = {pid: xy for pid, xy in zip(ids, proj)}
    cam_ann = annotations.get("cameras", {}).get(image_path.stem, {})
    if floor_grid and "point_ids" in floor_grid:
        grid_rows = floor_grid["point_ids"]
        for row in grid_rows:
            for a, b in zip(row, row[1:]):
                if a in cam_ann and b in cam_ann and cam_ann[a] is not None and cam_ann[b] is not None:
                    pa = cam_ann[a]
                    pb = cam_ann[b]
                    cv2.line(img, (int(pa[0]), int(pa[1])), (int(pb[0]), int(pb[1])),
                             (0, 180, 255), 1, cv2.LINE_AA)
                if a in proj_by_id and b in proj_by_id:
                    pa = proj_by_id[a]
                    pb = proj_by_id[b]
                    cv2.line(img, (int(pa[0]), int(pa[1])), (int(pb[0]), int(pb[1])),
                             (0, 0, 255), 1, cv2.LINE_AA)
        for col in range(len(grid_rows[0]) if grid_rows else 0):
            col_ids = [row[col] for row in grid_rows if col < len(row)]
            for a, b in zip(col_ids, col_ids[1:]):
                if a in cam_ann and b in cam_ann and cam_ann[a] is not None and cam_ann[b] is not None:
                    pa = cam_ann[a]
                    pb = cam_ann[b]
                    cv2.line(img, (int(pa[0]), int(pa[1])), (int(pb[0]), int(pb[1])),
                             (0, 180, 255), 1, cv2.LINE_AA)
                if a in proj_by_id and b in proj_by_id:
                    pa = proj_by_id[a]
                    pb = proj_by_id[b]
                    cv2.line(img, (int(pa[0]), int(pa[1])), (int(pb[0]), int(pb[1])),
                             (0, 0, 255), 1, cv2.LINE_AA)
    for pid, xy in cam_ann.items():
        cv2.circle(img, (int(round(xy[0])), int(round(xy[1]))), 5, (0, 220, 255), 2, cv2.LINE_AA)
        cv2.putText(img, pid, (int(xy[0]) + 6, int(xy[1]) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1, cv2.LINE_AA)
    for pid, xy in zip(ids, proj):
        cv2.drawMarker(img, (int(round(xy[0])), int(round(xy[1]))), (0, 0, 255),
                       cv2.MARKER_CROSS, 16, 2, cv2.LINE_AA)
        cv2.putText(img, pid, (int(xy[0]) + 6, int(xy[1]) + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), img)


class CalibrationSession:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.out_dir = Path(args.output_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.world_points, self.measure_meta = load_world_points(Path(args.world_points))
        self.cam_ids = []
        for idx, item in enumerate(args.image or []):
            cam_id, _path = parse_kv(item, f"cam{idx}")
            self.cam_ids.append(cam_id)
        offset = len(self.cam_ids)
        for idx, item in enumerate(args.cam or []):
            cam_id, _device = parse_kv(item, f"cam{idx + offset}")
            self.cam_ids.append(cam_id)
        self.runtime_intrinsics = load_intrinsics_yaml(Path(args.intrinsics), self.cam_ids)
        self.capture_intrinsics = load_intrinsics_yaml(
            Path(args.intrinsics),
            self.cam_ids,
            profile="capture_intrinsics",
            fallback_profile="intrinsics",
        )
        self.images = load_or_capture_images(args, self.out_dir, self.capture_intrinsics)
        self.annotations_path = self.out_dir / "annotations.json"
        if not self.annotations_path.exists():
            save_annotations(self.annotations_path, {
                "cameras": {cam_id: {} for cam_id in self.cam_ids},
                "grid_observations": {},
                "points": self.world_points,
                "floor_grid": self.measure_meta.get("floor_grid"),
            })
        self.last_quality: dict[str, Any] = {}
        self.grid_ids = {
            pid
            for row in self.measure_meta.get("floor_grid", {}).get("point_ids", [])
            for pid in row
        }

    def session_payload(self) -> dict[str, Any]:
        return {
            "cameras": [
                {
                    "id": cam_id,
                    "image_url": f"/calib_images/{self.images[cam_id].name}",
                    "width": int(self.capture_intrinsics[cam_id]["width"]),
                    "height": int(self.capture_intrinsics[cam_id]["height"]),
                }
                for cam_id in self.cam_ids
            ],
            "points": self.world_points,
            "floor_grid": self.measure_meta.get("floor_grid"),
            "annotations": load_annotations(self.annotations_path),
            "quality": self.last_quality,
        }

    def solve(self) -> dict[str, Any]:
        annotations = load_annotations(self.annotations_path)
        heights = measured_heights(self.measure_meta)
        extrinsics: dict[str, dict[str, Any]] = {}
        quality: dict[str, Any] = {}
        for cam_id in self.cam_ids:
            extrinsics[cam_id], quality[cam_id] = solve_camera_pose(
                cam_id,
                annotations,
                self.world_points,
                self.capture_intrinsics[cam_id],
                heights.get(cam_id),
                self.grid_ids,
            )
        centers = {
            cam_id: np.asarray(data["camera_center_w"], dtype=np.float64).reshape(3)
            for cam_id, data in extrinsics.items()
        }
        for a, b, measured in measured_baselines(self.measure_meta):
            if a in centers and b in centers:
                estimated = float(np.linalg.norm(centers[a] - centers[b]))
                quality[f"baseline_{a}_{b}"] = {
                    "measured_m": measured,
                    "estimated_m": estimated,
                    "error_m": estimated - measured,
                }
        out_yaml = self.out_dir / "cam_params.yaml"
        write_calibration_yaml(
            out_yaml,
            intrinsics=self.runtime_intrinsics,
            capture_intrinsics=self.capture_intrinsics,
            extrinsics=extrinsics,
            quality=quality,
            metadata={
                "tool": "measure_extrinsics_web.py",
                "world_points": self.world_points,
                "measurements": self.measure_meta,
            },
        )
        overlay_dir = self.out_dir / "overlays"
        for cam_id, image_path in self.images.items():
            draw_overlay(
                image_path, overlay_dir / f"{cam_id}_reprojection.jpg",
                annotations,
                self.world_points,
                self.capture_intrinsics[cam_id],
                extrinsics[cam_id],
                self.measure_meta.get("floor_grid"),
            )
        quality_path = self.out_dir / "quality.json"
        quality_path.write_text(json.dumps(quality, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.last_quality = quality
        return {"ok": True, "calibration": str(out_yaml), "quality": quality}


def build_app(session: CalibrationSession):
    from fastapi import FastAPI, HTTPException
    from fastapi.staticfiles import StaticFiles

    app = FastAPI()

    @app.get("/api/session")
    async def get_session():
        return session.session_payload()

    @app.post("/api/annotations")
    async def post_annotations(payload: dict[str, Any]):
        if "cameras" not in payload:
            raise HTTPException(status_code=400, detail="payload must contain cameras")
        save_annotations(session.annotations_path, payload)
        return {"ok": True}

    @app.post("/api/solve")
    async def post_solve():
        try:
            return session.solve()
        except Exception as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    app.mount("/calib_images", StaticFiles(directory=str(session.out_dir / "images")), name="calib_images")
    app.mount("/", StaticFiles(directory=str(WEB_DIR), html=True), name="static")
    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Measure-based multi-camera extrinsic calibration")
    parser.add_argument("--intrinsics", required=True, help="YAML produced by calibrate_intrinsics_charuco.py")
    parser.add_argument("--world-points", required=True,
                        help="JSON with points: [{id,x,y,z?}] and optional camera_heights_m/baselines_m")
    parser.add_argument("--image", action="append",
                        help="existing still image, or cam_id=path; repeat per camera")
    parser.add_argument("--cam", action="append",
                        help="V4L2 camera path, or cam_id=/dev/v4l/by-path/...; repeat per camera")
    parser.add_argument("--output-dir", default="calibrations/measure_session")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8010)
    parser.add_argument("--solve-now", action="store_true",
                        help="solve from existing annotations.json without serving the browser UI")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session = CalibrationSession(args)
    if args.solve_now:
        result = session.solve()
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    import uvicorn

    print(f"[ready] http://{args.host}:{args.port}/", file=sys.stderr)
    print(f"[data] annotations: {session.annotations_path}", file=sys.stderr)
    uvicorn.run(build_app(session), host=args.host, port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    sys.exit(main())
