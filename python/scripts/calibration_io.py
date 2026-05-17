"""Calibration file helpers shared by the measurement tools.

The project intentionally avoids PyYAML on Jetson.  The writer below emits an
OpenCV FileStorage-compatible YAML subset that both Python cv2 and C++ OpenCV
can read.
"""

from __future__ import annotations

import datetime as dt
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def _matrix_yaml(name: str, mat: np.ndarray, indent: int = 2) -> list[str]:
    arr = np.asarray(mat, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    pad = " " * indent
    data = ", ".join(f"{float(v):.12g}" for v in arr.reshape(-1))
    return [
        f"{pad}{name}: !!opencv-matrix",
        f"{pad}   rows: {arr.shape[0]}",
        f"{pad}   cols: {arr.shape[1]}",
        f"{pad}   dt: d",
        f"{pad}   data: [{data}]",
    ]


def _scalar_yaml(name: str, value: Any, indent: int = 2) -> str:
    pad = " " * indent
    if isinstance(value, bool):
        return f"{pad}{name}: {'true' if value else 'false'}"
    if isinstance(value, (int, np.integer)):
        return f"{pad}{name}: {int(value)}"
    if isinstance(value, (float, np.floating)):
        return f"{pad}{name}: {float(value):.12g}"
    text = str(value).replace("\\", "\\\\").replace('"', '\\"')
    return f'{pad}{name}: "{text}"'


def write_calibration_yaml(
    path: Path,
    *,
    intrinsics: dict[str, dict[str, Any]],
    capture_intrinsics: dict[str, dict[str, Any]] | None = None,
    extrinsics: dict[str, dict[str, Any]] | None = None,
    quality: dict[str, Any] | None = None,
    metadata: dict[str, Any] | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines: list[str] = [
        "%YAML:1.0",
        "---",
        'schema: "fitra_cam_calibration_v1"',
        'unit: "m"',
        'coordinate_system: "world: x/y measured on floor, z up; extrinsics are T_cw"',
        _scalar_yaml("created_at", dt.datetime.now(dt.timezone.utc).isoformat(), 0),
    ]

    def append_intrinsics_map(name: str, items: dict[str, dict[str, Any]]) -> None:
        lines.append(f"{name}:")
        for cam_id, data in items.items():
            lines.append(f"  {cam_id}:")
            lines.append(_scalar_yaml("width", int(data.get("width", 0)), 4))
            lines.append(_scalar_yaml("height", int(data.get("height", 0)), 4))
            lines.append(_scalar_yaml("source", data.get("source", ""), 4))
            lines.append(_scalar_yaml("rms_px", float(data.get("rms_px", 0.0)), 4))
            lines.extend(_matrix_yaml("K", np.asarray(data["K"], dtype=np.float64), 4))
            lines.extend(_matrix_yaml("dist", np.asarray(data["dist"], dtype=np.float64).reshape(1, -1), 4))

    append_intrinsics_map("intrinsics", intrinsics)
    if capture_intrinsics:
        append_intrinsics_map("capture_intrinsics", capture_intrinsics)

    if extrinsics:
        lines.append("extrinsics:")
        for cam_id, data in extrinsics.items():
            lines.append(f"  {cam_id}:")
            lines.append(_scalar_yaml("method", data.get("method", "measure_pnp"), 4))
            lines.extend(_matrix_yaml("T_cw", np.asarray(data["T_cw"], dtype=np.float64), 4))
            if "camera_center_w" in data:
                lines.extend(_matrix_yaml("camera_center_w", np.asarray(data["camera_center_w"], dtype=np.float64).reshape(1, 3), 4))
            if "rvec" in data:
                lines.extend(_matrix_yaml("rvec", np.asarray(data["rvec"], dtype=np.float64).reshape(3, 1), 4))
            if "tvec" in data:
                lines.extend(_matrix_yaml("tvec", np.asarray(data["tvec"], dtype=np.float64).reshape(3, 1), 4))
    else:
        lines.append("extrinsics: {}")

    if quality:
        lines.append("quality:")
        for key, value in quality.items():
            if isinstance(value, dict):
                lines.append(f"  {key}:")
                for sub_key, sub_value in value.items():
                    if isinstance(sub_value, (list, tuple, np.ndarray)):
                        lines.extend(_matrix_yaml(sub_key, np.asarray(sub_value, dtype=np.float64), 4))
                    else:
                        lines.append(_scalar_yaml(sub_key, sub_value, 4))
            else:
                lines.append(_scalar_yaml(key, value, 2))
    else:
        lines.append("quality: {}")

    if metadata:
        lines.append(_scalar_yaml("metadata_json", json.dumps(metadata, ensure_ascii=True, separators=(",", ":")), 0))

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def load_intrinsics_yaml(
    path: Path,
    cam_ids: list[str] | None = None,
    *,
    profile: str = "intrinsics",
    fallback_profile: str | None = None,
) -> dict[str, dict[str, Any]]:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"failed to open calibration YAML: {path}")
    try:
        root = fs.getNode(profile)
        if root.empty() and fallback_profile:
            root = fs.getNode(fallback_profile)
        if root.empty():
            raise RuntimeError(f"{path} does not contain a {profile} map")
        if cam_ids is None:
            cam_ids = []
            for i in range(int(root.size())):
                node = root.at(i)
                name = node.name()
                if name:
                    cam_ids.append(name)
        out: dict[str, dict[str, Any]] = {}
        for cam_id in cam_ids:
            node = root.getNode(cam_id)
            if node.empty():
                raise RuntimeError(f"missing intrinsics for {cam_id} in {path}")
            K = node.getNode("K").mat()
            dist = node.getNode("dist").mat()
            if K is None or K.shape != (3, 3):
                raise RuntimeError(f"invalid K for {cam_id} in {path}")
            if dist is None:
                raise RuntimeError(f"missing dist for {cam_id} in {path}")
            out[cam_id] = {
                "K": np.asarray(K, dtype=np.float64),
                "dist": np.asarray(dist, dtype=np.float64).reshape(-1),
                "width": int(node.getNode("width").real()),
                "height": int(node.getNode("height").real()),
                "source": node.getNode("source").string(),
                "rms_px": float(node.getNode("rms_px").real()),
            }
        return out
    finally:
        fs.release()
