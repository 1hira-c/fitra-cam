"""Pack a RoMa relative-geometry run and its original ChArUco intrinsics for markerless_hybrid."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def export(run, intrinsics, output):
    estimate = json.loads((run / "roma-geometry.json").read_text())
    if estimate.get("status") != "relative_candidate" or len(estimate.get("relative_poses", [])) != 3:
        raise ValueError("expected a successful three-camera relative solution")
    with np.load(run / "roma-pairs.npz", allow_pickle=False) as arrays:
        tracks = np.asarray(arrays["tracks"], dtype=np.float64)
    if tracks.ndim != 3 or tracks.shape[1:] != (3, 2) or not np.isfinite(tracks).all():
        raise ValueError("expected native image coordinates with shape (tracks, cameras, xy)")
    lens_file = cv2.FileStorage(str(intrinsics), cv2.FILE_STORAGE_READ)
    if not lens_file.isOpened():
        raise ValueError("cannot read the original intrinsics")
    try:
        data = cv2.FileStorage("background.yml", cv2.FILE_STORAGE_WRITE | cv2.FILE_STORAGE_MEMORY)
        data.write("schema", "fitra_markerless_background_v1")
        for i, pose in enumerate(estimate["relative_poses"]):
            camera_id = f"cam{i}"
            camera = lens_file.getNode("intrinsics").getNode(camera_id)
            if camera.empty():
                raise ValueError(f"missing intrinsics for {camera_id}")
            transform = np.eye(4)
            transform[:3, :3] = pose["R"]
            transform[:3, 3] = pose["t_in_baseline01_units"]
            data.write(f"id{i}", camera_id)
            data.write(f"T{i}", transform)
            data.write(f"K{i}", camera.getNode("K").mat())
            data.write(f"D{i}", camera.getNode("dist").mat())
            data.write(f"lens{i}", camera.getNode("distortion_model").string() or "pinhole")
            for key in ("width", "height"):
                data.write(f"{key}{i}", int(camera.getNode(key).real()))
        data.write("background_tracks", np.ascontiguousarray(tracks.reshape(-1, 6)))
        text = data.releaseAndGetString()
    finally:
        lens_file.release()
    with output.open("x", encoding="utf-8") as stream:
        stream.write(text)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="folder with roma-geometry.json and roma-pairs.npz")
    parser.add_argument("intrinsics", type=Path, help="the same intrinsics file used to solve the relative poses")
    parser.add_argument("new_output", type=Path)
    args = parser.parse_args()
    export(args.run, args.intrinsics, args.new_output)
