"""Extract a verified triplet from the archived capture; mask fiducials in either input type."""

import argparse
import hashlib
import json
from pathlib import Path

import cv2 as cv
import numpy as np


def sha(data):
    return hashlib.sha256(data).hexdigest()


def archive(source, group, dest):
    manifest = json.loads((source / "manifest.json").read_text())
    complete = json.loads((source / "complete.json").read_text())
    records = {}
    for name in ("plan.jsonl", "raw.jsonl"):
        data = (source / name).read_bytes()
        assert sha(data) == complete["artifacts"][name]["sha256"]
        records[name] = json.loads(data.splitlines()[group + 1])
    plan, raw = records["plan.jsonl"], records["raw.jsonl"]
    assert plan["seq"] == raw["seq"] and raw["detections"] == [0, 0, 0]
    assert int(plan["capture_max_ns"]) == raw["t_ns"]
    segments = {Path(s["directory"]).name: s for s in manifest["source"]["segments"]}
    info = {"group": group, "detected_people": raw["detections"], "frames": []}
    for i, row in enumerate(plan["frames"]):
        assert i == row["camera"]
        seg = segments[row["source_segment"]]
        payload = Path(row["payload_path"])
        assert payload == Path(seg["directory"]) / row["payload"]
        full = payload.read_bytes()
        assert sha(full) == seg["files"][row["payload"]]["sha256"]
        data = full[row["offset"] : row["offset"] + row["size"]]
        assert len(data) == row["size"]
        (dest / f"raw{i}.jpg").write_bytes(data)
        info["frames"].append(
            {
                "camera": i,
                "seq": row["seq"],
                "jpeg_sha256": sha(data),
                "capture_ns": str(row["v4l2_mono_ns"]),
            }
        )
    return info


def prepare(dest, extra):
    fields, preview = [], []
    for i in range(3):
        image = cv.imread(str(dest / f"raw{i}.jpg"))
        assert image is not None and image.shape == (960, 1280, 3)
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        targets, mask = {}, np.zeros(gray.shape, np.uint8)
        # Floor layout and tag lengths are unknown. IDs only identify withheld pixel observations.
        for dictionary, label in [
            (cv.aruco.DICT_APRILTAG_36h11, "36h11"),
            (cv.aruco.DICT_4X4_50, "4x4"),
        ]:
            prm = cv.aruco.DetectorParameters_create()
            prm.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
            prm.minMarkerPerimeterRate = 0.015
            for scale in (1.0, 0.75, 1.5):
                reduced = (
                    cv.resize(gray, None, fx=scale, fy=scale) if scale != 1.0 else gray
                )
                corners, ids, _ = cv.aruco.detectMarkers(
                    reduced,
                    cv.aruco.getPredefinedDictionary(dictionary),
                    parameters=prm,
                )
                if ids is None:
                    continue
                for k, corner in zip(ids.ravel(), corners):
                    xy = (corner.reshape(4, 2) + 0.5) / scale - 0.5
                    center = xy.mean(axis=0)
                    cv.fillConvexPoly(
                        mask, np.int32(np.round(center + 2.15 * (xy - center))), 255
                    )
                    targets.setdefault(f"{label}/{k}", xy.tolist())
        for cx, cy, rx, ry in extra.get(sha((dest / f"raw{i}.jpg").read_bytes()), []):
            cv.rectangle(mask, (cx - rx, cy - ry), (cx + rx, cy + ry), 255, cv.FILLED)
        # Remove the tag and paper border from the model input as well as excluding endpoints.
        masked = cv.inpaint(image, mask, 5, cv.INPAINT_TELEA)
        assert cv.imwrite(str(dest / f"input{i}.png"), masked)
        assert cv.imwrite(
            str(dest / f"valid{i}.png"),
            cv.bitwise_not(cv.dilate(mask, np.ones((13, 13), np.uint8))),
        )
        marked = image.copy()
        marked[mask > 0] = (20, 40, 255)
        preview.append(cv.resize(marked, (448, 336)))
        fields.append(
            {
                "image_sha256": sha((dest / f"raw{i}.jpg").read_bytes()),
                "excluded_fraction": float(np.mean(mask > 0)),
                "tags": targets,
            }
        )
    cv.imwrite(str(dest / "check.jpg"), np.hstack(preview))
    return fields


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("dest", type=Path)
    p.add_argument("--source", type=Path)
    p.add_argument("--group", type=int, default=2100)
    p.add_argument("--extra-mask", type=Path)
    args = p.parse_args()
    args.dest.mkdir(exist_ok=True)
    info = archive(args.source, args.group, args.dest) if args.source else {}
    extra = json.loads(args.extra_mask.read_text()) if args.extra_mask else {}
    info["views"] = prepare(args.dest, extra)
    (args.dest / "input.json").write_text(json.dumps(info, indent=2) + "\n")
    print(
        [
            (i, sorted(v["tags"]), round(v["excluded_fraction"], 4))
            for i, v in enumerate(info["views"])
        ]
    )
