#!/usr/bin/env python3
"""Reproducible two-view legacy-recording pilot, never a live latency benchmark."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

from import_dump import convert


def artifact(path):
    path = Path(path).resolve(strict=True)
    value = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            value.update(chunk)
    return {"path": str(path), "sha256": value.hexdigest()}


def recording_info(directory):
    directory = directory.resolve(strict=True)
    if (directory / "frames.csv").exists():
        raise ValueError("timestamp sidecar present: use a measured-time adapter, not this legacy pilot")
    meta_path = directory / "meta.json"
    meta = json.loads(meta_path.read_text())
    cameras = meta["cameras"]
    # Camera IDs are determined by exact filenames, not arbitrary list order.
    selected = []
    for cam in (0, 1):
        matches = [c for c in cameras if c["file"] in (f"raw_cam{cam}.avi", f"raw_cam{cam}.mp4")]
        if len(matches) != 1:
            raise ValueError(f"ambiguous/missing cam{cam} metadata")
        c = matches[0]
        if type(c["frames"]) is not int or c["frames"] <= 0:
            raise ValueError("invalid frame count")
        hz = c["fps_written"]
        if type(hz) not in (int, float) or not math.isfinite(hz) or not 5 <= hz <= 240:
            raise ValueError("invalid nominal cadence")
        selected.append({"id": f"cam{cam}", **c, "artifact": artifact(directory / c["file"])})
    return {"metadata": artifact(meta_path), "selected_cameras": selected,
            "recorded_camera_count": len(cameras), "nominal_hz": selected[0]["fps_written"],
            "pair_limit": min(c["frames"] for c in selected),
            "pairing": "equal_frame_index_cam0_cam1_not_measured_sync",
            "calibration_association": "operator_selected_not_proven_by_recording_metadata"}


def apply_labels(rows, labels):
    """Labels annotate scoring only; they must not reset filter continuity."""
    if labels.get("schema") != "fitra_temporal_labels_v1" or not labels.get("evidence"):
        raise ValueError("labels need schema and observation evidence")
    intervals = labels["intervals"]
    previous_end = 0
    for interval in intervals:
        start, end = interval["start_frame"], interval["end_frame"]
        if (type(start) is not int or type(end) is not int or start < previous_end
                or end <= start or end > rows[-1]["seq"] + 1):
            raise ValueError("labels must be ordered, nonoverlapping, in-range half-open frame intervals")
        if interval["phase"] not in ("static", "motion", "unlabelled"):
            raise ValueError("invalid phase")
        if type(interval.get("evaluate", True)) is not bool:
            raise ValueError("evaluate must be boolean")
        previous_end = end
    for row in rows:
        for interval in intervals:
            if interval["start_frame"] <= row["seq"] < interval["end_frame"]:
                row["phase"] = interval["phase"]
                row["evaluate"] = interval.get("evaluate", True)
                break


def write_json(path, value):
    with path.open("x") as handle:
        json.dump(value, handle, indent=2, allow_nan=False)
        handle.write("\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("output", type=Path, help="new directory; never resume/overwrite partial runs")
    for flag in ("dump-binary", "replay-binary", "calib", "det-engine", "pose-engine", "subject-profile"):
        parser.add_argument("--" + flag, type=Path, required=True)
    parser.add_argument("--subject-height-m", type=float, required=True)
    parser.add_argument("--max-frames", type=int, default=0, help="0 uses metadata pair limit")
    parser.add_argument("--labels", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.subject_height_m) or not 1 <= args.subject_height_m <= 2.5 or args.max_frames < 0:
        parser.error("invalid height or frame limit")
    if args.output.exists():
        parser.error("output directory already exists; choose a new path")
    info = recording_info(args.recording)
    assets = {k: artifact(getattr(args, k)) for k in
              ("dump_binary", "replay_binary", "calib", "det_engine", "pose_engine", "subject_profile")}
    labels = json.loads(args.labels.read_text()) if args.labels else None
    if args.labels:
        assets["labels"] = artifact(args.labels)
    out = args.output.resolve()
    frame_limit = min(args.max_frames or info["pair_limit"], info["pair_limit"])
    # Validate interval bounds before expensive inference and before making output.
    if labels is not None:
        apply_labels([{"seq": frame_limit - 1}], labels)
    dump = [assets["dump_binary"]["path"]]
    for camera in info["selected_cameras"]:
        dump += ["--video", camera["artifact"]["path"]]
    for flag in ("calib", "det_engine", "pose_engine"):
        dump += ["--" + flag.replace("_", "-"), assets[flag]["path"]]
    dump += ["--keypoint-format", "halpe26", "--no-kalman", "--no-ik",
             "--no-floor-contact-stability", "--max-frames", str(frame_limit),
             "--out", str(out / "raw.jsonl")]
    replay = [assets["replay_binary"]["path"], str(out / "input.jsonl"),
              str(out / "replay.jsonl"), assets["subject_profile"]["path"]]
    evaluate = [sys.executable, str(Path(__file__).with_name("evaluate.py").resolve()),
                str(out / "input.jsonl"), str(out / "replay.jsonl"), str(out / "report.json"),
                "--binary", assets["replay_binary"]["path"],
                "--subject-profile", assets["subject_profile"]["path"]]
    manifest = {"schema": "fitra_recording_pilot_v1", "recording": info, "assets": assets,
                "frame_limit": frame_limit, "commands": [dump, replay, evaluate],
                "scripts": {p.name: artifact(p) for p in Path(__file__).parent.glob("*.py")},
                "reference_kind": "none", "time_basis": "nominal_from_legacy_frames",
                "labels": labels, "subject_height_m": args.subject_height_m,
                "limitations": ["Two-view pilot, not three-camera live reconstruction.",
                    "Equal frame indices do not establish camera synchronization.",
                    "No teacher: tracking error/lag/amplitude accuracy remain unavailable.",
                    "Calibration/profile association is operator-selected, not independently verified."]}
    out.mkdir(parents=True, exist_ok=False)
    write_json(out / "manifest.json", manifest)
    with (out / "dump.log").open("x") as log:
        subprocess.run(dump, stdout=log, stderr=subprocess.STDOUT, check=True)
    header, rows = convert(out / "raw.jsonl", info["nominal_hz"], args.subject_height_m,
                           args.recording.resolve().name, "unlabelled")
    if len(rows) != frame_limit or [r["seq"] for r in rows] != list(range(frame_limit)):
        raise ValueError("unexpected decoded frame count/indices; do not score a partial run")
    if labels is not None:
        apply_labels(rows, labels)
    header["provenance"].update({"pilot_manifest": artifact(out / "manifest.json"),
                                 "recording": info, "assets": assets,
                                 "raw_stage": "generated_with_all_postprocessing_disabled"})
    with (out / "input.jsonl").open("x") as handle:
        for row in [header, *rows]:
            handle.write(json.dumps(row, allow_nan=False) + "\n")
    with (out / "replay.log").open("x") as log:
        subprocess.run(replay, stdout=log, stderr=subprocess.STDOUT, check=True)
    with (out / "evaluate.log").open("x") as log:
        subprocess.run(evaluate, stdout=log, stderr=subprocess.STDOUT, check=True)
    # Detect inputs replaced during a run. Never publish COMPLETE on mismatch.
    for item in [info["metadata"], *(c["artifact"] for c in info["selected_cameras"]),
                 *assets.values(), *manifest["scripts"].values()]:
        if artifact(item["path"]) != item:
            raise ValueError("source changed during run; result is not complete")
    write_json(out / "complete.json", {"reports": len(rows), "manifest": artifact(out / "manifest.json"),
        "artifacts": {name: artifact(out / name) for name in
                      ("raw.jsonl", "raw.jsonl.summary.json", "input.jsonl", "replay.jsonl", "report.json", "report.md")}})
    print(out / "report.md")


if __name__ == "__main__":
    main()
