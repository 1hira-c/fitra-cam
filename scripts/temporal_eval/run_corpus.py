#!/usr/bin/env python3
"""Measured-time raw three-camera corpus -> raw 3D -> fixed postprocessing replay."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import subprocess
import sys

from evaluate import read_jsonl, verify_input_binding, percentile
from inspect_corpus import load_session, group_frames, summary
from run_recording import artifact, write_json, apply_labels


def make_input(raw_header, raw_rows, groups, height, hz, session_id, provenance, labels=None):
    if (raw_header.get("schema") != "fitra_corpus_raw3d_v1" or raw_header.get("postprocessing") is not False
            or raw_header.get("kp_format") != "halpe26" or len(raw_rows) != len(groups) or not groups):
        raise ValueError("wrong raw stage/topology or incomplete inference output")
    if not math.isfinite(height) or not 1 <= height <= 2.5 or not math.isfinite(hz) or not 5 <= hz <= 240:
        raise ValueError("invalid height/cadence")
    origin = int(groups[0]["capture_max_ns"])
    rows = []
    for raw, group in zip(raw_rows, groups):
        if (type(raw["seq"]) is not int or type(raw["t_ns"]) is not int
                or raw["seq"] != group["seq"] or raw["t_ns"] != int(group["capture_max_ns"])):
            raise ValueError("raw output/group lineage mismatch")
        if type(raw["ambiguous_detection"]) is not bool or len(raw["joints"]) != 26:
            raise ValueError("malformed raw output")
        for joint in raw["joints"]:
            if joint is not None and (len(joint) != 4 or not all(type(v) in (int, float) and math.isfinite(v) for v in joint)
                                      or joint[3] < 0):
                raise ValueError("malformed raw joint")
        present = any(j is not None for j in raw["joints"])
        if raw["ambiguous_detection"] and present:
            raise ValueError("ambiguous group must not emit a person")
        rows.append({"seq": raw["seq"], "t_ns": raw["t_ns"]-origin, "segment": session_id,
                     "state": "fresh" if present else "boundary", "phase": "unlabelled",
                     "joints": raw["joints"] if present else None, "reference_joints": None,
                     "capture_max_ns": str(raw["t_ns"]), "source_skew_ms": group["skew_ms"]})
    if any(b["t_ns"] <= a["t_ns"] or b["seq"] <= a["seq"] for a,b in zip(rows, rows[1:])):
        raise ValueError("nonmonotonic measured sequence")
    if labels is not None:
        apply_labels(rows, labels)
    header = {"schema": "fitra_temporal_input_v1", "input_stage": "raw_triangulated",
              "kp_format": "halpe26", "coordinate_frame": "fitra_world_z_up_m",
              "nominal_hz": hz, "subject_height_m": height, "reference_kind": "none",
              "time_basis": "measured_capture_group", "capture_origin_ns": str(origin),
              "provenance": provenance}
    return header, rows


def raw_quality(rows):
    joints = []
    for k in range(26):
        available = sum(r["joints"][k] is not None for r in rows)
        quality = [r["joint_quality"][k] for r in rows if r["joint_quality"][k] is not None]
        joints.append({"halpe_index": k, "available": available, "reports": len(rows),
                       "coverage": available/len(rows), "synthetic_direction_endpoint": k == 0,
                       "inlier_view_counts": dict(Counter(q["views"] for q in quality)),
                       "reprojection_p50_px": percentile([q["reproj_px"] for q in quality], .5),
                       "reprojection_p95_px": percentile([q["reproj_px"] for q in quality], .95),
                       "ray_angle_p05_deg": percentile([q["ray_angle_deg"] for q in quality], .05)})
    return {"schema": "fitra_corpus_raw_quality_v1", "reports": len(rows),
            "ambiguous_groups": sum(r["ambiguous_detection"] for r in rows),
            "groups_with_body_3d": sum(r["valid_joints"] > 0 for r in rows),
            "per_joint": joints, "limitations": [
                "Reprojection is internal consistency, not measured 3D accuracy or calibration acceptance.",
                "Coverage includes entry/exit/empty views; no pose labels inferred from output stillness.",
                "A single person is assumed; multiple detections in any view invalidate the entire group."]}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("session", type=Path)
    p.add_argument("output", type=Path)
    for name in ("infer-binary", "replay-binary", "calib", "det-engine", "pose-engine", "subject-profile", "capture-run"):
        p.add_argument("--"+name, type=Path, required=True)
    p.add_argument("--height", type=float, default=1.7)
    p.add_argument("--window-ms", type=float, default=15)
    p.add_argument("--max-groups", type=int, default=0)
    p.add_argument("--labels", type=Path)
    args = p.parse_args()
    if args.output.exists() or args.max_groups < 0 or not math.isfinite(args.height) or not 1 <= args.height <= 2.5:
        p.error("invalid height/group limit or existing output")
    m, frames, source = load_session(args.session)
    if len(frames) != 3 or any(c["raw_encoding"] != "jpeg" for c in m["cameras"]):
        p.error("three-camera JPEG corpus required")
    groups, semantics = group_frames(frames, args.window_ms)
    qualification = summary(m, frames, groups, semantics, args.window_ms)
    groups = groups[:args.max_groups] if args.max_groups else groups
    if not groups:
        p.error("no measured-time groups available")
    assets = {name: artifact(getattr(args, name)) for name in
              ("infer_binary", "replay_binary", "calib", "det_engine", "pose_engine", "subject_profile")}
    capture = json.loads((args.capture_run / "capture.json").read_text())
    stopped = json.loads((args.capture_run / "stopped.json").read_text())
    if (str(args.session.resolve()) not in stopped["ready_sessions"] or stopped["returncode"] != 0
            or artifact(args.capture_run / "capture.json") != stopped["capture_manifest"]):
        p.error("session/capture provenance mismatch")
    # Alternative model engines are allowed, but geometry/profile must match capture provenance.
    for name in ("calib", "subject_profile"):
        if assets[name] not in capture["assets"].values():
            p.error("calibration/profile was not bound to this capture run")
    assets["capture_manifest"] = artifact(args.capture_run / "capture.json")
    assets["capture_stop"] = artifact(args.capture_run / "stopped.json")
    labels = json.loads(args.labels.read_text()) if args.labels else None
    if args.labels:
        assets["labels"] = artifact(args.labels)
        apply_labels([{"seq": groups[-1]["seq"]}], labels)
    out = args.output.resolve()
    infer = [assets["infer_binary"]["path"], str(out/"plan.jsonl"), assets["calib"]["path"],
             assets["det_engine"]["path"], assets["pose_engine"]["path"], str(out/"raw.jsonl")]
    replay = [assets["replay_binary"]["path"], str(out/"input.jsonl"), str(out/"replay.jsonl"), assets["subject_profile"]["path"]]
    evaluate = [sys.executable, str(Path(__file__).with_name("evaluate.py").resolve()), str(out/"input.jsonl"),
                str(out/"replay.jsonl"), str(out/"report.json"), "--binary", assets["replay_binary"]["path"],
                "--subject-profile", assets["subject_profile"]["path"]]
    scripts = {path.name: artifact(path) for path in Path(__file__).parent.glob("*.py")}
    manifest = {"schema": "fitra_corpus_evaluation_run_v1", "source": source, "qualification": qualification,
                "selected_groups": len(groups), "assets": assets, "scripts": scripts, "labels": labels,
                "commands": [infer, replay, evaluate], "height": args.height,
                "identity_assumption": "single-person task; ambiguous detections invalidate group",
                "time_basis": "measured_capture_group", "capture_semantics": semantics,
                "live_schedule_emulated": False, "reference_kind": "none"}
    out.mkdir(parents=True, exist_ok=False)
    write_json(out/"manifest.json", manifest)
    with (out/"plan.jsonl").open("x") as f:
        f.write(json.dumps({"schema": "fitra_corpus_inference_plan_v1", "cameras": m["cameras"],
                            "time_basis": semantics, "window_ns": round(args.window_ms*1e6)})+"\n")
        for g in groups:
            f.write(json.dumps(g, allow_nan=False)+"\n")
    with (out/"infer.log").open("x") as log:
        subprocess.run(infer, stdout=log, stderr=subprocess.STDOUT, check=True)
    rh, raw = read_jsonl(out/"raw.jsonl")
    verify_input_binding(out/"plan.jsonl", {"input_fnv1a64": rh["plan_fnv1a64"]})
    provenance = {"run_manifest": artifact(out/"manifest.json"), "raw": artifact(out/"raw.jsonl"),
                  "plan": artifact(out/"plan.jsonl"), "capture_semantics": semantics,
                  "camera_sync": "measured_software_grouping_not_hardware_sync"}
    header, rows = make_input(rh, raw, groups, args.height, m["cameras"][0]["requested_fps"], m["session_id"], provenance, labels)
    with (out/"input.jsonl").open("x") as f:
        for r in [header, *rows]:
            f.write(json.dumps(r, allow_nan=False)+"\n")
    write_json(out/"raw-quality.json", raw_quality(raw))
    with (out/"replay.log").open("x") as log:
        subprocess.run(replay, stdout=log, stderr=subprocess.STDOUT, check=True)
    with (out/"evaluate.log").open("x") as log:
        subprocess.run(evaluate, stdout=log, stderr=subprocess.STDOUT, check=True)
    check = [*assets.values(), *scripts.values(), source["manifest"]]
    for segment in source["segments"]:
        check += [segment["checksums"], *segment["files"].values()]
    if any(artifact(item["path"]) != item for item in check):
        raise ValueError("source changed during run; do not use as completed evaluation")
    write_json(out/"complete.json", {"groups": len(rows), "artifacts": {name: artifact(out/name) for name in
               ("manifest.json", "plan.jsonl", "raw.jsonl", "raw-quality.json", "input.jsonl", "replay.jsonl", "report.json", "report.md")}})
    print(out/"report.md")


if __name__ == "__main__":
    main()
