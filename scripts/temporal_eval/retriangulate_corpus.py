#!/usr/bin/env python3
"""Derived offline run using the same saved 2D observations and a traceable calibration proposal."""
import argparse
import copy
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

from calibrate_corpus import verified_calibration
from evaluate import read_jsonl, verify_input_binding
from pseudo_reference import completed_run
from run_corpus import make_input, raw_quality
from run_recording import artifact, write_json


def merge_geometry(original, geometry):
    if len(original) != len(geometry) or not original:
        raise ValueError("retriangulation row count mismatch")
    result = []
    for source, derived in zip(original, geometry):
        if set(derived) != {"seq", "t_ns", "joints", "joint_quality", "median_reproj_px", "valid_joints"}:
            raise ValueError("unexpected retriangulation fields")
        if (type(derived["seq"]) is not int or type(derived["t_ns"]) is not int
                or (source["seq"], source["t_ns"]) != (derived["seq"], derived["t_ns"])):
            raise ValueError("retriangulation lineage mismatch")
        if source["ambiguous_detection"] and any(j is not None for j in derived["joints"]):
            raise ValueError("ambiguous detection boundary lost")
        result.append({**source, **derived})
    return result


def verify_control(original, geometry):
    merged = merge_geometry(original, geometry)
    maximum = {"position_m": 0., "score": 0., "reprojection_px": 0., "ray_angle_deg": 0.}
    def difference(name, a, b):
        if type(a) not in (int, float) or type(b) not in (int, float) or not math.isfinite(a) or not math.isfinite(b):
            raise ValueError("nonfinite control result")
        maximum[name] = max(maximum[name], abs(a-b))
    for before, after in zip(original, merged):
        if before["valid_joints"] != after["valid_joints"]:
            raise ValueError("control valid count mismatch")
        if (before["median_reproj_px"] is None) != (after["median_reproj_px"] is None):
            raise ValueError("control median availability mismatch")
        if before["median_reproj_px"] is not None:
            difference("reprojection_px", before["median_reproj_px"], after["median_reproj_px"])
        if any(len(r[name]) != 26 for r in (before, after) for name in ("joints", "joint_quality")):
            raise ValueError("control topology mismatch")
        for a, b in zip(before["joints"], after["joints"]):
            if (a is None) != (b is None):
                raise ValueError("control joint availability mismatch")
            if a is not None:
                for i in range(3): difference("position_m", a[i], b[i])
                difference("score", a[3], b[3])
        for a, b in zip(before["joint_quality"], after["joint_quality"]):
            if (a is None) != (b is None) or (a is not None and a["views"] != b["views"]):
                raise ValueError("control geometry support mismatch")
            if a is not None:
                difference("reprojection_px", a["reproj_px"], b["reproj_px"])
                difference("ray_angle_deg", a["ray_angle_deg"], b["ray_angle_deg"])
    tolerances = {"position_m": 1e-5, "score": 1e-6, "reprojection_px": 1e-4, "ray_angle_deg": 1e-3}
    if any(maximum[k] > tolerances[k] for k in maximum):
        raise ValueError(f"same-calibration control does not reproduce original 3D: {maximum}")
    return {"passed": True, "maximum_absolute_difference": maximum, "tolerances": tolerances}


def validate_derivation(source_manifest, source_done, calibration_manifest, report):
    if "calibration_derivation" in source_manifest:
        raise ValueError("derive from an original run, not a previous correction")
    if (source_manifest["assets"]["calib"] != calibration_manifest["assets"]["source_calib"]
            or source_done["artifacts"]["plan.jsonl"]["sha256"] != calibration_manifest["source_plan"]["sha256"]
            or source_manifest["assets"]["capture_manifest"] != calibration_manifest["source_capture"]):
        raise ValueError("calibration proposal belongs to another capture or geometry")
    if report["updated_cameras"] < 1:
        raise ValueError("no camera passed calibration proposal validation")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("source_run", type=Path)
    p.add_argument("calibration_run", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--geometry-binary", type=Path, required=True)
    args = p.parse_args()
    if args.output.exists(): p.error("choose a new output directory")
    sm, sd, source = completed_run(args.source_run)
    cm, cd, cr = verified_calibration(args.calibration_run)
    validate_derivation(sm, sd, cm, cr)
    rh, _ = read_jsonl(args.source_run/"raw.jsonl")
    _, groups = read_jsonl(args.source_run/"plan.jsonl")
    ih, old_input = read_jsonl(args.source_run/"input.jsonl")
    binary = artifact(args.geometry_binary)
    assets = {**sm["assets"], "calib": cd["artifacts"]["geometry/candidate.yaml"], "geometry_binary": binary}
    for name in ("calib", "replay_binary", "subject_profile"):
        if artifact(assets[name]["path"]) != assets[name]: raise ValueError("replay/calibration asset changed")
    scripts = {path.name: artifact(path) for path in Path(__file__).parent.glob("*.py")}
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(args.source_run/"plan.jsonl", out/"plan.jsonl")
    derivation = {"source_complete": artifact(args.source_run/"complete.json"),
                  "calibration_complete": artifact(args.calibration_run/"complete.json"),
                  "original_calib": sm["assets"]["calib"], "geometry_binary": binary,
                  "observation_payload_unchanged": True, "runtime_applied": False}
    manifest = {**sm, "assets": assets, "scripts": scripts, "calibration_derivation": derivation,
                "commands": [], "reference_kind": "none"}
    for stage, calib in (("control", sm["assets"]["calib"]), ("derived", assets["calib"])):
        if artifact(calib["path"]) != calib: raise ValueError("calibration changed")
        command = [binary["path"], "triangulate", str(out/"plan.jsonl"), str((args.source_run/"raw.jsonl").resolve()),
                   calib["path"], str(out/(stage+".jsonl"))]
        manifest["commands"].append(command)
        with (out/(stage+".log")).open("x") as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        header, generated = read_jsonl(out/(stage+".jsonl"))
        if header["schema"] != "fitra_corpus_retriangulation_v1": raise ValueError("wrong geometry output")
        verify_input_binding(out/"plan.jsonl", {"input_fnv1a64": header["plan_fnv1a64"]})
        verify_input_binding(args.source_run/"raw.jsonl", {"input_fnv1a64": header["source_fnv1a64"]})
        if stage == "control": write_json(out/"control-diff.json", verify_control(source, generated))
        else: raw = merge_geometry(source, generated)
    rh = {**rh, "calibration_derivation": derivation}
    with (out/"raw.jsonl").open("x") as f:
        for row in [rh, *raw]: f.write(json.dumps(row, allow_nan=False)+"\n")
    replay = [assets["replay_binary"]["path"], str(out/"input.jsonl"), str(out/"replay.jsonl"), assets["subject_profile"]["path"]]
    evaluate = [sys.executable, str(Path(__file__).with_name("evaluate.py")), str(out/"input.jsonl"),
                str(out/"replay.jsonl"), str(out/"report.json"), "--binary", assets["replay_binary"]["path"],
                "--subject-profile", assets["subject_profile"]["path"]]
    manifest["commands"] += [replay, evaluate]
    write_json(out/"manifest.json", manifest)
    provenance = {"run_manifest": artifact(out/"manifest.json"), "raw": artifact(out/"raw.jsonl"),
                  "plan": artifact(out/"plan.jsonl"), "calibration_derivation": derivation,
                  "camera_sync": "measured_software_grouping_not_hardware_sync"}
    h, rows = make_input(rh, raw, groups, ih["subject_height_m"], ih["nominal_hz"], old_input[0]["segment"], provenance, sm["labels"])
    with (out/"input.jsonl").open("x") as f:
        for row in [h, *rows]: f.write(json.dumps(row, allow_nan=False)+"\n")
    write_json(out/"raw-quality.json", raw_quality(raw))
    for name, command in (("replay", replay), ("evaluate", evaluate)):
        with (out/(name+".log")).open("x") as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    completed_run(args.source_run)
    verified_calibration(args.calibration_run)
    for record in [binary, *scripts.values(), assets["replay_binary"], assets["subject_profile"],
                   derivation["source_complete"], derivation["calibration_complete"], derivation["original_calib"]]:
        if artifact(record["path"]) != record: raise ValueError("derived run inputs/code changed")
    write_json(out/"complete.json", {"groups": len(rows), "observation_payload_unchanged": True,
        "artifacts": {name: artifact(out/name) for name in
            ("manifest.json", "plan.jsonl", "raw.jsonl", "raw-quality.json", "input.jsonl", "replay.jsonl", "report.json", "report.md",
             "control.jsonl", "control-diff.json", "derived.jsonl")}})
    print(out/"report.md")


if __name__ == "__main__":
    main()
