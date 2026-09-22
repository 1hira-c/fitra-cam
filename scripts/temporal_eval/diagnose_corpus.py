#!/usr/bin/env python3
"""Sparse capture-bound geometry/pose diagnosis, not a temporal evaluation run.

Select image-reviewed windows before inference. Reuse production floor calibration
and triangulation; never alter live calibration or loosen teacher quality gates.
"""
import argparse
import json
from pathlib import Path
import subprocess

from calibrate_corpus import select_samples
from evaluate import percentile, read_jsonl, verify_input_binding
from inspect_corpus import load_session, group_frames
from pseudo_reference import SETTINGS, trusted
from retriangulate_corpus import merge_geometry, verify_control
from run_recording import artifact, write_json


def select_poses(groups, poses):
    if not groups or not poses or len(poses) > 32:
        raise ValueError("expected groups and 1..32 pose windows")
    origin = int(groups[0]["capture_max_ns"])
    selected, names, used = [], set(), set()
    previous_end = 0
    for pose in poses:
        name, start, end = pose["name"], pose["start_second"], pose["end_second"]
        if (not isinstance(name, str) or not name or name in names
                or type(start) is not int or type(end) is not int
                or start < previous_end or not 0 < end-start <= 60):
            raise ValueError("pose windows must be named, ordered, disjoint integer seconds")
        names.add(name)
        previous_end = end
        for second in range(start, end):
            target = origin+second*1_000_000_000
            group = min(groups, key=lambda g: abs(int(g["capture_max_ns"])-target))
            if abs(int(group["capture_max_ns"])-target) > 100_000_000 or group["seq"] in used:
                raise ValueError("missing/out-of-range/reused diagnostic sample")
            used.add(group["seq"])
            selected.append({**group, "diagnostic_case": name})
    return selected


def bind_capture(session, capture, stopped, assets):
    if (str(session.resolve()) not in stopped["ready_sessions"] or stopped["returncode"] != 0
            or assets["capture_manifest"] != stopped["capture_manifest"]):
        raise ValueError("session/capture provenance mismatch")
    for name in ("calib", "floor_map"):
        if assets[name] not in capture["assets"].values():
            raise ValueError(f"{name} was not bound to this capture")


def validate_raw(header, rows, groups):
    if (header.get("schema") != "fitra_corpus_raw3d_v1" or header.get("postprocessing") is not False
            or header.get("kp_format") != "halpe26" or len(rows) != len(groups)):
        raise ValueError("wrong/incomplete raw diagnostic result")
    for row, group in zip(rows, groups):
        if (type(row["seq"]) is not int or type(row["t_ns"]) is not int
                or (row["seq"], row["t_ns"]) != (group["seq"], int(group["capture_max_ns"]))
                or len(row["joints"]) != 26 or len(row["joint_quality"]) != 26
                or type(row["ambiguous_detection"]) is not bool):
            raise ValueError("diagnostic row lineage/topology mismatch")
        if row["ambiguous_detection"] and any(j is not None for j in row["joints"]):
            raise ValueError("ambiguous detection emitted a skeleton")


def pose_quality(rows, groups):
    if len(rows) != len(groups):
        raise ValueError("diagnostic row count mismatch")
    result = []
    for name in dict.fromkeys(g["diagnostic_case"] for g in groups):
        selected = [r for r, g in zip(rows, groups) if g["diagnostic_case"] == name]
        joints = []
        for k in range(26):
            values = [r["joint_quality"][k]["reproj_px"] for r in selected
                      if not r["ambiguous_detection"] and r["joint_quality"][k] is not None]
            passed = sum(not r["ambiguous_detection"] and trusted(r["joints"][k], r["joint_quality"][k], k)
                         for r in selected)
            joints.append({"halpe_index": k, "gate_passed": passed, "samples": len(selected),
                           "reprojection_p50_px": percentile(values, .5),
                           "reprojection_p95_px": percentile(values, .95)})
        result.append({"case": name, "samples": len(selected),
                       "ambiguous": sum(r["ambiguous_detection"] for r in selected), "joints": joints})
    return result


def write_rows(path, header, rows):
    with path.open("x") as handle:
        for row in [header, *rows]:
            handle.write(json.dumps(row, allow_nan=False)+"\n")


def markdown(report):
    def number(value):
        return "-" if value is None else f"{value:.2f}"
    lines = ["# Sparse pose/geometry diagnosis", "",
             "1 Hz image-selected samples; no temporal smoothing, lag, teacher coverage, or accuracy verdict.", "",
             "| Model | Case | Left hip reproj old -> candidate px | Right hip old -> candidate px | Candidate hip gate L / R |",
             "|---|---|---:|---:|---:|"]
    for model in report["models"]:
        for before, after in zip(model["original"], model["candidate"]):
            left, right = after["joints"][11], after["joints"][12]
            values = [f"{number(before['joints'][k]['reprojection_p50_px'])} -> {number(after['joints'][k]['reprojection_p50_px'])}"
                      for k in (11, 12)]
            lines.append(f"| {model['engine_name']} | {after['case']} | {values[0]} | {values[1]} | "
                         f"{left['gate_passed']}/{after['samples']} / {right['gate_passed']}/{after['samples']} |")
    return "\n".join(lines)+"\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("output", type=Path)
    for name in ("capture-run", "selection", "infer-binary", "geometry-binary", "calib", "floor-map", "det-engine"):
        parser.add_argument("--"+name, type=Path, required=True)
    parser.add_argument("--pose-engine", type=Path, action="append", required=True)
    parser.add_argument("--window-ms", type=float, default=15)
    args = parser.parse_args()
    if args.output.exists() or not 1 <= len(args.pose_engine) <= 4:
        parser.error("choose a new output directory and 1..4 pose engines")
    assets = {name: artifact(getattr(args, name)) for name in
              ("selection", "infer_binary", "geometry_binary", "calib", "floor_map", "det_engine")}
    engines = [artifact(path) for path in args.pose_engine]
    if len({e["sha256"] for e in engines}) != len(engines):
        parser.error("pose engines must be distinct")
    assets["capture_manifest"] = artifact(args.capture_run/"capture.json")
    assets["capture_stop"] = artifact(args.capture_run/"stopped.json")
    capture = json.loads((args.capture_run/"capture.json").read_text())
    stopped = json.loads((args.capture_run/"stopped.json").read_text())
    bind_capture(args.session, capture, stopped, assets)
    selection = json.loads(args.selection.read_text())
    if selection["schema"] != "fitra_corpus_diagnostic_selection_v1" or not selection.get("evidence"):
        parser.error("image-reviewed selection/evidence required")
    evidence = selection["evidence"]
    if any(artifact(e["path"]) != e for e in evidence):
        raise ValueError("selection evidence changed")
    manifest, frames, source = load_session(args.session)
    if len(frames) != 3 or any(c["raw_encoding"] != "jpeg" for c in manifest["cameras"]):
        parser.error("three-camera JPEG corpus required")
    groups, semantics = group_frames(frames, args.window_ms)
    poses = select_poses(groups, selection["poses"])
    calibration = select_samples(groups, selection["train_seconds"], selection["validation_seconds"])
    header = {"schema": "fitra_corpus_inference_plan_v1", "cameras": manifest["cameras"],
              "time_basis": semantics, "window_ns": round(args.window_ms*1e6)}
    scripts = {p.name: artifact(p) for p in Path(__file__).parent.glob("*.py")}
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    write_rows(out/"pose-plan.jsonl", header, poses)
    write_rows(out/"calibration-plan.jsonl", {**header, "schema": "fitra_corpus_calibration_plan_v1"}, calibration)
    commands = [[assets["geometry_binary"]["path"], "calibrate", str(out/"calibration-plan.jsonl"),
                 assets["calib"]["path"], assets["floor_map"]["path"], str(out/"geometry")]]
    for i, engine in enumerate(engines):
        commands.append([assets["infer_binary"]["path"], str(out/"pose-plan.jsonl"), assets["calib"]["path"],
                         assets["det_engine"]["path"], engine["path"], str(out/f"model-{i}-raw.jsonl")])
        for name, calib in (("control", assets["calib"]["path"]), ("candidate", str(out/"geometry/candidate.yaml"))):
            commands.append([assets["geometry_binary"]["path"], "triangulate", str(out/"pose-plan.jsonl"),
                             str(out/f"model-{i}-raw.jsonl"), calib, str(out/f"model-{i}-{name}.jsonl")])
    write_json(out/"manifest.json", {"schema": "fitra_corpus_diagnostic_run_v1", "source": source,
        "assets": assets, "engines": engines, "selection": selection, "scripts": scripts, "commands": commands,
        "floor_map_association": "capture_manifest_bound_file_not_physical_pose_guarantee",
        "sampling": "nearest_measured_group_at_1Hz_within_100ms_no_reuse", "runtime_applied": False,
        "not_a_temporal_evaluation_run": True})
    for i, command in enumerate(commands):
        print(f"step {i+1}/{len(commands)}: {Path(command[0]).name}", flush=True)
        with (out/f"step-{i}.log").open("x") as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    geometry = json.loads((out/"geometry/report.json").read_text())
    if geometry["schema"] != "fitra_corpus_calibration_report_v1" or geometry["runtime_applied"]:
        raise ValueError("unexpected calibration report")
    verify_input_binding(out/"calibration-plan.jsonl", {"input_fnv1a64": geometry["plan_fnv1a64"]})
    models = []
    for i, engine in enumerate(engines):
        rh, raw = read_jsonl(out/f"model-{i}-raw.jsonl")
        validate_raw(rh, raw, poses)
        verify_input_binding(out/"pose-plan.jsonl", {"input_fnv1a64": rh["plan_fnv1a64"]})
        rows = {}
        for name in ("control", "candidate"):
            gh, rows[name] = read_jsonl(out/f"model-{i}-{name}.jsonl")
            if gh["schema"] != "fitra_corpus_retriangulation_v1":
                raise ValueError("unexpected geometry output")
            verify_input_binding(out/"pose-plan.jsonl", {"input_fnv1a64": gh["plan_fnv1a64"]})
            verify_input_binding(out/f"model-{i}-raw.jsonl", {"input_fnv1a64": gh["source_fnv1a64"]})
        control = verify_control(raw, rows["control"])
        derived = merge_geometry(raw, rows["candidate"])
        models.append({"engine_name": Path(engine["path"]).stem, "control": control,
                       "original": pose_quality(raw, poses), "candidate": pose_quality(derived, poses)})
    report = {"schema": "fitra_corpus_diagnostic_report_v1", "calibration": geometry, "models": models,
              "joint_gate_settings": SETTINGS, "pose_samples": len(poses), "runtime_applied": False,
              "limitations": ["Sparse 1 Hz diagnosis; no lag, jitter, future teacher, or tracker coverage scoring.",
                              "Reprojection consistency is not physical accuracy; same-family model bias remains.",
                              "File binding does not prove unchanged camera/floor physical placement."]}
    write_json(out/"report.json", report)
    with (out/"report.md").open("x") as handle:
        handle.write(markdown(report))
    for record in [*assets.values(), *engines, *scripts.values(), *evidence]:
        if artifact(record["path"]) != record:
            raise ValueError("diagnostic input/code/evidence changed")
    if load_session(args.session)[2] != source:
        raise ValueError("raw source changed")
    write_json(out/"complete.json", {"schema": "fitra_corpus_diagnostic_complete_v1", "runtime_applied": False,
        "artifacts": {str(p.relative_to(out)): artifact(p) for p in sorted(out.rglob("*")) if p.is_file()}})
    print(out/"report.md")


if __name__ == "__main__":
    main()
