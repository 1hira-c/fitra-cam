#!/usr/bin/env python3
"""Offline-only, quality-masked future-context reference; never ground truth."""
import argparse
import copy
import json
import math
from pathlib import Path
import subprocess
import sys

from evaluate import read_jsonl, verify_input_binding
from run_recording import artifact, write_json, apply_labels

SETTINGS = {"radius_ns": 50_000_000, "max_gap_ns": 42_000_000,
            "minimum_each_side": 2, "minimum_score": .3,
            "maximum_reprojection_px": 6, "minimum_ray_angle_deg": 10}


def trusted(joint, quality, index):
    if index == 0 or joint is None or quality is None:
        return False  # Slot 0 is a synthetic direction endpoint.
    values = [*joint, quality["reproj_px"], quality["ray_angle_deg"]]
    if (len(joint) != 4 or not all(type(v) in (int, float) and math.isfinite(v) for v in values)
            or type(quality["views"]) is not int or not 2 <= quality["views"] <= 3):
        return False
    return (joint[3] >= SETTINGS["minimum_score"]
            and 0 <= quality["reproj_px"] <= SETTINGS["maximum_reprojection_px"]
            and SETTINGS["minimum_ray_angle_deg"] <= quality["ray_angle_deg"] <= 90)


def local_linear(samples, center_ns):
    """Unweighted symmetric-time-window regression, evaluated at the center timestamp."""
    times = [(t-center_ns)*1e-9 for t, _ in samples]
    n = len(samples)
    sx, sxx = sum(times), sum(t*t for t in times)
    denominator = n*sxx-sx*sx
    if denominator <= 0:
        raise ValueError("degenerate reference window")
    result = []
    for axis in range(3):
        sy = sum(j[axis] for _, j in samples)
        sxy = sum(t*j[axis] for t, (_, j) in zip(times, samples))
        result.append((sxx*sy-sx*sxy)/denominator)
    return [*result, min(j[3] for _, j in samples)]


def smooth_reference(rows):
    if not rows or any(type(r["seq"]) is not int or type(r["t_ns"]) is not int
                       or len(r["joints"]) != 26 or len(r["joint_quality"]) != 26 for r in rows):
        raise ValueError("malformed reference rows")
    if any(b["seq"] <= a["seq"] or b["t_ns"] <= a["t_ns"] for a, b in zip(rows, rows[1:])):
        raise ValueError("nonmonotonic reference")
    masks = [[not r["ambiguous_detection"] and trusted(j, q, k)
              for k, (j, q) in enumerate(zip(r["joints"], r["joint_quality"]))] for r in rows]
    reference = [[None]*26 for _ in rows]
    for i, center in enumerate(rows):
        for k in range(1, 26):
            if not masks[i][k]:
                continue
            window = [(center["t_ns"], center["joints"][k])]
            sides = []
            for direction in (-1, 1):
                count, previous, j = 0, i, i+direction
                while 0 <= j < len(rows):
                    r, last = rows[j], rows[previous]
                    if (abs(r["t_ns"]-center["t_ns"]) > SETTINGS["radius_ns"]
                            or abs(r["seq"]-last["seq"]) != 1
                            or abs(r["t_ns"]-last["t_ns"]) > SETTINGS["max_gap_ns"]
                            or not masks[j][k]):
                        break
                    window.append((r["t_ns"], r["joints"][k]))
                    count += 1
                    previous, j = j, j+direction
                sides.append(count)
            if min(sides) >= SETTINGS["minimum_each_side"]:
                reference[i][k] = local_linear(window, center["t_ns"])
    quality = {"settings": SETTINGS, "reports": len(rows), "per_joint": [
        {"halpe_index": k, "trusted_raw": sum(mask[k] for mask in masks),
         "future_supported": sum(r[k] is not None for r in reference),
         "coverage": sum(r[k] is not None for r in reference)/len(rows)} for k in range(26)]}
    return reference, quality


def completed_run(directory):
    done = json.loads((directory/"complete.json").read_text())
    required = {"manifest.json", "plan.jsonl", "raw.jsonl", "input.jsonl", "replay.jsonl", "report.json"}
    if not required <= done["artifacts"].keys():
        raise ValueError("incomplete source run")
    for name, record in done["artifacts"].items():
        if Path(name).name != name or artifact(directory/name) != record:
            raise ValueError("completed source artifact changed")
    manifest = json.loads((directory/"manifest.json").read_text())
    if manifest["schema"] != "fitra_corpus_evaluation_run_v1" or manifest["reference_kind"] != "none":
        raise ValueError("raw corpus run required")
    rh, raw = read_jsonl(directory/"raw.jsonl")
    if rh["schema"] != "fitra_corpus_raw3d_v1" or rh["postprocessing"] is not False or rh["kp_format"] != "halpe26":
        raise ValueError("raw Halpe26 reference required")
    verify_input_binding(directory/"plan.jsonl", {"input_fnv1a64": rh["plan_fnv1a64"]})
    if len(raw) != done["groups"] or len(raw) != manifest["selected_groups"]:
        raise ValueError("source count mismatch")
    return manifest, done, raw


def validate_pair(student_manifest, student_done, teacher_manifest, teacher_done):
    if student_done["artifacts"]["plan.jsonl"]["sha256"] != teacher_done["artifacts"]["plan.jsonl"]["sha256"]:
        raise ValueError("teacher and candidate must use identical frame groups")
    for key in ("calib", "subject_profile", "det_engine", "infer_binary", "capture_manifest"):
        if student_manifest["assets"][key] != teacher_manifest["assets"][key]:
            raise ValueError("teacher/candidate geometry, detector, or capture mismatch")
    if student_manifest["assets"]["pose_engine"]["sha256"] == teacher_manifest["assets"]["pose_engine"]["sha256"]:
        raise ValueError("teacher must use a distinct pose engine")
    student_derived = student_manifest.get("calibration_derivation")
    teacher_derived = teacher_manifest.get("calibration_derivation")
    if bool(student_derived) != bool(teacher_derived):
        raise ValueError("cannot mix capture-bound and derived calibration runs")
    if student_derived:
        for key in ("calibration_complete", "original_calib", "geometry_binary"):
            if student_derived[key] != teacher_derived[key]:
                raise ValueError("teacher/candidate calibration derivation mismatch")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("candidate_run", type=Path)
    p.add_argument("teacher_run", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--labels", type=Path)
    args = p.parse_args()
    if args.output.exists():
        p.error("choose a new output directory")
    sm, sd, student = completed_run(args.candidate_run)
    tm, td, teacher = completed_run(args.teacher_run)
    validate_pair(sm, sd, tm, td)
    if [(r["seq"], r["t_ns"]) for r in student] != [(r["seq"], r["t_ns"]) for r in teacher]:
        raise ValueError("teacher/candidate lineage mismatch")
    header, rows = read_jsonl(args.candidate_run/"input.jsonl")
    if (header["reference_kind"] != "none" or len(rows) != len(student)
            or header["time_basis"] != "measured_capture_group"
            or header["coordinate_frame"] != "fitra_world_z_up_m"):
        raise ValueError("unreferenced candidate input required")
    reference, quality = smooth_reference(teacher)
    provenance = {"candidate_complete": artifact(args.candidate_run/"complete.json"),
                  "teacher_complete": artifact(args.teacher_run/"complete.json"),
                  "generator": artifact(__file__), "teacher_engine": tm["assets"]["pose_engine"],
                  "scripts": {path.name: artifact(path) for path in Path(__file__).parent.glob("*.py")},
                  "settings": SETTINGS, "algorithm": "centered_local_linear_future_context",
                  "measured_ground_truth": False}
    labels = json.loads(args.labels.read_text()) if args.labels else None
    if labels:
        provenance["labels"] = artifact(args.labels)
        apply_labels(rows, labels)
    for row, raw, ref in zip(rows, student, reference):
        if row["seq"] != raw["seq"] or row["t_ns"]+int(header["capture_origin_ns"]) != raw["t_ns"]:
            raise ValueError("candidate input/raw mismatch")
        row["reference_joints"] = ref if row["state"] == "fresh" else None
    header = copy.deepcopy(header)
    header["reference_kind"] = "pseudo"
    header["provenance"]["pseudo_reference"] = provenance
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    write_json(out/"reference-quality.json", quality)
    with (out/"input.jsonl").open("x") as f:
        for row in [header, *rows]:
            f.write(json.dumps(row, allow_nan=False)+"\n")
    assets = sm["assets"]
    for name in ("replay_binary", "subject_profile"):
        if artifact(assets[name]["path"]) != assets[name]:
            raise ValueError("candidate replay asset changed")
    with (out/"replay.log").open("x") as log:
        subprocess.run([assets["replay_binary"]["path"], str(out/"input.jsonl"), str(out/"replay.jsonl"),
                        assets["subject_profile"]["path"]], stdout=log, stderr=subprocess.STDOUT, check=True)
    _, before = read_jsonl(args.candidate_run/"replay.jsonl")
    _, after = read_jsonl(out/"replay.jsonl")
    if len(before) != len(after) or any(a["candidates"] != b["candidates"] or a["reset"] != b["reset"]
                                     for a, b in zip(before, after)):
        raise ValueError("reference generation altered candidate output")
    with (out/"evaluate.log").open("x") as log:
        subprocess.run([sys.executable, str(Path(__file__).with_name("evaluate.py")), str(out/"input.jsonl"),
                        str(out/"replay.jsonl"), str(out/"report.json"), "--binary", assets["replay_binary"]["path"],
                        "--subject-profile", assets["subject_profile"]["path"]], stdout=log, stderr=subprocess.STDOUT, check=True)
    completed_run(args.candidate_run)
    completed_run(args.teacher_run)
    for name in ("candidate_complete", "teacher_complete", "generator", "labels"):
        if name in provenance and artifact(provenance[name]["path"]) != provenance[name]:
            raise ValueError("reference inputs changed during run")
    for record in [*provenance["scripts"].values(), assets["replay_binary"], assets["subject_profile"]]:
        if artifact(record["path"]) != record:
            raise ValueError("reference code or replay assets changed during run")
    write_json(out/"complete.json", {"candidate_output_unchanged": True, "provenance": provenance,
        "artifacts": {name: artifact(out/name) for name in
                      ("input.jsonl", "reference-quality.json", "replay.jsonl", "report.json", "report.md")}})
    print(out/"report.md")


if __name__ == "__main__":
    main()
