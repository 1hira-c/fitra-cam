#!/usr/bin/env python3
"""Propose offline extrinsics from recorded floor tags, with disjoint held-out images."""
import argparse
import json
import math
from pathlib import Path
import subprocess

from evaluate import read_jsonl, verify_input_binding
from inspect_corpus import load_session, group_frames
from pseudo_reference import completed_run
from run_recording import artifact, write_json


def select_samples(groups, train=(0, 13), validation=(28, 42)):
    ranges = (train, validation)
    if any(len(r) != 2 or any(type(v) is not int for v in r) or not 0 <= r[0] < r[1] for r in ranges):
        raise ValueError("sample ranges must be nonnegative half-open integer seconds")
    if max(train[0], validation[0]) < min(train[1], validation[1]):
        raise ValueError("training and validation must not overlap")
    if not groups:
        raise ValueError("no frame groups")
    origin = int(groups[0]["capture_max_ns"])
    selected, used = [], set()
    for split, (start, end) in zip(("train", "validation"), ranges):
        for second in range(start, end):
            target = origin+second*1_000_000_000
            row = min(groups, key=lambda r: abs(int(r["capture_max_ns"])-target))
            if abs(int(row["capture_max_ns"])-target) > 100_000_000:
                raise ValueError("requested sample missing or outside recorded range")
            if row["seq"] in used:
                raise ValueError("same frame reused across calibration samples")
            used.add(row["seq"])
            selected.append({**row, "split": split})
    return sorted(selected, key=lambda r: r["seq"])


def verified_calibration(directory):
    directory = Path(directory)
    done = json.loads((directory/"complete.json").read_text())
    expected = {"manifest.json", "plan.jsonl", "geometry/candidate.yaml", "geometry/report.json", "geometry/detections.jsonl"}
    if done.get("schema") != "fitra_corpus_calibration_complete_v1" or set(done["artifacts"]) != expected:
        raise ValueError("incomplete calibration proposal")
    for name, record in done["artifacts"].items():
        if artifact(directory/name) != record:
            raise ValueError("calibration artifact changed")
    manifest = json.loads((directory/"manifest.json").read_text())
    report = json.loads((directory/"geometry/report.json").read_text())
    if (manifest["schema"] != "fitra_corpus_calibration_run_v1" or report["schema"] != "fitra_corpus_calibration_report_v1"
            or report["runtime_applied"] or done["runtime_applied"]):
        raise ValueError("unsupported or live-applied calibration proposal")
    verify_input_binding(directory/"plan.jsonl", {"input_fnv1a64": report["plan_fnv1a64"]})
    return manifest, done, report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("source_run", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--geometry-binary", type=Path, required=True)
    p.add_argument("--floor-map", type=Path, required=True)
    p.add_argument("--confirm-floor-map-unchanged", action="store_true")
    p.add_argument("--train-seconds", nargs=2, type=int, default=(0, 13))
    p.add_argument("--validation-seconds", nargs=2, type=int, default=(28, 42))
    args = p.parse_args()
    if args.output.exists() or not args.confirm_floor_map_unchanged:
        p.error("new output and explicit unchanged floor-map association required")
    sm, sd, _ = completed_run(args.source_run)
    if "calibration_derivation" in sm:
        p.error("use an original capture-bound run, not a derived run")
    session = Path(sm["source"]["manifest"]["path"]).parent
    m, frames, source = load_session(session)
    if source != sm["source"]:
        raise ValueError("capture source changed")
    ph, groups = read_jsonl(args.source_run/"plan.jsonl")
    rebuilt, semantics = group_frames(frames, sm["qualification"]["window_ms"])
    if groups != rebuilt[:len(groups)] or ph["cameras"] != m["cameras"] or ph["time_basis"] != semantics:
        raise ValueError("frame plan does not match original capture")
    samples = select_samples(groups, args.train_seconds, args.validation_seconds)
    assets = {"source_calib": sm["assets"]["calib"], "floor_map": artifact(args.floor_map),
              "geometry_binary": artifact(args.geometry_binary), "source_complete": artifact(args.source_run/"complete.json")}
    if artifact(assets["source_calib"]["path"]) != assets["source_calib"]:
        raise ValueError("original calibration changed")
    scripts = {path.name: artifact(path) for path in Path(__file__).parent.glob("*.py")}
    out = args.output.resolve()
    command = [assets["geometry_binary"]["path"], "calibrate", str(out/"plan.jsonl"),
               assets["source_calib"]["path"], assets["floor_map"]["path"], str(out/"geometry")]
    out.mkdir(parents=True, exist_ok=False)
    write_json(out/"manifest.json", {"schema": "fitra_corpus_calibration_run_v1", "assets": assets,
        "source_plan": sd["artifacts"]["plan.jsonl"], "source_capture": sm["assets"]["capture_manifest"],
        "source": source, "scripts": scripts, "command": command, "runtime_applied": False,
        "train_seconds": args.train_seconds, "validation_seconds": args.validation_seconds,
        "floor_map_association": "operator_confirmed_unchanged_not_capture_manifest_bound",
        "limitations": ["Floor-only geometry may remain planar-degenerate.",
                        "Temporal holdout is not independent ground truth or new spatial tag coverage.",
                        "Missing/failed cameras retain original calibration, never silently re-fit."]})
    with (out/"plan.jsonl").open("x") as f:
        f.write(json.dumps({**ph, "schema": "fitra_corpus_calibration_plan_v1"})+"\n")
        for row in samples:
            f.write(json.dumps(row, allow_nan=False)+"\n")
    with (out/"calibrate.log").open("x") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    for record in [*assets.values(), *scripts.values()]:
        if artifact(record["path"]) != record:
            raise ValueError("calibration source/code changed during run")
    completed_run(args.source_run)
    if load_session(session)[2] != source:
        raise ValueError("raw capture changed during calibration")
    write_json(out/"complete.json", {"schema": "fitra_corpus_calibration_complete_v1", "runtime_applied": False,
        "artifacts": {name: artifact(out/name) for name in
                      ("manifest.json", "plan.jsonl", "geometry/candidate.yaml", "geometry/report.json", "geometry/detections.jsonl")}})
    verified_calibration(out)
    print(out/"geometry/report.json")


if __name__ == "__main__":
    main()
