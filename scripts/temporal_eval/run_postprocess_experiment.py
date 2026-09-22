#!/usr/bin/env python3
"""Offline IK ablations on synthetic motion or an immutable completed recording run."""
import argparse
import json
import math
from pathlib import Path
import shutil
import statistics
import subprocess

from evaluate import markdown, percentile, read_jsonl, score, verify_input_binding
from hip_sensitivity import CANDIDATES, HEAD_ISOLATED, candidate_names, constraint_summary
from run_recording import artifact, write_json
from synthetic import dataset, template_profile
from distance_controls import dataset as distance_dataset, score_distance, markdown as distance_markdown


def verified_source(directory):
    directory = directory.resolve(strict=True)
    done = json.loads((directory/"complete.json").read_text())
    manifest_record = done.get("manifest", done["artifacts"].get("manifest.json"))
    if manifest_record != artifact(directory/"manifest.json"):
        raise ValueError("source manifest mismatch")
    for name, record in done["artifacts"].items():
        if Path(name).name != name or artifact(directory/name) != record:
            raise ValueError("completed source changed")
    if not {"input.jsonl", "replay.jsonl"} <= done["artifacts"].keys():
        raise ValueError("complete raw replay source required")
    manifest = json.loads((directory/"manifest.json").read_text())
    if manifest["schema"] not in ("fitra_recording_pilot_v1", "fitra_corpus_evaluation_run_v1"):
        raise ValueError("sparse, partial, or experimental source is not a recording run")
    header, rows = read_jsonl(directory/"input.jsonl")
    if (header["input_stage"] != "raw_triangulated" or header["reference_kind"] != "none"
            or len(rows) != done.get("reports", done.get("groups"))):
        raise ValueError("complete raw unreferenced source required")
    profile = manifest["assets"]["subject_profile"]
    if artifact(profile["path"]) != profile:
        raise ValueError("source subject profile changed")
    return {"complete": artifact(directory/"complete.json"), "manifest": manifest_record,
            "input": done["artifacts"]["input.jsonl"], "baseline": done["artifacts"]["replay.jsonl"],
            "subject_profile": profile}, header, rows


def verify_baseline(control, experiment, names=CANDIDATES):
    if len(control) != len(experiment):
        raise ValueError("baseline row count mismatch")
    for a, b in zip(control, experiment):
        if any(a[k] != b[k] for k in ("seq", "t_ns", "reset", "reference")):
            raise ValueError("baseline lineage/reference changed")
        if any(a["candidates"][n] != b["candidates"][n] for n in names):
            raise ValueError("existing candidate changed during experiment")


def diagnostics(source, output, header):
    candidates = candidate_names(header)
    if candidates == CANDIDATES or len(source) != len(output):
        raise ValueError("complete experimental replay required")
    groups = {}
    for s, o in zip(source, output):
        if (s["seq"], s["t_ns"]) != (o["seq"], o["t_ns"]):
            raise ValueError("constraint lineage mismatch")
        if s["state"] == "boundary":
            if o["constraint_metrics"] is not None:
                raise ValueError("stale boundary constraints")
        elif s.get("evaluate", True):
            groups.setdefault((s["segment"], s.get("phase", "unlabelled")), []).append(o)
    details = []
    for (segment, phase), rows in groups.items():
        for name in candidates:
            times = [r["processing_ms"][name] for r in rows]
            if not all(type(t) in (int, float) and math.isfinite(t) and t >= 0 for t in times):
                raise ValueError("invalid processing time")
            details.append({"segment": segment, "phase": phase, "candidate": name, "reports": len(rows),
                "constraints": constraint_summary(rows, name),
                "processing_ms": {"p50": statistics.median(times), "p95": percentile(times, .95),
                                  "kind": "offline_filter_copy_cost_not_live_latency"}})
    return {"schema": "fitra_postprocess_constraints_v1", "details": details}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    p.add_argument("--binary", type=Path, required=True)
    p.add_argument("--source-run", type=Path, help="omit to use deterministic synthetic motion")
    p.add_argument("--isolate-head", action="store_true", help="append one-way head leaf ablations")
    p.add_argument("--diagnostics", action="store_true", help="retain stage copies without changing candidate values")
    p.add_argument("--synthetic-bone-prior", action="store_true",
                   help="synthetic input only: fixed template prior including head/torso/feet")
    p.add_argument("--distance-experiments", action="store_true", help="append 2/5 mm residual filters (implies head isolation)")
    p.add_argument("--distance-controls", action="store_true", help="small/slow synthetic fixture with template prior and distance candidates")
    p.add_argument("--noise-mm", type=float, help="distance controls only; per-axis noise std (default 3)")
    p.add_argument("--baseline-binary", type=Path, help="frozen head-leaf checkpoint binary; verify all eight old candidates")
    args = p.parse_args()
    if args.source_run and args.distance_controls:
        p.error("distance controls cannot replace a recording")
    if args.noise_mm is not None and not args.distance_controls:
        p.error("--noise-mm requires --distance-controls")
    if args.distance_controls:
        args.distance_experiments = args.synthetic_bone_prior = True
    if args.source_run and args.synthetic_bone_prior:
        p.error("synthetic prior cannot override a recorded subject profile")
    if args.output.exists(): p.error("choose a new output directory")
    source = None
    if args.source_run:
        source, header, rows = verified_source(args.source_run)
    else:
        header, rows = distance_dataset(3. if args.noise_mm is None else args.noise_mm) if args.distance_controls else dataset()
        if args.synthetic_bone_prior:
            header["provenance"]["bone_prior"] = "fixed_synthetic_template_not_fitted_to_run"
    root = Path(__file__).parents[2]
    assets = {"binary": artifact(args.binary), "replay_source": artifact(root/"cpp/tools/temporal_replay.cpp"),
              "spatial_source": artifact(root/"cpp/tools/temporal_weighted_bones.hpp"),
              "distance_source": artifact(root/"cpp/tools/temporal_distance_filter.hpp")}
    if args.baseline_binary: assets["baseline_binary"] = artifact(args.baseline_binary)
    scripts = {f.name: artifact(f) for f in Path(__file__).parent.glob("*.py")}
    out = args.output.resolve()
    command = [assets["binary"]["path"], str(out/"input.jsonl"), str(out/"replay.jsonl"), "--ik-experiments"]
    if args.isolate_head or args.distance_experiments or args.baseline_binary: command.append("--isolate-head")
    if args.distance_experiments: command.append("--distance-experiments")
    if args.diagnostics: command.append("--diagnostics")
    control = [assets.get("baseline_binary",assets["binary"])["path"], str(out/"input.jsonl"), str(out/"control.jsonl")]
    if args.baseline_binary: control += ["--ik-experiments", "--isolate-head"]
    if source:
        for argv in (command, control): argv.append(source["subject_profile"]["path"])
    elif args.synthetic_bone_prior:
        for argv in (command, control): argv.append(str(out/"synthetic-profile.json"))
    commands = [command] if source and not args.baseline_binary else [control, command]
    out.mkdir(parents=True, exist_ok=False)
    if args.synthetic_bone_prior:
        write_json(out/"synthetic-profile.json", template_profile())
        assets["synthetic_profile"] = artifact(out/"synthetic-profile.json")
    write_json(out/"manifest.json", {"schema": "fitra_postprocess_experiment_run_v1", "source": source,
        "assets": assets, "scripts": scripts, "commands": commands, "runtime_applied": False})
    if source: shutil.copyfile(source["input"]["path"], out/"input.jsonl")
    else:
        with (out/"input.jsonl").open("x") as handle:
            for r in [header, *rows]: handle.write(json.dumps(r, allow_nan=False)+"\n")
    frozen_input = artifact(out/"input.jsonl")
    for i, argv in enumerate(commands):
        print(f"replay {i+1}/{len(commands)}: {len(rows)} reports", flush=True)
        with (out/f"replay-{i}.log").open("x") as log:
            subprocess.run(argv, stdout=log, stderr=subprocess.STDOUT, check=True)
    oh, output = read_jsonl(out/"replay.jsonl")
    verify_input_binding(out/"input.jsonl", oh)
    if source:
        ch, baseline = read_jsonl(Path(source["baseline"]["path"]))
        verify_input_binding(out/"input.jsonl", ch)
        verify_baseline(baseline, output)
    if not source or args.baseline_binary:
        ch, baseline = read_jsonl(out/"control.jsonl")
        verify_input_binding(out/"input.jsonl", ch)
        if args.baseline_binary and candidate_names(ch) != HEAD_ISOLATED:
            raise ValueError("baseline binary is not an eight-candidate checkpoint")
        verify_baseline(baseline, output, HEAD_ISOLATED if args.baseline_binary else CANDIDATES)
    report = score(header, rows, oh, output)
    write_json(out/"report.json", report)
    with (out/"report.md").open("x") as handle: handle.write(markdown(report))
    write_json(out/"constraints.json", diagnostics(rows, output, oh))
    if args.distance_experiments:
        distance_report = score_distance(header, rows, oh, output)
        write_json(out/"distance.json", distance_report)
        with (out/"distance.md").open("x") as handle: handle.write(distance_markdown(distance_report))
    if source and verified_source(args.source_run)[0] != source:
        raise ValueError("source changed during run")
    if any(artifact(a["path"]) != a for a in [*assets.values(), *scripts.values(), frozen_input]):
        raise ValueError("experiment assets or input changed")
    write_json(out/"complete.json", {"schema": "fitra_postprocess_experiment_complete_v1",
        "reports": len(rows), "baseline_candidates_unchanged": True, "runtime_applied": False,
        "checkpoint_eight_candidates_unchanged": bool(args.baseline_binary),
        "artifacts": {f.name: artifact(f) for f in sorted(out.iterdir()) if f.is_file()}})
    print(out/"report.md")


if __name__ == "__main__":
    main()
