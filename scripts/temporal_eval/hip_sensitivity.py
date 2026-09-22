#!/usr/bin/env python3
"""Paired hip-only perturbations through production replay; not real-pose accuracy."""
import argparse
import copy
import json
import math
from pathlib import Path
import random
import subprocess

from evaluate import (ROLES, angle_deg, norm, percentile, read_jsonl, rms, spread,
                      sub, swing_twist_deg, validate_tracker, verify_input_binding)
from run_recording import artifact, write_json
from synthetic import pose

TARGETS = {"center": (19,), "hips_common": (11, 12), "hips_differential": (11, 12)}
BODY = tuple(range(5, 26))
CANDIDATES = ("minimal", "kalman", "current")
STAGES = ("raw", "post_kalman", "post_ik")


def cases():
    return [{"id": f"{motion}/{fault}/{target}", "motion": motion, "fault": fault, "target": target}
            for motion, faults in (("static", ("jitter", "fixed_bias")),
                                   ("forward_bend", ("jitter", "fixed_bias")),
                                   ("turn", ("view_bias",)))
            for fault in faults for target in TARGETS]


def clean_pose(motion, t):
    joints = pose()
    heading = 0.
    if motion == "forward_bend":
        theta = math.radians(70)*(.5-.5*math.cos(2*math.pi*t/3))
        for k in (0, 5, 6, 7, 8, 9, 10, 17, 18):
            x, y, z, _ = joints[k]
            z -= .95
            joints[k][:3] = [x, y*math.cos(theta)+z*math.sin(theta),
                             .95+z*math.cos(theta)-y*math.sin(theta)]
    elif motion == "turn":
        heading = math.pi*.8*math.sin(2*math.pi*t/4)
        for joint in joints:
            if joint:
                x, y = joint[:2]
                joint[:2] = [x*math.cos(heading)-y*math.sin(heading),
                             x*math.sin(heading)+y*math.cos(heading)]
    elif motion != "static":
        raise ValueError("unknown motion")
    return joints, heading


def dataset(amplitude_mm=20., hz=60, seconds=5, seed=20260922, selected=None):
    if (type(amplitude_mm) not in (int, float) or not math.isfinite(amplitude_mm)
            or not 0 <= amplitude_mm <= 200 or type(hz) is not int or not 5 <= hz <= 120
            or type(seconds) is not int or not 2 <= seconds <= 10 or type(seed) is not int):
        raise ValueError("invalid perturbation size, cadence, duration, or seed")
    selected = cases() if selected is None else selected
    known = {c["id"]: c for c in cases()}
    if (not selected or len({c["id"] for c in selected}) != len(selected)
            or any(c != known.get(c["id"]) for c in selected)):
        raise ValueError("unknown or duplicate sensitivity case")
    header = {"schema": "fitra_temporal_input_v1", "input_stage": "raw_triangulated",
              "kp_format": "halpe26", "coordinate_frame": "fitra_world_z_up_m",
              "nominal_hz": hz, "subject_height_m": 1.7, "reference_kind": "none",
              "time_basis": "synthetic_nominal", "provenance": {
                  "generator": "hip_sensitivity.py", "seed": seed, "cases": selected,
                  "amplitude_mm": amplitude_mm, "seconds_per_case": seconds,
                  "warmup_reports": hz, "kind": "controlled_pair_not_measured_ground_truth",
                  "fault_definition": {
                      "jitter": "independent Gaussian world axes, expected vector RMS=amplitude",
                      "fixed_bias": "constant world +Y displacement of amplitude",
                      "view_bias": "world +Y amplitude*sin(2*body_heading); surrogate, not a camera model",
                      "hips_differential": "equal and opposite displacements on indices 11 and 12"}}}
    clean, faulty = [], []
    amplitude = amplitude_mm*.001
    for case in selected:
        rng = random.Random(f"{seed}:{case['id']}")
        for i in range(hz*seconds):
            truth, heading = clean_pose(case["motion"], i/hz)
            changed = copy.deepcopy(truth)
            if case["fault"] == "jitter":
                delta = [rng.gauss(0, amplitude/math.sqrt(3)) for _ in range(3)]
            else:
                delta = [0., amplitude*(math.sin(2*heading) if case["fault"] == "view_bias" else 1.), 0.]
            for j, k in enumerate(TARGETS[case["target"]]):
                sign = -1 if case["target"] == "hips_differential" and j == 1 else 1
                changed[k][:3] = [truth[k][axis]+sign*delta[axis] for axis in range(3)]
            n = len(clean)
            row = {"seq": n, "t_ns": round(n*1e9/hz), "segment": case["id"], "state": "fresh",
                   "evaluate": i >= hz, "phase": "static" if case["motion"] == "static" else "motion"}
            clean.append({**row, "joints": truth})
            faulty.append({**row, "joints": changed})
    return header, clean, faulty


def validate_pair(header, clean, faulty):
    if (header.get("schema") != "fitra_temporal_input_v1" or header.get("input_stage") != "raw_triangulated"
            or header.get("kp_format") != "halpe26" or header.get("coordinate_frame") != "fitra_world_z_up_m"
            or header.get("time_basis") != "synthetic_nominal" or header.get("reference_kind") != "none"
            or header.get("provenance", {}).get("kind") != "controlled_pair_not_measured_ground_truth"
            or not clean or len(clean) != len(faulty)):
        raise ValueError("controlled synthetic pair required")
    definitions = {c["id"]: c for c in header["provenance"]["cases"]}
    for a, b in zip(clean, faulty):
        if (type(a["seq"]) is not int or type(a["t_ns"]) is not int or type(a["evaluate"]) is not bool
                or a["seq"] < 0 or a["t_ns"] < 0):
            raise ValueError("invalid controlled pair lineage")
        if {k: v for k, v in a.items() if k != "joints"} != {k: v for k, v in b.items() if k != "joints"}:
            raise ValueError("pair metadata/lineage mismatch")
        if len(a["joints"]) != 26 or len(b["joints"]) != 26 or a["state"] != "fresh":
            raise ValueError("expected fresh Halpe26 synthetic pair")
        targets = TARGETS[definitions[a["segment"]]["target"]]
        for k, (x, y) in enumerate(zip(a["joints"], b["joints"])):
            if (x is None) != (y is None) or (k not in targets and x != y):
                raise ValueError("perturbation changed missingness or a non-target joint")
            if x is not None:
                if (len(x) != 4 or len(y) != 4 or x[3] != y[3]
                        or not all(type(v) in (int, float) and math.isfinite(v) for v in [*x, *y])):
                    raise ValueError("invalid joint or changed confidence")


def position_difference(clean, faulty, input_rms_m):
    if len(clean) != len(faulty):
        raise ValueError("position pair count mismatch")
    if any(len(v) != 3 or not all(type(x) in (int, float) and math.isfinite(x) for x in v)
           for v in [*clean, *faulty] if v is not None):
        raise ValueError("invalid position")
    deltas = [sub(b, a) for a, b in zip(clean, faulty) if a is not None and b is not None]
    if any(len(d) != 3 or not all(math.isfinite(v) for v in d) for d in deltas):
        raise ValueError("invalid position delta")
    sizes = [norm(d) for d in deltas]
    value = rms(sizes)
    bias = [sum(d[c] for d in deltas)/len(deltas)*1000 for c in range(3)] if deltas else None
    return {"eligible": len(clean), "pairs": len(deltas),
            "clean_available": sum(v is not None for v in clean),
            "fault_available": sum(v is not None for v in faulty),
            "rms_mm": value*1000 if value is not None else None,
            "p95_mm": percentile([v*1000 for v in sizes], .95), "mean_delta_mm": bias,
            "centered_delta_rms_mm": spread(deltas)*1000 if len(deltas) >= 2 else None,
            "gain": value/input_rms_m if value is not None and input_rms_m > 1e-12 else None}


def tracker_difference(clean, faulty, input_rms_m):
    for t in [*clean, *faulty]:
        if t is not None:
            validate_tracker(t)
    result = position_difference([t["position_m"] if t else None for t in clean],
                                 [t["position_m"] if t else None for t in faulty], input_rms_m)
    angles, swings, twists = [], [], []
    unobservable, undefined = 0, 0
    for a, b in zip(clean, faulty):
        if a is None or b is None:
            continue
        angles.append(angle_deg(a["quat_wxyz"], b["quat_wxyz"]))
        swing, twist = swing_twist_deg(a["quat_wxyz"], b["quat_wxyz"])
        swings.append(swing)
        if a.get("roll_confidence", 0) <= 0 or b.get("roll_confidence", 0) <= 0:
            unobservable += 1
        elif twist is None:
            undefined += 1
        else:
            twists.append(abs(twist))
    result["rotation"] = {"quaternion_delta_p95_deg": percentile(angles, .95),
                          "swing_delta_p95_deg": percentile(swings, .95),
                          "supported_twist_delta_p95_deg": percentile(twists, .95),
                          "twist_supported_pairs": len(twists), "twist_unobservable_pairs": unobservable,
                          "twist_undefined_pairs": undefined,
                          "kind": "paired_output_difference_not_physical_rotation_error"}
    return result


def validate_replay(source, header, rows):
    if (header.get("schema") != "fitra_temporal_output_v1"
            or header.get("diagnostics_schema") != "fitra_temporal_diagnostics_v1"
            or header.get("candidates") != list(CANDIDATES) or len(rows) != len(source)):
        raise ValueError("complete diagnostic replay required")
    previous = None
    for s, r in zip(source, rows):
        if (type(r["seq"]) is not int or type(r["t_ns"]) is not int
                or (s["seq"], s["t_ns"]) != (r["seq"], r["t_ns"])
                or (previous and (s["seq"] <= previous["seq"] or s["t_ns"] <= previous["t_ns"]))):
            raise ValueError("replay lineage mismatch")
        reset = previous is None or s["segment"] != previous["segment"] or s["t_ns"]-previous["t_ns"] > 500_000_000
        if type(r["reset"]) is not bool or r["reset"] != reset:
            raise ValueError("replay reset mismatch")
        if (set(r["candidates"]) != set(CANDIDATES) or not isinstance(r["diagnostics"], dict)
                or set(r["diagnostics"]) != set(CANDIDATES)):
            raise ValueError("diagnostic candidates mismatch")
        for name in CANDIDATES:
            d = r["diagnostics"][name]
            if set(d) != {"post_kalman", "post_ik", "pre_smoothing"}:
                raise ValueError("diagnostic stages mismatch")
            for stage in ("post_kalman", "post_ik"):
                if len(d[stage]) != 26:
                    raise ValueError("diagnostic joint topology mismatch")
                for j in d[stage]:
                    if j is not None and (len(j) != 4 or not all(type(v) in (int, float) and math.isfinite(v) for v in j)):
                        raise ValueError("nonfinite diagnostic joint")
            for trackers in (d["pre_smoothing"], r["candidates"][name]):
                if len(trackers) != 10:
                    raise ValueError("diagnostic tracker topology mismatch")
                for t in trackers:
                    if t is not None:
                        validate_tracker(t)
        previous = s


def score_pair(header, clean, faulty, clean_header, clean_out, fault_header, fault_out):
    validate_pair(header, clean, faulty)
    validate_replay(clean, clean_header, clean_out)
    validate_replay(faulty, fault_header, fault_out)
    if ({k: v for k, v in clean_header.items() if k != "input_fnv1a64"}
            != {k: v for k, v in fault_header.items() if k != "input_fnv1a64"}):
        raise ValueError("paired replay configuration mismatch")
    details = []
    for case in header["provenance"]["cases"]:
        indices = [i for i, r in enumerate(clean) if r["segment"] == case["id"] and r["evaluate"]]
        if not indices:
            raise ValueError("empty scored case")
        targets = TARGETS[case["target"]]
        magnitude = rms([norm(sub(faulty[i]["joints"][k][:3], clean[i]["joints"][k][:3]))
                         for i in indices for k in targets])
        for candidate in CANDIDATES:
            joints = {}
            for stage in STAGES:
                def points(source, out, k):
                    return [(s[k][:3] if s[k] is not None else None) for i in indices
                            for s in [source[i]["joints"] if stage == "raw" else out[i]["diagnostics"][candidate][stage]]]
                joints[stage] = {str(k): position_difference(points(clean, clean_out, k),
                                                            points(faulty, fault_out, k), magnitude) for k in BODY}
            trackers = {}
            for stage in ("pre_smoothing", "final"):
                def values(out, k):
                    return [(out[i]["candidates"][candidate] if stage == "final"
                             else out[i]["diagnostics"][candidate][stage])[k] for i in indices]
                trackers[stage] = {role: tracker_difference(values(clean_out, k), values(fault_out, k), magnitude)
                                   for k, role in enumerate(ROLES)}
            details.append({"case": case, "candidate": candidate, "reports": len(indices),
                            "input_target_vector_rms_mm": magnitude*1000, "joints": joints, "trackers": trackers})
    return {"schema": "fitra_hip_sensitivity_report_v1", "details": details,
            "acceptance": "diagnostic_only_not_real_rig_accuracy_or_latency",
            "reference": "same_pipeline_unperturbed_control_not_anatomical_ground_truth",
            "limitations": ["Perturbation size is hypothetical, not converted from measured reprojection residuals.",
                            "View bias is a heading-dependent 3D surrogate, not a measured 2D annotation/camera model.",
                            "Both runs share IK priors; differences isolate added faults, not baseline reconstruction error.",
                            "One report per update; no live pacing, capture-to-display latency, or filter winner.",
                            "Stored stage positions exclude floor/extension correction; final trackers include both.",
                            "Swing/twist comparison uses tracker-local +Z; physical limb interpretation applies to legs."]}


def markdown(report):
    def maximum(values):
        finite = [v for v in values if v is not None]
        return f"{max(finite):.2f}" if finite else "-"
    lines = ["# Hip-only sensitivity", "", "Paired output differences, NOT physical pose error or live latency.",
             "Worst left/right value; counts, signed bias, jitter and intermediate stages are in report.json.", "",
             "| Case | Candidate | Input RMS mm | Post-IK knee RMS mm | Post-IK ankle RMS mm | Final foot RMS mm | Thigh swing p95 deg | Supported thigh twist p95 deg |",
             "|---|---|---:|---:|---:|---:|---:|---:|"]
    for d in report["details"]:
        j, t = d["joints"]["post_ik"], d["trackers"]["final"]
        lines.append(f"| {d['case']['id']} | {d['candidate']} | {d['input_target_vector_rms_mm']:.2f} | "
                     f"{maximum(j[str(k)]['rms_mm'] for k in (13,14))} | "
                     f"{maximum(j[str(k)]['rms_mm'] for k in (15,16))} | "
                     f"{maximum(t[role]['rms_mm'] for role in ('left_foot','right_foot'))} | "
                     f"{maximum(t[role]['rotation']['swing_delta_p95_deg'] for role in ('left_upper_leg','right_upper_leg'))} | "
                     f"{maximum(t[role]['rotation']['supported_twist_delta_p95_deg'] for role in ('left_upper_leg','right_upper_leg'))} |")
    return "\n".join(lines)+"\n"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    p.add_argument("--binary", type=Path, required=True)
    p.add_argument("--amplitude-mm", type=float, default=20.)
    args = p.parse_args()
    if args.output.exists():
        p.error("new output directory required")
    header, clean, faulty = dataset(args.amplitude_mm)
    validate_pair(header, clean, faulty)
    assets = {"binary": artifact(args.binary), "replay_source": artifact(Path(__file__).parents[2]/"cpp/tools/temporal_replay.cpp")}
    scripts = {f.name: artifact(f) for f in Path(__file__).parent.glob("*.py")}
    out = args.output.resolve()
    commands = {name: [assets["binary"]["path"], str(out/f"{name}-input.jsonl"),
                       str(out/f"{name}-output.jsonl"), "--diagnostics"] for name in ("clean", "fault")}
    out.mkdir(parents=True, exist_ok=False)
    write_json(out/"manifest.json", {"schema": "fitra_hip_sensitivity_run_v1", "assets": assets,
        "scripts": scripts, "commands": commands, "fixture": header, "runtime_applied": False})
    results, inputs = {}, []
    for name, rows in (("clean", clean), ("fault", faulty)):
        with (out/f"{name}-input.jsonl").open("x") as handle:
            for r in [header, *rows]:
                handle.write(json.dumps(r, allow_nan=False)+"\n")
        inputs.append(artifact(out/f"{name}-input.jsonl"))
        print(f"replay {name}: {len(rows)} reports", flush=True)
        with (out/f"{name}.log").open("x") as log:
            subprocess.run(commands[name], stdout=log, stderr=subprocess.STDOUT, check=True)
        results[name] = read_jsonl(out/f"{name}-output.jsonl")
        verify_input_binding(out/f"{name}-input.jsonl", results[name][0])
    report = score_pair(header, clean, faulty, *results["clean"], *results["fault"])
    write_json(out/"report.json", report)
    with (out/"report.md").open("x") as handle:
        handle.write(markdown(report))
    if any(artifact(a["path"]) != a for a in [*assets.values(), *scripts.values(), *inputs]):
        raise ValueError("code, binary or generated input changed during sensitivity run")
    write_json(out/"complete.json", {"schema": "fitra_hip_sensitivity_complete_v1", "runtime_applied": False,
        "artifacts": {f.name: artifact(f) for f in sorted(out.iterdir()) if f.is_file()}})
    print(out/"report.md")


if __name__ == "__main__":
    main()
