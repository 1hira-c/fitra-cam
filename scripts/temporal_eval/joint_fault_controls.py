#!/usr/bin/env python3
"""Counter-controls: knees/ankles, not hips, are noisy; high vs low declared score."""
import argparse
import copy
import math
from pathlib import Path
import random

from hip_sensitivity import cases, clean_pose, dataset, run_pair, validate_pair

TARGETS = {"knees": (13, 14), "ankles": (15, 16)}


def controls(amplitude_mm=20., hz=60, seconds=5, seed=20260922):
    header, _, _ = dataset(amplitude_mm, hz, seconds, seed, selected=[cases()[0]])
    definitions = [{"id": f"{motion}/{target}/{label}", "motion": motion, "fault": "jitter",
                    "target": target, "score": score}
                   for motion in ("static", "forward_bend") for target in TARGETS
                   for label, score in (("high_score", .95), ("low_score", .2))]
    header["provenance"].update({"generator": "joint_fault_controls.py", "cases": definitions,
        "fault_definition": {"jitter": "independent per joint/axis Gaussian, vector RMS expectation=amplitude",
            "score": "same in clean/fault; high/low-score cases have identical positions and noise",
            "purpose": "test upstream propagation when hip observations are unchanged"}})
    clean, fault = [], []
    for case in definitions:
        # Exclude score from seed: compare score alone, not different random draws.
        rng = random.Random(f"{seed}:{case['motion']}:{case['target']}")
        for i in range(hz*seconds):
            pose, _ = clean_pose(case["motion"], i/hz)
            for k in TARGETS[case["target"]]: pose[k][3] = case["score"]
            changed = copy.deepcopy(pose)
            for k in TARGETS[case["target"]]:
                for axis in range(3):
                    changed[k][axis] += rng.gauss(0, amplitude_mm*.001/math.sqrt(3))
            n = len(clean)
            row = {"seq": n, "t_ns": round(n*1e9/hz), "segment": case["id"], "state": "fresh",
                   "evaluate": i >= hz, "phase": "static" if case["motion"] == "static" else "motion"}
            clean.append({**row, "joints": pose})
            fault.append({**row, "joints": changed})
    validate_pair(header, clean, fault, TARGETS)
    return header, clean, fault


def markdown(report):
    def worst(values):
        values = [v for v in values if v is not None]
        return f"{max(values):.3f}" if values else "-"
    lines = ["# Knee/ankle fault counter-controls", "",
             "Paired numerical output differences, NOT anatomical accuracy or live latency.",
             "Hip inputs are unchanged; high/low-score cases differ only in declared target scores.", "",
             "| Case | Candidate | Input RMS mm | Post-IK hip RMS mm | Post-IK knee RMS mm | Post-IK ankle RMS mm | Final waist RMS mm | Final foot RMS mm |",
             "|---|---|---:|---:|---:|---:|---:|---:|"]
    for d in report["details"]:
        j, t = d["joints"]["post_ik"], d["trackers"]["final"]
        lines.append(f"| {d['case']['id']} | {d['candidate']} | {d['input_target_vector_rms_mm']:.3f} | "
                     + " | ".join(worst(j[str(k)]["rms_mm"] for k in keys)
                                  for keys in ((11,12,19), (13,14), (15,16)))
                     + " | " + worst([t["waist"]["rms_mm"]]) + " | "
                     + worst(t[k]["rms_mm"] for k in ("left_foot","right_foot")) + " |")
    lines += ["", "Full JSON keeps both runs' constraints, per-joint/stage differences, missingness and twist support.",
              "Lowering hip weight may transfer knee/ankle noise into hips. Scores are not measured uncertainty.", ""]
    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    p.add_argument("--binary", type=Path, required=True)
    p.add_argument("--amplitude-mm", type=float, default=20.)
    p.add_argument("--isolate-head", action="store_true")
    args = p.parse_args()
    run_pair(args.output, args.binary, *controls(args.amplitude_mm), experiments=True,
             target_sets=TARGETS, schema_prefix="fitra_joint_fault_control", formatter=markdown,
             isolate_head=args.isolate_head)


if __name__ == "__main__":
    main()
