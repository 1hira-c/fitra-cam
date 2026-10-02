#!/usr/bin/env python3
"""Causal joint-gate numerical controls; not measured pose accuracy or live acceptance."""
import argparse
import copy
import json
import math
from pathlib import Path
import shutil
import subprocess

from evaluate import (ROLES, angle_deg, norm, percentile, read_jsonl, rms, sub,
                      validate_tracker, verify_input_binding)
from hip_sensitivity import DISTANCE
from run_recording import artifact, write_json
from synthetic import pose, template_profile

GUARDED = "guarded_distance_5mm"
NAMES = DISTANCE + (GUARDED,)
MARKER = {"schema": "fitra_joint_outlier_experiment_v1", "base_candidate": "distance_5mm",
          "stage": "raw_3d_before_ik", "admission": "near_prediction_or_last_accepted",
          "residual_radius_m": .08, "acceleration_mps2": 30,
          "max_prediction_speed_mps": 3, "max_compensation_s": .05, "reacquire_reports": 3,
          "accepted_history_only": True, "velocity_seed": "consecutive_accepted_raw", "future_lookahead": False}
FLAGS = ("--ik-experiments", "--isolate-head", "--distance-experiments", "--diagnostics")


def cases():
    return ([{"id": f"{name}_{length}report", "target": index, "length_reports": length,
              "kind": "spike"}
             for name, index in (("knee", 13), ("ankle", 15), ("toe", 20)) for length in (1, 2)]
            + [{"id": "swapped_knees", "kind": "swap", "length_reports": 2},
               {"id": "sustained_plausible_shift", "kind": "true_shift"},
               {"id": "normal_start_stop", "kind": "start_stop"},
               {"id": "normal_high_speed", "kind": "high_speed"},
               {"id": "persistent_inconsistent_spikes", "kind": "expired"},
               {"id": "missing_left_foot", "kind": "missing"}])


def dataset(hz=60):
    if type(hz) is not int or not 20 <= hz <= 120:
        raise ValueError("control cadence must be an integer in [20,120]")
    definitions = cases()
    header = {"schema": "fitra_temporal_input_v1", "input_stage": "raw_triangulated",
              "kp_format": "halpe26", "coordinate_frame": "fitra_world_z_up_m",
              "nominal_hz": hz, "subject_height_m": 1.7, "reference_kind": "synthetic_known",
              "time_basis": "synthetic_nominal", "provenance": {
                  "generator": "outlier_controls.py", "cases": definitions,
                  "seconds_per_case": 3, "warmup_reports": hz,
                  "bone_prior": "fixed_synthetic_template_not_fitted_to_run",
                  "kind": "numerical_controls_not_measured_ground_truth",
                  "spike": "one left joint +0.3m world Y, score unchanged at .95",
                  "swap": "exchange left/right knee observations for two reports",
                  "shift": "all actual joints move +0.3m world X and remain there",
                  "expired": "one knee alternates +/-0.4m world Y for 0.3s",
                  "missing": "left ankle/bigtoe/smalltoe/heel absent for 0.3s"}}
    rows = []
    for case in definitions:
        for i in range(3*hz):
            t, truth = i/hz, pose()
            displacement = 0.
            if case["kind"] == "true_shift": displacement = .3 if i >= hz else 0.
            elif case["kind"] == "start_stop": displacement = .4*min(.5, max(0., t-1))
            elif case["kind"] == "high_speed": displacement = 3.*min(.25, max(0., t-1))
            for j in truth:
                if j is not None: j[0] += displacement
            observed = copy.deepcopy(truth)
            if case["kind"] == "spike" and hz <= i < hz+case["length_reports"]:
                observed[case["target"]][1] += .3
            elif case["kind"] == "swap" and hz <= i < hz+case["length_reports"]:
                observed[13], observed[14] = observed[14], observed[13]
            elif case["kind"] == "expired" and hz <= i < hz+round(.3*hz):
                observed[13][1] += .4 if i % 2 == 0 else -.4
            elif case["kind"] == "missing" and hz <= i < hz+round(.3*hz):
                for k in (15, 20, 22, 24): observed[k] = None
            n = len(rows)
            rows.append({"seq": n, "t_ns": round(n*1e9/hz), "segment": case["id"],
                         "state": "fresh", "phase": "motion", "evaluate": i >= hz,
                         "joints": observed, "reference_joints": truth})
    return header, rows


def verify_baselines(control_header, control, header, output):
    if tuple(control_header.get("candidates", [])) != DISTANCE or tuple(header.get("candidates", [])) != NAMES:
        raise ValueError("ten baseline and eleven experimental candidates required")
    if header.get("outlier_experiment") != MARKER:
        raise ValueError("joint gate semantics mismatch")
    if len(control) != len(output): raise ValueError("baseline report count mismatch")
    if {k: v for k, v in header.items() if k not in ("candidates", "outlier_experiment")} != {
            k: v for k, v in control_header.items() if k != "candidates"}:
        raise ValueError("existing replay header changed")
    for a, b in zip(control, output):
        if any(a[k] != b[k] for k in ("seq", "t_ns", "reset", "reference")):
            raise ValueError("baseline lineage/reference changed")
        for key in ("candidates", "diagnostics", "constraint_metrics"):
            if a[key] is None:
                if b[key] is not None: raise ValueError("baseline boundary metadata changed")
            elif any(a[key][n] != b[key][n] for n in DISTANCE):
                raise ValueError("existing candidate changed: "+key)


def validate_gate(diagnostic):
    if set(diagnostic) != {"post_kalman", "post_ik", "pre_smoothing", "post_gate", "joint_gate"}:
        raise ValueError("unexpected guarded diagnostic schema")
    joints, gate = diagnostic["post_gate"], diagnostic["joint_gate"]
    if len(joints) != 26 or len(gate) != 26:
        raise ValueError("26 gated joints/states required")
    for j, state in zip(joints, gate):
        if (set(state) != {"state", "rejected", "reacquired", "age_s"}
                or state["state"] not in ("accepted", "predicted", "unavailable")
                or type(state["rejected"]) is not bool or type(state["reacquired"]) is not bool):
            raise ValueError("invalid gate state")
        age = state["age_s"]
        if age is not None and (type(age) not in (int, float) or not math.isfinite(age) or age < 0):
            raise ValueError("invalid compensation age")
        if state["state"] == "accepted" and (j is None or age != 0):
            raise ValueError("accepted observation missing or stale")
        if state["state"] == "predicted" and (j is None or age is None or not 0 < age <= .05+1e-9):
            raise ValueError("prediction beyond deadline")
        if (state["state"] == "unavailable") != (j is None):
            raise ValueError("gate availability mismatch")
        if state["reacquired"] and state["state"] != "accepted":
            raise ValueError("reacquisition is not an accepted observation")
        if j is not None and (len(j) != 4 or any(type(x) not in (int, float) or not math.isfinite(x) for x in j)):
            raise ValueError("invalid gated joint")
        if state["state"] == "predicted" and j[3] != 0:
            raise ValueError("predicted geometry carries observation score")


def score_controls(input_header, source, output_header, output):
    if (tuple(output_header.get("candidates", [])) != NAMES or output_header.get("outlier_experiment") != MARKER
            or not source or len(source) != len(output)):
        raise ValueError("complete guarded replay required")
    groups = {}
    for s, o in zip(source, output):
        if (s["seq"], s["t_ns"]) != (o["seq"], o["t_ns"]):
            raise ValueError("source lineage mismatch")
        if s["state"] == "fresh":
            validate_gate(o["diagnostics"][GUARDED])
            for name in NAMES:
                if len(o["candidates"][name]) != len(ROLES): raise ValueError("tracker count mismatch")
                for t in o["candidates"][name]: validate_tracker(t)
        elif o["diagnostics"] is not None:
            raise ValueError("boundary diagnostics must be null")
        groups.setdefault(s["segment"], []).append((s, o))
    details, joint_states = [], []
    for segment, rows in groups.items():
        eligible = [(s, o) for s, o in rows if s["state"] == "fresh" and s.get("evaluate", True)]
        for k in range(26):
            states = [o["diagnostics"][GUARDED]["joint_gate"][k] for s, o in eligible]
            counts = {name: sum(x["state"] == name for x in states) for name in ("accepted", "predicted", "unavailable")}
            ages = [x["age_s"] for x in states if x["state"] == "predicted"]
            joint_states.append({"segment": segment, "joint": k, "eligible_reports": len(states), **counts,
                                 "rejected": sum(x["rejected"] for x in states),
                                 "reacquired": sum(x["reacquired"] for x in states),
                                 "max_prediction_age_s": max(ages) if ages else None})
        # A transition needs two consecutive, scored, fresh reports and no reset.
        # Its denominator is retained even when either output is invalid.
        transitions = [(a, b) for a, b in zip(rows, rows[1:]) if
                       a[0]["state"] == b[0]["state"] == "fresh"
                       and a[0].get("evaluate", True) and b[0].get("evaluate", True)
                       and b[0]["seq"] == a[0]["seq"]+1 and not b[1]["reset"]]
        for name in NAMES:
            costs = [o["processing_ms"][name] for s, o in eligible]
            if any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in costs):
                raise ValueError("invalid offline processing cost")
            for k, role in enumerate(ROLES):
                ts = [o["candidates"][name][k] for s, o in eligible]
                errors = [norm(sub(o["reference"][k]["position_m"], o["candidates"][name][k]["position_m"]))*1000
                          for s, o in eligible if o["reference"][k] is not None and o["candidates"][name][k] is not None]
                reference_support = sum(o["reference"][k] is not None for s, o in eligible)
                pairs = [(a[1]["candidates"][name][k], b[1]["candidates"][name][k]) for a, b in transitions]
                pairs = [(a, b) for a, b in pairs if a is not None and b is not None]
                position = [norm(sub(b["position_m"], a["position_m"]))*1000 for a, b in pairs]
                rotation = [angle_deg(a["quat_wxyz"], b["quat_wxyz"]) for a, b in pairs]
                details.append({"segment": segment, "candidate": name, "role": role,
                    "eligible_reports": len(eligible), "valid_reports": sum(t is not None for t in ts),
                    "reference_reports": reference_support, "reference_pairs": len(errors),
                    "same_time_reference_rmse_mm": rms(errors),
                    "eligible_transitions": len(transitions), "valid_transition_pairs": len(pairs),
                    "position_step_p95_mm": percentile(position, .95),
                    "position_step_max_mm": max(position) if position else None,
                    "position_steps_over_80mm": sum(x > 80 for x in position),
                    "rotation_step_p95_deg": percentile(rotation, .95),
                    "rotation_step_max_deg": max(rotation) if rotation else None,
                    "rotation_steps_over_45deg": sum(x > 45 for x in rotation),
                    "processing_ms": {"p50": percentile(costs, .5), "p95": percentile(costs, .95)}})
    return {"schema": "fitra_joint_outlier_controls_report_v1",
            "kind": "numerical_controls_not_real_pose_accuracy_or_live_acceptance",
            "reference_kind": input_header["reference_kind"], "gate": MARKER,
            "position_jump_threshold_mm": 80, "rotation_jump_threshold_deg": 45,
            "processing_kind": "offline_candidate_copy_cost_not_jetson_or_e2e_latency",
            "jump_pairing": "consecutive_scored_fresh_reports_no_boundary_or_reset",
            "joint_states": joint_states, "details": details}


def markdown(report):
    def fmt(value): return "-" if value is None else f"{value:.3f}"
    lines = ["# 関節別の外れ値棄却・短期補完の対照実験", "",
             ("人工入力による数値検証です。極端な飛びや回転の人工再現は、実際の姿勢精度や実機合格の証拠にはなりません。"
              if report["reference_kind"] == "synthetic_known" else
              "保存済みraw入力による数値比較です。正解参照がない区間の変化量は、実際の姿勢精度や実機合格の証拠にはなりません。"),
             "処理時間はオフラインの候補計算コストです。Jetson稼働時やE2Eの遅延は計測していません。",
             "raw入力を保持し、補完した関節のscoreは0にします。補完値で採用済み観測の履歴や期限を更新しません。", "",
             "| 場面 | 候補 | トラッカー | 有効 / 対象report | 有効差分 / 対象差分 | 位置変化最大 mm | 回転変化最大 deg | 参照RMSE mm |",
             "|---|---|---|---:|---:|---:|---:|---:|"]
    for d in report["details"]:
        if d["candidate"] not in ("distance_5mm", GUARDED) or d["role"] not in ("left_upper_leg", "left_lower_leg", "left_foot", "waist"):
            continue
        lines.append(f"| {d['segment']} | {d['candidate']} | {d['role']} | {d['valid_reports']} / {d['eligible_reports']} | "
                     f"{d['valid_transition_pairs']} / {d['eligible_transitions']} | {fmt(d['position_step_max_mm'])} | "
                     f"{fmt(d['rotation_step_max_deg'])} | {fmt(d['same_time_reference_rmse_mm'])} |")
    lines += ["", "無効な差分は未評価として残し、変化ゼロとは数えません。JSONには全11候補・26関節の状態と分母を残しています。",
              "参照RMSEは参照がある有効reportの同時刻比較です。欠測・補完率と併せて読む必要があります。",
              "持続する誤推定が再取得条件を満たす可能性もあります。時間方向の整合性だけでは姿勢の正しさを確定できません。", ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay_binary", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--input", type=Path, help="existing raw temporal JSONL; copied unchanged")
    args = parser.parse_args()
    if args.output.exists(): parser.error("choose a new output directory")
    binary = artifact(args.replay_binary)
    profile = artifact(args.profile) if args.profile else None
    external = artifact(args.input) if args.input else None
    header, source = read_jsonl(args.input) if args.input else dataset()
    root = Path(__file__).parents[2]
    assets = {"binary": binary, "runner": artifact(Path(__file__)),
              "replay_source": artifact(root/"cpp/tools/temporal_replay.cpp"),
              "candidate_source": artifact(root/"cpp/tools/temporal_candidate.hpp"),
              "gate_source": artifact(root/"cpp/tools/temporal_outlier_gate.hpp"),
              "spatial_source": artifact(root/"cpp/tools/temporal_weighted_bones.hpp"),
              "distance_source": artifact(root/"cpp/tools/temporal_distance_filter.hpp")}
    if profile: assets["profile"] = profile
    if external: assets["source_input"] = external
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    source_file = output/"input.jsonl"
    if external: shutil.copyfile(external["path"], source_file)
    else:
        with source_file.open("x") as handle:
            for row in [header, *source]: handle.write(json.dumps(row, allow_nan=False)+"\n")
    if profile: profile_file = Path(profile["path"])
    elif external: profile_file = None  # Existing input uses its height prior, not an artificial template.
    else:
        profile_file = output/"synthetic-profile.json"
        write_json(profile_file, template_profile())
        assets["synthetic_profile"] = artifact(profile_file)
    assets["input"] = artifact(source_file)
    commands = []
    for filename, guarded in (("control.jsonl", False), ("replay.jsonl", True)):
        command = [binary["path"], str(source_file), str(output/filename)]
        if profile_file: command.append(str(profile_file))
        command += list(FLAGS)
        if guarded: command.append("--outlier-experiments")
        commands.append(command)
    write_json(output/"manifest.json", {"schema": "fitra_joint_outlier_controls_run_v1", "assets": assets,
        "commands": commands, "gate": MARKER, "runtime_applied": False,
        "source_kind": "external_raw_input" if external else "deterministic_synthetic_controls"})
    for i, command in enumerate(commands):
        with (output/f"replay-{i}.log").open("x") as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    ch, control = read_jsonl(output/"control.jsonl")
    oh, replay = read_jsonl(output/"replay.jsonl")
    for h in (ch, oh): verify_input_binding(source_file, h)
    verify_baselines(ch, control, oh, replay)
    report = score_controls(header, source, oh, replay)
    report["existing_ten_candidates_unchanged"] = True
    write_json(output/"report.json", report)
    with (output/"report.md").open("x") as handle: handle.write(markdown(report))
    if any(artifact(a["path"]) != a for a in assets.values()): raise ValueError("run assets changed")
    write_json(output/"complete.json", {"schema": "fitra_joint_outlier_controls_complete_v1",
        "reports": len(source), "existing_ten_candidates_unchanged": True, "runtime_applied": False,
        "artifacts": {f.name: artifact(f) for f in sorted(output.iterdir()) if f.is_file()}})
    print(output/"report.md")


if __name__ == "__main__": main()
