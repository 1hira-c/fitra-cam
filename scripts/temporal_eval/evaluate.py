#!/usr/bin/env python3
"""Score time-aligned tracker outputs. Python standard library; offline only."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics

ROLES = ("left_upper_arm", "right_upper_arm", "chest", "waist", "left_upper_leg",
         "right_upper_leg", "left_lower_leg", "right_lower_leg", "left_foot", "right_foot")


def read_jsonl(path):
    with Path(path).open() as handle:
        rows = [json.loads(line) for line in handle]
    if len(rows) < 2:
        raise ValueError("header and at least one report required")
    return rows[0], rows[1:]


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def sub(a, b):
    return [x-y for x, y in zip(a, b)]


def rms(values):
    return math.sqrt(statistics.mean(x*x for x in values)) if values else None


def percentile(values, fraction):
    if not values:
        return None
    v = sorted(values)
    p = (len(v)-1)*fraction
    lo = math.floor(p)
    return v[lo] + (v[math.ceil(p)]-v[lo])*(p-lo)


def unit_quat(q):
    if len(q) != 4 or not all(math.isfinite(x) for x in q) or norm(q) < 1e-8:
        raise ValueError("invalid quaternion")
    n = norm(q)
    return [x/n for x in q]


def angle_deg(a, b):
    a, b = unit_quat(a), unit_quat(b)
    return math.degrees(2*math.acos(min(1, abs(sum(x*y for x, y in zip(a, b))))))


def spread(vectors):
    if len(vectors) < 2:
        return None
    center = [statistics.mean(v[c] for v in vectors) for c in range(len(vectors[0]))]
    return rms([norm(sub(v, center)) for v in vectors])


def rotation_spread(quats):
    if len(quats) < 2:
        return None
    qs = [unit_quat(q) for q in quats]
    aligned = [[-x for x in q] if sum(a*b for a, b in zip(q, qs[0])) < 0 else q for q in qs]
    center = [statistics.mean(q[c] for q in aligned) for c in range(4)]
    if norm(center) < 1e-8:
        return None
    return rms([angle_deg(q, center) for q in qs])


def swing_twist_deg(reference, output):
    """Relative rotation in reference-local coordinates, forward axis +Z.

    Swing is the angle between bone directions. Twist is the signed shortest
    residual about +Z, not an Euler angle. At an antiparallel forward direction
    (180-degree swing), twist is not uniquely defined and is returned as None.
    """
    aw, ax, ay, az = unit_quat(reference)
    bw, bx, by, bz = unit_quat(output)
    # conjugate(reference) * output
    w = aw*bw + ax*bx + ay*by + az*bz
    x = aw*bx - ax*bw - ay*bz + az*by
    y = aw*by + ax*bz - ay*bw - az*bx
    z = aw*bz - ax*by + ay*bx - az*bw
    projected = math.hypot(w, z)
    swing = math.degrees(2*math.atan2(math.hypot(x, y), projected))
    twist = None if projected < 1e-6 else (math.degrees(2*math.atan2(z, w))+180) % 360-180
    return swing, twist


def forward_vector(quat):
    w, x, y, z = unit_quat(quat)
    return [2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)]


def rotation_metrics(times, seqs, refs, outs, nominal_s, jump_deg):
    swings, twists = [], []
    reference_pairs = 0
    unobservable_reference = 0
    for ref, out in zip(refs, outs):
        if ref is None or out is None:
            continue
        reference_pairs += 1
        swing, twist = swing_twist_deg(ref["quat_wxyz"], out["quat_wxyz"])
        swings.append(swing)
        # A forward-only reference may contain an arbitrary valid quaternion.
        # Missing legacy observability metadata is unknown, not full confidence.
        if ref.get("roll_confidence", 0) <= 0:
            unobservable_reference += 1
        elif twist is not None:
            twists.append(abs(twist))
    rates, steps, excesses = [], [], []
    reference_steps, output_steps = [], []
    for i in range(1, len(times)):
        dt = times[i]-times[i-1]
        # Never turn a dropout or reappearance into an apparent per-frame jump.
        if seqs[i] != seqs[i-1]+1 or not 0 < dt <= 2.5*nominal_s:
            continue
        if outs[i-1] is None or outs[i] is None:
            continue
        _, out_twist = swing_twist_deg(outs[i-1]["quat_wxyz"], outs[i]["quat_wxyz"])
        if out_twist is None:
            continue
        step = abs(out_twist)
        steps.append(step)
        rates.append(step/dt)
        if (refs[i-1] is None or refs[i] is None or refs[i-1].get("roll_confidence", 0) <= 0
                or refs[i].get("roll_confidence", 0) <= 0):
            continue
        _, ref_twist = swing_twist_deg(refs[i-1]["quat_wxyz"], refs[i]["quat_wxyz"])
        if ref_twist is not None:
            reference_steps.append(abs(ref_twist))
            output_steps.append(step)
            excesses.append(max(0, step-abs(ref_twist)))
    ref_path = sum(reference_steps)
    return {"same_time_swing_p95_deg": percentile(swings, .95),
            "same_time_twist_p95_deg": percentile(twists, .95),
            "twist_reference_pairs": reference_pairs, "twist_defined_pairs": len(twists),
            "twist_unobservable_reference_pairs": unobservable_reference,
            "twist_undefined_pairs": reference_pairs-len(twists)-unobservable_reference,
            "twist_step_p95_deg": percentile(steps, .95),
            "twist_step_max_deg": max(steps) if steps else None,
            "twist_rate_p95_deg_s": percentile(rates, .95), "twist_step_support": len(steps),
            "twist_jump_count": sum(s >= jump_deg for s in steps) if steps else None,
            "excess_twist_step_p95_deg": percentile(excesses, .95),
            "twist_path_pair_support": len(reference_steps),
            "reference_twist_path_deg": ref_path if reference_steps else None,
            # Path length, not amplitude: high-frequency jitter can inflate it.
            # Always read alongside same-time error and jump/coverage metrics.
            "twist_path_length_ratio": sum(output_steps)/ref_path if ref_path >= 5 else None}


def estimate_lag(times, reference, output, nominal_s, max_steps=15):
    """Diagnostic only. Positive lag means output follows an older reference.

    Same support at every candidate shift, no interpolation across missing values
    or >2.5-report gaps. Primary errors are NEVER computed on this shifted data.
    Returns no estimate for frozen outputs or insufficient/nonunique evidence.
    """
    import bisect

    empty = {"lag_steps": None, "lag_ms": None, "correlation": None,
             "support": 0, "at_search_boundary": False}
    for vectors, reason in ((reference, "reference_motion_unobservable"),
                            (output, "frozen_or_unobservable_output")):
        available = [v for v in vectors if v is not None]
        if len(available) >= 2 and spread(available) < 1e-5:
            return {**empty, "reason": reason}

    def interpolate(t):
        j = bisect.bisect_left(times, t)
        if j < len(times) and abs(times[j]-t) < 1e-8:
            return reference[j]
        if j == 0 or j == len(times):
            return None
        if reference[j-1] is None or reference[j] is None or times[j]-times[j-1] > 2.5*nominal_s:
            return None
        f = (t-times[j-1])/(times[j]-times[j-1])
        return [a+(b-a)*f for a, b in zip(reference[j-1], reference[j])]

    shifts = list(range(-max_steps, max_steps+1))
    pairs = []
    for t, y in zip(times, output):
        if y is None:
            continue
        refs = [interpolate(t-step*nominal_s) for step in shifts]
        if all(r is not None for r in refs):
            pairs.append((refs, y))
    empty = {"lag_steps": None, "lag_ms": None, "correlation": None,
             "support": len(pairs), "at_search_boundary": False}
    if len(pairs) < 12:
        return {**empty, "reason": "insufficient_contiguous_motion"}
    ys = [y for _, y in pairs]
    cy = [statistics.mean(y[c] for y in ys) for c in range(3)]
    dy = [sub(y, cy) for y in ys]
    vy = sum(norm(y)**2 for y in dy)
    if vy/len(ys) < 1e-10:
        return {**empty, "reason": "frozen_or_unobservable_output"}
    correlations = []
    for k in range(len(shifts)):
        rs = [r[k] for r, _ in pairs]
        cr = [statistics.mean(r[c] for r in rs) for c in range(3)]
        dr = [sub(r, cr) for r in rs]
        vr = sum(norm(r)**2 for r in dr)
        correlations.append(sum(sum(a*b for a, b in zip(r, y)) for r, y in zip(dr, dy)) /
                            math.sqrt(vr*vy) if vr/len(rs) > 1e-10 else -2)
    best = max(range(len(shifts)), key=lambda k: correlations[k])
    if correlations[best] < 0.5:
        return {**empty, "reason": "weak_motion_match"}
    # Constant-velocity / periodic ambiguity must not masquerade as zero lag.
    near = [k for k, c in enumerate(correlations) if c > correlations[best]-1e-5]
    if len(near) > 1:
        return {**empty, "reason": "ambiguous_motion_match"}
    return {"lag_steps": shifts[best], "lag_ms": shifts[best]*nominal_s*1000,
            "correlation": correlations[best], "support": len(pairs),
            "at_search_boundary": abs(shifts[best]) == max_steps, "reason": None}


def validate_tracker(t):
    if t is None:
        return
    p = t["position_m"]
    if len(p) != 3 or not all(math.isfinite(x) for x in p):
        raise ValueError("invalid output position")
    unit_quat(t["quat_wxyz"])
    for field in ("roll_confidence", "swing_confidence"):
        if field in t and (not math.isfinite(t[field]) or not 0 <= t[field] <= 1):
            raise ValueError("invalid rotation confidence")


def score(input_header, source, output_header, outputs, twist_jump_deg=45.0):
    if input_header["schema"] != "fitra_temporal_input_v1" or output_header["schema"] != "fitra_temporal_output_v1":
        raise ValueError("unsupported schema")
    if not source or len(source) != len(outputs):
        raise ValueError("output dropped/added reports")
    if not math.isfinite(twist_jump_deg) or not 0 < twist_jump_deg <= 180:
        raise ValueError("twist jump diagnostic threshold must be in (0, 180]")
    hz = float(input_header["nominal_hz"])
    if not math.isfinite(hz) or not 5 <= hz <= 240 or hz != output_header["nominal_hz"]:
        raise ValueError("invalid/mismatched report frequency")
    candidates = output_header["candidates"]
    if not candidates or len(candidates) != len(set(candidates)):
        raise ValueError("missing/duplicate candidate names")
    groups, previous_time, previous_seq, previous_segment = {}, -1, -1, None
    previous_phase = None
    previous_fresh = False
    run = 0
    missing_sequence_reports = 0
    for s, o in zip(source, outputs):
        if any(type(s[k]) is not int or s[k] < 0 for k in ("seq", "t_ns")):
            raise ValueError("source time and sequence must be nonnegative integers")
        if not isinstance(s["segment"], str) or not s["segment"]:
            raise ValueError("nonempty continuity segment required")
        if (s["seq"], s["t_ns"]) != (o["seq"], o["t_ns"]):
            raise ValueError("reference/output time or sequence misalignment")
        if s["t_ns"] <= previous_time:
            raise ValueError("duplicate/nonmonotonic source time")
        if s["segment"] == previous_segment:
            if s["seq"] <= previous_seq:
                raise ValueError("duplicate/nonmonotonic source sequence")
            missing_sequence_reports += s["seq"]-previous_seq-1
        if s["state"] not in ("fresh", "boundary"):
            raise ValueError("unknown source state")
        fresh = s["state"] == "fresh"
        required_reset = (not fresh or not previous_fresh or s["segment"] != previous_segment
                          or s["t_ns"]-previous_time > 500_000_000)
        if type(o["reset"]) is not bool or (required_reset and not o["reset"]):
            raise ValueError("output omitted required lifecycle reset")
        phase = s.get("phase", "unlabelled")
        # Never combine separate still poses into a false static-jitter window.
        if (o["reset"] or s["segment"] != previous_segment or phase != previous_phase
                or s["t_ns"]-previous_time > 500_000_000):
            run += 1
        if set(o["candidates"]) != set(candidates):
            raise ValueError("candidate set changed")
        for ts in [o["reference"], *o["candidates"].values()]:
            if len(ts) != len(ROLES):
                raise ValueError("expected ten tracker slots")
            for t in ts:
                validate_tracker(t)
            if s["state"] == "boundary" and any(t is not None for t in ts):
                raise ValueError("boundary output must be unavailable")
        if input_header["reference_kind"] == "none" and any(r is not None for r in o["reference"]):
            raise ValueError("reference present despite reference_kind=none")
        if phase not in ("static", "motion", "unlabelled"):
            raise ValueError("unknown phase")
        if type(s.get("evaluate", True)) is not bool:
            raise ValueError("evaluate must be boolean")
        if not s.get("evaluate", True):
            run += 1
        if s["state"] == "fresh" and s.get("evaluate", True):
            groups.setdefault((run, s["segment"], phase), []).append((s, o))
        previous_time, previous_seq, previous_segment = s["t_ns"], s["seq"], s["segment"]
        previous_phase = phase
        previous_fresh = fresh

    details = []
    for (run, segment, phase), rows in groups.items():
        times = [(s["t_ns"]-rows[0][0]["t_ns"])*1e-9 for s, _ in rows]
        seqs = [s["seq"] for s, _ in rows]
        for role, name in enumerate(ROLES):
            refs = [o["reference"][role] for _, o in rows]
            expected = sum(r is not None for r in refs)
            for candidate in candidates:
                outs = [o["candidates"][candidate][role] for _, o in rows]
                pairs = [(r, y) for r, y in zip(refs, outs) if r is not None and y is not None]
                errors = [1000*norm(sub(y["position_m"], r["position_m"])) for r, y in pairs]
                # A forward-only reference has arbitrary roll: full-quaternion error is unknown.
                angles = [angle_deg(y["quat_wxyz"], r["quat_wxyz"]) for r, y in pairs
                          if r.get("roll_confidence", 0) > 0]
                values = [y["position_m"] for y in outs if y is not None]
                rotations = [y["quat_wxyz"] for y in outs if y is not None]
                ref_spread = spread([r["position_m"] for r, _ in pairs])
                out_spread = spread([y["position_m"] for _, y in pairs])
                lag = estimate_lag(times, [r["position_m"] if r else None for r in refs],
                                   [y["position_m"] if y else None for y in outs], 1/hz) if phase == "motion" else None
                swing_lag = estimate_lag(times,
                    [forward_vector(r["quat_wxyz"]) if r else None for r in refs],
                    [forward_vector(y["quat_wxyz"]) if y else None for y in outs], 1/hz) if phase == "motion" else None
                details.append({"run": run, "segment": segment, "phase": phase, "role": name,
                    "candidate": candidate, "reports": len(rows), "reference_available": expected,
                    "paired": len(pairs), "output_available": len(values),
                    "coverage": len(pairs)/expected if expected else None,
                    "same_time_rmse_mm": rms(errors), "same_time_p95_mm": percentile(errors, .95),
                    "same_time_angle_p95_deg": percentile(angles, .95),
                    "same_time_angle_support": len(angles),
                    "static_jitter_rms_mm": 1000*spread(values) if phase == "static" and len(values) > 1 else None,
                    "static_rotation_rms_deg": rotation_spread(rotations) if phase == "static" else None,
                    "motion_amplitude_ratio": out_spread/ref_spread if phase == "motion" and
                        ref_spread is not None and ref_spread > .001 else None,
                    "position_lag": lag, "swing_lag": swing_lag,
                    "rotation": rotation_metrics(times, seqs, refs, outs, 1/hz, twist_jump_deg)})
    summary = {}
    for candidate in candidates:
        ds = [d for d in details if d["candidate"] == candidate]
        processing = [o["processing_ms"][candidate] for s,o in zip(source,outputs)
                      if s["state"] == "fresh" and "processing_ms" in o]
        if any(not math.isfinite(v) or v < 0 for v in processing):
            raise ValueError("invalid processing time")
        def median_field(field):
            vs = [d[field] for d in ds if d[field] is not None]
            return statistics.median(vs) if vs else None
        lags = [d["position_lag"]["lag_ms"] for d in ds if d["position_lag"] and
                d["position_lag"]["lag_ms"] is not None and not d["position_lag"]["at_search_boundary"]]
        expected_reports = sum(d["reports"] for d in ds)
        summary[candidate] = {"median_static_jitter_mm": median_field("static_jitter_rms_mm"),
            "offline_call_p50_ms": percentile(processing,.5), "offline_call_p95_ms": percentile(processing,.95),
            "median_static_rotation_deg": median_field("static_rotation_rms_deg"),
            "median_position_lag_ms": statistics.median(lags) if lags else None,
            "position_lag_estimates": len(lags),
            "median_same_time_rmse_mm": median_field("same_time_rmse_mm"),
            "median_motion_amplitude_ratio": median_field("motion_amplitude_ratio"),
            "reference_availability": sum(d["reference_available"] for d in ds)/expected_reports
                if expected_reports else None,
            "output_availability": sum(d["output_available"] for d in ds)/expected_reports
                if expected_reports else None,
            "reference_coverage": sum(d["paired"] for d in ds)/sum(d["reference_available"] for d in ds)
                if sum(d["reference_available"] for d in ds) else None}
    cases = []
    for segment in dict.fromkeys(d["segment"] for d in details):
        for candidate in candidates:
            ds = [d for d in details if d["segment"] == segment and d["candidate"] == candidate]
            defined = [d for d in ds if d["rotation"]["same_time_twist_p95_deg"] is not None]
            worst = max(defined, key=lambda d: d["rotation"]["same_time_twist_p95_deg"]) if defined else None
            supported = [d for d in ds if d["rotation"]["twist_step_support"] > 0]
            cases.append({"segment": segment, "candidate": candidate,
                "worst_twist_p95_deg": worst["rotation"]["same_time_twist_p95_deg"] if worst else None,
                "worst_twist_role": worst["role"] if worst else None,
                "twist_jump_count": sum(d["rotation"]["twist_jump_count"] for d in supported) if supported else None,
                "twist_step_support": sum(d["rotation"]["twist_step_support"] for d in ds),
                "twist_unobservable_reference_pairs": sum(d["rotation"]["twist_unobservable_reference_pairs"] for d in ds),
                "twist_undefined_pairs": sum(d["rotation"]["twist_undefined_pairs"] for d in ds)})
    return {"schema": "fitra_temporal_report_v1", "metrics_revision": 3,
            "reference_kind": input_header["reference_kind"],
            "time_basis": input_header["time_basis"], "source_provenance": input_header.get("provenance", {}),
            "replay_settings": output_header, "reports": len(source),
            "unscored_reports": sum(not s.get("evaluate", True) for s in source),
            "missing_sequence_reports": missing_sequence_reports,
            "acceptance": "not_calibrated_against_human_AB",
            "diagnostic_thresholds": {"twist_jump_deg": twist_jump_deg,
                                      "minimum_reference_twist_path_deg": 5},
            "limitations": ["Offline filter response, not capture-to-display latency.",
                "One update per report; live paced re-sampling is not emulated.",
                "Position/swing lag are curve diagnostics, not onset/stop/twist or display delay.",
                "Twist is undefined at antiparallel bone directions; missing decomposition is not zero error.",
                "Reference roll_confidence=0 or missing is unobservable, never a zero twist error.",
                "Twist jumps/rates are diagnostics, not defects without motion/reference context.",
                "No automatic winner: coverage, amplitude and per-role failures must be checked.",
                "Missing reference is unavailable, never zero error; synthetic/pseudo labels are not measured ground truth."],
            "summary": summary, "case_summary": cases, "details": details}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_input_binding(path, output_header):
    value = 14695981039346656037
    for byte in Path(path).read_bytes():
        value = ((value ^ byte)*1099511628211) & ((1 << 64)-1)
    if str(value) != output_header.get("input_fnv1a64"):
        raise ValueError("replay output belongs to different input bytes")


def markdown(report):
    def f(v):
        return "—" if v is None else f"{v:.3f}"
    lines = ["# 3D後段・オフライン比較", "", f"参照: `{report['reference_kind']}` / 時刻: `{report['time_basis']}`",
             "", "以下は部位・区間別指標の中央値。体感遅延の実測値や合否ではありません。",
             f"入力で指定した採点除外: {report['unscored_reports']} reports（全候補共通、履歴更新は継続）。",
             "参照有効率は採点対象の部位×レポートに対する参照の割合、対応率は有効参照に対する候補出力の割合です。",
             "", "| 設定 | 静止揺れ mm | 静止回転 ° | 位置遅れ ms | 同時刻誤差 mm | 振幅比 | 参照有効率 | 参照との対応率 |",
             "|---|---:|---:|---:|---:|---:|---:|---:|"]
    fields = ("median_static_jitter_mm", "median_static_rotation_deg", "median_position_lag_ms",
              "median_same_time_rmse_mm", "median_motion_amplitude_ratio", "reference_availability", "reference_coverage")
    for name, s in report["summary"].items():
        lines.append(f"| {name} | " + " | ".join(f(s[k]) for k in fields) + " |")
    lines.extend(["", "## ケース別の回転診断", "",
        "全体中央値に埋もれる局所的な暴れを確認する表です。ひねり誤差は同時刻参照に対する値。",
        f"急変は連続レポート間で {report['diagnostic_thresholds']['twist_jump_deg']:g}° 以上という診断用しきい値で、合否ではありません。",
        "", "| ケース | 設定 | 最悪部位のひねり誤差p95 ° | 部位 | 急変回数 / 有効step | 参照不明 / 分解未定義 |",
        "|---|---|---:|---|---:|---:|"])
    def cell(value):
        return str(value).replace("|", "\\|").replace("\n", " ").replace("\r", " ")
    for c in report["case_summary"]:
        jumps = "—" if c["twist_jump_count"] is None else str(c["twist_jump_count"])
        lines.append(f"| {cell(c['segment'])} | {cell(c['candidate'])} | {f(c['worst_twist_p95_deg'])} | "
                     f"{c['worst_twist_role'] or '—'} | {jumps} / {c['twist_step_support']} | "
                     f"{c['twist_unobservable_reference_pairs']} / {c['twist_undefined_pairs']} |")
    lines.extend(["", "`minimal`もIK・床補正・信頼度ゲートを含みます。小さい揺れだけで優勝を決めません。",
                  "擬似教師との同時刻誤差は時間合わせなし。lag探索は別指標です。",
                  "部位・区間別、欠測率、探索端に達したlagは同名JSONで確認してください。", ""])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path, help="C++ replay output")
    parser.add_argument("report", type=Path, help="new JSON report; also writes .md")
    parser.add_argument("--binary", type=Path)
    parser.add_argument("--subject-profile", type=Path)
    parser.add_argument("--twist-jump-deg", type=float, default=45,
                        help="diagnostic jump threshold, not an acceptance threshold")
    args = parser.parse_args()
    md = args.report.with_suffix(".md")
    if args.report == md or args.report.exists() or md.exists():
        parser.error("choose new .json report and .md paths")
    ih, rows = read_jsonl(args.input)
    oh, outputs = read_jsonl(args.output)
    verify_input_binding(args.input, oh)
    report = score(ih, rows, oh, outputs, args.twist_jump_deg)
    report["artifacts"] = {name: {"path": str(path.resolve()), "sha256": digest(path)}
                           for name, path in (("input", args.input), ("output", args.output),
                                              ("binary", args.binary), ("subject_profile", args.subject_profile)) if path}
    with args.report.open("x") as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
        handle.write("\n")
    with md.open("x") as handle:
        handle.write(markdown(report))
    print(markdown(report))


if __name__ == "__main__":
    main()
