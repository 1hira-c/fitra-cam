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


def estimate_lag(times, reference, output, nominal_s, max_steps=15):
    """Diagnostic only. Positive lag means output follows an older reference.

    Same support at every candidate shift, no interpolation across missing values
    or >2.5-report gaps. Primary errors are NEVER computed on this shifted data.
    Returns no estimate for frozen outputs or insufficient/nonunique evidence.
    """
    import bisect

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


def score(input_header, source, output_header, outputs):
    if input_header["schema"] != "fitra_temporal_input_v1" or output_header["schema"] != "fitra_temporal_output_v1":
        raise ValueError("unsupported schema")
    if len(source) != len(outputs):
        raise ValueError("output dropped/added reports")
    hz = float(input_header["nominal_hz"])
    if not math.isfinite(hz) or hz <= 0 or hz != output_header["nominal_hz"]:
        raise ValueError("invalid/mismatched report frequency")
    candidates = output_header["candidates"]
    if not candidates or len(candidates) != len(set(candidates)):
        raise ValueError("missing/duplicate candidate names")
    groups, previous_time, previous_seq, previous_segment = {}, -1, -1, None
    previous_phase = None
    run = 0
    missing_sequence_reports = 0
    for s, o in zip(source, outputs):
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

    details = []
    for (run, segment, phase), rows in groups.items():
        times = [(s["t_ns"]-rows[0][0]["t_ns"])*1e-9 for s, _ in rows]
        for role, name in enumerate(ROLES):
            refs = [o["reference"][role] for _, o in rows]
            expected = sum(r is not None for r in refs)
            for candidate in candidates:
                outs = [o["candidates"][candidate][role] for _, o in rows]
                pairs = [(r, y) for r, y in zip(refs, outs) if r is not None and y is not None]
                errors = [1000*norm(sub(y["position_m"], r["position_m"])) for r, y in pairs]
                angles = [angle_deg(y["quat_wxyz"], r["quat_wxyz"]) for r, y in pairs]
                values = [y["position_m"] for y in outs if y is not None]
                rotations = [y["quat_wxyz"] for y in outs if y is not None]
                ref_spread = spread([r["position_m"] for r, _ in pairs])
                out_spread = spread([y["position_m"] for _, y in pairs])
                lag = estimate_lag(times, [r["position_m"] if r else None for r in refs],
                                   [y["position_m"] if y else None for y in outs], 1/hz) if phase == "motion" else None
                details.append({"run": run, "segment": segment, "phase": phase, "role": name,
                    "candidate": candidate, "reports": len(rows), "reference_available": expected,
                    "paired": len(pairs), "output_available": len(values),
                    "coverage": len(pairs)/expected if expected else None,
                    "same_time_rmse_mm": rms(errors), "same_time_p95_mm": percentile(errors, .95),
                    "same_time_angle_p95_deg": percentile(angles, .95),
                    "static_jitter_rms_mm": 1000*spread(values) if phase == "static" and len(values) > 1 else None,
                    "static_rotation_rms_deg": rotation_spread(rotations) if phase == "static" else None,
                    "motion_amplitude_ratio": out_spread/ref_spread if phase == "motion" and
                        ref_spread is not None and ref_spread > .001 else None,
                    "position_lag": lag})
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
        summary[candidate] = {"median_static_jitter_mm": median_field("static_jitter_rms_mm"),
            "offline_call_p50_ms": percentile(processing,.5), "offline_call_p95_ms": percentile(processing,.95),
            "median_static_rotation_deg": median_field("static_rotation_rms_deg"),
            "median_position_lag_ms": statistics.median(lags) if lags else None,
            "position_lag_estimates": len(lags),
            "median_same_time_rmse_mm": median_field("same_time_rmse_mm"),
            "median_motion_amplitude_ratio": median_field("motion_amplitude_ratio"),
            "reference_coverage": sum(d["paired"] for d in ds)/sum(d["reference_available"] for d in ds)
                if sum(d["reference_available"] for d in ds) else None}
    return {"schema": "fitra_temporal_report_v1", "reference_kind": input_header["reference_kind"],
            "time_basis": input_header["time_basis"], "source_provenance": input_header.get("provenance", {}),
            "replay_settings": output_header, "reports": len(source),
            "unscored_reports": sum(not s.get("evaluate", True) for s in source),
            "missing_sequence_reports": missing_sequence_reports,
            "acceptance": "not_calibrated_against_human_AB",
            "limitations": ["Offline filter response, not capture-to-display latency.",
                "One update per report; live paced re-sampling is not emulated.",
                "Lag is a position-curve diagnostic, not onset/stop/rotation delay.",
                "No automatic winner: coverage, amplitude and per-role failures must be checked.",
                "Missing reference is unavailable, never zero error; synthetic/pseudo labels are not measured ground truth."],
            "summary": summary, "details": details}


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
             "", "| 設定 | 静止揺れ mm | 静止回転 ° | 位置遅れ ms | 同時刻誤差 mm | 振幅比 | 参照coverage |",
             "|---|---:|---:|---:|---:|---:|---:|"]
    fields = ("median_static_jitter_mm", "median_static_rotation_deg", "median_position_lag_ms",
              "median_same_time_rmse_mm", "median_motion_amplitude_ratio", "reference_coverage")
    for name, s in report["summary"].items():
        lines.append(f"| {name} | " + " | ".join(f(s[k]) for k in fields) + " |")
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
    args = parser.parse_args()
    md = args.report.with_suffix(".md")
    if args.report == md or args.report.exists() or md.exists():
        parser.error("choose new .json report and .md paths")
    ih, rows = read_jsonl(args.input)
    oh, outputs = read_jsonl(args.output)
    verify_input_binding(args.input, oh)
    report = score(ih, rows, oh, outputs)
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
