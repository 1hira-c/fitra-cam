"""Small/slow numerical controls and target-distance diagnostics; not live GT."""
import copy
import math
import random
import statistics

from evaluate import ROLES, norm, percentile, rms, sub
from hip_sensitivity import DISTANCE, candidate_names
from synthetic import pose

BASE = "soft_bones_head_leaf"
CAPS = {"distance_2mm": .002, "distance_5mm": .005}


def cases():
    return [
        {"id": "static"},
        {"id": "slow_5mm_s", "events": [{"id": "rise_half", "start_s": 1., "end_s": 5., "level_m": .01, "direction": 1}], "plateau_start_s": 5.},
        {"id": "micro_1mm", "frequency_hz": .5, "amplitude_m": .001},
        {"id": "small_5mm", "frequency_hz": .5, "amplitude_m": .005},
        {"id": "start_stop_reverse", "events": [
            {"id": "rise_half", "start_s": 1., "end_s": 2., "level_m": .01, "direction": 1},
            {"id": "reverse_zero", "start_s": 3., "end_s": 5., "level_m": 0., "direction": -1}], "plateau_start_s": 5.},
        {"id": "step_100mm", "events": [
            {"id": "rise_half", "start_s": 2., "end_s": 4., "level_m": .05, "direction": 1},
            {"id": "fall_half", "start_s": 5., "end_s": 7., "level_m": .05, "direction": -1}], "plateau_start_s": 5.},
        {"id": "dropout_slow"}]


def displacement(case, t):
    name = case["id"]
    if name == "static": return 0.
    if name in ("slow_5mm_s", "dropout_slow"): return .005*min(4., max(0., t-1))
    if "frequency_hz" in case:
        return case["amplitude_m"]*math.sin(2*math.pi*case["frequency_hz"]*(t-1)) if t >= 1 else 0.
    if name == "start_stop_reverse":
        return .02*min(1., max(0., t-1))-.02*min(2., max(0., t-3))
    if name == "step_100mm": return .1 if 2 <= t < 5 else 0.
    raise ValueError("unknown distance control")


def dataset(noise_mm=3., hz=60, seed=20260922):
    if (type(noise_mm) not in (int, float) or not math.isfinite(noise_mm) or not 0 <= noise_mm <= 20
            or type(hz) is not int or not 5 <= hz <= 120 or type(seed) is not int):
        raise ValueError("invalid distance control noise/cadence/seed")
    definitions = cases()
    header = {"schema": "fitra_temporal_input_v1", "input_stage": "raw_triangulated", "kp_format": "halpe26",
              "coordinate_frame": "fitra_world_z_up_m", "reference_kind": "synthetic_known",
              "time_basis": "synthetic_nominal", "nominal_hz": hz, "subject_height_m": 1.7,
              "provenance": {"generator": "distance_controls.py", "distance_controls": definitions,
                  "noise_mm_per_axis_std": noise_mm, "seed": seed, "seconds_per_case": 7,
                  "warmup_reports": hz, "axis": "world_x", "kind": "numerical_controls_not_real_pose_accuracy"}}
    rows = []
    for case in definitions:
        rng = random.Random(f"{seed}:{case['id']}")
        for i in range(7*hz):
            t = i/hz
            truth = pose()
            for j in truth:
                if j: j[0] += displacement(case, t)
            noisy = copy.deepcopy(truth)
            for j in noisy:
                if j:
                    for a in range(3): j[a] += rng.gauss(0, noise_mm*.001)
            boundary = case["id"] == "dropout_slow" and 2.5 <= t < 2.6
            if case["id"] == "dropout_slow" and 4 <= t < 4.1:
                for k in (15, 20, 22, 24): noisy[k] = None
            n = len(rows)
            rows.append({"seq": n, "t_ns": round(n*1e9/hz), "segment": case["id"],
                         "phase": "static" if case["id"] == "static" else "motion", "evaluate": i >= hz,
                         "state": "boundary" if boundary else "fresh", "joints": None if boundary else noisy,
                         "reference_joints": None if boundary else truth})
    return header, rows


def harmonic_fit(times, values, frequency):
    """OLS on constant + sin + cos, also valid for partial cycles. No resampling."""
    if len(times) != len(values): raise ValueError("harmonic sample count mismatch")
    if type(frequency) not in (int, float) or not math.isfinite(frequency) or frequency <= 0:
        raise ValueError("invalid harmonic frequency")
    if len(times) < 12: return None
    if not all(type(v) in (int, float) and math.isfinite(v) for v in [*times, *values]):
        raise ValueError("invalid harmonic samples")
    s, c = ([f(2*math.pi*frequency*t) for t in times] for f in (math.sin, math.cos))
    sm, cm, ym = (statistics.mean(v) for v in (s, c, values))
    s, c, y = ([v-m for v in vs] for vs, m in ((s, sm), (c, cm), (values, ym)))
    ss, cc, sc = sum(v*v for v in s), sum(v*v for v in c), sum(a*b for a,b in zip(s,c))
    sy, cy = sum(a*b for a,b in zip(s,y)), sum(a*b for a,b in zip(c,y))
    det = ss*cc-sc*sc
    if det <= 1e-12*max(1., ss*cc): return None
    a, b = (sy*cc-cy*sc)/det, (cy*ss-sy*sc)/det
    return {"sin_m": a, "cos_m": b, "amplitude_m": math.hypot(a,b),
            "residual_rms_m": rms([v-a*x-b*z for v,x,z in zip(y,s,c)]), "support": len(times)}


def harmonic_comparison(times, reference, output, frequency):
    if len(times) != len(reference) or len(times) != len(output):
        raise ValueError("harmonic sample count mismatch")
    pairs = [(t,r,y) for t,r,y in zip(times,reference,output) if r is not None and y is not None]
    if not pairs: return {"pairs": 0, "amplitude_ratio": None, "phase_lag_ms": None}
    t, r, y = map(list, zip(*pairs))
    rf, yf = harmonic_fit(t,r,frequency), harmonic_fit(t,y,frequency)
    result = {"pairs": len(pairs), "reference_fit": rf, "output_fit": yf,
              "amplitude_ratio": None, "phase_lag_ms": None}
    if rf and yf and rf["amplitude_m"] > 1e-7:
        result["amplitude_ratio"] = yf["amplitude_m"]/rf["amplitude_m"]
        # A nearly frozen output has no meaningful phase; don't report zero lag.
        if result["amplitude_ratio"] >= .1:
            angle = math.atan2(rf["cos_m"], rf["sin_m"])-math.atan2(yf["cos_m"], yf["sin_m"])
            result["phase_lag_ms"] = math.atan2(math.sin(angle),math.cos(angle))/(2*math.pi*frequency)*1000
    return result


def crossing(times, seqs, values, event, hz):
    # Retrospective metric: first sample of three consecutive qualifying reports.
    # The confirmation samples never enter a candidate filter.
    for i in range(len(times)-2):
        if not event["start_s"] <= times[i] < event["end_s"]: continue
        window = range(i,i+3)
        if (any(values[j] is None or times[j] >= event["end_s"] or
                event["direction"]*(values[j]-event["level_m"]) < -1e-8 for j in window)
                or any(seqs[j] != seqs[j-1]+1 or times[j]-times[j-1] > 2.5/hz for j in range(i+1,i+3))):
            continue
        return times[i]
    return None


def score_distance(header, source, out_header, output):
    if candidate_names(out_header) != DISTANCE or len(source) != len(output):
        raise ValueError("complete ten-candidate distance replay required")
    marker = out_header["distance_experiment"]
    if marker.get("base_candidate") != BASE or marker.get("position_only") is not True or marker.get("cap_m") != CAPS:
        raise ValueError("distance experiment semantics mismatch")
    grouped = {}
    for s,o in zip(source,output):
        if (s["seq"],s["t_ns"]) != (o["seq"],o["t_ns"]): raise ValueError("distance lineage mismatch")
        grouped.setdefault(s["segment"], []).append((s,o))
    caps = []
    for name,cap in CAPS.items():
        for k,role in enumerate(ROLES):
            errors, eligible = [], 0
            for s,o in zip(source,output):
                a,b = o["candidates"][BASE][k], o["candidates"][name][k]
                if (a is None) != (b is None): raise ValueError("distance filter changed output availability")
                if a is not None:
                    if {f:v for f,v in a.items() if f != "position_m"} != {f:v for f,v in b.items() if f != "position_m"}:
                        raise ValueError("distance filter changed orientation/confidence")
                    delta = norm(sub(a["position_m"],b["position_m"]))
                    if not math.isfinite(delta) or delta > cap+2e-6: raise ValueError("distance cap violated")
                    if s["state"] == "boundary": raise ValueError("distance output across boundary")
                    if s.get("evaluate",True): errors.append(delta*1000)
                if s["state"] == "fresh" and s.get("evaluate",True): eligible += 1
            caps.append({"candidate": name, "role": role, "eligible": eligible, "pairs": len(errors),
                         "max_deviation_mm": max(errors) if errors else None,
                         "p95_deviation_mm": percentile(errors,.95), "cap_mm": cap*1000})
    details = []
    definitions = header.get("provenance",{}).get("distance_controls",[])
    if definitions and (header["time_basis"] != "synthetic_nominal" or header["reference_kind"] != "synthetic_known"
                        or definitions != cases()):
        raise ValueError("controlled motion metadata mismatch")
    for case in definitions:
        rows = grouped[case["id"]]
        origin = rows[0][0]["t_ns"]
        scored = [(s,o) for s,o in rows if s["state"] == "fresh" and s.get("evaluate",True)]
        times = [(s["t_ns"]-origin)*1e-9 for s,o in scored]
        seqs = [s["seq"] for s,o in scored]
        for k,role in enumerate(ROLES):
            first = next((o["reference"][k] for s,o in rows if o["reference"][k] is not None), None)
            if first is None: raise ValueError("controlled tracker reference missing")
            x0 = first["position_m"][0]
            refs = [o["reference"][k] for s,o in scored]
            rx = [r["position_m"][0]-x0 if r else None for r in refs]
            for name in DISTANCE:
                ys = [o["candidates"][name][k] for s,o in scored]
                yx = [y["position_m"][0]-x0 if y else None for y in ys]
                errors = [norm(sub(r["position_m"],y["position_m"]))*1000 for r,y in zip(refs,ys) if r and y]
                d = {"case": case["id"], "candidate": name, "role": role, "eligible": len(scored),
                     "pairs": len(errors), "same_time_rmse_mm": rms(errors), "harmonic": None, "events": [],
                     "plateau_error_p95_mm": None}
                if "frequency_hz" in case:
                    d["harmonic"] = harmonic_comparison(times,rx,yx,case["frequency_hz"])
                for event in case.get("events",[]):
                    rt = crossing(times,seqs,rx,event,header["nominal_hz"])
                    yt = crossing(times,seqs,yx,event,header["nominal_hz"])
                    d["events"].append({"id": event["id"], "reference_time_s": rt, "output_time_s": yt,
                        "crossing_delay_ms": (yt-rt)*1000 if yt is not None and rt is not None else None,
                        "output_censored": yt is None})
                if "plateau_start_s" in case:
                    d["plateau_error_p95_mm"] = percentile([norm(sub(r["position_m"],y["position_m"]))*1000
                        for t,r,y in zip(times,refs,ys) if t >= case["plateau_start_s"] and r and y],.95)
                details.append(d)
    return {"schema": "fitra_distance_report_v1", "kind": "numerical_diagnostics_not_live_acceptance",
            "caps": caps, "controls": details}


def markdown(report):
    def fmt(value): return "-" if value is None else f"{value:.3f}"
    lines = ["# Distance residual diagnostics", "", "Target-distance caps are NOT anatomical error or live latency bounds.", "",
             "| Candidate | Role | Pairs / eligible | Max deviation mm | Cap mm |", "|---|---|---:|---:|---:|"]
    for d in report["caps"]:
        lines.append(f"| {d['candidate']} | {d['role']} | {d['pairs']} / {d['eligible']} | {fmt(d['max_deviation_mm'])} | {d['cap_mm']} |")
    if report["controls"]:
        lines += ["", "## Controlled waist motion (all roles in JSON)", "",
                  "Harmonic gain/phase are fitted to the known frequency, not whole-clip lag or human ratings.",
                  "Crossing uses three consecutive reports; missing crossings remain censored, not zero.", "",
                  "| Case | Candidate | RMSE mm | Harmonic gain | Phase lag ms | Event crossing delays ms |",
                  "|---|---|---:|---:|---:|---|"]
        for d in report["controls"]:
            if d["role"] != "waist" or d["candidate"] not in ("minimal","kalman","current",BASE,*CAPS): continue
            h = d["harmonic"] or {}
            events = ", ".join(e["id"]+":"+fmt(e["crossing_delay_ms"]) for e in d["events"])
            lines.append(f"| {d['case']} | {d['candidate']} | {fmt(d['same_time_rmse_mm'])} | "
                         f"{fmt(h.get('amplitude_ratio'))} | {fmt(h.get('phase_lag_ms'))} | {events} |")
    return "\n".join(lines)+"\n"
