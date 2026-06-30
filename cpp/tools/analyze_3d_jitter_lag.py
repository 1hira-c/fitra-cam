#!/usr/bin/env python3
"""analyze_3d_jitter_lag — jitter / lag metrics for dump_keypoints_3d output.

Companion analyzer for the spatial-filtering verification harness
(docs/design/pose-3d-spatial-filtering.md, milestone M-infra). It reads the
per-frame JSONL emitted by ``tools/dump_keypoints_3d`` and scores two things,
one per subcommand:

  jitter  — per-joint 3D position standard deviation over a *stationary* clip.
            The lower the better. Run it on one file, or on several (e.g. a
            stage ON vs OFF run of the same clip) to get a side-by-side table
            with deltas. Core joints {neck, shoulders, hip_center, hips} are
            highlighted because the rigid-fit stages target them first.

  lag     — temporal delay of a *candidate* run relative to a *baseline* run of
            the same *motion* clip (e.g. tri-only vs tri+Kalman, or weak vs
            strong temporal smoothing), estimated by cross-correlating one
            joint's trajectory. Positive lag = candidate is delayed.

Dependencies: numpy + stdlib only (NumPy 1.x on the Jetson venv is fine).

Examples
--------
  # Stationary jitter of a single run (core joints + overall median)
  analyze_3d_jitter_lag.py jitter still.jsonl

  # Compare two stationary runs (stage OFF vs ON) on the same clip
  analyze_3d_jitter_lag.py jitter still_kalman_off.jsonl still_kalman_on.jsonl

  # Lag of a filtered run vs the raw triangulation on a motion clip
  analyze_3d_jitter_lag.py lag bend_no_kalman.jsonl bend_kalman.jsonl --joint 19
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass

import numpy as np

# Halpe26 joint names (COCO17 = indices 0..16, identical meaning).
JOINT_NAMES = {
    0: "nose", 1: "l_eye", 2: "r_eye", 3: "l_ear", 4: "r_ear",
    5: "l_shoulder", 6: "r_shoulder", 7: "l_elbow", 8: "r_elbow",
    9: "l_wrist", 10: "r_wrist", 11: "l_hip", 12: "r_hip",
    13: "l_knee", 14: "r_knee", 15: "l_ankle", 16: "r_ankle",
    17: "head_top", 18: "neck", 19: "hip_center",
    20: "l_big_toe", 21: "r_big_toe", 22: "l_small_toe", 23: "r_small_toe",
    24: "l_heel", 25: "r_heel",
}

# Core rigid-body joints the spatial stages stabilise first: pelvis {19,11,12}
# + shoulder girdle {18,5,6}.
CORE_JOINTS = [18, 5, 6, 19, 11, 12]


def joint_label(idx: int) -> str:
    return f"{idx}:{JOINT_NAMES.get(idx, '?')}"


@dataclass
class Loaded:
    path: str
    kp_format: str
    n_frames: int
    n_joints: int
    # positions[frame, joint, 0:3] = xyz (meters); valid[frame, joint] = bool.
    positions: np.ndarray
    valid: np.ndarray


def load_jsonl(path: str) -> Loaded:
    """Load a dump_keypoints_3d JSONL into dense (frame x joint x 3) arrays.

    Person 0 only (the harness MVP triangulates a single person). Frames whose
    person list is empty leave every joint marked invalid for that row.
    """
    frames = []
    kp_format = ""
    n_joints = 0
    with open(path, "r", encoding="utf-8") as fh:
        for line_no, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError as exc:
                raise SystemExit(f"{path}:{line_no}: bad JSON: {exc}") from exc
            kp_format = rec.get("kp_format", kp_format)
            persons = rec.get("persons_3d", [])
            joints = persons[0]["joints"] if persons else []
            n_joints = max(n_joints, len(joints))
            frames.append(joints)
    if not frames:
        raise SystemExit(f"{path}: no frames")

    n_frames = len(frames)
    positions = np.zeros((n_frames, n_joints, 3), dtype=np.float64)
    valid = np.zeros((n_frames, n_joints), dtype=bool)
    for fi, joints in enumerate(frames):
        for ji, j in enumerate(joints):
            # joint = [x, y, z, score, valid_bool]
            positions[fi, ji, 0] = j[0]
            positions[fi, ji, 1] = j[1]
            positions[fi, ji, 2] = j[2]
            valid[fi, ji] = bool(j[4]) if len(j) > 4 else False
    return Loaded(path, kp_format or "unknown", n_frames, n_joints, positions, valid)


# --------------------------------------------------------------------------- #
# jitter
# --------------------------------------------------------------------------- #

def joint_jitter_mm(data: Loaded, joint: int, min_frames: int) -> dict | None:
    """Per-joint 3D position jitter over frames where the joint is valid.

    Returns std-dev per axis and the combined 3D RMS scatter
    (sqrt(var_x + var_y + var_z)), all in millimetres, plus the valid frame
    count. None if the joint never reaches min_frames valid samples.
    """
    if joint >= data.n_joints:
        return None
    mask = data.valid[:, joint]
    n = int(mask.sum())
    if n < min_frames:
        return None
    pts = data.positions[mask, joint, :]  # (n, 3) meters
    std_axis = pts.std(axis=0) * 1000.0  # mm
    rms3d = float(np.sqrt((std_axis ** 2).sum()))
    return {
        "joint": joint,
        "name": JOINT_NAMES.get(joint, "?"),
        "valid_frames": n,
        "std_x_mm": float(std_axis[0]),
        "std_y_mm": float(std_axis[1]),
        "std_z_mm": float(std_axis[2]),
        "rms_3d_mm": rms3d,
    }


def run_jitter(args: argparse.Namespace) -> int:
    files = [load_jsonl(p) for p in args.files]
    if args.joints:
        joints = [int(x) for x in args.joints.split(",") if x.strip() != ""]
    else:
        # Core joints first, then every other joint present in the topology.
        joints = list(CORE_JOINTS)
        joints += [j for j in range(files[0].n_joints) if j not in CORE_JOINTS]

    # Per-file per-joint results.
    results = []
    for data in files:
        per_joint = {}
        for j in joints:
            r = joint_jitter_mm(data, j, args.min_frames)
            if r is not None:
                per_joint[j] = r
        core_vals = [per_joint[j]["rms_3d_mm"] for j in CORE_JOINTS if j in per_joint]
        all_vals = [r["rms_3d_mm"] for r in per_joint.values()]
        results.append({
            "path": data.path,
            "kp_format": data.kp_format,
            "frames": data.n_frames,
            "per_joint": per_joint,
            "core_median_rms_mm": float(np.median(core_vals)) if core_vals else None,
            "core_mean_rms_mm": float(np.mean(core_vals)) if core_vals else None,
            "overall_median_rms_mm": float(np.median(all_vals)) if all_vals else None,
        })

    _print_jitter_table(results, joints)

    out = {
        "metric": "jitter",
        "min_frames": args.min_frames,
        "files": results,
    }
    if len(results) == 2:
        out["delta"] = _jitter_delta(results[0], results[1], joints)
    _maybe_write(args.out, out)
    return 0


def _print_jitter_table(results: list[dict], joints: list[int]) -> None:
    print("== jitter (3D position RMS, mm; lower is better) ==")
    for i, r in enumerate(results):
        tag = f"[{i}]"
        print(f"{tag} {r['path']}  ({r['kp_format']}, {r['frames']} frames)")
    print()
    header = f"{'joint':<16}" + "".join(f"{('['+str(i)+'] rms_mm'):>16}" for i in range(len(results)))
    print(header)
    print("-" * len(header))
    for j in joints:
        cells = []
        present = False
        for r in results:
            pj = r["per_joint"].get(j)
            if pj is None:
                cells.append(f"{'--':>16}")
            else:
                present = True
                cells.append(f"{pj['rms_3d_mm']:>16.2f}")
        if not present:
            continue
        marker = "*" if j in CORE_JOINTS else " "
        print(f"{marker}{joint_label(j):<15}" + "".join(cells))
    print("-" * len(header))
    for key, label in (("core_median_rms_mm", "core median"),
                       ("core_mean_rms_mm", "core mean"),
                       ("overall_median_rms_mm", "all median")):
        cells = []
        for r in results:
            v = r[key]
            cells.append(f"{'--':>16}" if v is None else f"{v:>16.2f}")
        print(f"{label:<16}" + "".join(cells))
    print("\n(* = core rigid-body joint: neck/shoulders/hip_center/hips)")


def _jitter_delta(base: dict, cand: dict, joints: list[int]) -> dict:
    """[1] relative to [0]: negative pct = candidate has less jitter."""
    per_joint = {}
    for j in joints:
        b = base["per_joint"].get(j)
        c = cand["per_joint"].get(j)
        if b is None or c is None:
            continue
        bv, cv = b["rms_3d_mm"], c["rms_3d_mm"]
        per_joint[j] = {
            "joint": j,
            "name": JOINT_NAMES.get(j, "?"),
            "base_rms_mm": bv,
            "cand_rms_mm": cv,
            "delta_mm": cv - bv,
            "delta_pct": (100.0 * (cv - bv) / bv) if bv > 1e-9 else None,
        }
    print("\n== delta [1] vs [0] (negative = [1] less jitter) ==")
    for j in joints:
        d = per_joint.get(j)
        if d is None:
            continue
        pct = "n/a" if d["delta_pct"] is None else f"{d['delta_pct']:+.1f}%"
        marker = "*" if j in CORE_JOINTS else " "
        print(f"{marker}{joint_label(j):<15} {d['base_rms_mm']:>8.2f} -> "
              f"{d['cand_rms_mm']:>8.2f} mm  ({pct})")
    return {"per_joint": per_joint}


# --------------------------------------------------------------------------- #
# lag
# --------------------------------------------------------------------------- #

def _joint_signal(data: Loaded, joint: int, axis: int) -> np.ndarray:
    """One axis of a joint's trajectory, with invalid frames interpolated.

    Returns a float array of length n_frames (NaN only if there are zero valid
    samples). Linear interpolation over the valid indices keeps cross-
    correlation well-defined across short dropouts.
    """
    mask = data.valid[:, joint]
    x = data.positions[:, joint, axis].astype(np.float64)
    idx = np.arange(data.n_frames)
    valid_idx = idx[mask]
    if valid_idx.size == 0:
        return np.full(data.n_frames, np.nan)
    return np.interp(idx, valid_idx, x[mask])


def _best_axis(data: Loaded, joint: int) -> int:
    """Axis with the most motion (largest variance) for the chosen joint."""
    mask = data.valid[:, joint]
    if mask.sum() < 2:
        return 0
    var = data.positions[mask, joint, :].var(axis=0)
    return int(np.argmax(var))


def _xcorr_lag(base: np.ndarray, cand: np.ndarray, max_lag: int) -> dict:
    """Estimate the integer (and sub-sample) lag of cand relative to base.

    For each integer lag in [-max_lag, max_lag] we compute the Pearson
    correlation of base[t] with cand[t+lag] over their overlap. The maximizing
    lag is the candidate's delay (positive = cand later than base). A parabolic
    fit around the peak gives a sub-frame refinement.
    """
    b = base - np.nanmean(base)
    c = cand - np.nanmean(cand)
    n = len(b)
    lags = range(-max_lag, max_lag + 1)
    corrs = {}
    for lag in lags:
        if lag >= 0:
            bb, cc = b[: n - lag], c[lag:]
        else:
            bb, cc = b[-lag:], c[: n + lag]
        if len(bb) < 8:
            continue
        sb, sc = bb.std(), cc.std()
        if sb < 1e-12 or sc < 1e-12:
            corrs[lag] = 0.0
        else:
            corrs[lag] = float(np.mean((bb - bb.mean()) * (cc - cc.mean())) / (sb * sc))
    if not corrs:
        return {"best_lag_frames": 0, "best_lag_subframe": 0.0, "peak_corr": 0.0,
                "corr_by_lag": {}}
    best_lag = max(corrs, key=corrs.get)
    # Parabolic interpolation around the integer peak for a sub-frame estimate.
    sub = float(best_lag)
    if (best_lag - 1) in corrs and (best_lag + 1) in corrs:
        ym1, y0, yp1 = corrs[best_lag - 1], corrs[best_lag], corrs[best_lag + 1]
        denom = (ym1 - 2 * y0 + yp1)
        if abs(denom) > 1e-12:
            sub = best_lag + 0.5 * (ym1 - yp1) / denom
    return {
        "best_lag_frames": best_lag,
        "best_lag_subframe": sub,
        "peak_corr": corrs[best_lag],
        "corr_by_lag": corrs,
    }


def run_lag(args: argparse.Namespace) -> int:
    base = load_jsonl(args.baseline)
    cand = load_jsonl(args.candidate)
    joint = args.joint
    if joint < 0:
        # Default: hip_center (19) if present, else l_hip (11, COCO17 root).
        joint = 19 if base.n_joints > 19 else 11
    if joint >= base.n_joints or joint >= cand.n_joints:
        raise SystemExit(f"joint {joint} out of range for the loaded topologies")

    n = min(base.n_frames, cand.n_frames)
    if base.n_frames != cand.n_frames:
        print(f"warning: frame counts differ (baseline={base.n_frames}, "
              f"candidate={cand.n_frames}); using first {n}", file=sys.stderr)

    axis = args.axis if args.axis >= 0 else _best_axis(base, joint)
    axis_name = "xyz"[axis]
    b_sig = _joint_signal(base, joint, axis)[:n]
    c_sig = _joint_signal(cand, joint, axis)[:n]
    if np.isnan(b_sig).all() or np.isnan(c_sig).all():
        raise SystemExit(f"joint {joint} never valid in one of the inputs")

    motion_mm = float(np.nanstd(b_sig) * 1000.0)
    res = _xcorr_lag(b_sig, c_sig, args.max_lag_frames)
    ms_per_frame = 1000.0 / args.fps
    lag_ms = res["best_lag_subframe"] * ms_per_frame

    print(f"== lag (candidate vs baseline; positive = candidate delayed) ==")
    print(f"baseline : {base.path}")
    print(f"candidate: {cand.path}")
    print(f"joint    : {joint_label(joint)}  axis={axis_name}  "
          f"(baseline motion sigma={motion_mm:.1f} mm over {n} frames)")
    if motion_mm < args.min_motion_mm:
        print(f"WARNING: baseline motion sigma {motion_mm:.1f} mm < "
              f"--min-motion-mm {args.min_motion_mm:.1f}; this looks like a "
              f"stationary clip — lag is meaningless. Use a motion clip.")
    print(f"fps      : {args.fps}  ({ms_per_frame:.2f} ms/frame)")
    print(f"best lag : {res['best_lag_frames']} frames "
          f"(sub-frame {res['best_lag_subframe']:+.2f}) => {lag_ms:+.1f} ms")
    print(f"peak corr: {res['peak_corr']:.4f}")

    out = {
        "metric": "lag",
        "baseline": base.path,
        "candidate": cand.path,
        "joint": joint,
        "joint_name": JOINT_NAMES.get(joint, "?"),
        "axis": axis_name,
        "fps": args.fps,
        "frames_compared": n,
        "baseline_motion_sigma_mm": motion_mm,
        "best_lag_frames": res["best_lag_frames"],
        "best_lag_subframe": res["best_lag_subframe"],
        "lag_ms": lag_ms,
        "peak_corr": res["peak_corr"],
    }
    _maybe_write(args.out, out)
    return 0


# --------------------------------------------------------------------------- #

def _maybe_write(path: str | None, obj: dict) -> None:
    if not path:
        return
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=2)
    print(f"\nwrote {path}")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="jitter / lag metrics for dump_keypoints_3d JSONL output",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    pj = sub.add_parser("jitter", help="per-joint 3D position std-dev (stationary clip)")
    pj.add_argument("files", nargs="+", help="dump_keypoints_3d JSONL (1+; 2 => delta table)")
    pj.add_argument("--joints", default="",
                    help="comma-separated joint indices (default: core + all present)")
    pj.add_argument("--min-frames", type=int, default=10,
                    help="min valid frames for a joint to be scored (default 10)")
    pj.add_argument("--out", default="", help="write metrics JSON here")
    pj.set_defaults(func=run_jitter)

    pl = sub.add_parser("lag", help="temporal lag of candidate vs baseline (motion clip)")
    pl.add_argument("baseline", help="baseline JSONL (e.g. tri-only / weak smoothing)")
    pl.add_argument("candidate", help="candidate JSONL (same clip, e.g. with smoothing)")
    pl.add_argument("--joint", type=int, default=-1,
                    help="joint index to track (default hip_center 19, or l_hip 11)")
    pl.add_argument("--axis", type=int, default=-1,
                    help="0=x 1=y 2=z; default = highest-variance axis in baseline")
    pl.add_argument("--fps", type=float, default=30.0,
                    help="clip fps for ms conversion (default 30)")
    pl.add_argument("--max-lag-frames", type=int, default=30,
                    help="search +/- this many frames (default 30)")
    pl.add_argument("--min-motion-mm", type=float, default=20.0,
                    help="warn if baseline motion sigma is below this (default 20)")
    pl.add_argument("--out", default="", help="write metrics JSON here")
    pl.set_defaults(func=run_lag)
    return p


def main(argv: list[str]) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
