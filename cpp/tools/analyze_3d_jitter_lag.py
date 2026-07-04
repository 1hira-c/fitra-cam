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

  trackers— per-SlimeVR-tracker metrics from a ``--dump-trackers`` JSONL:
            position jitter (mm), inferred-roll (twist) jitter (deg) and
            per-bone relative-to-parent angular velocity (deg/s). The
            spatiotemporal-filter harness (M-C1): run OFF vs ON on the same clip
            for a per-tracker delta table.

Dependencies: numpy + stdlib only (NumPy 1.x on the Jetson venv is fine).

Examples
--------
  # Stationary jitter of a single run (core joints + overall median)
  analyze_3d_jitter_lag.py jitter still.jsonl

  # Compare two stationary runs (stage OFF vs ON) on the same clip
  analyze_3d_jitter_lag.py jitter still_kalman_off.jsonl still_kalman_on.jsonl

  # Lag of a filtered run vs the raw triangulation on a motion clip
  analyze_3d_jitter_lag.py lag bend_no_kalman.jsonl bend_kalman.jsonl --joint 19

  # Per-tracker jitter of a --dump-trackers run (OFF baseline)
  analyze_3d_jitter_lag.py trackers still_trackers.jsonl --fps 8.6
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
    # max(1, ...): --min-frames <= 0 must not let an empty slice through
    # (std of an empty array is NaN and it would leak into the JSON output).
    if n < max(1, min_frames):
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
        # Core joints first, then every other joint present in ANY file's
        # topology (use the max so a longer candidate file isn't truncated to
        # the first file's joint count).
        max_joints = max(f.n_joints for f in files)
        joints = list(CORE_JOINTS)
        joints += [j for j in range(max_joints) if j not in CORE_JOINTS]

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
# trackers  (spatiotemporal-filter harness, M-C1)
# --------------------------------------------------------------------------- #
#
# Scores the RAW SlimeVR 10-tracker trajectories emitted by
# `dump_keypoints_3d --dump-trackers`: per-tracker position jitter (the VMT /
# WebUI signal the filter attacks), inferred-roll jitter (the twist the filter
# also targets on arms/legs), and per-bone relative-to-parent angular velocity
# (the angle-domain / 案6 hypothesis data). This is the OFF baseline the
# spatiotemporal filter (docs/design/pose-3d-spatiotemporal-filter.md) is
# measured against; re-run ON once the filter is wired to get an A/B delta.

N_TRACKERS = 10

# TrackerRole order == array index (see slimevr::TrackerRole).
TRACKER_NAMES = {
    0: "l_upper_arm", 1: "r_upper_arm", 2: "chest", 3: "waist",
    4: "l_upper_leg", 5: "r_upper_leg", 6: "l_lower_leg", 7: "r_lower_leg",
    8: "l_foot", 9: "r_foot",
}

# Kinematic parent for the relative-angular-velocity metric (-1 = world root).
# Matches the apply_quat_smoothing transport tree: arms ride the chest, legs
# ride the waist, distals chain up their own limb.
TRACKER_PARENT = {
    0: 2, 1: 2,      # upper arms -> chest
    2: 3,            # chest      -> waist
    3: -1,           # waist      -> world root
    4: 3, 5: 3,      # upper legs -> waist
    6: 4, 7: 5,      # lower legs -> upper legs
    8: 6, 9: 7,      # feet       -> lower legs
}


def tracker_label(idx: int) -> str:
    return f"{idx}:{TRACKER_NAMES.get(idx, '?')}"


@dataclass
class TrackersLoaded:
    path: str
    n_frames: int
    n_trackers: int
    # pos[frame, tracker, 0:3] = xyz (m); quat[frame, tracker, 0:4] = wxyz;
    # valid[frame, tracker] = bool; roll_conf[frame, tracker] = [0,1].
    pos: np.ndarray
    quat: np.ndarray
    valid: np.ndarray
    roll_conf: np.ndarray


def load_trackers_jsonl(path: str) -> TrackersLoaded:
    """Load the per-frame ``trackers`` array from a dump_keypoints_3d JSONL.

    Each tracker entry is [role, px,py,pz, qw,qx,qy,qz, valid, roll_conf].
    Lines without a ``trackers`` field (e.g. a dump made without
    --dump-trackers) leave that frame's trackers all-invalid; if NO line has
    the field the file is rejected.
    """
    rows = []
    n_trk = 0
    any_trackers = False
    with open(path, "r", encoding="utf-8") as fh:
        for line_no, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError as exc:
                raise SystemExit(f"{path}:{line_no}: bad JSON: {exc}") from exc
            trk = rec.get("trackers")
            if trk is None:
                trk = []
            else:
                any_trackers = True
            n_trk = max(n_trk, len(trk))
            rows.append(trk)
    if not rows:
        raise SystemExit(f"{path}: no frames")
    if not any_trackers:
        raise SystemExit(
            f"{path}: no 'trackers' field on any line — re-run "
            f"dump_keypoints_3d with --dump-trackers")

    n_frames = len(rows)
    n_trk = max(n_trk, 1)
    pos = np.zeros((n_frames, n_trk, 3), dtype=np.float64)
    quat = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (n_frames, n_trk, 1))
    valid = np.zeros((n_frames, n_trk), dtype=bool)
    roll_conf = np.zeros((n_frames, n_trk), dtype=np.float64)
    for fi, trk in enumerate(rows):
        for entry in trk:
            # [role, px,py,pz, qw,qx,qy,qz, valid, roll_conf]
            ti = int(entry[0])
            if ti < 0 or ti >= n_trk:
                continue
            pos[fi, ti, :] = entry[1:4]
            quat[fi, ti, :] = entry[4:8]
            valid[fi, ti] = bool(entry[8]) if len(entry) > 8 else False
            roll_conf[fi, ti] = float(entry[9]) if len(entry) > 9 else 0.0
    return TrackersLoaded(path, n_frames, n_trk, pos, quat, valid, roll_conf)


# ---- quaternion helpers (wxyz) -------------------------------------------- #

def _q_norm(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def _q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _geo_angle(a: np.ndarray, b: np.ndarray) -> float:
    """Geodesic angle (rad, [0, pi]) between two orientation quaternions."""
    d = abs(float(np.dot(_q_norm(a), _q_norm(b))))
    return 2.0 * float(np.arccos(min(1.0, max(0.0, d))))


def _mean_quat(qs: np.ndarray) -> np.ndarray:
    """Sign-aligned normalized average — a robust reference orientation."""
    q0 = _q_norm(qs[0])
    acc = np.zeros(4)
    for q in qs:
        qn = _q_norm(q)
        acc += qn if np.dot(qn, q0) >= 0.0 else -qn
    return _q_norm(acc)


def _rms_deg(rads: list[float]) -> float:
    if not rads:
        return 0.0
    return float(np.degrees(np.sqrt(np.mean(np.square(rads)))))


def tracker_metrics(data: TrackersLoaded, t: int, min_frames: int,
                    fps: float) -> dict | None:
    """Per-tracker jitter metrics over frames where the tracker is valid.

    * pos_rms_mm  : 3D position scatter (sqrt of summed per-axis variance).
    * ori_rms_deg : full orientation scatter about the mean orientation.
    * roll_rms_deg: twist (roll about the bone forward) scatter about the mean.
    * rel_dps     : relative-to-parent angular velocity RMS (deg/s), the
                    angle-domain (案6) signal. rel-step RMS (deg) x fps.
    None if the tracker never reaches min_frames valid samples.
    """
    if t >= data.n_trackers:
        return None
    mask = data.valid[:, t]
    n = int(mask.sum())
    # max(1, ...): --min-frames <= 0 must not let a never-valid tracker through
    # (_mean_quat indexes qs[0] and would raise IndexError on an empty array).
    if n < max(1, min_frames):
        return None

    pts = data.pos[mask, t, :]                    # (n, 3) meters
    std_axis = pts.std(axis=0) * 1000.0           # mm
    pos_rms_mm = float(np.sqrt((std_axis ** 2).sum()))

    qs = data.quat[mask, t, :]                    # (n, 4) wxyz
    ref = _mean_quat(qs)
    ref_conj = _q_conj(ref)
    ori_angs, roll_angs = [], []
    for q in qs:
        qn = _q_norm(q)
        ori_angs.append(_geo_angle(ref, qn))
        d = _q_mul(ref_conj, qn)                  # ref^-1 * q
        if d[0] < 0.0:
            d = -d                                # canonical w >= 0
        roll_angs.append(2.0 * float(np.arctan2(d[3], d[0])))  # twist about +Z
    ori_rms_deg = _rms_deg(ori_angs)
    roll_rms_deg = _rms_deg(roll_angs)

    # Relative-to-parent per-frame angular step, over frames where both this
    # tracker and its parent are valid (root: parent always "valid").
    parent = TRACKER_PARENT.get(t, -1)
    pvalid = (np.ones(data.n_frames, dtype=bool) if parent < 0
              else data.valid[:, parent])
    rel_steps = []
    prev_rel = None
    prev_ok = False
    for fi in range(data.n_frames):
        ok = bool(mask[fi] and pvalid[fi])
        if not ok:
            prev_ok = False
            continue
        qc = _q_norm(data.quat[fi, t, :])
        rel = qc if parent < 0 else _q_mul(_q_conj(data.quat[fi, parent, :]), qc)
        if prev_ok:
            rel_steps.append(_geo_angle(prev_rel, rel))
        prev_rel = rel
        prev_ok = True
    rel_step_rms_deg = _rms_deg(rel_steps)
    rel_dps = rel_step_rms_deg * fps

    return {
        "tracker": t,
        "name": TRACKER_NAMES.get(t, "?"),
        "parent": parent,
        "valid_frames": n,
        "pos_rms_mm": pos_rms_mm,
        "ori_rms_deg": ori_rms_deg,
        "roll_rms_deg": roll_rms_deg,
        "rel_step_rms_deg": rel_step_rms_deg,
        "rel_dps": rel_dps,
        "roll_conf_mean": float(data.roll_conf[mask, t].mean()),
    }


def run_trackers(args: argparse.Namespace) -> int:
    files = [load_trackers_jsonl(p) for p in args.files]
    if args.trackers:
        sel = [int(x) for x in args.trackers.split(",") if x.strip() != ""]
    else:
        sel = list(range(N_TRACKERS))

    results = []
    for data in files:
        per = {}
        for t in sel:
            m = tracker_metrics(data, t, args.min_frames, args.fps)
            if m is not None:
                per[t] = m
        results.append({
            "path": data.path,
            "frames": data.n_frames,
            "fps": args.fps,
            "per_tracker": per,
        })

    _print_trackers_table(results, sel)

    out = {"metric": "trackers", "min_frames": args.min_frames,
           "fps": args.fps, "files": results}
    if len(results) == 2:
        out["delta"] = _trackers_delta(results[0], results[1], sel)
    _maybe_write(args.out, out)
    return 0


def _print_trackers_table(results: list[dict], sel: list[int]) -> None:
    print("== trackers (RAW, no temporal smoothing; lower is better) ==")
    for i, r in enumerate(results):
        print(f"[{i}] {r['path']}  ({r['frames']} frames, {r['fps']:g} fps)")
    print()
    cols = ("pos_rms_mm", "ori_rms_deg", "roll_rms_deg", "rel_dps")
    for r in results:
        idx = results.index(r)
        print(f"-- [{idx}] {r['path']} --")
        header = f"{'tracker':<16}" + "".join(f"{c:>13}" for c in cols) + f"{'roll_cf':>9}"
        print(header)
        print("-" * len(header))
        for t in sel:
            m = r["per_tracker"].get(t)
            if m is None:
                continue
            print(f"{tracker_label(t):<16}"
                  f"{m['pos_rms_mm']:>13.2f}{m['ori_rms_deg']:>13.2f}"
                  f"{m['roll_rms_deg']:>13.2f}{m['rel_dps']:>13.1f}"
                  f"{m['roll_conf_mean']:>9.2f}")
        print()
    print("pos_rms_mm  = 3D position scatter (VMT/WebUI signal)")
    print("roll_rms_deg= inferred-roll (twist) scatter  |  rel_dps = "
          "relative-to-parent angular velocity (deg/s, 案6)")


def _trackers_delta(base: dict, cand: dict, sel: list[int]) -> dict:
    """[1] relative to [0]: negative pct = candidate less jitter."""
    per = {}
    print("== delta [1] vs [0] (negative = [1] less jitter) ==")
    print(f"{'tracker':<16}{'pos_rms_mm':>24}{'roll_rms_deg':>24}")
    for t in sel:
        b = base["per_tracker"].get(t)
        c = cand["per_tracker"].get(t)
        if b is None or c is None:
            continue

        def _cell(key):
            bv, cv = b[key], c[key]
            pct = (100.0 * (cv - bv) / bv) if bv > 1e-9 else None
            pcs = "n/a" if pct is None else f"{pct:+.1f}%"
            return f"{bv:>7.2f}->{cv:>7.2f} ({pcs})", pct

        pos_s, pos_pct = _cell("pos_rms_mm")
        roll_s, roll_pct = _cell("roll_rms_deg")
        per[t] = {"tracker": t, "name": TRACKER_NAMES.get(t, "?"),
                  "pos_rms_mm": {"base": b["pos_rms_mm"], "cand": c["pos_rms_mm"],
                                 "delta_pct": pos_pct},
                  "roll_rms_deg": {"base": b["roll_rms_deg"], "cand": c["roll_rms_deg"],
                                   "delta_pct": roll_pct}}
        print(f"{tracker_label(t):<16}{pos_s:>24}{roll_s:>24}")
    return {"per_tracker": per}


def _tracker_signal(data: "TrackersLoaded", tracker: int, axis: int) -> np.ndarray:
    """One axis of a tracker's position trajectory, invalid frames interpolated."""
    mask = data.valid[:, tracker]
    x = data.pos[:, tracker, axis].astype(np.float64)
    idx = np.arange(data.n_frames)
    vi = idx[mask]
    if vi.size == 0:
        return np.full(data.n_frames, np.nan)
    return np.interp(idx, vi, x[mask])


def _best_tracker_axis(data: "TrackersLoaded", tracker: int) -> int:
    mask = data.valid[:, tracker]
    if mask.sum() < 2:
        return 0
    return int(np.argmax(data.pos[mask, tracker, :].var(axis=0)))


def run_trackers_lag(args: argparse.Namespace) -> int:
    """Temporal lag of a tracker's POSITION between two --dump-trackers runs.

    Same cross-correlation as `lag`, but the signal is a tracker (post-smoothing)
    trajectory rather than a skeleton joint — so it measures the lag the tracker
    stage's smoothing (One Euro vs st) adds. Positive = candidate delayed vs
    baseline. Use raw as the baseline (no smoothing = ground-truth timing) and
    one_euro / st as candidates to compare how much lag each mode adds.
    """
    base = load_trackers_jsonl(args.baseline)
    cand = load_trackers_jsonl(args.candidate)
    t = args.tracker
    if t < 0 or t >= min(base.n_trackers, cand.n_trackers):
        raise SystemExit(f"tracker {t} out of range")
    n = min(base.n_frames, cand.n_frames)
    if base.n_frames != cand.n_frames:
        print(f"warning: frame counts differ ({base.n_frames} vs {cand.n_frames}); "
              f"using first {n}", file=sys.stderr)
    axis = args.axis if args.axis >= 0 else _best_tracker_axis(base, t)
    b_sig = _tracker_signal(base, t, axis)[:n]
    c_sig = _tracker_signal(cand, t, axis)[:n]
    if np.isnan(b_sig).all() or np.isnan(c_sig).all():
        raise SystemExit(f"tracker {t} never valid in one of the inputs")
    motion_mm = float(np.nanstd(b_sig) * 1000.0)
    res = _xcorr_lag(b_sig, c_sig, args.max_lag_frames)
    ms_per_frame = 1000.0 / args.fps
    lag_ms = res["best_lag_subframe"] * ms_per_frame

    print("== tracker lag (candidate vs baseline; positive = candidate delayed) ==")
    print(f"baseline : {base.path}")
    print(f"candidate: {cand.path}")
    print(f"tracker  : {tracker_label(t)}  axis={'xyz'[axis]}  "
          f"(baseline motion sigma={motion_mm:.1f} mm over {n} frames)")
    if motion_mm < args.min_motion_mm:
        print(f"WARNING: baseline motion sigma {motion_mm:.1f} mm < --min-motion-mm "
              f"{args.min_motion_mm:.1f}; looks stationary — lag is meaningless.")
    print(f"fps      : {args.fps}  ({ms_per_frame:.2f} ms/frame)")
    print(f"best lag : {res['best_lag_frames']} frames "
          f"(sub-frame {res['best_lag_subframe']:+.2f}) => {lag_ms:+.1f} ms")
    print(f"peak corr: {res['peak_corr']:.4f}")

    _maybe_write(args.out, {
        "metric": "trackers_lag", "baseline": base.path, "candidate": cand.path,
        "tracker": t, "tracker_name": TRACKER_NAMES.get(t, "?"), "axis": "xyz"[axis],
        "fps": args.fps, "frames_compared": n, "baseline_motion_sigma_mm": motion_mm,
        "best_lag_frames": res["best_lag_frames"], "best_lag_subframe": res["best_lag_subframe"],
        "lag_ms": lag_ms, "peak_corr": res["peak_corr"],
    })
    return 0


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
    # No global de-mean needed: each lag's overlap is re-centered per-slice below
    # (bb - bb.mean()), and .std() is shift-invariant, so subtracting the global
    # mean here would be overwritten work.
    b = base
    c = cand
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

    pt = sub.add_parser("trackers",
                        help="per-tracker pos/roll jitter + relative angular "
                             "velocity (spatiotemporal-filter harness)")
    pt.add_argument("files", nargs="+",
                    help="dump_keypoints_3d --dump-trackers JSONL (1+; 2 => delta)")
    pt.add_argument("--trackers", default="",
                    help="comma-separated tracker indices 0..9 (default all)")
    pt.add_argument("--min-frames", type=int, default=10,
                    help="min valid frames for a tracker to be scored (default 10)")
    pt.add_argument("--fps", type=float, default=30.0,
                    help="clip fps for the relative-angular-velocity deg/s "
                         "column (default 30; jitter columns are fps-independent)")
    pt.add_argument("--out", default="", help="write metrics JSON here")
    pt.set_defaults(func=run_trackers)

    ptl = sub.add_parser("trackers-lag",
                         help="temporal lag of a tracker position between two runs "
                              "(e.g. raw vs st on a motion clip)")
    ptl.add_argument("baseline", help="baseline --dump-trackers JSONL (e.g. raw)")
    ptl.add_argument("candidate", help="candidate --dump-trackers JSONL (e.g. st / one_euro)")
    ptl.add_argument("--tracker", type=int, default=9,
                     help="tracker index 0..9 to track (default 9 r_foot)")
    ptl.add_argument("--axis", type=int, default=-1,
                     help="0=x 1=y 2=z; default = highest-variance axis in baseline")
    ptl.add_argument("--fps", type=float, default=30.0, help="clip fps for ms conversion")
    ptl.add_argument("--max-lag-frames", type=int, default=30, help="search +/- this many frames")
    ptl.add_argument("--min-motion-mm", type=float, default=20.0,
                     help="warn if baseline motion sigma is below this (default 20)")
    ptl.add_argument("--out", default="", help="write metrics JSON here")
    ptl.set_defaults(func=run_trackers_lag)

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
