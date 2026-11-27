"""Compare independently solved three-camera rigs and leave-one-camera-out checks on fixed observations."""

from itertools import combinations
from pathlib import Path
import json
import sys
import numpy as np
from geometry import Cameras, EDGES, rotation_angle
from score import assess

out, reference, intrinsics = map(Path, sys.argv[1:])
cams = Cameras(intrinsics)
folders = {
    "open": reference,
    **{s: out / s for s in ("closed-a", "closed-b", "closed-c")},
}


def load(folder, stem):
    return json.loads((folder / (stem + ".json")).read_text())


def percentile(v, q):
    return [float(x) for x in np.percentile(v, q)] if len(v) else []


def ang(a, b):
    return float(
        np.rad2deg(
            np.arccos(
                np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1)
            )
        )
    )


def included(item):
    return item["n"] >= 2 and item.get("center_spread_p95_px", 999) <= 2.0


def centers(rig):
    return np.asarray([-r.T @ t for r, t in rig])


def deltas(a, b):
    x, y = centers(a), centers(b)
    result = {}
    for i, j in EDGES:
        ra = a[j][0] @ a[i][0].T
        rb = b[j][0] @ b[i][0].T
        va = a[i][0] @ (x[j] - x[i])
        vb = b[i][0] @ (y[j] - y[i])
        result[f"{i}{j}"] = {
            "rotation_delta_deg": rotation_angle(rb @ ra.T),
            "direction_delta_deg": ang(va, vb),
            "first_length_over_01": float(np.linalg.norm(va)),
            "second_length_over_01": float(np.linalg.norm(vb)),
        }
    return result


def native_triangulate(rig, ids, pixels):
    # Native lenses are inverted separately before multi-view homogeneous DLT.
    blocks = []
    for col, cam in enumerate(ids):
        rays = cams.rays(cam, pixels[:, col])
        P = np.column_stack(rig[cam])
        blocks.extend([rays[:, 0, None] * P[2] - P[0], rays[:, 1, None] * P[2] - P[1]])
    _, _, V = np.linalg.svd(np.stack(blocks, axis=1), full_matrices=False)
    X = V[:, -1, :3] / V[:, -1, 3, None]
    ok = np.isfinite(X).all(1)
    for k, cam in enumerate(ids):
        uv, z = cams.project(cam, rig[cam], X)
        ok &= (z > 0) & (np.linalg.norm(uv - pixels[:, k], axis=1) <= 4.0)
    return X, ok


def height_support(rig, obs, folder):
    # Plane locations are reconstructed from the masked tag centers for diagnostics *after* calibration.
    floor = []
    for tag in set().union(*(set(row) for row in obs)):
        ids = [i for i in range(3) if tag in obs[i] and included(obs[i][tag])]
        if len(ids) < 2:
            continue
        px = np.array([[obs[i][tag]["center_px"] for i in ids]])
        xyz, ok = native_triangulate(rig, ids, px)
        if ok[0]:
            floor.append(xyz[0])
    if len(floor) < 3:
        return None
    floor = np.asarray(floor)
    origin = floor.mean(0)
    _, s, v = np.linalg.svd(floor - origin)
    assert s[1] > 0.01
    n = v[-1]
    if (centers(rig).mean(0) - origin) @ n < 0:
        n = -n
    info = {
        "masked_floor_centers": len(floor),
        "plane_scatter_p95_over_01": float(
            np.percentile(abs((floor - origin) @ n), 95)
        ),
        "tracks": {},
    }
    with np.load(folder / "roma-pairs.npz") as src:
        for ids, key in [((0, 1, 2), "tracks")] + [
            (e, f"p{e[0]}{e[1]}") for e in EDGES
        ]:
            xyz, ok = native_triangulate(rig, ids, src[key])
            h = (xyz[ok] - origin) @ n
            info["tracks"][key] = {
                "positive_max4px": int(ok.sum()),
                "above_005_baseline": int((h > 0.05).sum()),
                "within_002_baseline": int((np.abs(h) <= 0.02).sum()),
                "height_p05_p50_p95": percentile(h, [5, 50, 95]),
            }
    return info


def compact(score):
    hv = score["held_out_centers"]
    errors = []
    by_camera = {}
    for c in range(3):
        per = [v["held_view"][c] for v in hv.values()]
        assert all(k["positive_depth"] for k in per)
        nums = [v["error_px"] for v in per]
        errors += nums
        by_camera[c] = percentile(nums, [50, 95])
    return dict(
        ids=sorted(int(t) for t in hv),
        n=len(errors),
        p50_p95_max=percentile(errors, [50, 95, 100]),
        by_camera_p50_p95=by_camera,
        errors_by_id={
            k: [v["held_view"][c]["error_px"] for c in range(3)] for k, v in hv.items()
        },
        corner_tests=score["leave_one_camera_out"],
    )


cap = load(out, "captures")
summary = {"settings": {}, "crossfit": {}, "pose_deltas": {}, "tag_repeatability": {}}
rigs = {}
observed = {}
for s, f in folders.items():
    observed[s] = load(f, "combined-corners")
    geom = load(f, "roma-geometry")
    row = {
        "status": geom["status"],
        "three_view_tracks": geom["three_view_tracks"],
        "cam1_clipped_pct": cap[s]["gray_ge250_fraction"][1] * 100,
        "pre_ba_loop_deg": geom.get("pair_loop", {}).get("rotation_gap_deg"),
    }
    if geom["status"] == "relative_candidate":
        rig = [
            (np.array(q["R"]), np.array(q["t_in_baseline01_units"]))
            for q in geom["relative_poses"]
        ]
        assert np.allclose(np.linalg.det([x[0] for x in rig]), 1)
        rigs[s] = rig
        g = geom["joint_refinement"]
        row.update(
            baselines=g["baseline_ratios_to_01"],
            landmarks=g["landmarks"],
            positive=g["all_positive_depth"],
            fit_px=g["fit_px_after_p50_p95"],
            tag_center_tests=compact(assess(cams, rig, observed[s])),
            height_support=height_support(rig, observed[s], f),
        )
    summary["settings"][s] = row
for key, rig in rigs.items():
    summary["crossfit"][key] = {
        k: compact(assess(cams, rig, v)) for k, v in observed.items()
    }
for x, y in combinations(rigs, 2):
    summary["pose_deltas"][x + "_vs_" + y] = deltas(rigs[x], rigs[y])
for x, y in [("open", "closed-c"), ("closed-b", "closed-c")]:
    stats = []
    for c in range(3):
        a = observed[x][c]
        b = observed[y][c]
        ids = sorted(
            k for k in a.keys() & b.keys() if included(a[k]) and included(b[k])
        )
        delta = [
            np.linalg.norm(np.array(a[k]["center_px"]) - b[k]["center_px"]) for k in ids
        ]
        stats.append(
            {"ids": ids, "measured_shift_p50_p95_px": percentile(delta, [50, 95])}
        )
    summary["tag_repeatability"][x + "_vs_" + y] = stats
(out / "comparison.json").write_text(
    json.dumps(summary, indent=2, allow_nan=False) + "\n"
)
for k, v in summary["settings"].items():
    print(
        k,
        v["status"],
        v["three_view_tracks"],
        v.get("tag_center_tests", {}).get("p50_p95_max"),
        v.get("baselines"),
    )
for k, v in summary["pose_deltas"].items():
    print(
        k,
        [
            (round(z["rotation_delta_deg"], 4), round(z["direction_delta_deg"], 4))
            for z in v.values()
        ],
    )
