"""Score only newly detected fiducial corners completely withheld from the geometry fit."""

import argparse
import json
from pathlib import Path

import numpy as np
from geometry import Cameras, EDGES, quantiles, triangulate, visible


def assess(cams, poses, observed):
    views = [
        {
            int(k): np.asarray(e["pixels"])
            for k, e in v.items()
            if e["n"] >= 2 and e["spread_p95_px"] <= 2.0
        }
        for v in observed
    ]
    stats = {}
    for i, j in EDGES:
        keys = sorted(views[i].keys() & views[j].keys())
        s = {"common_ids": keys}
        stats[f"{i}{j}"] = s
        if not keys:
            continue
        pts = np.stack(
            [np.concatenate([views[a][key] for key in keys]) for a in (i, j)], 1
        )
        X, ok, errs = visible(cams, poses, (i, j), pts)
        s.update(
            corners=len(pts),
            positive_depth=int(ok.sum()),
            max_of_two_views_px_p50_p95=quantiles(errs[ok].max(1)),
        )
        shapes = {}
        for n, tag in enumerate(keys):
            if not ok[n * 4 : n * 4 + 4].all():
                continue
            xyz = X[n * 4 : n * 4 + 4]
            dif = np.roll(xyz, -1, axis=0) - xyz
            lengths = np.linalg.norm(dif, axis=1)
            unit = dif / lengths[:, None]
            angles = np.rad2deg(
                np.arccos(
                    np.clip(np.sum(-np.roll(unit, 1, axis=0) * unit, axis=1), -1, 1)
                )
            )
            shapes[tag] = {
                "side_ratio_long_to_short": float(lengths.max() / lengths.min()),
                "corner_max_deviation_from_90_deg": float(abs(angles - 90).max()),
            }
        s["square_shape"] = shapes
    keys = sorted(set.intersection(*(set(v) for v in views)))
    hold = {"common_ids": keys}
    if keys:
        for k in range(3):
            i, j = [c for c in range(3) if c != k]
            X = triangulate(
                poses[i],
                poses[j],
                cams.rays(i, np.concatenate([views[i][t] for t in keys])),
                cams.rays(j, np.concatenate([views[j][t] for t in keys])),
            )
            uv, z = cams.project(k, poses[k], X)
            expected = np.concatenate([views[k][t] for t in keys])
            positive = np.all(
                np.stack([cams.project(c, poses[c], X)[1] > 0 for c in range(3)]),
                axis=0,
            )
            hold[str(k)] = {
                "positive_depth": int(positive.sum()),
                "corners": len(X),
                "withheld_view_px_p50_p95": quantiles(
                    np.linalg.norm(uv - expected, axis=1)[positive]
                ),
            }
    center_views = [
        {
            int(tag): e
            for tag, e in v.items()
            if e["n"] >= 2 and e.get("center_spread_p95_px", 999) <= 2.0
        }
        for v in observed
    ]
    center_keys = sorted(set.intersection(*(set(v) for v in center_views)))
    centers = {}
    for tag in center_keys:
        measured = [np.array([v[tag]["center_px"]]) for v in center_views]
        result = {}
        for k in range(3):
            i, j = [c for c in range(3) if c != k]
            X = triangulate(
                poses[i], poses[j], cams.rays(i, measured[i]), cams.rays(j, measured[j])
            )
            uv, _ = cams.project(k, poses[k], X)
            positive = all(cams.project(c, poses[c], X)[1][0] > 0 for c in range(3))
            result[k] = {
                "error_px": float(np.linalg.norm(uv - measured[k])),
                "positive_depth": bool(positive),
                "observed": measured[k][0].tolist(),
                "predicted": uv[0].tolist(),
            }
        centers[tag] = {
            "held_view": result,
            "localization_spread_p95_px": [
                v[tag]["center_spread_p95_px"] for v in center_views
            ],
        }
    return {"pairs": stats, "leave_one_camera_out": hold, "held_out_centers": centers}


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("folder", type=Path)
    p.add_argument("intrinsics", type=Path)
    p.add_argument("--observations", type=Path)
    a = p.parse_args()
    cams = Cameras(a.intrinsics)
    cfg = json.loads((a.folder / "roma-geometry.json").read_text())
    poses = [
        (np.array(p["R"]), np.array(p["t_in_baseline01_units"]))
        for p in cfg["relative_poses"]
    ]
    obs = a.observations or (a.folder / "withheld-corners.json")
    v = json.loads(obs.read_text())
    s = assess(cams, poses, v)
    (a.folder / (obs.stem + "-score.json")).write_text(json.dumps(s, indent=2) + "\n")
    print(json.dumps(s))
