"""Jointly fit three camera poses and 3D landmarks by robust native-pixel bundle adjustment."""

import argparse
import json
from pathlib import Path

import cv2 as cv
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

from geometry import Cameras, EDGES, IDENTITY, quantiles, rotation_angle, solve, visible


def bundle(cameras, poses, data):
    ids, camera_ids, pixels, xyz0 = [], [], [], []
    groups = {}
    rng = np.random.default_rng(66)
    for views, key in [((0, 1, 2), "tracks")] + [(p, f"p{p[0]}{p[1]}") for p in EDGES]:
        uv = data[key]
        if not len(uv):
            continue
        xyz, positive, err = visible(cameras, poses, views, uv)
        good = np.flatnonzero(positive & (err.max(1) < 4))
        # Keep coverage in both endpoints; avoid fitting a dense patch repeatedly.
        used = [set() for _ in views]
        selected = []
        for idx in rng.permutation(good):
            cell = [tuple((u // 32).astype(int)) for u in uv[idx]]
            if any(c in seen for c, seen in zip(cell, used)):
                continue
            for c, seen in zip(cell, used):
                seen.add(c)
            selected.append(idx)
            if len(selected) >= 200:
                break
        groups[key] = len(selected)
        for idx in selected:
            landmark = len(xyz0)
            xyz0.append(xyz[idx])
            for cam, q in zip(views, uv[idx]):
                ids.append(landmark)
                camera_ids.append(cam)
                pixels.append(q)
    assert len(xyz0) >= 12 and len(set(camera_ids)) == 3
    ids = np.array(ids)
    camera_ids = np.array(camera_ids)
    pixels = np.array(pixels)
    t1 = poses[1][1]
    assert abs(np.linalg.norm(t1) - 1) < 1e-6
    tangent = np.linalg.svd(t1[None, :])[2][1:].T
    start = np.r_[
        cv.Rodrigues(poses[1][0])[0].ravel(),
        [0.0, 0.0],
        cv.Rodrigues(poses[2][0])[0].ravel(),
        poses[2][1],
        np.ravel(xyz0),
    ]

    def unpack(params):
        t = t1 + tangent @ params[3:5]
        return [
            IDENTITY,
            (cv.Rodrigues(params[:3])[0], t / np.linalg.norm(t)),
            (cv.Rodrigues(params[5:8])[0], params[8:11]),
        ]

    def fun(params):
        pose = unpack(params)
        xyz = params[11:].reshape(-1, 3)
        residual = np.empty_like(pixels, dtype=float)
        for i in range(3):
            which = camera_ids == i
            uv, _ = cameras.project(i, pose[i], xyz[ids[which]])
            residual[which] = uv - pixels[which]
        return residual.ravel()

    jac = lil_matrix((2 * len(ids), len(start)), dtype=np.uint8)
    for n, (point, cam) in enumerate(zip(ids, camera_ids)):
        rows = slice(2 * n, 2 * n + 2)
        jac[rows, 11 + 3 * point : 14 + 3 * point] = 1
        if cam == 1:
            jac[rows, :5] = 1
        if cam == 2:
            jac[rows, 5:11] = 1
    res = least_squares(
        fun,
        start,
        jac_sparsity=jac.tocsr(),
        loss="soft_l1",
        f_scale=1.0,
        x_scale="jac",
        max_nfev=100,
        ftol=1e-6,
        gtol=1e-6,
        xtol=1e-6,
    )
    after = unpack(res.x)
    depths = []
    for cam in range(3):
        which = camera_ids == cam
        _, z = cameras.project(cam, after[cam], res.x[11:].reshape(-1, 3)[ids[which]])
        depths.extend(z)
    centers = np.array([-r.T @ v for r, v in after])
    result = {
        "landmark_groups": groups,
        "landmarks": len(xyz0),
        "observations": len(ids),
        "status": int(res.status),
        "iterations": int(res.nfev),
        "all_positive_depth": bool(np.min(depths) > 0),
        "fit_px_before_p50_p95": quantiles(
            np.linalg.norm(fun(start).reshape(-1, 2), axis=1)
        ),
        "fit_px_after_p50_p95": quantiles(
            np.linalg.norm(res.fun.reshape(-1, 2), axis=1)
        ),
        "seed_to_final_rotation_deg": [
            rotation_angle(after[i][0] @ poses[i][0].T) for i in (1, 2)
        ],
        "baseline_ratios_to_01": {
            f"{i}{j}": float(np.linalg.norm(centers[i] - centers[j])) for i, j in EDGES
        },
    }
    return after, result


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("folder", type=Path)
    p.add_argument("method")
    p.add_argument("intrinsics", type=Path)
    a = p.parse_args()
    cv.setNumThreads(2)
    cams = Cameras(a.intrinsics)
    with np.load(a.folder / f"{a.method}-pairs.npz") as source:
        data = dict(source)
    report, poses = solve(cams, data)
    if poses is not None:
        poses, stats = bundle(cams, poses, data)
        report["joint_refinement"] = stats
        report["relative_poses"] = [
            {"R": r.tolist(), "t_in_baseline01_units": t.tolist()} for r, t in poses
        ]
        report["status"] = (
            "relative_candidate"
            if stats["status"] > 0 and stats["all_positive_depth"]
            else "refinement_unverified"
        )
        print(stats)
    (a.folder / f"{a.method}-geometry.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
