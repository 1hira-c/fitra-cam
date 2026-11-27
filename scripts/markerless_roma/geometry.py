"""Fixed-intrinsic geometry. All poses map camera 0 coordinates into each camera."""

from itertools import combinations
from pathlib import Path
import argparse
import json

import cv2 as cv
import numpy as np

EDGES = list(combinations(range(3), 2))
IDENTITY = (np.eye(3), np.zeros(3))


def rotation_angle(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def quantiles(values):
    return (
        None
        if len(values) == 0
        else [float(x) for x in np.percentile(values, [50, 95])]
    )


class Cameras:
    def __init__(self, path):
        f = cv.FileStorage(str(path), cv.FILE_STORAGE_READ)
        self.params = []
        for i in range(3):
            c = f.getNode("intrinsics").getNode(f"cam{i}")
            assert c.getNode("distortion_model").string() == "fisheye"
            assert [int(c.getNode(k).real()) for k in ("width", "height")] == [
                1280,
                960,
            ]
            self.params.append((c.getNode("K").mat(), c.getNode("dist").mat()))
        f.release()

    def rays(self, i, pixels):
        if not len(pixels):
            return np.empty((0, 2))
        K, D = self.params[i]
        return cv.fisheye.undistortPoints(
            np.ascontiguousarray(pixels, dtype=np.float64).reshape(-1, 1, 2), K, D
        ).reshape(-1, 2)

    def focal(self, i):
        K, _ = self.params[i]
        return float(np.mean([K[0, 0], K[1, 1]]))

    def project(self, i, pose, points):
        R, t = pose
        xyz = points @ R.T + t
        pix = cv.fisheye.projectPoints(
            np.ascontiguousarray(points).reshape(-1, 1, 3),
            cv.Rodrigues(R)[0],
            t,
            *self.params[i],
        )[0]
        return pix.reshape(-1, 2), xyz[:, 2]


def triangulate(a, b, ra, rb):
    P = [np.column_stack([R, t]) for R, t in (a, b)]
    X = cv.triangulatePoints(*P, ra.T, rb.T).T
    with np.errstate(divide="ignore", invalid="ignore"):
        return X[:, :3] / X[:, 3, None]


def get_edges(cams, data):
    edges = {}
    stats = {}
    for i, j in EDGES:
        pts = data[f"p{i}{j}"]
        si = {"input": len(pts)}
        stats[f"{i}{j}"] = si
        if len(pts) < 8:
            continue
        a, b = cams.rays(i, pts[:, 0]), cams.rays(j, pts[:, 1])
        f = (cams.focal(i) + cams.focal(j)) * 0.5
        cv.setRNGSeed(66)
        E, em = cv.findEssentialMat(
            a,
            b,
            np.eye(3),
            method=cv.RANSAC,
            prob=0.999,
            threshold=2.0 / f,
            maxIters=10000,
        )
        if E is None or E.shape != (3, 3):
            continue
        _, R, t, cheir = cv.recoverPose(E, a, b, np.eye(3), mask=em.copy())
        t = t.ravel()
        good = cheir.ravel() > 0
        si.update(essential_support=int(em.sum()), positive=int(good.sum()))
        if good.sum() >= 4:
            H, hm = cv.findHomography(
                a[good], b[good], cv.RANSAC, 3 / f, maxIters=5000, confidence=0.999
            )
            si["one_plane_support"] = int(hm.sum()) if H is not None else 0
            si["coverage_cells"] = [
                len(set(map(tuple, (v // [160, 120]).astype(int))))
                for v in (pts[good, 0], pts[good, 1])
            ]
        if good.sum() >= 5:
            edges[i, j] = (R, t, pts[good])
    return edges, stats


def loop_seed(edges):
    if len(edges) != 3:
        return None, {}
    A, ta, _ = edges[0, 1]
    B, tb, _ = edges[0, 2]
    C, tc, _ = edges[1, 2]
    M = np.column_stack((tc, -tb))
    scales, _, _, singular = np.linalg.lstsq(M, -C @ ta, rcond=None)
    discrepancy = float(np.linalg.norm(C @ ta + M @ scales))
    angle = rotation_angle(B.T @ C @ A)
    stat = {
        "rotation_gap_deg": angle,
        "translation_loop_norm_over_baseline01": discrepancy,
        "signed_baselines_12_02": scales.tolist(),
        "direction_matrix_condition": float(singular[0] / max(singular[-1], 1e-12)),
    }
    # A negative edge length violates cheirality. A large loop disagreement gives no consistent seed.
    if (
        min(scales) <= 0
        or angle > 10
        or discrepancy > 0.15
        or stat["direction_matrix_condition"] > 40
    ):
        return None, stat
    return [IDENTITY, (A, ta), (B, tb * scales[1])], stat


def pnp_seeds(cams, edges, tracks):
    if len(tracks) < 8:
        return []
    seeds = []
    for (i, j), (R, t, _) in edges.items():
        k = 3 - i - j
        xa, xb = cams.rays(i, tracks[:, i]), cams.rays(j, tracks[:, j])
        xyz = triangulate(IDENTITY, (R, t), xa, xb)
        pa, za = cams.project(i, IDENTITY, xyz)
        pb, zb = cams.project(j, (R, t), xyz)
        ok = (
            (za > 0.01)
            & (zb > 0.01)
            & (np.linalg.norm(pa - tracks[:, i], axis=1) < 4)
            & (np.linalg.norm(pb - tracks[:, j], axis=1) < 4)
        )
        if ok.sum() < 8:
            continue
        cv.setRNGSeed(66)
        good, rv, tv, ids = cv.solvePnPRansac(
            xyz[ok],
            cams.rays(k, tracks[ok, k]),
            np.eye(3),
            None,
            iterationsCount=2000,
            reprojectionError=3 / cams.focal(k),
            confidence=0.999,
            flags=cv.SOLVEPNP_EPNP,
        )
        if not good or ids is None or len(ids) < 8:
            continue
        poses = {i: IDENTITY, j: (R, t), k: (cv.Rodrigues(rv)[0], tv.ravel())}
        R0, t0 = poses[0]
        poses = [(r @ R0.T, v - r @ R0.T @ t0) for r, v in [poses[c] for c in range(3)]]
        norm = np.linalg.norm(poses[1][1])
        if norm > 1e-6:
            seeds.append((f"pnp_{i}{j}", [(r, v / norm) for r, v in poses]))
    return seeds


def visible(cams, poses, view_ids, pixels):
    # Triangulate from the first two views, measure each included pixel in its native lens model.
    i, j = view_ids[:2]
    X = triangulate(
        poses[i], poses[j], cams.rays(i, pixels[:, 0]), cams.rays(j, pixels[:, 1])
    )
    positive = np.isfinite(X).all(axis=1)
    error = []
    for idx, c in enumerate(view_ids):
        xy, z = cams.project(c, poses[c], X)
        error.append(np.linalg.norm(xy - pixels[:, idx], axis=1))
        positive &= z > 0.005
    return X, positive, np.stack(error, 1)


def ranking(cams, poses, data):
    per_pair = []
    for i, j in EDGES:
        pts = data[f"p{i}{j}"]
        if not len(pts):
            per_pair.append(0)
            continue
        _, pos, err = visible(cams, poses, (i, j), pts)
        per_pair.append(int(np.count_nonzero(pos & (err.max(axis=1) < 4))))
    t = data["tracks"]
    tri = 0
    if len(t):
        _, pos, err = visible(cams, poses, (0, 1, 2), t)
        tri = int(np.count_nonzero(pos & (err.max(axis=1) < 4)))
    return (tri, min(per_pair), sum(per_pair))


def solve(cams, data):
    edges, stats = get_edges(cams, data)
    seed, cl = loop_seed(edges)
    report = {"pairs": stats, "three_view_tracks": len(data["tracks"]), "pair_loop": cl}
    seeds = pnp_seeds(cams, edges, data["tracks"])
    if seed is not None and all(len(e[2]) >= 8 for e in edges.values()):
        seeds.append(("pair_loop", seed))
    report["initializations"] = {s: list(ranking(cams, v, data)) for s, v in seeds}
    if not seeds:
        report["status"] = "no_consistent_three_camera_initialization"
        return report, None
    method, poses = max(seeds, key=lambda ent: ranking(cams, ent[1], data))
    report["initialization"] = method
    report["status"] = "candidate_needs_validation"
    return report, poses


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
        report["relative_poses"] = [
            {"R": r.tolist(), "t_in_baseline01_units": t.tolist()} for r, t in poses
        ]
    (a.folder / f"{a.method}-geometry.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    print(json.dumps({k: v for k, v in report.items() if k != "relative_poses"}))
