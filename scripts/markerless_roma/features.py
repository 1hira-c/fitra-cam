"""Create pairs and explicit, cycle-checked three-view tracks in native pixel coordinates."""

import argparse
from itertools import combinations
from pathlib import Path

import cv2 as cv
import numpy as np


EDGES = list(combinations(range(3), 2))
W, H = 1280, 960


def thin(points, scores, cell):
    used = [set() for _ in range(points.shape[1])]
    keep = []
    for idx in np.argsort(-scores, kind="stable"):
        cells = [tuple(q) for q in np.floor(points[idx] / cell).astype(int)]
        if any(q in seen for q, seen in zip(cells, used)):
            continue
        for q, seen in zip(cells, used):
            seen.add(q)
        keep.append(idx)
    return points[keep]


def rootsift(folder, masks):
    sift = cv.SIFT_create(nfeatures=6000)
    desc, views = [], []
    for i, mask in enumerate(masks):
        gray = cv.imread(str(folder / f"input{i}.png"), cv.IMREAD_GRAYSCALE)
        k, v = sift.detectAndCompute(gray, mask)
        desc.append(np.sqrt(v / np.maximum(v.sum(axis=1, keepdims=True), 1e-9)))
        views.append(np.float32([kp.pt for kp in k]))
    matcher = cv.BFMatcher(cv.NORM_L2)
    links, result = {}, {}
    for i, j in EDGES:
        two = {}
        for a, b in ((i, j), (j, i)):
            nn = matcher.knnMatch(desc[a], desc[b], k=2)
            two[a, b] = {
                best.queryIdx: (
                    best.trainIdx,
                    1 - best.distance / max(1e-12, second.distance),
                )
                for best, second in nn
                if best.distance < 0.80 * second.distance
            }
        matches = {
            x: (y, s)
            for x, (y, s) in two[i, j].items()
            if two[j, i].get(y, (-1, 0))[0] == x
        }
        links[i, j] = matches
        pairs = np.float32(
            [[views[i][x], views[j][y]] for x, (y, _) in matches.items()]
        ).reshape(-1, 2, 2)
        result[f"p{i}{j}"] = thin(pairs, np.array([s for y, s in matches.values()]), 12)
    triples, score = [], []
    for x, (y, sa) in links[0, 1].items():
        if x in links[0, 2] and y in links[1, 2]:
            z, sb = links[0, 2][x]
            z2, sc = links[1, 2][y]
            if z == z2:
                triples.append([views[0][x], views[1][y], views[2][z]])
                score.append(min(sa, sb, sc))
    result["tracks"] = thin(np.float32(triples).reshape(-1, 3, 2), np.array(score), 12)
    return result


def roma(folder, masks):
    field = {}
    for i, j in EDGES:
        with np.load(folder / f"fields{i}{j}.npz") as d:
            for a, b, side in ((i, j, "AB"), (j, i, "BA")):
                field[a, b] = np.concatenate(
                    (d[f"warp_{side}"], d[f"overlap_{side}"]), axis=-1
                )

    def query(a, b, pts):
        f = field[a, b]
        h, w = f.shape[:2]
        xy = ((pts + 0.5) * np.float32([w / W, h / H]) - 0.5).reshape(-1, 1, 2)
        hit = cv.remap(
            f, xy, None, cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT
        ).reshape(-1, 3)
        target = (hit[:, :2] + 1) * np.float32([W / 2, H / 2]) - 0.5
        return target, hit[:, 2]

    def allowed(a, pts):
        x, y = np.rint(pts).astype(int).T
        return (
            (x >= 0)
            & (x < W)
            & (y >= 0)
            & (y < H)
            & (masks[a][np.clip(y, 0, H - 1), np.clip(x, 0, W - 1)] > 0)
        )

    def reciprocal(a, b, pts):
        pos, c = query(a, b, pts)
        back, d = query(b, a, pos)
        ok = (
            allowed(a, pts)
            & allowed(b, pos)
            & (c >= 0.05)
            & (d >= 0.05)
            & (np.linalg.norm(back - pts, axis=1) <= 3.0)
        )
        return pos, ok, np.minimum(c, d)

    y, x = np.mgrid[5:H:10, 5:W:10]
    grid = np.column_stack([x.ravel(), y.ravel()]).astype(np.float32)
    result = {}
    for i, j in EDGES:
        forward, good, cs = reciprocal(i, j, grid)
        back, good2, ds = reciprocal(j, i, grid)
        pairs = np.concatenate(
            (
                np.stack((grid[good], forward[good]), 1),
                np.stack((back[good2], grid[good2]), 1),
            )
        )
        result[f"p{i}{j}"] = thin(pairs, np.r_[cs[good], ds[good2]], 12)
    triples = []
    scores = []
    for i in range(3):
        j, k = tuple(n for n in range(3) if n != i)
        qj, j_ok, sj = reciprocal(i, j, grid)
        qk, k_ok, sk = reciprocal(i, k, grid)
        jk, ok_jk, sjk = reciprocal(j, k, qj)
        kj, ok_kj, skj = reciprocal(k, j, qk)
        ok = (
            j_ok
            & k_ok
            & ok_jk
            & ok_kj
            & (np.linalg.norm(jk - qk, axis=1) <= 4)
            & (np.linalg.norm(kj - qj, axis=1) <= 4)
        )
        per = {i: grid, j: qj, k: qk}
        triples.append(np.stack([per[c][ok] for c in range(3)], 1))
        scores.append(np.minimum.reduce([sj, sk, sjk, skj])[ok])
    result["tracks"] = thin(np.concatenate(triples), np.concatenate(scores), 12)
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("frames", type=Path)
    p.add_argument("method", choices=("rootsift", "roma"))
    args = p.parse_args()
    cv.setNumThreads(2)
    cv.setRNGSeed(66)
    masks = [cv.imread(str(args.frames / f"valid{i}.png"), 0) for i in range(3)]
    result = {"rootsift": rootsift, "roma": roma}[args.method](args.frames, masks)
    np.savez_compressed(args.frames / f"{args.method}-pairs.npz", **result)
    print({k: len(v) for k, v in result.items()})
