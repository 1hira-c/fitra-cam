"""Decode high-contrast portions of clipped floor tags for held-out image checks only."""

import cv2 as cv
import numpy as np
import json
import sys
from pathlib import Path

p = Path(sys.argv[1])
cfg = json.loads((p.parent / sys.argv[2]).read_text())
inp = json.loads((p / "input.json").read_text())
dic = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h11)
bits = []
for n in range(len(dic.bytesList)):
    tag = cv.aruco.drawMarker(dic, n, 80)
    a = tag[15:70:10, 15:70:10] > 0
    bits.extend(np.rot90(a, k) for k in range(4))
bits = np.reshape(bits, (-1, 36))
# Map the 36-bit word to bytes, vectorize Hamming with a small popcount table.
packed = np.packbits(bits, axis=1)
lut = np.array([int(i).bit_count() for i in range(256)], np.uint8)


def examine(g, q):
    center = q.mean(axis=0)
    q = q[np.argsort(np.arctan2(q[:, 1] - center[1], q[:, 0] - center[0]))]
    q = np.roll(q, -np.argmin(q.sum(axis=1)), axis=0)
    dest = np.float32([[-0.5, -0.5], [79.5, -0.5], [79.5, 79.5], [-0.5, 79.5]])
    warp = cv.warpPerspective(
        g,
        cv.getPerspectiveTransform(np.float32(q), dest),
        (80, 80),
        flags=cv.INTER_LINEAR,
    )
    cells = warp.reshape(8, 10, 8, 10)[:, 2:8, :, 2:8].mean(axis=(1, 3))
    v = cells[1:-1, 1:-1].ravel()
    plo, phi = np.percentile(warp, [4, 85])
    cert = {}
    for frac in (0.28, 0.4, 0.5, 0.6, 0.7, 0.78, 0.86):
        th = plo + (phi - plo) * frac
        # The printed tags use a black 1-cell border. Ignore near-empty patches.
        if (
            np.concatenate([cells[0], cells[-1], cells[1:-1, 0], cells[1:-1, -1]]) > th
        ).sum() > 10:
            continue
        word = np.packbits(v > th)
        d = lut[packed ^ word].sum(axis=1)
        ix = np.argpartition(d, 2)[:2]
        ix = ix[np.argsort(d[ix])]
        best, second = map(int, d[ix])
        if best > 4 or second - best < 4:
            continue
        tag, rot = divmod(int(ix[0]), 4)
        key = (tag, rot)
        rank = (best, -second, frac)
        if key not in cert or rank < cert[key][0]:
            cert[key] = (rank, q.copy())
    return cert


out = {}
standard = json.loads((p / "withheld-corners.json").read_text())
combined = json.loads(json.dumps(standard))
agreement = []
for i in [1, 2]:
    im = cv.imread(str(p / f"raw{i}.jpg"), 0)
    for cx, cy, rx, ry in cfg.get(inp["views"][i]["image_sha256"], []):
        left = max(0, cx - rx)
        top = max(0, cy - ry)
        g = im[top : cy + ry + 1, left : cx + rx + 1]
        quad = {}
        for th in list(range(20, 254, 6)) + [246, 250, 252, 253, 254]:
            binary = np.uint8(g < th) * 255
            comps = cv.findContours(binary, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[0]
            y, x = np.where(binary)
            mask = ((x - (cx - left)) / (rx * 0.86)) ** 2 + (
                (y - (cy - top)) / (ry * 0.86)
            ) ** 2 < 1
            if mask.sum() > 10:
                comps = list(comps) + [
                    np.int32(np.c_[x[mask], y[mask]]).reshape(-1, 1, 2)
                ]
            for ct in comps:
                hull = cv.convexHull(ct)
                per = cv.arcLength(hull, True)
                if per < 35:
                    continue
                for ep in (0.025, 0.045, 0.065):
                    q = cv.approxPolyDP(hull, ep * per, True).reshape(-1, 2)
                    if len(q) != 4 or cv.contourArea(np.int32(q)) < 65:
                        continue
                    if np.linalg.norm(q.mean(0) - [cx - left, cy - top]) > 16:
                        continue
                    quad[tuple(q.ravel())] = q
        found = {}
        for q in quad.values():
            center = q.mean(axis=0)
            for sc in (0.91, 1.0, 1.08):
                for (n, k), (rank, qr) in examine(
                    g, center + (q - center) * sc
                ).items():
                    found.setdefault((n, k), []).append((rank, qr))
        rank = sorted(
            [(kk, len(v), min(e[0][:2] for e in v)) for kk, v in found.items()],
            key=lambda e: (e[2][0], -e[1]),
        )
        print(i, (cx, cy), len(quad), rank[:6], flush=True)
        if not rank:
            continue
        ids = {label[0] for label, _, _ in rank}
        if len(ids) != 1:
            continue
        n = next(iter(ids))
        candidates = []
        for (tag, k), values in found.items():
            for quality, corners in values:
                if quality[0] <= 2 and -quality[1] >= 8:
                    candidates.append(
                        (quality, np.roll(corners, k, axis=0) + [left, top])
                    )
        if len(candidates) < 5:
            continue
        best = min(quality[0] for quality, _ in candidates)
        quads = np.stack([q for quality, q in candidates if quality[0] <= best + 1])
        xy = np.median(quads, axis=0)
        spread = float(
            np.percentile(np.linalg.norm(quads - xy, axis=2).max(axis=1), 95)
        )
        best_centers = []
        for quad in quads:
            hq = np.c_[quad, np.ones(4)]
            center = np.cross(np.cross(hq[0], hq[2]), np.cross(hq[1], hq[3]))
            best_centers.append(center[:2] / center[2])
        centers = np.stack(best_centers)
        center = np.median(centers, axis=0)
        obs = {
            "center_px": center.tolist(),
            "center_spread_p95_px": float(
                np.percentile(np.linalg.norm(centers - center, axis=1), 95)
            ),
            "pixels": xy.tolist(),
            "n": len(quads),
            "spread_p95_px": spread,
            "best_hamming": int(best),
            "source": "masked ROI full 36h11 dictionary",
        }
        out[f"{i}_{cx}_{cy}"] = {"id": int(n), **obs}
        if str(n) in standard[i]:
            ref = np.array(standard[i][str(n)]["pixels"])
            agreement.append(
                {
                    "camera": i,
                    "id": int(n),
                    "corner_max_px": float(np.linalg.norm(ref - xy, axis=1).max()),
                }
            )
        else:
            combined[i][str(n)] = obs
(p / "supplementary-decodes.json").write_text(
    json.dumps({"detections": out, "agreement": agreement}, indent=2) + "\n"
)
(p / "combined-corners.json").write_text(json.dumps(combined, indent=2) + "\n")
print(
    "accepted observations",
    [(k, v["id"], v["n"], round(v["spread_p95_px"], 2)) for k, v in out.items()],
)
print("independent decoder comparison:", agreement)
