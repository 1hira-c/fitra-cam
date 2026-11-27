"""Redetect coded corners within the masks on the original stills for independent image checks."""

import argparse
import json
from pathlib import Path

import cv2 as cv
import numpy as np


def detect(folder):
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h11)
    results = []
    for i in range(3):
        gray = cv.imread(str(folder / f"raw{i}.jpg"), 0)
        valid = cv.imread(str(folder / f"valid{i}.png"), 0)
        contours, _ = cv.findContours(
            255 - valid, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
        )
        found = {}
        for contour in contours:
            x, y, w, h = cv.boundingRect(contour)
            x0, y0 = max(0, x - 18), max(0, y - 18)
            patch = gray[
                y0 : min(y + h + 18, gray.shape[0]), x0 : min(x + w + 18, gray.shape[1])
            ]
            lo, hi = np.percentile(patch, [1, 98.5])
            sharp = np.uint8(
                np.clip((patch.astype(float) - lo) * (255 / max(1, hi - lo)), 0, 255)
            )
            clahe = cv.createCLAHE(3, (4, 4)).apply(patch)
            for label, src in (("raw", patch), ("stretch", sharp), ("clahe", clahe)):
                for mag in (2.0, 4.0):
                    img = cv.resize(
                        src, None, fx=mag, fy=mag, interpolation=cv.INTER_CUBIC
                    )
                    for mode in (
                        cv.aruco.CORNER_REFINE_SUBPIX,
                        cv.aruco.CORNER_REFINE_APRILTAG,
                    ):
                        prm = cv.aruco.DetectorParameters_create()
                        prm.cornerRefinementMethod = mode
                        prm.minMarkerPerimeterRate = 0.018
                        prm.polygonalApproxAccuracyRate = 0.05
                        cs, ids, _ = cv.aruco.detectMarkers(
                            img, dictionary, parameters=prm
                        )
                        if ids is None:
                            continue
                        for tag, pts in zip(ids.ravel(), cs):
                            xy = (
                                (pts.reshape(4, 2) + 0.5) / mag
                                - 0.5
                                + np.array([x0, y0])
                            )
                            xi, yi = np.rint(xy).astype(int).T
                            if np.all(valid[yi, xi] == 0):
                                found.setdefault(int(tag), []).append(
                                    (xy, f"{label}/{mag}/{mode}")
                                )
        reliable = {}
        for tag, v in found.items():
            arr = np.array([u[0] for u in v])
            # subpixel jitter can be systematic: report it per tag rather than assuming exact GT.
            xy = np.median(arr, axis=0)
            spread = np.linalg.norm(arr - xy, axis=2).max(axis=1)
            centers = []
            for vv in arr:
                hq = np.c_[vv, np.ones(4)]
                diagonal = np.cross(np.cross(hq[0], hq[2]), np.cross(hq[1], hq[3]))
                centers.append(diagonal[:2] / diagonal[2])
            centers = np.stack(centers)
            med = np.median(centers, axis=0)
            reliable[tag] = {
                "center_px": med.tolist(),
                "center_spread_p95_px": float(
                    np.percentile(np.linalg.norm(centers - med, axis=1), 95)
                ),
                "pixels": xy.tolist(),
                "n": len(v),
                "spread_p95_px": float(np.percentile(spread, 95)),
            }
        results.append(reliable)
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("folder", type=Path)
    a = p.parse_args()
    cv.setNumThreads(2)
    v = detect(a.folder)
    (a.folder / "withheld-corners.json").write_text(json.dumps(v, indent=2) + "\n")
    print(
        [
            {
                tag: (entry["n"], round(entry["spread_p95_px"], 2))
                for tag, entry in cam.items()
            }
            for cam in v
        ]
    )
