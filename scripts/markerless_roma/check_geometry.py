"""Check strided fisheye coordinates and metric-free three-camera recovery with known projections."""

from itertools import combinations
from pathlib import Path
import sys

import cv2 as cv
import numpy as np
from geometry import Cameras, IDENTITY, rotation_angle, solve
from refine import bundle


def main():
    cameras = Cameras(Path(sys.argv[1]))
    rng = np.random.default_rng(66)
    pts = rng.uniform([-1.1, -0.8, 4.0], [1.1, 0.8, 7.5], size=(110, 3))
    target = [
        IDENTITY,
        (
            cv.Rodrigues(np.array([-0.02, 0.12, 0.01]))[0],
            np.array([-0.99, -0.08, 0.05]),
        ),
        (cv.Rodrigues(np.array([0.06, -0.08, 0.01]))[0], np.array([0.32, -0.57, 0.15])),
    ]
    pix = np.stack(
        [cameras.project(i, pose, pts)[0] for i, pose in enumerate(target)], axis=1
    )
    assert not pix[:, 1].flags.c_contiguous
    for i in range(3):
        rays = cameras.rays(i, pix[:, i])
        reproj = cameras.project(i, IDENTITY, np.c_[rays, np.ones(len(rays))])[0]
        np.testing.assert_allclose(reproj, pix[:, i], rtol=0, atol=1e-8)
    data = {"tracks": pix}
    for i, j in combinations(range(3), 2):
        data[f"p{i}{j}"] = pix[:, [i, j]]
    _, seed = solve(cameras, data)
    assert seed is not None
    poses, fit = bundle(cameras, seed, data)
    for (r, t), (ra, ta) in zip(poses, target):
        assert rotation_angle(r @ ra.T) < 0.05
        np.testing.assert_allclose(t, ta / np.linalg.norm(target[1][1]), atol=1e-3)
    assert fit["all_positive_depth"] and fit["fit_px_after_p50_p95"][1] < 1e-3
    print("Three-camera synthetic recovery and noncontiguous pixel inputs passed.")


if __name__ == "__main__":
    main()
