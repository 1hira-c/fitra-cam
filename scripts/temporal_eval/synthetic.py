#!/usr/bin/env python3
"""Deterministic fixture, not a model-accuracy or real-rig acceptance dataset."""
import argparse
import json
import math
from pathlib import Path
import random


def pose():
    values = [None]*26
    points = {0:(0,.08,1.62), 17:(0,0,1.7), 18:(0,0,1.45), 19:(0,0,.95),
              5:(.18,0,1.42), 6:(-.18,0,1.42), 7:(.46,.03,1.37), 8:(-.46,.03,1.37),
              9:(.70,.1,1.32), 10:(-.70,.1,1.32), 11:(.1,0,.95), 12:(-.1,0,.95),
              13:(.1,.025,.52), 14:(-.1,.025,.52), 15:(.1,0,.09), 16:(-.1,0,.09),
              20:(.1,.17,.01), 21:(-.1,.17,.01), 22:(.15,.13,.01), 23:(-.15,.13,.01),
              24:(.1,-.05,.01), 25:(-.1,-.05,.01)}
    for i, xyz in points.items():
        values[i] = [*xyz, .95]
    return values


def dataset(hz=60, seed=20260908):
    rng = random.Random(seed)
    header = {"schema":"fitra_temporal_input_v1", "input_stage":"raw_triangulated",
              "kp_format":"halpe26", "coordinate_frame":"fitra_world_z_up_m",
              "nominal_hz":hz, "subject_height_m":1.7, "reference_kind":"synthetic_known",
              "time_basis":"synthetic_nominal", "provenance":{"generator":"synthetic.py", "seed":seed,
                  "startup_unscored_reports_per_scene":hz}}
    rows = []
    for scene in ("static", "translation", "arm_swing", "start_stop", "dropout", "forward_bend_fixed_feet"):
        for i in range(6*hz):
            t = i/hz
            truth = pose()
            if scene in ("translation", "dropout"):
                dx = .13*math.sin(2*math.pi*.65*t)+.035*math.sin(2*math.pi*1.1*t)
                for p in truth:
                    if p: p[0] += dx
            elif scene == "arm_swing":
                theta = .6*math.sin(2*math.pi*.8*t)
                for side, shoulder, elbow, wrist in ((1,5,7,9),(-1,6,8,10)):
                    for idx, length in ((elbow,.28),(wrist,.53)):
                        truth[idx][:3] = [truth[shoulder][0]+side*length*math.cos(theta),
                                          .04, truth[shoulder][2]+length*math.sin(theta)]
            elif scene == "start_stop":
                dx = .25*max(0,min(t-1,1))-.25*max(0,min(t-3,1))
                for p in truth:
                    if p: p[0] += dx
            elif scene == "forward_bend_fixed_feet":
                # Control case: torso leans while the true leg/foot pose is fixed.
                # This is not a reproduction or causal explanation of the user's
                # observed thigh-only twist instability.
                theta = math.radians(70)*(.5-.5*math.cos(2*math.pi*t/3))
                for k in (0,5,6,7,8,9,10,17,18):
                    x,y,z,_ = truth[k]
                    z -= .95
                    truth[k][:3] = [x,y*math.cos(theta)+z*math.sin(theta),
                                   .95+z*math.cos(theta)-y*math.sin(theta)]
            noisy = [[p[c]+rng.gauss(0,.003) for c in range(3)]+[p[3]] if p else None for p in truth]
            if scene == "forward_bend_fixed_feet":
                # Stable observed ankles/toes/heels. Hip/knee/body noise remains.
                for k in (15,16,20,21,22,23,24,25):
                    noisy[k] = list(truth[k])
            boundary = scene == "dropout" and 2*hz <= i < 2*hz+6
            if scene == "dropout" and 3*hz <= i < 3*hz+5:
                for k in (15,20,22,24): noisy[k] = None
            n = len(rows)
            rows.append({"seq":n, "t_ns":round(n*1e9/hz), "segment":scene,
                         "evaluate":i >= hz,
                         "state":"boundary" if boundary else "fresh",
                         "phase":"static" if scene == "static" else "motion",
                         "joints":noisy if not boundary else None,
                         "reference_joints":truth if not boundary else None})
    return header, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    header, rows = dataset()
    with args.output.open("x") as f:
        for row in [header, *rows]:
            f.write(json.dumps(row, allow_nan=False)+"\n")


if __name__ == "__main__":
    main()
