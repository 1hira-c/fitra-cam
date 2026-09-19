#!/usr/bin/env python3
"""Explicit adapter for legacy dump_keypoints_3d --no-kalman --no-ik --no-floor-contact-stability.

No measured timestamps are invented. Unknown provenance must be resolved by the
operator, not guessed from smooth-looking coordinates or filenames.
"""
import argparse
import json
import math
from pathlib import Path
from evaluate import digest


def convert(path, hz, height, segment, phase):
    if not math.isfinite(hz) or not 5 <= hz <= 240 or not math.isfinite(height) or not 1 <= height <= 2.5:
        raise ValueError("invalid nominal frequency or subject height")
    header = {"schema":"fitra_temporal_input_v1", "input_stage":"raw_triangulated",
              "kp_format":"halpe26", "coordinate_frame":"fitra_world_z_up_m", "nominal_hz":hz,
              "subject_height_m":height, "reference_kind":"none", "time_basis":"nominal_from_legacy_frames",
              "provenance":{"source_path":str(path.resolve()), "source_sha256":digest(path),
                            "raw_stage":"operator_confirmed", "camera_sync":"unverified"}}
    rows, previous = [], -1
    with path.open() as f:
        for line in f:
            source = json.loads(line)
            frame = source["frame"]
            if type(frame) is not int or frame < 0 or frame <= previous:
                raise ValueError("nonmonotonic legacy frame; import one session/clip at a time")
            if source["kp_format"] != "halpe26":
                raise ValueError("Halpe26 required")
            people = source["persons_3d"]
            if len(people) > 1:
                raise ValueError("multi-person identity is not resolved")
            joints = None
            if people:
                js = people[0]["joints"]
                if len(js) != 26:
                    raise ValueError("26 joints required")
                joints = []
                for j in js:
                    if len(j) != 5 or type(j[4]) is not bool:
                        raise ValueError("joint must be [x,y,z,score,valid]")
                    # SimCC maxima are nonnegative scores, not probabilities.
                    # Production RTMPose/triangulation preserve values above 1.
                    if j[4] and (not all(type(x) in (int,float) and math.isfinite(x) for x in j[:4])
                                  or j[3] < 0):
                        raise ValueError("invalid valid-joint value")
                    joints.append(j[:4] if j[4] else None)
            fresh = joints is not None and any(j is not None for j in joints)
            rows.append({"seq":frame, "t_ns":round(frame*1e9/hz), "segment":segment,
                         "phase":phase, "state":"fresh" if fresh else "boundary",
                         "joints":joints if fresh else None, "reference_joints":None})
            previous = frame
    if not rows:
        raise ValueError("empty dump")
    return header, rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--confirm-unfiltered", action="store_true", required=True,
                   help="confirm dump was generated with all three post-3D stages disabled")
    p.add_argument("--nominal-hz", required=True, type=float)
    p.add_argument("--subject-height-m", required=True, type=float)
    p.add_argument("--segment", required=True)
    p.add_argument("--phase", choices=("static","motion","unlabelled"), default="unlabelled")
    args = p.parse_args()
    header, rows = convert(args.input, args.nominal_hz, args.subject_height_m, args.segment, args.phase)
    with args.output.open("x") as f:
        for row in [header, *rows]:
            f.write(json.dumps(row, allow_nan=False)+"\n")


if __name__ == "__main__":
    main()
