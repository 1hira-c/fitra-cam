#!/usr/bin/env python3
"""Dump a YOLOX INT8 calibration blob from recorded mp4 frames.

Output layout: raw float32 little-endian, N consecutive tensors of shape
(3, S, S) in CHW order (no header). The C++ Int8EntropyCalibrator2 expects
exactly this layout; the size of the file divided by `3 * S * S * 4` must
equal N.

Preprocessing must match cpp/src/infer/yolox.cpp::letterbox bit-for-bit:
  - r = min(S / h, S / w)
  - resize to (nh, nw) = (round(h * r), round(w * r)) with INTER_LINEAR
  - pad to S x S with value 114, top-left aligned
  - BGR raw, NO normalization
  - CHW float32

Usage:
  python python/scripts/dump_yolox_calibration_blobs.py \
      --video outputs/recorded_rtmpose/20260515_064342/raw_cam0.mp4 \
      --video outputs/recorded_rtmpose/20260515_064342/raw_cam1.mp4 \
      --input-size 640 --target 200 \
      --output outputs/tensorrt_engines/calib_yolox_640.bin
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def yolox_letterbox(img: np.ndarray, target: int) -> np.ndarray:
    """Mirror of cpp/src/infer/yolox.cpp::letterbox / pose_pipeline._yolox_letterbox."""
    h, w = img.shape[:2]
    r = min(target / h, target / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    padded = np.full((target, target, 3), 114, dtype=np.uint8)
    padded[:nh, :nw] = resized
    return padded


def evenly_spaced_indices(total: int, target: int) -> list[int]:
    if total <= 0 or target <= 0:
        return []
    if target >= total:
        return list(range(total))
    step = total / target
    return [int(round(i * step)) for i in range(target)]


def iter_video_frames(path: Path, indices: list[int]):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open video: {path}")
    try:
        # Sequential read with frame skip; CAP_PROP_POS_FRAMES is unreliable
        # on some V4L2-captured mp4s, so we step.
        wanted = sorted(set(indices))
        wanted_iter = iter(wanted)
        next_target = next(wanted_iter, None)
        frame_idx = 0
        while next_target is not None:
            ok, frame = cap.read()
            if not ok or frame is None:
                break
            if frame_idx == next_target:
                yield frame_idx, frame
                next_target = next(wanted_iter, None)
            frame_idx += 1
    finally:
        cap.release()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--video", action="append", required=True,
                    help="input mp4 (repeatable). Frames are sampled evenly across the union.")
    ap.add_argument("--input-size", "-s", type=int, required=True,
                    help="YOLOX input size S (416 for tiny, 640 for s/m/x).")
    ap.add_argument("--target", "-N", type=int, default=200,
                    help="total frames to sample across all videos (default 200).")
    ap.add_argument("--output", "-o", type=Path, required=True,
                    help="output raw float32 (N,3,S,S) binary.")
    args = ap.parse_args()

    videos = [Path(v) for v in args.video]
    for v in videos:
        if not v.exists():
            print(f"video not found: {v}", file=sys.stderr)
            return 2

    # Probe frame counts (CAP_PROP_FRAME_COUNT is reliable for re-encoded
    # mp4s; on raw V4L2 captures it can be off by a frame or two, but that
    # is fine for even-spacing sampling).
    frame_counts: list[int] = []
    for v in videos:
        cap = cv2.VideoCapture(str(v))
        if not cap.isOpened():
            print(f"cannot open: {v}", file=sys.stderr)
            return 2
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        if n <= 0:
            print(f"video reports 0 frames: {v}", file=sys.stderr)
            return 2
        frame_counts.append(n)

    total = sum(frame_counts)
    per_video_targets = [max(1, round(args.target * n / total)) for n in frame_counts]
    # Trim/pad to exactly args.target.
    diff = args.target - sum(per_video_targets)
    if diff != 0:
        per_video_targets[0] += diff
    print(f"sampling plan: total_pool={total} target={args.target} "
          f"per_video={list(zip([str(v) for v in videos], per_video_targets))}",
          file=sys.stderr)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    written = 0
    with open(args.output, "wb") as out:
        for v, n, k in zip(videos, frame_counts, per_video_targets):
            if k <= 0:
                continue
            idxs = evenly_spaced_indices(n, k)
            for _frame_idx, frame in iter_video_frames(v, idxs):
                padded = yolox_letterbox(frame, args.input_size)
                blob = padded.transpose(2, 0, 1).astype(np.float32, copy=False)
                out.write(blob.tobytes())
                written += 1
                if written % 50 == 0:
                    print(f"  wrote {written} frames", file=sys.stderr)

    bytes_per_image = 3 * args.input_size * args.input_size * 4
    final_size = args.output.stat().st_size
    expected = written * bytes_per_image
    print(f"done: wrote {written} frames -> {args.output} "
          f"({final_size} bytes, expected {expected}, "
          f"bytes_per_image={bytes_per_image})", file=sys.stderr)
    if final_size != expected:
        print("ERROR: blob size mismatch", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
