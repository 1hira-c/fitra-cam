#!/usr/bin/env python3
"""Detect floor AprilTags (tag36h11) from each USB camera.

Experiment helper: confirms that the 6 floor-placed 11cm tag36h11 markers
are visible from each camera. Captures a short burst per camera, runs the
OpenCV aruco AprilTag detector, reports the union of detected IDs, and writes
an annotated snapshot for visual confirmation.

Resolution defaults to 1280x960 (4:3, same field of view as 640x480 VGA but
double the linear resolution) so small/far tags resolve to enough pixels.
"""
import argparse
import time
from collections import Counter

import cv2
import numpy as np

CAMS = {
    "cam0": "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.3:1.0-video-index0",
    "cam1": "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4:1.0-video-index0",
}


def open_cam(dev, w, h, fourcc="MJPG", focus=None):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    if focus is not None:
        # disable continuous autofocus, then pin manual focus
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_FOCUS, focus)
    return cap


def make_detector():
    # cv2 4.5.4: legacy aruco API (ArucoDetector class arrived in 4.7)
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return dictionary, params


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=960)
    ap.add_argument("--frames", type=int, default=30, help="frames to scan per camera")
    ap.add_argument("--warmup", type=int, default=10, help="frames to discard first")
    ap.add_argument("--outdir", default="/home/hitohira/Documents/fitra-cam/outputs/apriltag_check")
    ap.add_argument("--fourcc", default="MJPG", help="MJPG or YUYV")
    ap.add_argument("--focus", type=int, default=None,
                    help="manual focus 0-1023 (disables autofocus). omit = keep autofocus")
    ap.add_argument("--suffix", default="", help="suffix for output filenames")
    ap.add_argument("--clahe", action="store_true",
                    help="apply CLAHE local-contrast enhancement before detection "
                         "(recovers soft/low-contrast tags)")
    args = ap.parse_args()
    clahe = cv2.createCLAHE(2.0, (8, 8)) if args.clahe else None

    import os
    os.makedirs(args.outdir, exist_ok=True)
    dictionary, params = make_detector()

    for name, dev in CAMS.items():
        cap = open_cam(dev, args.width, args.height, args.fourcc, args.focus)
        if not cap.isOpened():
            print(f"[{name}] FAILED to open {dev}")
            continue
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        for _ in range(args.warmup):
            cap.read()

        seen = Counter()           # id -> frames it appeared in
        best_frame = None
        best_corners = None
        best_ids = None
        best_count = -1
        scanned = 0

        for _ in range(args.frames):
            ok, frame = cap.read()
            if not ok:
                continue
            scanned += 1
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if clahe is not None:
                gray = clahe.apply(gray)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
            n = 0 if ids is None else len(ids)
            if ids is not None:
                for i in ids.flatten():
                    seen[int(i)] += 1
            if n > best_count:
                best_count = n
                best_frame = frame.copy()
                best_corners = corners
                best_ids = ids
        cap.release()

        ids_sorted = sorted(seen.keys())
        print(f"[{name}] {dev}")
        print(f"  resolution: {aw}x{ah}  scanned {scanned} frames")
        print(f"  detected {len(ids_sorted)} unique tag IDs: {ids_sorted}")
        for tid in ids_sorted:
            print(f"    id={tid:<3} seen in {seen[tid]}/{scanned} frames")

        if best_frame is not None:
            vis = best_frame
            if best_ids is not None:
                cv2.aruco.drawDetectedMarkers(vis, best_corners, best_ids)
            label = f"{name}  {aw}x{ah}  ids={sorted(int(i) for i in best_ids.flatten())}" \
                if best_ids is not None else f"{name}  {aw}x{ah}  none"
            cv2.putText(vis, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0), 2, cv2.LINE_AA)
            path = os.path.join(args.outdir, f"{name}{args.suffix}.png")
            cv2.imwrite(path, vis)
            print(f"  saved: {path}")
        print()


if __name__ == "__main__":
    main()
