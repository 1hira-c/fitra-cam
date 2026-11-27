"""Take one local MJPEG still per idle device from the configured camera slots."""

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import time

import cv2 as cv
import yaml


def main():
    cfg = yaml.safe_load(Path(sys.argv[1]).read_text())["cameras"]
    dest = Path(sys.argv[2])
    dest.mkdir(mode=0o700)

    def grab(i):
        dev = cfg[f"cam{i}"]
        image = dest / f"raw{i}.jpg"
        args = [
            "v4l2-ctl",
            "-d",
            dev,
            f"--set-fmt-video=width={cfg['width']},height={cfg['height']},pixelformat=MJPG",
            "--stream-mmap=4",
            "--stream-skip=120",
            "--stream-count=1",
            f"--stream-to={image}",
        ]
        begin = time.monotonic_ns()
        cp = subprocess.run(args, timeout=15, capture_output=True, text=True)
        end = time.monotonic_ns()
        cp.check_returncode()
        data = image.read_bytes()
        assert data.startswith(b"\xff\xd8") and data.rstrip(b"\x00").endswith(
            b"\xff\xd9"
        )
        assert cv.imread(str(image)).shape[:2] == (cfg["height"], cfg["width"])
        return {
            "camera": i,
            "device": dev,
            "realpath": str(Path(dev).resolve()),
            "sha256": hashlib.sha256(data).hexdigest(),
            "bytes": len(data),
            "host_call_monotonic_ns": [str(begin), str(end)],
        }

    start = datetime.now(timezone.utc).isoformat()
    with ThreadPoolExecutor(max_workers=3) as pool:
        views = list(pool.map(grab, range(3)))
    (dest / "capture.json").write_text(
        json.dumps(
            {"utc": start, "views": views, "size": [cfg["width"], cfg["height"]]},
            indent=2,
        )
        + "\n"
    )
    print(start, [(v["camera"], v["bytes"]) for v in views])


if __name__ == "__main__":
    main()
