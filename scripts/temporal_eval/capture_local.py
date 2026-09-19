#!/usr/bin/env python3
"""Bounded local-only smoke capture using the existing event recorder binary.

Does not modify the supplied configuration. Persons trigger raw recording;
successful process shutdown alone does not prove any session was recorded.
"""
import argparse
import signal
import subprocess
from pathlib import Path

from run_recording import artifact, write_json


def command(binary, config, out):
    return [str(binary), "--config", str(config), "--no-web", "--no-idle",
            "--no-vmt-out", "--no-vmt-discovery", "--corpus-record",
            "--corpus-spool", str(out / "spool"), "--corpus-remote", "",
            "--corpus-spool-max-gib", "4", "--corpus-min-free-gib", "10"]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("binary", type=Path)
    p.add_argument("config", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--seconds", type=int, default=90)
    p.add_argument("--asset", action="append", type=Path, default=[],
                   help="freeze provenance of engines/calibration/profile in addition to config")
    args = p.parse_args()
    if not 10 <= args.seconds <= 300 or args.output.exists():
        p.error("choose 10..300 seconds and a new output directory")
    files = {str(path): artifact(path) for path in [args.binary, args.config, *args.asset]}
    out = args.output.resolve()
    cmd = command(args.binary.resolve(), args.config.resolve(), out)
    out.mkdir(parents=True, exist_ok=False)
    write_json(out / "capture.json", {"schema": "fitra_local_capture_run_v1", "argv": cmd,
        "seconds": args.seconds, "assets": files, "launcher": artifact(__file__),
        "limits": "4 GiB spool; 10 GiB minimum free; no remote upload or VMT output",
        "purpose": "capture qualification; not labelled motion or live latency ground truth"})
    with (out / "capture.log").open("x") as log:
        process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        print(f"started pid={process.pid}; auto-stop after {args.seconds}s; log={out / 'capture.log'}", flush=True)
        timed_stop = False
        try:
            process.wait(timeout=args.seconds)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            timed_stop = True
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if process.returncode != 0:
            raise RuntimeError(f"capture exited {process.returncode}; inspect log, keep partial artifacts")
    if any(artifact(item["path"]) != item for item in files.values()):
        raise RuntimeError("capture inputs changed; provenance is not stable")
    sessions = sorted(str(path) for path in (out / "spool" / "ready").glob("*"))
    write_json(out / "stopped.json", {"returncode": process.returncode, "stop_requested": timed_stop,
        "ready_sessions": sessions, "capture_manifest": artifact(out / "capture.json"),
        "log": artifact(out / "capture.log"), "qualification": "not_yet_inspected"})
    print(f"stopped cleanly; ready sessions={len(sessions)}", flush=True)


if __name__ == "__main__":
    main()
