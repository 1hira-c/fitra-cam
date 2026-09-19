#!/usr/bin/env python3
"""Validate closed raw corpus sessions and index measured-time camera groups.

This is capture-data qualification, not live inference scheduling or 3D replay.
Only kernel timestamps with identical SOE/EOF semantics may form a group.
"""
import argparse
import json
import math
from pathlib import Path

from evaluate import percentile
from run_recording import artifact, write_json


def uint(value, bits=64):
    if type(value) is str and value.isascii() and value.isdecimal():
        value = int(value)
    if type(value) is not int or not 0 <= value < 2**bits:
        raise ValueError("invalid unsigned integer")
    return value


def local_file(directory, name):
    if not isinstance(name, str) or not name or Path(name).name != name or name in (".", ".."):
        raise ValueError("expected a flat relative file name")
    path = directory / name
    if path.is_symlink() or not path.is_file():
        raise ValueError("missing file or unexpected symlink")
    return path


def verify_segment(directory):
    sums = local_file(directory, "checksums.sha256")
    checked = {}
    for line in sums.read_text().splitlines():
        expected, name = line.split("  ", 1)
        if name in checked or name == "checksums.sha256":
            raise ValueError("duplicate or self-referential checksum")
        info = artifact(local_file(directory, name))
        if info["sha256"] != expected:
            raise ValueError(f"checksum mismatch: {name}")
        checked[name] = info
    actual = {p.name for p in directory.iterdir() if p.name != "checksums.sha256"}
    if actual != set(checked) or not {"segment.json", "frames.jsonl"} <= actual:
        raise ValueError("checksum coverage does not match segment contents")
    return checked


def load_session(session):
    session = session.resolve(strict=True)
    manifest_path = local_file(session, "manifest.json")
    manifest_info = artifact(manifest_path)
    m = json.loads(manifest_path.read_text())
    if m["schema"] != "fitra_event_corpus_v1" or (session / "RECOVERED_INCOMPLETE.txt").exists():
        raise ValueError("only complete event corpus sessions are supported")
    if session.parent.name != "ready":
        raise ValueError("inspect closed ready sessions, never a running open session")
    cameras = m["cameras"]
    ids = [c["index"] for c in cameras]
    if len(ids) not in (2, 3) or ids != list(range(len(ids))):
        raise ValueError("expected compact ordered two/three-camera manifest")
    frames = [[] for _ in cameras]
    last_kernel_time = [None] * len(cameras)
    if any(c["raw_encoding"] not in ("jpeg", "yuyv422") for c in cameras):
        raise ValueError("unsupported raw encoding")
    verified = []
    directories = sorted((session / "segments").iterdir())
    if not directories or any(p.is_symlink() or not p.is_dir() or p.name.endswith(".partial") for p in directories):
        raise ValueError("missing, partial, or unsupported segments")
    for segment_index, directory in enumerate(directories):
        checked = verify_segment(directory)
        verified.append({"directory": str(directory), "files": checked,
                         "checksums": artifact(directory / "checksums.sha256")})
        sm = json.loads((directory / "segment.json").read_text())
        if (sm["schema"] != m["schema"] or uint(sm["segment_index"]) != segment_index
                or directory.name != f"{segment_index:06d}"):
            raise ValueError("segment schema/index mismatch")
        ends, counts = {}, [0] * len(cameras)
        for line in (directory / "frames.jsonl").read_text().splitlines():
            f = json.loads(line)
            camera = uint(f["camera"])
            if camera >= len(cameras):
                raise ValueError("unknown camera")
            for field in ("seq", "captured_mono_ns", "offset", "size"):
                f[field] = uint(f[field])
            f["v4l2_sequence"] = uint(f["v4l2_sequence"], 32)
            if f["v4l2_mono_ns"] is not None:
                f["v4l2_mono_ns"] = uint(f["v4l2_mono_ns"])
            semantics = f["v4l2_semantics"]
            if semantics not in ("unavailable", "monotonic_soe", "monotonic_eof"):
                raise ValueError("unknown timestamp semantics")
            if (semantics == "unavailable") != (f["v4l2_mono_ns"] is None):
                raise ValueError("timestamp and semantics disagree")
            expected_payload = f"cam{camera}." + ("mjpeg" if cameras[camera]["raw_encoding"] == "jpeg" else "yuyv")
            if f["payload"] != expected_payload or f["payload"] not in checked:
                raise ValueError("payload does not match camera/encoding")
            payload = local_file(directory, f["payload"])
            if f["size"] == 0 or f["offset"] != ends.get(f["payload"], 0):
                raise ValueError("empty, overlapping, or noncontiguous payload range")
            ends[f["payload"]] = f["offset"] + f["size"]
            if ends[f["payload"]] > payload.stat().st_size:
                raise ValueError("payload range outside file")
            f["payload_path"] = str(payload)
            f["source_segment"] = directory.name
            previous = frames[camera][-1] if frames[camera] else None
            if previous:
                if f["seq"] <= previous["seq"] or f["captured_mono_ns"] <= previous["captured_mono_ns"]:
                    raise ValueError("source sequence/host clock regression; explicit split required")
                delta = (f["v4l2_sequence"] - previous["v4l2_sequence"]) % 2**32
                if delta == 0 or delta >= 2**31:
                    raise ValueError("kernel sequence regression; explicit split required")
            if f["v4l2_mono_ns"] is not None:
                if last_kernel_time[camera] is not None and f["v4l2_mono_ns"] <= last_kernel_time[camera]:
                    raise ValueError("kernel clock regression; explicit split required")
                last_kernel_time[camera] = f["v4l2_mono_ns"]
            frames[camera].append(f)
            counts[camera] += 1
        if counts != sm["frame_counts"]:
            raise ValueError("segment frame count mismatch")
        payload_names = set(checked) - {"segment.json", "frames.jsonl"}
        if payload_names != set(ends) or any(ends[n] != (directory / n).stat().st_size for n in ends):
            raise ValueError("unindexed payload bytes")
    if [len(fs) for fs in frames] != [c["frame_count"] for c in cameras]:
        raise ValueError("session frame count mismatch")
    # Closed sessions are immutable by contract; fail if another writer changed
    # any source while qualification was reading it.
    inputs = [manifest_info]
    for segment in verified:
        inputs += [segment["checksums"], *segment["files"].values()]
    if any(artifact(item["path"]) != item for item in inputs):
        raise ValueError("source changed during qualification")
    return m, frames, {"manifest": manifest_info, "segments": verified}


def group_frames(frames, window_ms):
    if not math.isfinite(window_ms) or not 0 <= window_ms <= 100:
        raise ValueError("invalid grouping window")
    semantics = {f["v4l2_semantics"] for fs in frames for f in fs if f["v4l2_mono_ns"] is not None}
    if not semantics:
        return [], "unavailable"
    if len(semantics) != 1:
        raise ValueError("mixed SOE/EOF semantics")
    usable = [[f for f in fs if f["v4l2_mono_ns"] is not None] for fs in frames]
    indices, groups = [0] * len(frames), []
    while all(i < len(fs) for i, fs in zip(indices, usable)):
        chosen = [fs[i] for i, fs in zip(indices, usable)]
        ts = [f["v4l2_mono_ns"] for f in chosen]
        if max(ts) - min(ts) <= round(window_ms * 1e6):
            groups.append({"seq": chosen[0]["seq"], "capture_max_ns": str(max(ts)),
                           "skew_ms": (max(ts) - min(ts)) / 1e6, "frames": chosen})
            indices = [i + 1 for i in indices]
        else:
            indices[ts.index(min(ts))] += 1
    return groups, semantics.pop()


def summary(m, frames, groups, semantics, window_ms):
    cameras = []
    for camera, fs in zip(m["cameras"], frames):
        known = [f for f in fs if f["v4l2_mono_ns"] is not None]
        # Cadence diagnostic spans missing kernel timestamps; missing count is separate.
        dt = [(b["v4l2_mono_ns"] - a["v4l2_mono_ns"]) / 1e6 for a, b in zip(known, known[1:])]
        host_dt = [(b["captured_mono_ns"] - a["captured_mono_ns"]) / 1e6 for a, b in zip(fs, fs[1:])]
        kernel_gaps = [((b["v4l2_sequence"] - a["v4l2_sequence"]) % 2**32) - 1 for a, b in zip(fs, fs[1:])]
        cameras.append({"camera": camera["index"], "config_slot": camera["config_slot"],
            "frames": len(fs), "timestamp_missing": len(fs) - len(known),
            "source_sequence_gaps": sum(b["seq"] - a["seq"] - 1 for a, b in zip(fs, fs[1:])),
            "kernel_sequence_gaps": sum(kernel_gaps),
            "kernel_interval_p50_ms": percentile(dt, .5), "kernel_interval_p95_ms": percentile(dt, .95),
            "kernel_interval_max_ms": max(dt, default=None), "host_interval_p95_ms": percentile(host_dt, .95),
            "grouped_frames": len(groups), "group_adoption_ratio": len(groups)/len(fs) if fs else None,
            "ungrouped_frames": len(fs) - len(groups)})
    skews = [g["skew_ms"] for g in groups]
    return {"schema": "fitra_corpus_qualification_v1", "session_id": m["session_id"],
        "close_reason": m["close_reason"], "cameras": cameras, "groups": len(groups),
        "grouping": "offline_greedy_head_match_no_reuse_not_live_inference_schedule",
        "window_ms": window_ms, "time_basis": semantics, "skew_p50_ms": percentile(skews, .5),
        "skew_p95_ms": percentile(skews, .95), "skew_max_ms": max(skews, default=None),
        "recorder_drops": m["recorder_drops"], "acceptance": "capture_qualification_only",
        "limitations": ["Kernel SOE/EOF is not exposure-midpoint or hardware synchronization proof.",
            "Groups use all recorded raw frames, not the frames consumed by live inference.",
            "No 3D reconstruction, teacher, pose response lag, or capture-to-display evaluation.",
            "Sequence gaps include unrecorded frames; they do not identify which stage dropped them."]}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("session", type=Path)
    p.add_argument("output", type=Path, help="new directory")
    p.add_argument("--window-ms", type=float, default=15)
    args = p.parse_args()
    if args.output.exists():
        p.error("choose a new output directory")
    m, frames, provenance = load_session(args.session)
    groups, semantics = group_frames(frames, args.window_ms)
    report = summary(m, frames, groups, semantics, args.window_ms)
    report["provenance"] = provenance
    report["tool"] = artifact(__file__)
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output / "report.json", report)
    with (args.output / "groups.jsonl").open("x") as out:
        out.write(json.dumps({"schema": "fitra_corpus_groups_v1", "time_basis": semantics,
                              "qualification": artifact(args.output / "report.json")}) + "\n")
        for group in groups:
            encoded = dict(group, seq=str(group["seq"]), frames=[dict(f) for f in group["frames"]])
            for frame in encoded["frames"]:
                for field in ("seq", "offset", "captured_mono_ns", "v4l2_mono_ns"):
                    if frame[field] is not None:
                        frame[field] = str(frame[field])
            out.write(json.dumps(encoded, allow_nan=False) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "provenance"}, indent=2))


if __name__ == "__main__":
    main()
