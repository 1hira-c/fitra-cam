#!/usr/bin/env python3
"""Read-only capture-integrity/timing QC. No pose or calibration accuracy claim."""
import argparse
import hashlib
import json
import math
import re
import sys
from pathlib import Path


def require(condition, message):
    if not condition:
        raise ValueError(message)


def unique_object(pairs):
    out = {}
    for key, value in pairs:
        require(key not in out, "duplicate JSON key")
        out[key] = value
    return out


def parse(text):
    return json.loads(text, object_pairs_hook=unique_object,
                      parse_constant=lambda _: (_ for _ in ()).throw(ValueError("nonfinite JSON")))


def uint(value):
    require(isinstance(value, str) and re.fullmatch(r"0|[1-9][0-9]*", value), "invalid uint string")
    out = int(value)
    require(out < 2**64, "uint64 overflow")
    return out


def integer(value):
    require(type(value) is int and value >= 0, "expected unsigned JSON integer")
    return value


def lines(path):
    require(path.stat().st_size <= 256 * 1024 * 1024, "sidecar exceeds 256 MiB")
    with path.open("rb") as stream:
        while True:
            line = stream.readline(65537)
            if not line:
                return
            require(len(line) <= 65536, "sidecar line exceeds 64 KiB")
            yield parse(line)


def local_file(root, name):
    require(isinstance(name, str) and re.fullmatch(r"[A-Za-z0-9_.-]+", name)
            and name not in (".", ".."), "invalid bundle filename")
    path = root / name
    require(path.is_file() and not path.is_symlink(), "missing file or symlink in bundle: " + name)
    return path


def digest(path):
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(block)
    return checksum.hexdigest()


def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    index = (len(values) - 1) * q
    lo = int(index)
    hi = min(lo + 1, len(values) - 1)
    return values[lo] + (values[hi] - values[lo]) * (index - lo)


def match_frames(cameras, window_ns):
    """Earliest complete set, advance oldest on miss; one use per frame."""
    positions = [0] * len(cameras)
    while all(positions[i] < len(frames) for i, frames in enumerate(cameras)):
        group = [frames[positions[i]] for i, frames in enumerate(cameras)]
        times = [row[0] for row in group]
        if max(times) - min(times) <= window_ns:
            yield group
            positions = [pos + 1 for pos in positions]
        else:
            oldest = min(range(len(times)), key=times.__getitem__)
            positions[oldest] += 1


def check_bundle(root, sync_window_ms=20.0):
    root = Path(root).resolve(strict=True)
    require(math.isfinite(sync_window_ms) and 0 <= sync_window_ms <= 1000, "invalid sync window")
    manifest = parse(local_file(root, "manifest.json").read_bytes())
    plan = parse(local_file(root, "plan.json").read_bytes())
    require(manifest["schema"] == "fitra_markerless_capture_v1", "unsupported capture schema")
    require(plan["schema"] == "fitra_markerless_capture_plan_v1", "unsupported plan schema")
    camera_specs = manifest["cameras"]
    require(2 <= len(camera_specs) <= 3, "expected 2..3 cameras")
    ids = [camera["id"] for camera in camera_specs]
    require(len(set(ids)) == len(ids), "duplicate camera ids")
    payloads = [camera["payload"] for camera in camera_specs]
    require(len(set(payloads)) == len(payloads), "duplicate payload files")
    expected = {"manifest.json", "plan.json", "frames.jsonl", "cues.jsonl", "intrinsics.yaml", *payloads}
    if manifest["reference_file"] is not None:
        require(manifest["reference_file"] == "reference.yaml", "unexpected reference filename")
        expected.add("reference.yaml")
    entries = {}
    for line in local_file(root, "checksums.sha256").read_text().splitlines():
        match = re.fullmatch(r"([0-9a-f]{64})  ([A-Za-z0-9_.-]+)", line)
        require(match is not None, "invalid checksum entry")
        checksum, name = match.groups()
        require(name not in entries, "duplicate checksum entry")
        entries[name] = checksum
    require(set(entries) == expected, "checksum inventory differs from required bundle files")
    for name, checksum in entries.items():
        require(digest(local_file(root, name)) == checksum, "checksum mismatch: " + name)
    steps = plan["steps"]
    require(len(steps) <= 128 and len({step["id"] for step in steps}) == len(steps), "invalid plan steps")
    cue_events = list(lines(local_file(root, "cues.jsonl")))
    require(len(cue_events) <= len(steps), "too many cue events")
    starts = []
    for index, cue in enumerate(cue_events):
        step = steps[index]
        require(all(cue[a] == step[b] for a, b in
                    (("cue_id", "id"), ("take", "take"), ("role", "role"),
                     ("duration_s", "duration_s"), ("settle_s", "settle_s"))), "cue differs from plan")
        stamp = uint(cue["start_mono_ns"])
        require(stamp > 0 and (not starts or stamp > starts[-1]), "cue timestamp regression")
        starts.append(stamp)
    samples = {name: [] for name in ids}
    host_times = {name: [] for name in ids}
    offsets = dict.fromkeys(ids, 0)
    previous_source = {}
    previous_kernel = {}
    kinds = set()
    unknown = regressions = jpeg_warnings = payload_warnings = 0
    clock_after_dq = 0
    dq_delays = []
    for row in lines(local_file(root, "frames.jsonl")):
        camera = row["camera_id"]
        require(camera in samples, "unknown frame camera")
        spec = camera_specs[ids.index(camera)]
        require(row["payload"] == spec["payload"], "frame points at wrong camera payload")
        source, host = uint(row["source_seq"]), uint(row["host_capture_mono_ns"])
        require(camera not in previous_source or (source > previous_source[camera]
                and host > host_times[camera][-1]), "source sequence/host clock regression")
        previous_source[camera] = source
        host_times[camera].append(host)
        offset, size = uint(row["offset"]), integer(row["size"])
        require(offset == offsets[camera] and size > 0, "payload offset gap/overlap or empty frame")
        offsets[camera] += size
        require(offsets[camera] <= local_file(root, row["payload"]).stat().st_size, "frame outside payload")
        require(type(row["jpeg_markers_ok"]) is bool, "invalid jpeg marker flag")
        require(type(row["payload_shape_ok"]) is bool, "invalid payload shape flag")
        jpeg_warnings += not row["jpeg_markers_ok"]
        payload_warnings += not row["payload_shape_ok"]
        if row["capture_mono_ns"] is None or row["timestamp_semantics"] not in ("monotonic_soe", "monotonic_eof"):
            unknown += 1
            continue
        kernel = uint(row["capture_mono_ns"])
        if kernel > host:
            clock_after_dq += 1
        dq_delays.append((host - kernel) / 1e6)
        if kernel == 0 or (camera in previous_kernel and kernel <= previous_kernel[camera]):
            regressions += 1
            continue
        previous_kernel[camera] = kernel
        kinds.add(row["timestamp_semantics"])
        samples[camera].append((kernel, row))
    for camera, spec in zip(ids, camera_specs):
        require(offsets[camera] == local_file(root, spec["payload"]).stat().st_size, "unindexed payload tail")
        require(len(host_times[camera]) == uint(spec["frames"]), "manifest frame count mismatch")
    timing_available = unknown == 0 and regressions == 0 and clock_after_dq == 0 and len(kinds) == 1
    matched = list(match_frames([samples[name] for name in ids], sync_window_ms * 1e6)) if timing_available else []
    skews = [(max(row[0] for row in group) - min(row[0] for row in group)) / 1e6 for group in matched]
    capacity = min(len(samples[name]) for name in ids)
    candidates = []
    window_stats = []
    for index, cue in enumerate(cue_events):
        if cue["role"] not in ("fit", "validation", "repeat", "floor_scale"):
            continue
        start = starts[index] + (integer(cue["settle_s"]) * 1000000000) + 500000000
        end = (starts[index + 1] if index + 1 < len(starts)
               else starts[index] + integer(cue["duration_s"]) * 1000000000) - 500000000
        groups = [group for group in matched if min(row[0] for row in group) >= start
                  and max(row[0] for row in group) < end]
        window_stats.append({"cue_id": cue["cue_id"], "take": cue["take"], "role": cue["role"],
                             "matched_sets": len(groups), "enough_timing_samples": len(groups) >= 6})
        if len(groups) < 6:
            continue
        for chosen in (len(groups) // 4, 3 * len(groups) // 4):
            group = groups[chosen]
            candidate = {"cue_id": cue["cue_id"], "take": cue["take"], "partition": cue["role"],
                         "pose_stability_verified": False, "frames": []}
            for stamp, row in group:
                candidate["frames"].append({key: row[key] for key in
                    ("camera_id", "source_seq", "payload", "offset", "size", "capture_mono_ns", "timestamp_semantics")})
            candidates.append(candidate)
    planned_windows = sum(step["role"] in ("fit", "validation", "repeat", "floor_scale") for step in steps)
    enough_roles = all(sum(row["role"] == role for row in window_stats) >= minimum
                       for role, minimum in (("fit", 12), ("validation", 8), ("repeat", 12), ("floor_scale", 4)))
    coverage = enough_roles and len(window_stats) == planned_windows and all(row["enough_timing_samples"] for row in window_stats)
    complete = manifest["recording_complete"] is True and len(cue_events) == len(steps)
    last_end = starts[-1] + integer(cue_events[-1]["duration_s"]) * 1000000000 if starts else 0
    delivery_ok = all(len(times) > 1 and starts and times[0] <= starts[0] + 3000000000
        and times[-1] >= last_end - 3000000000
        and max((b - a for a, b in zip(times, times[1:])), default=0) <= 3000000000
        for times in host_times.values())
    report = {"schema": "fitra_markerless_capture_qc_v1", "integrity_ok": True,
        "capture_qc_pass": bool(complete and timing_available and coverage and delivery_ok
                                and jpeg_warnings == 0 and payload_warnings == 0),
        "recording_complete": complete, "timing_available": timing_available,
        "frame_delivery_ok": bool(delivery_ok),
        "calibration_ready": False, "person_visibility_checked": False, "geometry_checked": False,
        "scale_floor_validated": False, "unknown_timestamps": unknown, "timestamp_regressions": regressions,
        "timestamp_semantics": sorted(kinds), "incomplete_jpeg_markers": jpeg_warnings,
        "invalid_payload_shapes": payload_warnings,
        "kernel_timestamp_after_dq": clock_after_dq,
        "capture_to_dq_ms": {"median": percentile(dq_delays, .5), "p95": percentile(dq_delays, .95)},
        "sync_window_ms": sync_window_ms, "matching": "earliest_complete_set_one_use_per_source",
        "matched_sets": len(matched), "potential_sets_min_camera_count": capacity,
        "adoption_fraction": len(matched) / capacity if capacity else 0,
        "missed_potential_sets": capacity - len(matched),
        "skew_ms": {"median": percentile(skews, .5), "p95": percentile(skews, .95), "max": max(skews, default=None)},
        "cameras": [], "windows": window_stats,
        "candidates": {"schema": "fitra_markerless_capture_candidates_v1", "frame_sets": candidates}}
    for camera, spec in zip(ids, camera_specs):
        times = host_times[camera]
        report["cameras"].append({"id": camera, "frames": len(times),
            "measured_fps": (len(times) - 1) * 1e9 / (times[-1] - times[0]) if len(times) > 1 else None,
            "source_gaps": uint(spec["source_gaps"]), "driver_gaps": uint(spec["driver_gaps"]),
            "max_frame_gap_ms": max((b - a for a, b in zip(times, times[1:])), default=0) / 1e6,
            "unmatched_frames": len(samples[camera]) - len(matched)})
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--sync-window-ms", type=float, default=20.0)
    parser.add_argument("--output", type=Path, help="new QC JSON outside the immutable recording bundle")
    args = parser.parse_args()
    try:
        if args.output is not None:
            require(args.session.resolve() not in args.output.resolve().parents,
                    "QC output must be outside the immutable capture bundle")
        report = check_bundle(args.session, args.sync_window_ms)
        encoded = json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2)
        if args.output is not None:
            with args.output.open("x", encoding="utf-8") as output:
                output.write(encoded + "\n")
            summary = {key: value for key, value in report.items() if key not in ("windows", "candidates")}
            summary["insufficient_windows"] = [row["cue_id"] for row in report["windows"]
                                               if not row["enough_timing_samples"]]
            summary["candidate_sets"] = len(report["candidates"]["frame_sets"])
            print(json.dumps(summary, ensure_ascii=False, allow_nan=False, indent=2))
        else:
            print(encoded)
        return 0 if report["capture_qc_pass"] else 2
    except (ValueError, KeyError, TypeError, OSError, OverflowError) as error:
        print("check_markerless_capture: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
