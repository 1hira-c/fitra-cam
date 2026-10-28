#!/usr/bin/env python3
"""Synthetic bundle tests invoked by test_markerless_recording, never cameras."""
import importlib.util
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

module_spec = importlib.util.spec_from_file_location("capture_qc", Path(__file__).with_name("check_markerless_capture.py"))
qc = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(qc)
SOURCE = Path(sys.argv[1])
RECORDER = sys.argv[2]


def refresh(root):
    checksums = root / "checksums.sha256"
    names = [line.split("  ")[1] for line in checksums.read_text().splitlines()]
    checksums.write_text("".join(qc.digest(root / name) + "  " + name + "\n" for name in names))


class CaptureQC(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="markerless-qc-")
        self.root = Path(self.temp.name) / "bundle"
        shutil.copytree(SOURCE, self.root)

    def tearDown(self):
        self.temp.cleanup()

    def rewrite(self, name, transform):
        path = self.root / name
        if name.endswith(".jsonl"):
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            transform(rows)
            path.write_text("".join(json.dumps(row) + "\n" for row in rows))
        else:
            row = json.loads(path.read_text())
            transform(row)
            path.write_text(json.dumps(row))
        refresh(self.root)

    def test_complete_and_partitioned(self):
        report = qc.check_bundle(self.root)
        self.assertTrue(report["capture_qc_pass"])
        self.assertFalse(report["calibration_ready"])
        self.assertFalse(report["person_visibility_checked"])
        self.assertEqual(report["unknown_timestamps"], 0)
        self.assertEqual(report["skew_ms"]["max"], 5)
        self.assertEqual(report["adoption_fraction"], 1)
        sets = report["candidates"]["frame_sets"]
        self.assertEqual(len(sets), 72)
        self.assertEqual(sum(row["partition"] == "fit" for row in sets), 24)
        self.assertEqual(sum(row["partition"] == "validation" for row in sets), 16)
        self.assertEqual(sum(row["partition"] == "repeat" for row in sets), 24)
        self.assertEqual(sum(row["partition"] == "floor_scale" for row in sets), 8)
        train = {row["frames"][0]["source_seq"] for row in sets if row["partition"] == "fit"}
        held = {row["frames"][0]["source_seq"] for row in sets if row["partition"] == "validation"}
        self.assertTrue(train.isdisjoint(held))
        self.assertGreater(int(sets[0]["frames"][0]["capture_mono_ns"]), 2**53)

    def test_corruption_is_detected(self):
        with (self.root / "cam0.mjpeg").open("ab") as out:
            out.write(b"damage")
        with self.assertRaises(ValueError):
            qc.check_bundle(self.root)

    def test_missing_index_tail(self):
        self.rewrite("frames.jsonl", lambda rows: rows.pop())
        with self.assertRaises(ValueError):
            qc.check_bundle(self.root)

    def test_unknown_clock_is_not_host_fallback(self):
        def alter(rows):
            for row in rows:
                if row["camera_id"] == "cam0":
                    row["capture_mono_ns"] = None
                    row["timestamp_semantics"] = "unavailable"
        self.rewrite("frames.jsonl", alter)
        report = qc.check_bundle(self.root)
        self.assertFalse(report["capture_qc_pass"])
        self.assertFalse(report["timing_available"])
        self.assertEqual(report["matched_sets"], 0)

    def test_mixed_semantics_are_not_accepted(self):
        def alter(rows):
            for row in rows:
                if row["camera_id"] == "cam0":
                    row["timestamp_semantics"] = "monotonic_eof"
        self.rewrite("frames.jsonl", alter)
        self.assertFalse(qc.check_bundle(self.root)["capture_qc_pass"])

    def test_camera_skew_misses_are_reported(self):
        def alter(rows):
            for row in rows:
                if row["camera_id"] == "cam0":
                    row["capture_mono_ns"] = str(int(row["capture_mono_ns"]) - 60000000)
        self.rewrite("frames.jsonl", alter)
        report = qc.check_bundle(self.root)
        self.assertTrue(report["timing_available"])
        self.assertFalse(report["capture_qc_pass"])
        self.assertEqual(report["matched_sets"], 0)
        self.assertGreater(report["missed_potential_sets"], 0)

    def test_shortened_take_does_not_pass(self):
        self.rewrite("manifest.json", lambda row: row.update(recording_complete=False))
        self.assertFalse(qc.check_bundle(self.root)["capture_qc_pass"])

    def test_outside_payload_is_rejected(self):
        self.rewrite("frames.jsonl", lambda rows: rows[0].update(offset="999999999999"))
        with self.assertRaises(ValueError):
            qc.check_bundle(self.root)

    def test_unsafe_path_is_rejected(self):
        self.rewrite("manifest.json", lambda row: row["cameras"][0].update(payload="../outside"))
        with self.assertRaises(ValueError):
            qc.check_bundle(self.root)

    def test_duplicate_json_key_is_rejected(self):
        with self.assertRaises(ValueError):
            qc.parse('{"camera_id":"cam0","camera_id":"cam1"}')
        with self.assertRaises(ValueError):
            qc.parse('{"size":NaN}')

    def test_no_record_is_a_safe_plan(self):
        result = subprocess.run([RECORDER, "--plan-json"], capture_output=True, text=True, check=True)
        plan = json.loads(result.stdout)
        self.assertEqual(plan["duration_s"], 370)
        self.assertEqual(len(plan["steps"]), 44)
        self.assertTrue(plan["duration_is_provisional"])
        # No output creation, even if --out names a path, without --record.
        output = Path(self.temp.name) / "must-not-exist"
        result = subprocess.run([RECORDER, "--out", str(output)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        self.assertFalse(output.exists())
        result = subprocess.run([RECORDER, "--max-gib", "nan"], capture_output=True)
        self.assertNotEqual(result.returncode, 0)

    def test_camera_only_preflight_ignores_unrelated_outputs(self):
        config = Path(self.temp.name) / "cameras.yaml"
        config.write_text("schema: fitra_main_config_v1\ncameras:\n"
            "  cam0: /dev/synthetic-camera-0\n  cam1: /dev/synthetic-camera-1\n"
            "  width: 640\n  height: 480\n  fps: 60\n"
            "slimevr:\n  slimevr_out: true\ninference:\n  pose_engine: not-opened.engine\n")
        output = Path(self.temp.name) / "no-recording"
        command = [RECORDER, "--config", str(config), "--intrinsics", str(self.root / "intrinsics.yaml"),
                   "--out", str(output)]
        result = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(output.exists())
        config.write_text(config.read_text().replace("  fps: 60", "  wrong_camera_key: 60"))
        self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
