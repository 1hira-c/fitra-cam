"""Contracts for diagnostic HTTP-snapshot import; no cameras or profile needed."""
import copy
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class SnapshotImportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = os.environ.get("TEMPORAL_SNAPSHOT_IMPORT_BINARY")
        if not cls.binary:
            raise unittest.SkipTest("TEMPORAL_SNAPSHOT_IMPORT_BINARY not set")

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.input = self.directory / "snapshot.jsonl"
        self.calibration = self.directory / "calib.yaml"
        self.output = self.directory / "input.jsonl"
        self.centers = [(-1., 0., 0.), (1., 0., 0.), (0., -1., 0.)]
        self.points = [[.01*k, .02*(k % 3), 3.+.01*k] for k in range(26)]

        def matrix(rows, columns, data):
            return f"!!opencv-matrix {{ rows: {rows}, cols: {columns}, dt: d, data: {json.dumps(data)} }}"

        yaml = "%YAML:1.0\n---\nschema: fitra_calibration_v1\nunit: m\ncoordinate_system: fitra_world_z_up_m\nintrinsics:\n"
        for c in range(3):
            yaml += f"  cam{c}:\n    width: 640\n    height: 480\n    distortion_model: pinhole\n"
            yaml += "    K: " + matrix(3, 3, [300, 0, 320, 0, 300, 240, 0, 0, 1]) + "\n"
            yaml += "    dist: " + matrix(1, 4, [0, 0, 0, 0]) + "\n"
        yaml += "extrinsics:\n"
        for c, (x, y, z) in enumerate(self.centers):
            yaml += f"  cam{c}:\n    T_cw: " + matrix(4, 4, [1, 0, 0, -x, 0, 1, 0, -y, 0, 0, 1, -z, 0, 0, 0, 1]) + "\n"
        self.calibration.write_text(yaml)

    def snapshot(self, index=0):
        observed = 2**54 + 7 + index*32_000_000
        cameras = []
        for c, (cx, cy, _) in enumerate(self.centers):
            points = [[300*(x-cx)/z+320, 300*(y-cy)/z+240, 1.03] for x, y, z in self.points]
            cameras.append({"id": c, "w": 640, "h": 480,
                            "stats": {"processed": 100+index, "captured_at_ms": 1_000_000+index*32+c*3},
                            "persons": [{"kpts": points}]})
        return {"observed_mono_ns": observed, "read_done_mono_ns": observed+2_000_000,
                "snapshot": {"seq": 10+index, "ts_ms": 1_000_030+index*32,
                             "kp_format": "halpe26", "cameras": cameras}}

    def run_import(self, rows, options=(), success=True, output=None):
        self.input.write_text("".join(json.dumps(row)+"\n" for row in rows))
        before = self.input.read_bytes(), self.calibration.read_bytes()
        target = output or self.output
        result = subprocess.run([self.binary, str(self.input), str(self.calibration), str(target), *options],
                                capture_output=True, text=True)
        self.assertEqual(before, (self.input.read_bytes(), self.calibration.read_bytes()))
        if not success:
            self.assertNotEqual(result.returncode, 0, result.stderr)
            return result
        self.assertEqual(result.returncode, 0, result.stderr)
        values = [json.loads(line) for line in target.read_text().splitlines()]
        return values[0], values[1:]

    def test_production_geometry_report_time_and_profile_independence(self):
        a, b = self.snapshot(), self.snapshot(1)
        b["observed_mono_ns"] = a["observed_mono_ns"]+1
        b["read_done_mono_ns"] = b["observed_mono_ns"]+2_000_000
        header, rows = self.run_import([a, b])
        self.assertEqual(header["time_basis"], "measured_report")
        self.assertEqual(header["reference_kind"], "none")
        self.assertEqual([row["t_ns"] for row in rows], [0, 1])
        self.assertEqual([row["seq"] for row in rows], [0, 1])
        self.assertEqual(rows[0]["source_observed_mono_ns"], str(2**54+7))
        self.assertEqual(rows[0]["camera_processed"], [100, 100, 100])
        self.assertEqual(rows[0]["camera_capture_wall_ms"], [1_000_000, 1_000_003, 1_000_006])
        self.assertEqual(rows[0]["source_skew_ms"], 6)
        self.assertIsNone(rows[0]["reference_joints"])
        self.assertIsNone(rows[0]["joint_quality"][0])
        for actual, expected in zip(rows[0]["joints"][19][:3], self.points[19]):
            self.assertAlmostEqual(actual, expected, places=5)
        self.assertEqual(rows[0]["joint_quality"][19]["views"], 3)
        self.assertAlmostEqual(rows[0]["joints"][19][3], 1.03, places=6)
        other_header, other_rows = self.run_import([a, b], ("--height", "1.95"), output=self.directory / "other.jsonl")
        self.assertEqual(other_header["subject_height_m"], 1.95)
        self.assertEqual(other_rows, rows)
        self.assertEqual(header["provenance"]["observation_kind"], "approximate_asynchronous_http_snapshot")
        self.assertFalse(header["provenance"]["original_live_3d_reproduced"])
        self.assertEqual(header["provenance"]["camera_sync"], "not_kernel_or_hardware_synchronized")

    def test_camera_identity_maps_reordered_display_array(self):
        row = self.snapshot()
        row["snapshot"]["cameras"].reverse()
        _, rows = self.run_import([row])
        for actual, expected in zip(rows[0]["joints"][19][:3], self.points[19]):
            self.assertAlmostEqual(actual, expected, places=5)

    def test_source_dedup_ignores_http_sequence_but_preserves_partial_updates(self):
        a, duplicate, partial = self.snapshot(), self.snapshot(1), self.snapshot(2)
        duplicate["snapshot"]["cameras"] = copy.deepcopy(a["snapshot"]["cameras"])
        partial["snapshot"]["cameras"][1:] = copy.deepcopy(a["snapshot"]["cameras"][1:])
        # Permit this asynchronous source tuple explicitly; this is not a
        # reconstruction of the original production sync matcher.
        partial["snapshot"]["cameras"][0]["stats"]["captured_at_ms"] = 1_000_010
        _, rows = self.run_import([a, duplicate, partial])
        self.assertEqual([row["source_snapshot_seq"] for row in rows], [10, 12])
        self.assertEqual([row["t_ns"] for row in rows], [0, 64_000_000])
        self.assertEqual(rows[1]["camera_processed"], [102, 100, 100])

    def test_presence_ambiguity_skew_and_missing_timestamp_are_boundaries(self):
        cases = ("no_person", "ambiguous_detection", "missing_camera", "excessive_source_skew", "missing_capture_timestamp")
        for reason in cases:
            with self.subTest(reason=reason):
                row = self.snapshot()
                cameras = row["snapshot"]["cameras"]
                if reason == "no_person": cameras[0]["persons"] = []
                elif reason == "ambiguous_detection": cameras[0]["persons"] *= 2
                elif reason == "missing_camera": cameras.pop()
                elif reason == "excessive_source_skew": cameras[2]["stats"]["captured_at_ms"] += 100
                else: cameras[0]["stats"].pop("captured_at_ms")
                _, rows = self.run_import([row], output=self.directory / (reason+".jsonl"))
                self.assertEqual(rows[0]["state"], "boundary")
                self.assertEqual(rows[0]["boundary_reason"], reason)
                self.assertIsNone(rows[0]["joints"])
                self.assertIsNone(rows[0]["joint_quality"])
                self.assertEqual(rows[0]["valid_joints"], 0)

    def test_boundary_with_unchanged_counters_is_never_suppressed(self):
        for reason in ("no_person", "ambiguous_detection", "excessive_source_skew", "no_triangulated_body"):
            with self.subTest(reason=reason):
                a, boundary, recovered = self.snapshot(), self.snapshot(1), self.snapshot(2)
                boundary["snapshot"]["cameras"] = copy.deepcopy(a["snapshot"]["cameras"])
                cameras = boundary["snapshot"]["cameras"]
                if reason == "no_person": cameras[0]["persons"] = []
                elif reason == "ambiguous_detection": cameras[0]["persons"] *= 2
                elif reason == "excessive_source_skew": cameras[2]["stats"]["captured_at_ms"] += 100
                else:
                    for camera in cameras:
                        for point in camera["persons"][0]["kpts"]:
                            point[2] = 0.
                _, rows = self.run_import([a, boundary, recovered], output=self.directory / ("same-counter-"+reason+".jsonl"))
                self.assertEqual([row["state"] for row in rows], ["fresh", "boundary", "fresh"])
                self.assertEqual(rows[1]["camera_processed"], rows[0]["camera_processed"])
                self.assertEqual(rows[1]["boundary_reason"], reason)
                self.assertIsNone(rows[1]["joints"])

    def test_skew_threshold_is_inclusive_and_options_are_recorded(self):
        row = self.snapshot()
        row["snapshot"]["cameras"][2]["stats"]["captured_at_ms"] = 1_000_015
        header, rows = self.run_import([row], ("--max-skew-ms", "15", "--nominal-hz", "29"))
        self.assertEqual(rows[0]["state"], "fresh")
        self.assertEqual(header["provenance"]["options"], {"max_skew_ms": 15, "omit_camera": None, "height_m": 1.7, "nominal_hz": 29})
        _, rows = self.run_import([row], ("--max-skew-ms", "14"), output=self.directory / "tight.jsonl")
        self.assertEqual(rows[0]["boundary_reason"], "excessive_source_skew")

    def test_omit_is_counterfactual_and_preserves_original_source_metadata(self):
        row = self.snapshot()
        for point in row["snapshot"]["cameras"][0]["persons"][0]["kpts"]:
            point[0] += 40
        header, rows = self.run_import([row], ("--omit-camera", "0"))
        self.assertTrue(header["provenance"]["counterfactual"])
        self.assertEqual(header["provenance"]["options"]["omit_camera"], 0)
        self.assertEqual(rows[0]["camera_processed"], [100, 100, 100])
        self.assertEqual(rows[0]["camera_capture_wall_ms"], [1_000_000, 1_000_003, 1_000_006])
        self.assertEqual(rows[0]["source_skew_ms"], 6)
        self.assertEqual(rows[0]["joint_quality"][19]["views"], 2)
        for actual, expected in zip(rows[0]["joints"][19][:3], self.points[19]):
            self.assertAlmostEqual(actual, expected, places=5)
        row["snapshot"]["cameras"][0]["stats"]["captured_at_ms"] -= 100
        _, rows = self.run_import([row], ("--omit-camera", "0"), output=self.directory / "omitted-skew.jsonl")
        self.assertEqual(rows[0]["boundary_reason"], "excessive_source_skew")

    def test_source_counter_regression_resets_the_report_stream(self):
        a, b, c = [self.snapshot(i) for i in range(3)]
        b["snapshot"]["cameras"][0]["stats"]["processed"] = 1
        c["snapshot"]["cameras"][0]["stats"]["processed"] = 2
        _, rows = self.run_import([a, b, c])
        self.assertEqual([r["state"] for r in rows], ["fresh", "boundary", "fresh"])
        self.assertEqual(rows[1]["boundary_reason"], "source_counter_regression")

    def test_malformed_geometry_topology_and_clocks_are_rejected_without_output(self):
        for case in ("id", "duplicate_id", "resolution", "topology", "truncated", "negative_score", "clock", "read_done"):
            with self.subTest(case=case):
                row = self.snapshot()
                cameras = row["snapshot"]["cameras"]
                if case == "id": cameras[0]["id"] = 5
                elif case == "duplicate_id": cameras[0]["id"] = 1
                elif case == "resolution": cameras[0]["w"] += 1
                elif case == "topology": row["snapshot"]["kp_format"] = "coco17"
                elif case == "truncated": cameras[0]["persons"][0]["kpts"].pop()
                elif case == "negative_score": cameras[0]["persons"][0]["kpts"][13][2] = -.1
                elif case == "read_done": row["read_done_mono_ns"] = row["observed_mono_ns"]-1
                source = [row, copy.deepcopy(row)] if case == "clock" else [row]
                path = self.directory / (case+".jsonl")
                self.run_import(source, success=False, output=path)
                self.assertFalse(path.exists())

    def test_existing_output_is_preserved(self):
        self.output.write_bytes(b"user-owned output\n")
        self.run_import([self.snapshot()], success=False)
        self.assertEqual(self.output.read_bytes(), b"user-owned output\n")


if __name__ == "__main__":
    unittest.main()
