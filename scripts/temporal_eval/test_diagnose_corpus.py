import copy
import json
from pathlib import Path
import tempfile
import unittest

from diagnose_corpus import bind_capture, pose_quality, select_poses, validate_raw
from pseudo_reference import completed_run


class DiagnosticTests(unittest.TestCase):
    def groups(self):
        return [{"seq": 100+i*60, "capture_max_ns": str(2**54+i*1_000_000_000), "frames": []}
                for i in range(10)]

    def poses(self):
        return [{"name": "a", "start_second": 1, "end_second": 3},
                {"name": "b", "start_second": 5, "end_second": 8}]

    def test_selection_preserves_clock_and_source_sequence(self):
        groups = self.groups()
        selected = select_poses(groups, self.poses())
        self.assertEqual([g["seq"] for g in selected], [160, 220, 400, 460, 520])
        self.assertEqual(selected[0]["capture_max_ns"], str(2**54+1_000_000_000))
        self.assertEqual([g["diagnostic_case"] for g in selected], ["a", "a", "b", "b", "b"])
        self.assertNotIn("diagnostic_case", groups[1])

    def test_missing_samples_are_not_replaced_with_distant_frames(self):
        for removed in (1, 2, 5, 6, 7):
            groups = self.groups()
            groups.pop(removed)
            with self.subTest(removed=removed), self.assertRaises(ValueError):
                select_poses(groups, self.poses())

    def test_overlapping_duplicate_or_bad_windows_rejected(self):
        for changes in ({"name": "a"}, {"start_second": 2}, {"end_second": 12},
                        {"start_second": True}, {"end_second": 5}, {"end_second": 200}):
            poses = self.poses()
            poses[1].update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                select_poses(self.groups(), poses)
        with self.assertRaises(ValueError): select_poses([], self.poses())
        with self.assertRaises(ValueError): select_poses(self.groups(), [])

    def test_capture_requires_same_session_calibration_and_bound_floor_map(self):
        session = Path("/tmp/diagnostic-test-session")
        assets = {"calib": {"path": "calib", "sha256": "c"},
                  "floor_map": {"path": "map", "sha256": "f"},
                  "capture_manifest": {"path": "capture", "sha256": "m"}}
        capture = {"assets": {"calib": assets["calib"], "floor": assets["floor_map"]}}
        stopped = {"ready_sessions": [str(session)], "returncode": 0,
                   "capture_manifest": assets["capture_manifest"]}
        bind_capture(session, capture, stopped, assets)
        for mutation in ("session", "exit", "capture", "map_hash", "map_unbound", "calib"):
            a, c, s = (copy.deepcopy(value) for value in (assets, capture, stopped))
            if mutation == "session": s["ready_sessions"] = []
            if mutation == "exit": s["returncode"] = 1
            if mutation == "capture": s["capture_manifest"] = {}
            if mutation == "map_hash": a["floor_map"]["sha256"] = "changed"
            if mutation == "map_unbound": del c["assets"]["floor"]
            if mutation == "calib": a["calib"] = {}
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                bind_capture(session, c, s, a)

    def fixture(self):
        groups = select_poses(self.groups(), self.poses())
        header = {"schema": "fitra_corpus_raw3d_v1", "kp_format": "halpe26", "postprocessing": False}
        rows = [{"seq": g["seq"], "t_ns": int(g["capture_max_ns"]), "ambiguous_detection": False,
                 "joints": [[0, 0, 1, 1.03] for _ in range(26)],
                 "joint_quality": [None]+[{"views": 3, "reproj_px": 2, "ray_angle_deg": 40}
                                          for _ in range(25)]} for g in groups]
        return header, rows, groups

    def test_raw_lineage_and_stage_checked(self):
        header, rows, groups = self.fixture()
        validate_raw(header, rows, groups)
        for mutation in ("time", "seq", "topology", "stage", "count", "ambiguous"):
            h, r, g = copy.deepcopy((header, rows, groups))
            if mutation == "time": r[0]["t_ns"] += 1
            if mutation == "seq": r[0]["seq"] = True
            if mutation == "topology": r[0]["joint_quality"].pop()
            if mutation == "stage": h["postprocessing"] = True
            if mutation == "count": r.pop()
            if mutation == "ambiguous": r[0]["ambiguous_detection"] = True
            with self.subTest(mutation=mutation), self.assertRaises(ValueError): validate_raw(h, r, g)

    def test_quality_keeps_missing_and_ambiguous_in_denominator(self):
        _, rows, groups = self.fixture()
        rows[0]["joints"][11] = None
        rows[0]["joint_quality"][11] = None
        rows[3]["ambiguous_detection"] = True
        rows[3]["joints"] = [None]*26
        rows[3]["joint_quality"] = [None]*26
        result = pose_quality(rows, groups)
        self.assertEqual(result[0]["joints"][11]["gate_passed"], 1)
        self.assertEqual(result[0]["joints"][11]["samples"], 2)
        self.assertEqual(result[1]["joints"][11]["gate_passed"], 2)
        self.assertEqual(result[1]["ambiguous"], 1)
        self.assertEqual(result[0]["joints"][0]["gate_passed"], 0)
        self.assertIsNone(result[0]["joints"][0]["reprojection_p50_px"])

    def test_unobservable_is_null_not_zero_error(self):
        _, rows, groups = self.fixture()
        for r in rows:
            r["joints"][12] = None
            r["joint_quality"][12] = None
        for case in pose_quality(rows, groups):
            self.assertEqual(case["joints"][12]["gate_passed"], 0)
            self.assertIsNone(case["joints"][12]["reprojection_p50_px"])

    def test_sparse_completion_is_not_a_temporal_reference_run(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path/"complete.json").write_text(json.dumps({
                "schema": "fitra_corpus_diagnostic_complete_v1",
                "runtime_applied": False,
                "artifacts": {name: {} for name in
                              ("manifest.json", "pose-plan.jsonl", "report.json", "model-0-raw.jsonl")}}))
            with self.assertRaisesRegex(ValueError, "incomplete source run"):
                completed_run(path)


if __name__ == "__main__":
    unittest.main()
