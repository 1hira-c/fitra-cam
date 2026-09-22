import copy
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from calibrate_corpus import select_samples
from retriangulate_corpus import merge_geometry, verify_control, validate_derivation
from pseudo_reference import validate_pair


class RescueTests(unittest.TestCase):
    def test_disjoint_sampling_preserves_large_clock(self):
        groups = [{"seq": i, "capture_max_ns": str(2**54+i*1_000_000_000)} for i in range(43)]
        result = select_samples(groups)
        self.assertEqual([r["seq"] for r in result if r["split"] == "train"], list(range(13)))
        self.assertEqual([r["seq"] for r in result if r["split"] == "validation"], list(range(28, 42)))
        self.assertEqual(result[0]["capture_max_ns"], str(2**54))
        self.assertNotIn("split", groups[0])
        for train, valid in (((0, 13), (12, 20)), ((0, 13), (40, 50)), ((-1, 3), (4, 6))):
            with self.assertRaises(ValueError): select_samples(groups, train, valid)

    def fixture(self):
        source = [{"seq": 1, "t_ns": 2**54, "joints": [[1., 2., 3., 1.03]]*26,
                   "valid_joints": 21, "median_reproj_px": 1., "joint_quality": [None]+[
                       {"views": 3, "reproj_px": 1., "ray_angle_deg": 45.}]*25,
                   "observations_2d": [[[1., 2., 1.03]]*26]*3, "ambiguous_detection": False,
                   "detections": [1, 1, 1]}]
        geometry = [{k: copy.deepcopy(source[0][k]) for k in
                     ("seq", "t_ns", "joints", "joint_quality", "valid_joints", "median_reproj_px")}]
        return source, geometry

    def test_merge_preserves_observations_and_does_not_overwrite_source(self):
        source, geometry = self.fixture()
        geometry[0]["joints"][19] = [4., 5., 6., 1.03]
        result = merge_geometry(source, geometry)
        self.assertEqual(result[0]["observations_2d"], source[0]["observations_2d"])
        self.assertEqual(source[0]["joints"][19][0], 1.)
        self.assertEqual(result[0]["joints"][19][0], 4.)
        geometry[0]["t_ns"] += 1
        with self.assertRaises(ValueError): merge_geometry(source, geometry)

    def test_same_calibration_control_fails_on_changed_support_or_positions(self):
        s, g = self.fixture()
        self.assertTrue(verify_control(s, g)["passed"])
        for case in ("position", "joint", "views", "nan", "count"):
            s, g = self.fixture()
            if case == "position": g[0]["joints"][19][0] += .001
            if case == "joint": g[0]["joints"][19] = None
            if case == "views": g[0]["joint_quality"][19]["views"] = 2
            if case == "nan": g[0]["joints"][19][0] = float("nan")
            if case == "count": g[0]["valid_joints"] = 20
            with self.subTest(case=case), self.assertRaises(ValueError): verify_control(s, g)

    def test_ambiguous_or_unexpected_field_rejected(self):
        s, g = self.fixture()
        s[0]["ambiguous_detection"] = True
        with self.assertRaises(ValueError): merge_geometry(s, g)
        s, g = self.fixture()
        g[0]["observations_2d"] = []
        with self.assertRaises(ValueError): merge_geometry(s, g)

    def test_calibration_must_belong_to_original_run_and_have_valid_updates(self):
        sm = {"assets": {"calib": "calib", "capture_manifest": "capture"}}
        sd = {"artifacts": {"plan.jsonl": {"sha256": "plan"}}}
        cm = {"assets": {"source_calib": "calib"}, "source_capture": "capture", "source_plan": {"sha256": "plan"}}
        validate_derivation(sm, sd, cm, {"updated_cameras": 1})
        with self.assertRaises(ValueError): validate_derivation(sm, sd, cm, {"updated_cameras": 0})
        cm["source_plan"]["sha256"] = "another"
        with self.assertRaises(ValueError): validate_derivation(sm, sd, cm, {"updated_cameras": 1})

    def test_teacher_requires_same_calibration_derivation(self):
        assets = {k: {"path": k, "sha256": k} for k in
                  ("calib", "subject_profile", "det_engine", "infer_binary", "capture_manifest", "pose_engine")}
        a = {"assets": assets, "calibration_derivation": {"calibration_complete": "c", "original_calib": "o", "geometry_binary": "g"}}
        b = copy.deepcopy(a)
        b["assets"]["pose_engine"]["sha256"] = "teacher"
        done = {"artifacts": {"plan.jsonl": {"sha256": "p"}}}
        validate_pair(a, done, b, done)
        b["calibration_derivation"]["calibration_complete"] = "other"
        with self.assertRaises(ValueError): validate_pair(a, done, b, done)
        del b["calibration_derivation"]
        with self.assertRaises(ValueError): validate_pair(a, done, b, done)


class GeometryToolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = os.environ.get("FITRA_CORPUS_GEOMETRY")
        if not cls.binary: raise unittest.SkipTest("FITRA_CORPUS_GEOMETRY not set")

    def run_tool(self, mutation=None):
        with tempfile.TemporaryDirectory() as directory:
            d = Path(directory)
            cameras = [{"index": i, "config_slot": i, "capture_width": 640, "capture_height": 480} for i in range(3)]
            header = {"schema": "fitra_corpus_inference_plan_v1", "cameras": cameras}
            group = {"seq": 7, "capture_max_ns": str(2**54+7)}
            plan = json.dumps(header)+"\n"+json.dumps(group)+"\n"
            (d/"plan.jsonl").write_text(plan)
            fnv = 14695981039346656037
            for b in plan.encode(): fnv = ((fnv ^ b)*1099511628211) % 2**64
            rh = {"schema": "fitra_corpus_raw3d_v1", "postprocessing": False, "kp_format": "halpe26",
                  "plan_fnv1a64": str(fnv), "kp_conf_thresh": .3, "max_reproj_px": 6}
            centers = [(-1, 0, 0), (1, 0, 0), (0, -1, 0)]
            points = [[.01*k, .02*(k % 3), 3+.01*k] for k in range(26)]
            observations = [[[300*(x-cx)/z+320, 300*(y-cy)/z+240, 1.03] for x, y, z in points]
                            for cx, cy, _ in centers]
            r = {"seq": 7, "t_ns": 2**54+7, "ambiguous_detection": False,
                 "detections": [1, 1, 1], "observations_2d": observations}
            if mutation == "binding": rh["plan_fnv1a64"] = "wrong"
            if mutation == "clock": r["t_ns"] += 1
            if mutation == "ambiguous": r["ambiguous_detection"] = True
            if mutation == "negative": r["observations_2d"][0][0][2] = -1
            if mutation == "truncated": r["observations_2d"][0].pop()
            if mutation == "stage": rh["postprocessing"] = True
            (d/"raw.jsonl").write_text(json.dumps(rh)+"\n"+json.dumps(r)+"\n")
            def matrix(rows, cols, data):
                return f"!!opencv-matrix {{ rows: {rows}, cols: {cols}, dt: d, data: {json.dumps(data)} }}"
            yaml = "%YAML:1.0\n---\nschema: fitra_calibration_v1\nunit: m\ncoordinate_system: fitra_world_z_up_m\nintrinsics:\n"
            for i in range(3):
                yaml += f"  cam{i}:\n    width: 640\n    height: 480\n    distortion_model: pinhole\n"
                yaml += "    K: "+matrix(3, 3, [300, 0, 320, 0, 300, 240, 0, 0, 1])+"\n"
                yaml += "    dist: "+matrix(1, 4, [0, 0, 0, 0])+"\n"
            yaml += "extrinsics:\n"
            for i, (x, y, z) in enumerate(centers):
                yaml += f"  cam{i}:\n    T_cw: "+matrix(4, 4, [1,0,0,-x,0,1,0,-y,0,0,1,-z,0,0,0,1])+"\n"
            (d/"calib.yaml").write_text(yaml)
            command = [self.binary, "triangulate", str(d/"plan.jsonl"), str(d/"raw.jsonl"), str(d/"calib.yaml"), str(d/"out.jsonl")]
            completed = subprocess.run(command, capture_output=True, text=True)
            if mutation:
                self.assertNotEqual(completed.returncode, 0)
                return
            self.assertEqual(completed.returncode, 0, completed.stderr)
            result = json.loads((d/"out.jsonl").read_text().splitlines()[1])
            self.assertEqual(result["t_ns"], 2**54+7)
            for actual, expected in zip(result["joints"][19][:3], points[19]): self.assertAlmostEqual(actual, expected, places=5)
            self.assertAlmostEqual(result["joints"][19][3], 1.03, places=6)
            self.assertEqual(result["joint_quality"][19]["views"], 3)
            self.assertIsNone(result["joint_quality"][0])
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)

    def test_saved_2d_cpu_roundtrip_and_no_overwrite(self): self.run_tool()

    def test_wrong_source_rejected(self):
        for case in ("binding", "clock", "ambiguous", "negative", "truncated", "stage"):
            with self.subTest(case=case): self.run_tool(case)


if __name__ == "__main__": unittest.main()
