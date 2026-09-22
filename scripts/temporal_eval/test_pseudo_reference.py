import copy
import unittest

from pseudo_reference import smooth_reference, trusted, validate_pair


class PseudoReferenceTests(unittest.TestCase):
    def fixture(self):
        # Large absolute clock and nonuniform intervals must not lose nanoseconds.
        origin = 2**54
        times = [0, 15, 33, 48, 65, 81, 98, 115, 132, 148, 165]
        return [{"seq": i, "t_ns": origin+t*1_000_000, "ambiguous_detection": False,
                 "joints": [[1+t*.001, 2-t*.002, .4, 1.0332] for _ in range(26)],
                 "joint_quality": [None]+[{"views": 3, "reproj_px": 2., "ray_angle_deg": 60.}
                                          for _ in range(25)]} for i, t in enumerate(times)]

    def test_linear_motion_and_score_preserved_without_source_mutation(self):
        rows = self.fixture()
        original = copy.deepcopy(rows)
        ref, quality = smooth_reference(rows)
        self.assertEqual(rows, original)
        self.assertIsNone(ref[0][1])
        self.assertIsNone(ref[-1][1])
        self.assertTrue(all(r[0] is None for r in ref))
        for k in range(4):
            self.assertAlmostEqual(ref[5][1][k], rows[5]["joints"][1][k], places=10)
        self.assertEqual(quality["per_joint"][0]["future_supported"], 0)

    def test_future_is_required_and_really_used(self):
        rows = self.fixture()
        base, _ = smooth_reference(rows)
        for i in (6, 7):
            rows[i]["joints"][1][0] += .1
        changed, _ = smooth_reference(rows)
        self.assertGreater(changed[5][1][0], base[5][1][0])
        truncated, _ = smooth_reference(rows[:7])
        self.assertIsNone(truncated[5][1])

    def test_no_bridging_quality_missingness_sequence_or_time_gaps(self):
        for case in ("quality", "missing", "ambiguous", "seq", "time"):
            rows = self.fixture()
            if case == "quality": rows[6]["joint_quality"][1]["reproj_px"] = 6.01
            if case == "missing": rows[6]["joints"][1] = None
            if case == "ambiguous": rows[6]["ambiguous_detection"] = True
            if case == "seq":
                for row in rows[6:]: row["seq"] += 1
            if case == "time":
                for row in rows[6:]: row["t_ns"] += 100_000_000
            with self.subTest(case=case):
                ref, _ = smooth_reference(rows)
                self.assertIsNone(ref[5][1])

    def test_quality_fail_closed(self):
        joint = [1., 2., 3., 1.03]
        quality = {"views": 2, "reproj_px": 6., "ray_angle_deg": 10.}
        self.assertTrue(trusted(joint, quality, 1))
        self.assertFalse(trusted(joint, quality, 0))
        for key, value in (("views", 1), ("views", True), ("reproj_px", -1),
                           ("reproj_px", float("nan")), ("ray_angle_deg", 9.9)):
            self.assertFalse(trusted(joint, {**quality, key: value}, 1))
        self.assertFalse(trusted([1., 2., 3., .29], quality, 1))

    def test_pair_requires_identical_geometry_and_distinct_teacher(self):
        assets = {k: {"path": k, "sha256": k} for k in
                  ("calib", "subject_profile", "det_engine", "infer_binary", "capture_manifest", "pose_engine")}
        student = {"assets": assets}
        teacher = copy.deepcopy(student)
        teacher["assets"]["pose_engine"]["sha256"] = "teacher"
        done = {"artifacts": {"plan.jsonl": {"sha256": "groups"}}}
        validate_pair(student, done, teacher, done)
        with self.assertRaises(ValueError): validate_pair(student, done, student, done)
        teacher["assets"]["calib"]["sha256"] = "other"
        with self.assertRaises(ValueError): validate_pair(student, done, teacher, done)

    def test_out_of_order_reference_rejected(self):
        rows = self.fixture()
        rows[1]["t_ns"] = rows[0]["t_ns"]
        with self.assertRaises(ValueError): smooth_reference(rows)


if __name__ == "__main__":
    unittest.main()
