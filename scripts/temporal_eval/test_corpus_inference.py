import copy
import unittest
from run_corpus import make_input, raw_quality


class CorpusInferenceTests(unittest.TestCase):
    def fixture(self):
        h = {"schema": "fitra_corpus_raw3d_v1", "postprocessing": False, "kp_format": "halpe26"}
        ts = [2**54, 2**54+16_000_000, 2**54+53_000_000]
        groups = [{"seq": s, "capture_max_ns": str(t), "skew_ms": 12.5} for s,t in zip([10,11,13],ts)]
        rows = [{"seq": g["seq"], "t_ns": int(g["capture_max_ns"]), "ambiguous_detection": False,
                 "joints": [[0,0,1,1.0332]]*26, "valid_joints": 25, "joint_quality": [None]+[
                     {"views": 3, "reproj_px": 2, "ray_angle_deg": 35}]*25} for g in groups]
        return h, rows, groups

    def test_capture_clock_and_source_holes_are_preserved(self):
        h,r,g = self.fixture()
        ih, rows = make_input(h,r,g,1.7,60,"session",{})
        self.assertEqual(ih["time_basis"], "measured_capture_group")
        self.assertEqual(ih["capture_origin_ns"], str(2**54))
        self.assertEqual([s["t_ns"] for s in rows], [0,16_000_000,53_000_000])
        self.assertEqual([s["seq"] for s in rows], [10,11,13])
        self.assertEqual(rows[0]["joints"][0][3], 1.0332)
        self.assertTrue(all(s["phase"] == "unlabelled" and s["reference_joints"] is None for s in rows))

    def test_ambiguous_or_empty_is_boundary(self):
        h,r,g = self.fixture()
        r[1]["joints"] = [None]*26
        r[1]["ambiguous_detection"] = True
        _, rows = make_input(h,r,g,1.7,60,"session",{})
        self.assertEqual(rows[1]["state"], "boundary")
        self.assertIsNone(rows[1]["joints"])

    def test_bad_lineage_stage_or_joint_rejected(self):
        for mutation in ("time", "sequence", "ambiguous", "negative", "boolean", "nan", "stage", "count"):
            h,r,g = self.fixture()
            if mutation=="time": r[0]["t_ns"]+=1
            if mutation=="sequence": r[0]["seq"]+=1
            if mutation=="ambiguous": r[0]["ambiguous_detection"]=True
            if mutation=="negative": r[0]["joints"][0][3]=-1
            if mutation=="boolean": r[0]["joints"][0][3]=True
            if mutation=="nan": r[0]["joints"][0][0]=float("nan")
            if mutation=="stage": h["postprocessing"]=True
            if mutation=="count": r.pop()
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                make_input(h,r,g,1.7,60,"session",{})

    def test_quality_excludes_synthetic_nose_endpoint(self):
        _,r,_ = self.fixture()
        report = raw_quality(r)
        self.assertIsNone(report["per_joint"][0]["reprojection_p95_px"])
        self.assertEqual(report["per_joint"][1]["inlier_view_counts"], {3: 3})


if __name__ == "__main__":
    unittest.main()
