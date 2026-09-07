import copy
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from evaluate import angle_deg, estimate_lag, score, read_jsonl, verify_input_binding
from synthetic import dataset
from import_dump import convert


def track(x):
    return {"position_m":[x,0,1], "quat_wxyz":[1,0,0,0]}


class MetricsTests(unittest.TestCase):
    def series(self, delay=0, scale=1):
        ts = [i/60 for i in range(240)]
        def signal(t): return math.sin(4*t)+.3*math.sin(9*t)
        return ts, [[signal(t),0,0] for t in ts], [[scale*signal(t-delay),0,0] for t in ts]

    def test_known_delay_keeps_sign(self):
        t, r, y = self.series(delay=9/60)
        result = estimate_lag(t,r,y,1/60)
        self.assertEqual(result["lag_steps"], 9)
        self.assertAlmostEqual(result["lag_ms"],150)

    def test_prediction_negative_lag(self):
        t,r,y = self.series(delay=-2/60)
        self.assertEqual(estimate_lag(t,r,y,1/60)["lag_steps"],-2)

    def test_frozen_output_has_no_valid_lag(self):
        t,r,_ = self.series()
        self.assertIsNone(estimate_lag(t,r,[[0,0,0]]*len(t),1/60)["lag_steps"])

    def test_constant_velocity_is_ambiguous(self):
        t,r,_ = self.series()
        r = [[v,0,0] for v in t]
        self.assertEqual(estimate_lag(t,r,r,1/60)["reason"], "ambiguous_motion_match")

    def test_quaternion_double_cover(self):
        self.assertAlmostEqual(angle_deg([1,0,0,0],[-1,0,0,0]),0)
        with self.assertRaises(ValueError): angle_deg([0,0,0,0],[1,0,0,0])

    def sample(self, delayed=False, missing=False):
        h, _ = dataset()
        t,r,y = self.series(delay=.15 if delayed else 0,scale=.5)
        source = [{"seq":i,"t_ns":round(v*1e9),"segment":"motion","state":"fresh","phase":"motion"} for i,v in enumerate(t)]
        output = [{"seq":s["seq"],"t_ns":s["t_ns"],"reset":i==0,
                   "reference":[track(r[i][0])]*10,
                   "candidates":{"test":[None if missing else track(y[i][0])]*10}} for i,s in enumerate(source)]
        oh = {"schema":"fitra_temporal_output_v1","candidates":["test"],"nominal_hz":60}
        return h,source,oh,output

    def test_shrunk_motion_not_rewarded_as_accuracy(self):
        result=score(*self.sample(delayed=True))
        s=result["summary"]["test"]
        self.assertGreater(s["median_same_time_rmse_mm"],100)
        self.assertLess(s["median_motion_amplitude_ratio"],.7)
        self.assertEqual(s["median_position_lag_ms"],150)

    def test_missing_output_not_zero_error(self):
        s=score(*self.sample(missing=True))["summary"]["test"]
        self.assertEqual(s["reference_coverage"],0)
        self.assertIsNone(s["median_same_time_rmse_mm"])

    def test_misaligned_output_rejected(self):
        args=self.sample()
        args[3][2]["t_ns"]+=1
        with self.assertRaises(ValueError): score(*args)

    def test_phase_windows_do_not_merge(self):
        args=self.sample()
        for i,s in enumerate(args[1]): s["phase"]="motion" if 60 <= i < 120 else "static"
        result=score(*args)
        self.assertEqual(len({d["run"] for d in result["details"]}),3)

    def test_shared_warmup_exclusion(self):
        args=self.sample()
        for s in args[1][:60]: s["evaluate"]=False
        report=score(*args)
        self.assertEqual(report["unscored_reports"],60)
        self.assertTrue(all(d["reports"]==180 for d in report["details"]))

    def test_nominal_adapter_preserves_frame_gaps(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/"raw.jsonl"
            p.write_text("\n".join(json.dumps({"frame":i,"kp_format":"halpe26",
                "persons_3d":[{"joints":[[0,0,1,.9,True]]*26}]}) for i in (0,2)))
            h,rows=convert(p,60,1.7,"clip","static")
            self.assertEqual(h["time_basis"],"nominal_from_legacy_frames")
            self.assertEqual(rows[1]["t_ns"],33333333)
            self.assertIsNone(rows[0]["reference_joints"])


@unittest.skipUnless(os.environ.get("FITRA_TEMPORAL_REPLAY"), "set FITRA_TEMPORAL_REPLAY to test C++ replay")
class ReplayTests(unittest.TestCase):
    def run_replay(self, rows, header=None, expect_ok=True):
        h,_=dataset()
        with tempfile.TemporaryDirectory() as d:
            source,out=Path(d)/"input.jsonl",Path(d)/"out.jsonl"
            source.write_text("\n".join(json.dumps(r) for r in [header or h,*rows])+"\n")
            p=subprocess.run([os.environ["FITRA_TEMPORAL_REPLAY"],str(source),str(out)],capture_output=True,text=True)
            if not expect_ok:
                self.assertNotEqual(p.returncode,0)
                return
            self.assertEqual(p.returncode,0,p.stderr)
            header, data = read_jsonl(out)
            verify_input_binding(source, header)
            with source.open("a") as f: f.write("\n")
            with self.assertRaises(ValueError): verify_input_binding(source, header)
            # Numerical causality/determinism excludes wall-clock profiling noise.
            for row in data: row.pop("processing_ms", None)
            return header,data

    def test_determinism_prefix_causality_and_teacher_isolation(self):
        _,rows=dataset()
        rows=rows[:90]
        _,a=self.run_replay(rows)
        _,b=self.run_replay(rows[:40])
        self.assertEqual(a[:40],b)
        altered=copy.deepcopy(rows)
        for r in altered: r["reference_joints"]=None
        _,c=self.run_replay(altered)
        self.assertEqual([r["candidates"] for r in a],[r["candidates"] for r in c])
        self.assertEqual(a,self.run_replay(rows)[1])

    def test_boundary_and_gap_reset(self):
        _,rows=dataset()
        rows=rows[:60]
        rows[20]["state"]="boundary"
        rows[20]["joints"]=None
        rows[20]["reference_joints"]=None
        _,out=self.run_replay(rows)
        self.assertTrue(all(t is None for t in out[20]["candidates"]["current"]))
        self.assertTrue(out[21]["reset"])
        isolated=copy.deepcopy(rows[21:])
        _,fresh=self.run_replay(isolated)
        self.assertEqual(out[21]["candidates"],fresh[0]["candidates"])
        for r in rows[30:]: r["t_ns"]+=1_000_000_000
        _,out=self.run_replay(rows)
        self.assertTrue(out[30]["reset"])

    def test_duplicate_nonfinite_and_wrong_stage_rejected(self):
        h,rows=dataset()
        self.run_replay([rows[0],rows[0]],expect_ok=False)
        rows[0]["joints"][19][0]=float("nan")
        self.run_replay(rows[:1],expect_ok=False)
        h["input_stage"]="post_kalman"
        self.run_replay(rows[1:2],header=h,expect_ok=False)

    def test_synthetic_three_candidates_score_without_nan(self):
        h,rows=dataset()
        oh,out=self.run_replay(rows)
        report=score(h,rows,oh,out)
        json.dumps(report,allow_nan=False)
        self.assertEqual(set(report["summary"]),{"minimal","kalman","current"})
        self.assertEqual(report["reference_kind"],"synthetic_known")
        self.assertEqual(report["acceptance"],"not_calibrated_against_human_AB")


if __name__ == "__main__": unittest.main()
