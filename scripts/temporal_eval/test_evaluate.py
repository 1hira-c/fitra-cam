import copy
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from evaluate import (angle_deg, estimate_lag, score, read_jsonl, verify_input_binding,
                      swing_twist_deg, forward_vector, rotation_metrics)
from synthetic import dataset
from import_dump import convert


def track(x):
    return {"position_m":[x,0,1], "quat_wxyz":[1,0,0,0]}


def rotation(axis, degrees):
    half = math.radians(degrees)/2
    q = [math.cos(half),0,0,0]
    q[axis+1] = math.sin(half)
    return q


def rotated_track(q):
    return {"position_m":[0,0,1], "quat_wxyz":q, "roll_confidence":1, "swing_confidence":1}


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

    def test_source_boundary_cannot_be_hidden_by_output_reset(self):
        args=self.sample()
        args[1][80]["state"]="boundary"
        args[3][80]["reference"]=[None]*10
        args[3][80]["candidates"]["test"]=[None]*10
        with self.assertRaisesRegex(ValueError,"reset"): score(*args)

    def test_source_time_and_reference_contract(self):
        args=self.sample()
        args[1][0]["t_ns"]=False
        with self.assertRaises(ValueError): score(*args)
        args=self.sample()
        args[0]["reference_kind"]="none"
        with self.assertRaises(ValueError): score(*args)


class RotationTests(unittest.TestCase):
    def test_twist_axis_is_reference_local_not_world_z(self):
        # q_out = Rx(90deg) * Rz(60deg), with reference Rx(90deg).
        c = math.sqrt(.5)
        out = [c*math.cos(math.pi/6), c*math.cos(math.pi/6),
               -c*math.sin(math.pi/6), c*math.sin(math.pi/6)]
        swing,twist=swing_twist_deg(rotation(0,90),out)
        self.assertAlmostEqual(swing,0)
        self.assertAlmostEqual(twist,60)

    def test_unobservable_reference_does_not_invent_twist_ground_truth(self):
        for confidence in (None,0):
            ref=rotated_track([1,0,0,0])
            if confidence is None:
                del ref["roll_confidence"]
            else:
                ref["roll_confidence"]=confidence
            out=rotated_track(rotation(2,100))
            m=rotation_metrics([0],[0],[ref],[out],1/60,45)
            self.assertIsNone(m["same_time_twist_p95_deg"])
            self.assertEqual(m["twist_unobservable_reference_pairs"],1)
            self.assertAlmostEqual(m["same_time_swing_p95_deg"],0)

    def test_swing_and_twist_are_separate_and_sign_invariant(self):
        identity = [1,0,0,0]
        swing,twist = swing_twist_deg(identity,rotation(0,60))
        self.assertAlmostEqual(swing,60)
        self.assertAlmostEqual(twist,0)
        swing,twist = swing_twist_deg(identity,rotation(2,80))
        self.assertAlmostEqual(swing,0)
        self.assertAlmostEqual(twist,80)
        for a,b in zip(swing_twist_deg(identity,rotation(2,80)),
                       swing_twist_deg(identity,[-x for x in rotation(2,80)])):
            self.assertAlmostEqual(a,b,places=9)

    def test_antiparallel_swing_is_not_zero_twist_error(self):
        swing,twist=swing_twist_deg([1,0,0,0],rotation(0,180))
        self.assertAlmostEqual(swing,180)
        self.assertIsNone(twist)
        result=rotation_metrics([0],[0],[rotated_track([1,0,0,0])],
                                [rotated_track(rotation(0,180))],1/60,45)
        self.assertIsNone(result["same_time_twist_p95_deg"])
        self.assertEqual(result["twist_undefined_pairs"],1)

    def test_wraparound_is_a_small_step_not_full_turn(self):
        qs=[rotated_track(rotation(2,179)),rotated_track(rotation(2,-179))]
        result=rotation_metrics([0,1/60],[0,1],qs,qs,1/60,45)
        self.assertAlmostEqual(result["twist_step_max_deg"],2)
        self.assertEqual(result["twist_jump_count"],0)

    def test_thigh_only_noise_is_not_hidden_by_stable_feet(self):
        h,source,oh,outputs=MetricsTests().sample()
        for i,out in enumerate(outputs):
            out["reference"]=[rotated_track([1,0,0,0]) for _ in range(10)]
            out["candidates"]["test"]=[rotated_track([1,0,0,0]) for _ in range(10)]
            out["candidates"]["test"][4]=rotated_track(rotation(2,80 if i%2 else -80))
        report=score(h,source,oh,outputs)
        thigh=next(d for d in report["details"] if d["role"]=="left_upper_leg")
        foot=next(d for d in report["details"] if d["role"]=="left_foot")
        self.assertAlmostEqual(thigh["rotation"]["same_time_twist_p95_deg"],80)
        self.assertGreater(thigh["rotation"]["twist_jump_count"],100)
        self.assertEqual(foot["rotation"]["twist_jump_count"],0)
        self.assertEqual(report["case_summary"][0]["worst_twist_role"],"left_upper_leg")
        self.assertAlmostEqual(thigh["rotation"]["same_time_swing_p95_deg"],0)

    def test_freeze_cannot_win_on_jump_count_alone(self):
        times=[i/60 for i in range(120)]
        refs=[rotated_track(rotation(2,i*.8)) for i in range(120)]
        outs=[rotated_track([1,0,0,0])]*120
        m=rotation_metrics(times,list(range(120)),refs,outs,1/60,45)
        self.assertEqual(m["twist_jump_count"],0)
        self.assertEqual(m["twist_path_length_ratio"],0)
        self.assertGreater(m["same_time_twist_p95_deg"],80)

    def test_no_rate_or_jump_across_missing_reports(self):
        identity=rotated_track([1,0,0,0])
        turn=rotated_track(rotation(2,100))
        for times,seqs,qs in (([0,1/60,2/60],[0,1,2],[identity,None,turn]),
                              ([0,1/60],[0,2],[identity,turn]),
                              ([0,1],[0,1],[identity,turn])):
            m=rotation_metrics(times,seqs,qs,qs,1/60,45)
            self.assertEqual(m["twist_step_support"],0)
            self.assertIsNone(m["twist_jump_count"])

    def test_swing_lag_known_delay_with_no_position_motion(self):
        times=[i/60 for i in range(240)]
        def q(t): return rotation(0,40*math.sin(4*t)+10*math.sin(9*t))
        refs=[forward_vector(q(t)) for t in times]
        outs=[forward_vector(q(t-.1)) for t in times]
        self.assertEqual(estimate_lag(times,refs,outs,1/60)["lag_steps"],6)

    def test_forward_bend_control_keeps_observed_feet_fixed(self):
        _,rows=dataset()
        rows=[r for r in rows if r["segment"]=="forward_bend_fixed_feet"]
        self.assertEqual(len(rows),360)
        for k in (15,16,20,21,22,23,24,25):
            self.assertTrue(all(r["joints"][k]==rows[0]["joints"][k] for r in rows))
        self.assertGreater(max(r["reference_joints"][18][1] for r in rows),.4)


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
