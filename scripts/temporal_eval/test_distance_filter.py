import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from distance_controls import (BASE, CAPS, cases, crossing, dataset, harmonic_comparison,
                               harmonic_fit, score_distance)
from evaluate import norm, read_jsonl, sub, verify_input_binding
from hip_sensitivity import DISTANCE, HEAD_ISOLATED, candidate_names
from synthetic import pose, template_profile


@unittest.skipUnless(os.environ.get("FITRA_TEMPORAL_REPLAY"), "C++ replay required")
class DistanceFilterTests(unittest.TestCase):
    def source(self, offsets):
        h, _ = dataset(0)
        rows = []
        for i,offset in enumerate(offsets):
            p = pose()
            for j in p:
                if j: j[0] += offset
            rows.append({"seq": i, "t_ns": round(i*1e9/60), "segment": "test", "state": "fresh",
                         "phase": "motion", "joints": p, "reference_joints": copy.deepcopy(p)})
        return h, rows

    def replay(self, h, rows, distance=True, diagnostic=True):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, output, profile = root/"input.jsonl", root/"out.jsonl", root/"profile.json"
            source.write_text("\n".join(json.dumps(r) for r in [h,*rows])+"\n")
            profile.write_text(json.dumps(template_profile()))
            args = [os.environ["FITRA_TEMPORAL_REPLAY"], str(source), str(output), str(profile),
                    "--ik-experiments", "--isolate-head"]
            if diagnostic: args.append("--diagnostics")
            if distance: args.append("--distance-experiments")
            result = subprocess.run(args, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            oh, out = read_jsonl(output)
            verify_input_binding(source, oh)
            for r in out: r.pop("processing_ms")
            return oh, out

    def test_optional_candidates_leave_all_eight_baselines_exact(self):
        h, rows = dataset(3,hz=5)
        old_h, old = self.replay(h,rows,distance=False)
        new_h, new = self.replay(h,rows)
        self.assertEqual(candidate_names(old_h),HEAD_ISOLATED)
        self.assertEqual(candidate_names(new_h),DISTANCE)
        for a,b in zip(old,new):
            for key in ("seq","t_ns","reset","reference"): self.assertEqual(a[key],b[key])
            for key in ("candidates","diagnostics","constraint_metrics"):
                if a[key] is None: self.assertIsNone(b[key]); continue
                for n in HEAD_ISOLATED: self.assertEqual(a[key][n],b[key][n])

    def test_distance_formula_cap_and_position_only(self):
        h,rows = self.source([0,.004,.001,.009,.01,.035,.035,.035,.001,0]+[0]*20)
        _,out = self.replay(h,rows)
        for name,cap in CAPS.items():
            previous = [None]*10
            errors = []
            for r in out:
                for k,(a,b) in enumerate(zip(r['candidates'][BASE],r['candidates'][name])):
                    self.assertEqual(a is None,b is None)
                    if a is None: previous[k]=None; continue
                    self.assertEqual({f:v for f,v in a.items() if f!='position_m'},
                                     {f:v for f,v in b.items() if f!='position_m'})
                    target = a['position_m']
                    if previous[k] is None: expected=target
                    else:
                        delta=sub(target,previous[k]); d2=sum(v*v for v in delta)
                        alpha=d2/(d2+4*cap*cap)
                        expected=[y+alpha*d for y,d in zip(previous[k],delta)]
                    self.assertLess(norm(sub(expected,b['position_m'])),1e-6)
                    previous[k]=b['position_m']
                    error=norm(sub(target,b['position_m']))
                    self.assertLessEqual(error,cap+2e-6)
                    errors.append(error)
                self.assertEqual(r['diagnostics'][BASE],r['diagnostics'][name])
                self.assertEqual(r['constraint_metrics'][BASE],r['constraint_metrics'][name])
            self.assertGreater(max(errors),cap*.9)
            # Fixed target converges; no hard dead zone or permanent frozen offset.
            before=norm(sub(out[10]['candidates'][BASE][3]['position_m'],out[10]['candidates'][name][3]['position_m']))
            after=norm(sub(out[-1]['candidates'][BASE][3]['position_m'],out[-1]['candidates'][name][3]['position_m']))
            self.assertLess(after,before)

    def test_prefix_teacher_independence_and_boundary_reset(self):
        h,rows=self.source([0,.001,.004,.008,.01,.014,.02,.015,.006,0])
        rows[4]['state']='boundary'; rows[4]['joints']=rows[4]['reference_joints']=None
        _,out=self.replay(h,rows)
        self.assertEqual(self.replay(h,rows[:8])[1],out[:8])
        self.assertEqual(self.replay(h,rows[5:])[1][0],out[5])
        changed=copy.deepcopy(rows)
        for r in changed:
            if r['reference_joints']:
                for j in r['reference_joints']:
                    if j: j[0]+=.3
        _,other=self.replay(h,changed)
        for a,b in zip(out,other):
            for key in ('candidates','diagnostics','constraint_metrics'): self.assertEqual(a[key],b[key])
        for name in CAPS:
            self.assertTrue(all(t is None for t in out[4]['candidates'][name]))
            for a,b in zip(out[5]['candidates'][BASE],out[5]['candidates'][name]): self.assertEqual(a,b)

    def test_missing_tracker_discards_history_without_filling(self):
        h,rows=self.source([0,.003,.006,.008,.02])
        rows[3]['joints']=[None]*26
        _,out=self.replay(h,rows)
        missing=[i for i,t in enumerate(out[3]['candidates'][BASE]) if t is None]
        self.assertTrue(missing)
        for name in CAPS:
            for i in missing:
                self.assertIsNone(out[3]['candidates'][name][i])
                self.assertEqual(out[4]['candidates'][BASE][i],out[4]['candidates'][name][i])

    def test_gap_reset_and_diagnostic_on_off(self):
        h,rows=self.source([0,.004,.008,.015])
        rows[3]['t_ns']+=700_000_000
        _,full=self.replay(h,rows)
        self.assertEqual(self.replay(h,rows[3:])[1][0],full[3])
        _,plain=self.replay(h,rows,diagnostic=False)
        for a,b in zip(full,plain):
            a.pop('diagnostics'); self.assertEqual(a,b)

    def test_world_translation_and_yaw_equivariance(self):
        def transform(j):
            if j is None: return None
            x,y,z,*tail=j
            return [x*math.cos(.6)-y*math.sin(.6)+.4,x*math.sin(.6)+y*math.cos(.6)-.2,z,*tail]
        h,rows=self.source([0,.001,.004,.012,.025,0])
        changed=copy.deepcopy(rows)
        for r in changed: r['joints']=[transform(j) for j in r['joints']]
        _,a=self.replay(h,rows); _,b=self.replay(h,changed)
        for x,y in zip(a,b):
            for name in CAPS:
                for p,q in zip(x['candidates'][name],y['candidates'][name]):
                    self.assertEqual(p is None,q is None)
                    if p: self.assertLess(norm(sub(transform(p['position_m']),q['position_m'])),3e-6)

    def test_control_scoring_and_caps_reject_changed_fields(self):
        h,rows=dataset(0,hz=5)
        oh,out=self.replay(h,rows)
        report=score_distance(h,rows,oh,out)
        self.assertEqual(len(report['controls']),7*10*10)
        self.assertEqual(len(report['caps']),2*10)
        for field in ('position_m','roll_confidence'):
            bad=copy.deepcopy(out)
            if field=='position_m': bad[0]['candidates']['distance_2mm'][3][field][0]+=1
            else: bad[0]['candidates']['distance_2mm'][3][field]=.123
            with self.assertRaises(ValueError): score_distance(h,rows,oh,bad)
        bad=copy.deepcopy(oh); bad.pop('distance_experiment')
        with self.assertRaises(ValueError): candidate_names(bad)

    def test_cli_rejects_missing_prerequisites_and_duplicate_flags(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for flags in (['--distance-experiments'],['--ik-experiments','--isolate-head','--distance-experiments','--distance-experiments']):
                result=subprocess.run([os.environ['FITRA_TEMPORAL_REPLAY'],str(root/'unused'),str(root/'out'),*flags],capture_output=True,text=True)
                self.assertNotEqual(result.returncode,0)
                self.assertIn('--distance-experiments',result.stderr)
                self.assertFalse((root/'out').exists())


class DistanceMetricTests(unittest.TestCase):
    def test_harmonic_fit_detects_small_motion_gain_and_phase(self):
        times=[1.2+i/60 for i in range(217)] # Partial cycles, arbitrary offset.
        reference=[.001*math.sin(math.pi*t) for t in times]
        output=[4+.0005*math.sin(math.pi*(t-.1)) for t in times]
        m=harmonic_comparison(times,reference,output,.5)
        self.assertAlmostEqual(m['amplitude_ratio'],.5,places=8)
        self.assertAlmostEqual(m['phase_lag_ms'],100,places=6)

    def test_frozen_harmonic_output_has_no_meaningful_phase(self):
        times=[i/60 for i in range(120)]
        ref=[.001*math.sin(math.pi*t) for t in times]
        m=harmonic_comparison(times,ref,[3.]*120,.5)
        self.assertAlmostEqual(m['amplitude_ratio'],0)
        self.assertIsNone(m['phase_lag_ms'])
        self.assertEqual(harmonic_comparison(times,ref,[None]*120,.5)['pairs'],0)
        self.assertIsNone(harmonic_fit([0.]*20,[1.]*20,.5))
        with self.assertRaises(ValueError): harmonic_comparison(times,ref[:-1],ref,.5)
        with self.assertRaises(ValueError): harmonic_fit(times,ref[:-1],.5)
        for frequency in (True,0,float('nan')):
            with self.assertRaises(ValueError): harmonic_fit(times,ref,frequency)

    def test_crossing_is_sustained_and_does_not_bridge_holes(self):
        e={'start_s':0.,'end_s':2.,'level_m':.005,'direction':1}
        t=[i/10 for i in range(9)]; seq=list(range(9))
        self.assertAlmostEqual(crossing(t,seq,[0,.006,0,0,.006,.006,.006,.006,.006],e,10),.4)
        self.assertIsNone(crossing(t[:3],[0,1,3],[.006]*3,e,10))
        self.assertIsNone(crossing(t[:3],seq[:3],[.006,None,.006],e,10))
        self.assertIsNone(crossing(t,seq,[0.]*9,e,10))

    def test_crossing_falling_direction(self):
        e={'start_s':0.,'end_s':2.,'level_m':0.,'direction':-1}
        self.assertAlmostEqual(crossing([i/10 for i in range(6)],list(range(6)),[.01,.004,0,-.001,-.002,-.003],e,10),.2)

    def test_fixture_is_reproducible_and_reference_is_noise_independent(self):
        h,rows=dataset(0,hz=5); noisy_h,noisy=dataset(3,hz=5)
        self.assertEqual((h,rows),dataset(0,hz=5))
        self.assertEqual(len(rows),245)
        for a,b in zip(rows,noisy):
            self.assertEqual({k:v for k,v in a.items() if k!='joints'},{k:v for k,v in b.items() if k!='joints'})
        xs=[r['reference_joints'][19][0] for r in rows if r['segment']=='micro_1mm']
        self.assertLessEqual(max(abs(x) for x in xs),.001)
        self.assertGreater(max(xs)-min(xs),.0018)
        for noise in (-1,float('nan'),True):
            with self.assertRaises(ValueError): dataset(noise,hz=5)

    def test_runner_rejects_recording_replacement_and_unscoped_noise(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for flags in (['--source-run','unused','--distance-controls'],['--noise-mm','1']):
                result=subprocess.run([sys.executable,str(Path(__file__).with_name('run_postprocess_experiment.py')),
                    str(root/'out'),'--binary','unused',*flags],capture_output=True,text=True)
                self.assertEqual(result.returncode,2)
                self.assertFalse((root/'out').exists())


if __name__ == '__main__': unittest.main()
