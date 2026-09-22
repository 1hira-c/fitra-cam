import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from evaluate import norm, read_jsonl, sub, verify_input_binding
from hip_sensitivity import (EXPERIMENTAL, HEAD_ISOLATED, candidate_names, dataset,
                             score_pair)
from synthetic import pose, template_profile


@unittest.skipUnless(os.environ.get("FITRA_TEMPORAL_REPLAY"), "C++ replay required")
class HeadLeafTests(unittest.TestCase):
    def profile(self):
        return template_profile()

    def source(self, poses):
        h, _, _ = dataset(hz=20, seconds=2)
        rows = [{"seq": i, "t_ns": i*50_000_000, "segment": "head-control", "state": "fresh",
                 "phase": "static", "joints": p} for i, p in enumerate(poses)]
        return h, rows

    def replay(self, h, rows, isolate=True, profile=True, diagnostic=True):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, output = root/"input.jsonl", root/"replay.jsonl"
            source.write_text("\n".join(json.dumps(r) for r in [h, *rows])+"\n")
            args = [os.environ["FITRA_TEMPORAL_REPLAY"], str(source), str(output), "--ik-experiments"]
            if isolate: args.append("--isolate-head")
            if diagnostic: args.append("--diagnostics")
            if profile:
                path = root/"profile.json"
                path.write_text(json.dumps(self.profile()))
                args.append(str(path))
            result = subprocess.run(args, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            oh, out = read_jsonl(output)
            verify_input_binding(source, oh)
            for r in out: r.pop("processing_ms")
            return oh, out

    def test_opt_in_preserves_all_six_existing_candidates(self):
        h, _, fault = dataset(hz=5, seconds=2)
        old_h, old = self.replay(h, fault, isolate=False)
        new_h, new = self.replay(h, fault)
        self.assertEqual(candidate_names(old_h), EXPERIMENTAL)
        self.assertEqual(candidate_names(new_h), HEAD_ISOLATED)
        for a, b in zip(old, new):
            for key in ("seq", "t_ns", "reset", "reference"): self.assertEqual(a[key], b[key])
            for n in EXPERIMENTAL:
                for key in ("candidates", "diagnostics", "constraint_metrics"):
                    self.assertEqual(a[key][n], b[key][n])
        bad = copy.deepcopy(new_h)
        bad["ik_experiment"]["schema"] = "fitra_soft_bones_experiment_v1"
        with self.assertRaises(ValueError): candidate_names(bad)

    def test_head_only_outlier_cannot_move_body_in_isolated_variants(self):
        a, b = pose(), pose()
        b[17][0] += .3
        b[17][2] += .8
        h, rows = self.source([a, b])
        _, out = self.replay(h, rows)
        for n in ("soft_bones_head_leaf", "soft_hips_head_leaf"):
            x, y = [r["diagnostics"][n]["post_ik"] for r in out]
            for k in range(5, 26):
                if k != 17: self.assertEqual(x[k], y[k])
            self.assertNotEqual(x[17], y[17])
        for n in ("soft_bones", "soft_hips"):
            x, y = [r["diagnostics"][n]["post_ik"] for r in out]
            self.assertGreater(norm(sub(x[19][:3], y[19][:3])), .001)

    def test_absent_head_prior_is_exact_no_op(self):
        a, b = pose(), pose()
        b[17][1] += 1
        h, rows = self.source([a, b])
        _, out = self.replay(h, rows, profile=False)
        for r in out:
            for n in ("soft_bones", "soft_hips"):
                for key in ("candidates", "diagnostics", "constraint_metrics"):
                    self.assertEqual(r[key][n], r[key][n+"_head_leaf"])

    def test_head_projection_preserves_score_and_missingness(self):
        poses = []
        for missing in (None, 17, 18):
            p = pose()
            p[17][2] += 1
            p[17][3] = 1.8
            if missing is not None: p[missing] = None
            poses.append(p)
        h, rows = self.source(poses)
        oh, out = self.replay(h, rows)
        target = next(length for _, child, length in oh["ik_experiment"]["bone_targets"] if child == 17)
        for n in ("soft_bones_head_leaf", "soft_hips_head_leaf"):
            for p, r in zip(poses, out):
                x = r["diagnostics"][n]["post_ik"]
                for k in range(5, 26):
                    self.assertEqual(p[k] is None, x[k] is None)
                    if p[k]: self.assertAlmostEqual(p[k][3], x[k][3], places=6)
                if x[17] and x[18]:
                    self.assertAlmostEqual(norm(sub(x[17][:3], x[18][:3])), target, places=6)

    def test_coincident_head_does_not_invent_direction(self):
        p = [None]*26
        p[17] = p[18] = [1, 2, 3, .95]
        h, rows = self.source([p])
        _, out = self.replay(h, rows)
        for n in ("soft_bones_head_leaf", "soft_hips_head_leaf"):
            joints = out[0]["diagnostics"][n]["post_ik"]
            self.assertEqual(joints[17], joints[18])
            self.assertGreater(out[0]["constraint_metrics"][n]["bone_rms_mm"], 200)

    def test_cli_requires_experiment_and_rejects_duplicate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for flags in (("--isolate-head",), ("--ik-experiments", "--isolate-head", "--isolate-head")):
                result = subprocess.run([os.environ["FITRA_TEMPORAL_REPLAY"], str(root/"unused-input"),
                    str(root/"out"), *flags], capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("--isolate-head", result.stderr)
                self.assertFalse((root/"out").exists())

    def test_spatial_equivariance(self):
        def transform(j):
            if j is None: return None
            x, y, z, s = j
            # Rotation about X, plus arbitrary world translation.
            return [x+.2, y*math.cos(.7)-z*math.sin(.7)-.4,
                    y*math.sin(.7)+z*math.cos(.7)+.6, s]
        p = pose()
        p[17][0] += .4
        h, rows = self.source([p, [transform(j) for j in p]])
        _, out = self.replay(h, rows)
        for n in ("soft_bones_head_leaf", "soft_hips_head_leaf"):
            x, y = [r["diagnostics"][n]["post_ik"] for r in out]
            for k in range(5, 26): self.assertLess(norm(sub(transform(x[k])[:3], y[k][:3])), 2e-6)

    def test_prefix_causality_boundary_reset_and_diagnostic_parity(self):
        h, rows = self.source([pose() for _ in range(10)])
        rows[3]["state"], rows[3]["joints"] = "boundary", None
        rows[7]["joints"][17][1] += .5
        _, out = self.replay(h, rows)
        self.assertEqual(self.replay(h, rows[:7])[1], out[:7])
        self.assertEqual(self.replay(h, rows[4:])[1][0], out[4])
        self.assertIsNone(out[3]["diagnostics"])
        self.assertIsNone(out[3]["constraint_metrics"])
        _, plain = self.replay(h, rows, diagnostic=False)
        for a, b in zip(out, plain):
            a.pop("diagnostics")
            self.assertEqual(a, b)

    def test_pair_scorer_keeps_eight_candidates_and_constraints(self):
        h, clean, fault = dataset(0, hz=5, seconds=2)
        ch, co = self.replay(h, clean)
        fh, fo = self.replay(h, fault)
        result = score_pair(h, clean, fault, ch, co, fh, fo)
        self.assertEqual(len(result["details"]), 15*8)
        for d in result["details"]:
            self.assertEqual(d["constraints"]["clean"], d["constraints"]["fault"])
            for j in d["joints"]["post_ik"].values(): self.assertEqual(j["rms_mm"], 0)


class SyntheticPriorTests(unittest.TestCase):
    def test_profile_is_fixed_template_and_fresh_object(self):
        p = template_profile()
        self.assertAlmostEqual(p["bone_lengths_m"][17], .25)
        self.assertAlmostEqual(p["bone_lengths_m"][18], .5)
        self.assertAlmostEqual(p["bone_lengths_m"][11], .1)
        p["bone_lengths_m"][17] = 1
        self.assertAlmostEqual(template_profile()["bone_lengths_m"][17], .25)
        self.assertIn("not_real_subject", p["provenance"])

    def test_synthetic_prior_cannot_override_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            result = subprocess.run([sys.executable, str(Path(__file__).with_name("run_postprocess_experiment.py")),
                str(root/"out"), "--source-run", str(root/"source"), "--binary", "unused",
                "--synthetic-bone-prior"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn("cannot override", result.stderr)
            self.assertFalse((root/"out").exists())


if __name__ == "__main__":
    unittest.main()
