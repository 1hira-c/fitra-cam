import copy
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from evaluate import norm, read_jsonl, sub, verify_input_binding
from hip_sensitivity import CANDIDATES, EXPERIMENTAL, constraint_summary, dataset, score_pair
from synthetic import pose
from run_postprocess_experiment import verify_baseline, verified_source
from run_recording import artifact, write_json
from joint_fault_controls import TARGETS as CONTROL_TARGETS, controls


@unittest.skipUnless(os.environ.get("FITRA_TEMPORAL_REPLAY"), "C++ replay required")
class SoftBoneTests(unittest.TestCase):
    def source(self, poses):
        h, _, _ = dataset(hz=20, seconds=2)
        rows = [{"seq": i, "t_ns": i*50_000_000, "segment": "test", "state": "fresh",
                 "phase": "motion", "joints": p} for i, p in enumerate(poses)]
        return h, rows

    def replay(self, h, rows, experimental=True, profile=None, flags=None):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, out = root/"input.jsonl", root/"output.jsonl"
            source.write_text("\n".join(json.dumps(r) for r in [h, *rows])+"\n")
            args = [os.environ["FITRA_TEMPORAL_REPLAY"], str(source), str(out), "--diagnostics"]
            if experimental: args.append("--ik-experiments")
            if profile:
                path = root/"profile.json"
                path.write_text(json.dumps(profile))
                args.append(str(path))
            if flags: args.extend(flags)
            completed = subprocess.run(args, capture_output=True, text=True)
            if flags:
                self.assertNotEqual(completed.returncode, 0)
                return
            self.assertEqual(completed.returncode, 0, completed.stderr)
            oh, output = read_jsonl(out)
            verify_input_binding(source, oh)
            for row in output: row.pop("processing_ms")
            return oh, output

    def test_existing_candidates_unchanged_and_experiment_is_explicit(self):
        h, _, faulty = dataset(hz=5, seconds=2)
        ph, plain = self.replay(h, faulty, False)
        eh, extra = self.replay(h, faulty)
        self.assertEqual(eh["candidates"], list(EXPERIMENTAL))
        self.assertNotIn("ik", eh["common"])
        self.assertTrue(ph["common"]["ik"])
        for p, e in zip(plain, extra):
            for key in ("seq", "t_ns", "reset", "reference"):
                self.assertEqual(p[key], e[key])
            for name in CANDIDATES:
                self.assertEqual(p["candidates"][name], e["candidates"][name])
                self.assertEqual(p["diagnostics"][name], e["diagnostics"][name])

    def test_prefix_causality_boundaries_and_stage_statelessness(self):
        h, rows = self.source([pose() for _ in range(12)])
        rows[4]["state"], rows[4]["joints"] = "boundary", None
        rows[7]["joints"][13][1] += .2
        _, all_rows = self.replay(h, rows)
        self.assertEqual(self.replay(h, rows[:8])[1], all_rows[:8])
        self.assertIsNone(all_rows[4]["constraint_metrics"])
        self.assertEqual(self.replay(h, rows[5:])[1][0], all_rows[5])
        for name in ("soft_bones", "soft_hips"):
            self.assertEqual(all_rows[8]["diagnostics"][name]["post_ik"],
                             all_rows[0]["diagnostics"][name]["post_ik"])

    def test_spatial_translation_and_rotation_equivariance(self):
        def transform(j):
            theta = .7
            x, y, z = j[:3]
            return [math.cos(theta)*x-math.sin(theta)*y+.4,
                    math.sin(theta)*x+math.cos(theta)*y-.2, z+.3, j[3]]
        original = pose()
        h, rows = self.source([original, [transform(j) if j else None for j in original]])
        _, output = self.replay(h, rows)
        for name in ("soft_bones", "soft_hips"):
            a, b = [r["diagnostics"][name]["post_ik"] for r in output]
            for k in range(5, 26):
                self.assertLess(norm(sub(transform(a[k])[:3], b[k][:3])), 2e-6)

    def test_missingness_scores_and_unconstrained_body_points_preserved(self):
        p = pose()
        p[13] = None
        p[11][3] = 1.8
        p[12][3] = .001
        h, rows = self.source([p, [None]*26])
        _, output = self.replay(h, rows)
        for name in ("no_ik", "soft_bones", "soft_hips"):
            joints = output[0]["diagnostics"][name]["post_ik"]
            for k in range(5, 26):
                if p[k] is None: self.assertIsNone(joints[k])
                else: self.assertAlmostEqual(joints[k][3], p[k][3], places=6)
            for k in (17, 18, 20, 21, 22, 23, 24, 25):
                # Neck has shoulder constraints, all other listed points do not.
                if k != 18: self.assertLess(norm(sub(joints[k][:3], p[k][:3])), 1e-6)
            self.assertTrue(all(v is None for v in output[1]["diagnostics"][name]["post_ik"]))
            self.assertIsNone(output[1]["constraint_metrics"][name]["bone_rms_mm"])

    def test_hip_weight_shifts_correction_to_hip_not_knee(self):
        p = [None]*26
        p[11], p[13] = [0, 0, 1, .95], [0, 0, .3, .95]
        h, rows = self.source([p])
        _, out = self.replay(h, rows)
        uniform, soft = [out[0]["diagnostics"][n]["post_ik"] for n in ("soft_bones", "soft_hips")]
        self.assertGreater(norm(sub(soft[11][:3], p[11][:3])), norm(sub(uniform[11][:3], p[11][:3])))
        self.assertLess(norm(sub(soft[13][:3], p[13][:3])), norm(sub(uniform[13][:3], p[13][:3]))/3)

    def test_coincident_edge_does_not_invent_direction(self):
        p = [None]*26
        p[11] = p[13] = [.3, -.2, 1, .95]
        h, rows = self.source([p])
        _, output = self.replay(h, rows)
        for name in ("soft_bones", "soft_hips"):
            joints = output[0]["diagnostics"][name]["post_ik"]
            self.assertLess(norm(sub(joints[11][:3], p[11][:3])), 1e-6)
            self.assertEqual(joints[11], joints[13])
            self.assertGreater(output[0]["constraint_metrics"][name]["bone_rms_mm"], 300)

    def assert_prior_parity(self, profile=None):
        h, rows = self.source([pose()])
        oh, _ = self.replay(h, rows, profile=profile)
        bones = oh["ik_experiment"]["bone_targets"]
        isolated = []
        for parent, child, length in bones:
            p = [None]*26
            p[parent], p[child] = [0, 0, 1, 1], [.3, .2, .3, 1]
            isolated.append(p)
        h, rows = self.source(isolated)
        _, out = self.replay(h, rows, profile=profile)
        for row, (parent, child, length) in zip(out, bones):
            joints = row["diagnostics"]["minimal"]["post_ik"]
            actual = norm(sub(joints[parent][:3], joints[child][:3]))
            self.assertAlmostEqual(actual, length, places=6)
        return oh

    def test_height_prior_matches_production_on_each_isolated_bone(self):
        header = self.assert_prior_parity()
        self.assertEqual(len(header["ik_experiment"]["bone_targets"]), 12)

    def test_subject_profile_and_optional_cli_remain_supported(self):
        profile = {"schema": "fitra_subject_profile_v2", "subject_id": "synthetic_test_only",
                   "subject_height_m": 1.7, "quality_status": "pass", "shoulder_width_m": .37,
                   "hip_width_m": .25, "bone_lengths_m": [0.]*26}
        for k in (5, 6, 7, 8, 9, 10, 13, 14, 15, 16): profile["bone_lengths_m"][k] = .2+k*.001
        header = self.assert_prior_parity(profile)
        self.assertTrue(header["profile_loaded"])
        h, rows = self.source([pose()])
        for flag in ("--ik-experiments", "--unknown"):
            self.replay(h, rows, flags=[flag])

    def test_pair_scoring_keeps_constraints_and_zero_fault(self):
        h, clean, fault = dataset(0, hz=5, seconds=2)
        ch, co = self.replay(h, clean)
        fh, fo = self.replay(h, fault)
        report = score_pair(h, clean, fault, ch, co, fh, fo)
        self.assertEqual(len(report["details"]), 15*6)
        for d in report["details"]:
            self.assertEqual(d["constraints"]["clean"], d["constraints"]["fault"])
            for k, j in d["joints"]["post_ik"].items():
                self.assertEqual(j["rms_mm"], 0)
        bad = copy.deepcopy(fo)
        bad[0]["constraint_metrics"]["soft_hips"]["bone_pairs"] = True
        with self.assertRaises(ValueError): score_pair(h, clean, fault, ch, co, fh, bad)

    def test_nonhip_fault_controls_score_all_stages(self):
        h, clean, fault = controls(hz=5, seconds=2)
        ch, co = self.replay(h, clean)
        fh, fo = self.replay(h, fault)
        report = score_pair(h, clean, fault, ch, co, fh, fo, target_sets=CONTROL_TARGETS)
        self.assertEqual(len(report["details"]), 8*6)
        for d in report["details"]:
            for k in (11, 12, 19): self.assertEqual(d["joints"]["raw"][str(k)]["rms_mm"], 0)
        self.assertTrue(any(d["candidate"] == "soft_hips" and d["joints"]["post_ik"]["11"]["rms_mm"] > 0
                            for d in report["details"]))


class CounterControlTests(unittest.TestCase):
    def test_only_declared_targets_change_and_scores_are_shared(self):
        from hip_sensitivity import validate_pair
        h, clean, fault = controls(hz=5, seconds=2)
        self.assertEqual(len(clean), 80)
        validate_pair(h, clean, fault, CONTROL_TARGETS)
        for a, b in zip(clean, fault):
            for k in (11, 12, 19): self.assertEqual(a["joints"][k], b["joints"][k])
        with self.assertRaises(ValueError): validate_pair(h, clean, fault)
        fault[0]["seq"] = False
        with self.assertRaises(ValueError): validate_pair(h, clean, fault, CONTROL_TARGETS)

    def test_score_conditions_have_identical_positions_and_random_errors(self):
        _, clean, fault = controls(hz=5, seconds=2)
        for series in (clean, fault):
            for start in range(0, len(series), 20):
                for high, low in zip(series[start:start+10], series[start+10:start+20]):
                    for a, b in zip(high["joints"], low["joints"]):
                        if a is not None: self.assertEqual(a[:3], b[:3])

    def test_zero_fault_control_and_seed_reproducibility(self):
        self.assertEqual(controls(hz=5, seconds=2), controls(hz=5, seconds=2))
        _, clean, fault = controls(0, hz=5, seconds=2)
        self.assertEqual(clean, fault)


class ConstraintMetricTests(unittest.TestCase):
    def test_weighted_residual_and_missing_denominators(self):
        def row(n, value):
            return {"constraint_metrics": {"test": {"bone_pairs": n, "bone_rms_mm": value,
                    "bone_max_mm": value, "hinge_pairs": 0, "hinge_outside_5_175": 0}}}
        a = constraint_summary([row(0, None), row(1, 2), row(3, 4)], "test")
        self.assertEqual(a["bone_pairs"], 4)
        self.assertAlmostEqual(a["bone_rms_mm"], math.sqrt(13))
        self.assertIsNone(constraint_summary([row(0, None)], "test")["bone_rms_mm"])

    def test_baseline_guard_rejects_candidate_and_lineage_changes(self):
        row = {"seq": 0, "t_ns": 0, "reset": True, "reference": [None]*10,
               "candidates": {n: [None]*10 for n in CANDIDATES}}
        verify_baseline([row], [copy.deepcopy(row)])
        for field in ("seq", "candidate", "count"):
            changed = copy.deepcopy(row)
            if field == "seq": changed["seq"] += 1
            if field == "candidate": changed["candidates"]["minimal"][0] = {}
            with self.assertRaises(ValueError): verify_baseline([row], [] if field == "count" else [changed])

    def source_fixture(self, root):
        h, rows, _ = dataset(hz=5, seconds=2)
        (root/"input.jsonl").write_text("\n".join(json.dumps(r) for r in [h, *rows])+"\n")
        (root/"replay.jsonl").write_text("fixture only\n")
        write_json(root/"profile.json", {})
        write_json(root/"manifest.json", {"schema": "fitra_recording_pilot_v1",
            "assets": {"subject_profile": artifact(root/"profile.json")}})
        write_json(root/"complete.json", {"reports": len(rows), "manifest": artifact(root/"manifest.json"),
            "artifacts": {n: artifact(root/n) for n in ("input.jsonl", "replay.jsonl")}})

    def test_completed_source_rejects_replaced_input_profile_or_manifest(self):
        for target in ("input.jsonl", "profile.json", "manifest.json"):
            with tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                self.source_fixture(root)
                _, _, rows = verified_source(root)
                self.assertEqual(len(rows), 150)
                with (root/target).open("a") as handle: handle.write("\n")
                with self.assertRaises(ValueError): verified_source(root)

    def test_partial_source_cannot_be_promoted(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.source_fixture(root)
            (root/"complete.json").unlink()
            with self.assertRaises(FileNotFoundError): verified_source(root)


if __name__ == "__main__":
    unittest.main()
