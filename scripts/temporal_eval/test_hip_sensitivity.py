import copy
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from evaluate import read_jsonl, verify_input_binding
from hip_sensitivity import (BODY, CANDIDATES, TARGETS, cases, dataset, position_difference,
                             score_pair, tracker_difference, validate_pair)


class FixtureTests(unittest.TestCase):
    def small(self, amplitude=20):
        return dataset(amplitude, hz=10, seconds=2)

    def test_target_only_injection_preserves_timing_confidence_and_missingness(self):
        h, clean, faulty = self.small()
        validate_pair(h, clean, faulty)
        self.assertEqual(len(h["provenance"]["cases"]), 15)
        self.assertEqual(len(clean), 300)
        definition = {c["id"]: c for c in cases()}
        for a, b in zip(clean, faulty):
            self.assertEqual(a.keys(), b.keys())
            for k in a.keys()-{"joints"}:
                self.assertEqual(a[k], b[k])
            targets = TARGETS[definition[a["segment"]]["target"]]
            for k in range(26):
                if k not in targets:
                    self.assertEqual(a["joints"][k], b["joints"][k])
                else:
                    self.assertEqual(a["joints"][k][3], b["joints"][k][3])
        self.assertFalse(clean[0]["evaluate"])
        self.assertTrue(clean[10]["evaluate"])
        self.assertEqual(clean[-1]["seq"], 299)
        self.assertEqual(h["reference_kind"], "none")

    def test_deterministic_independent_storage_and_zero_fault_control(self):
        a = self.small()
        self.assertEqual(a, self.small())
        _, clean, faulty = self.small(0)
        self.assertEqual(clean, faulty)
        a[2][0]["joints"][19][0] += 1
        self.assertNotEqual(a[1][0]["joints"][19], a[2][0]["joints"][19])
        self.assertEqual(a[1], self.small()[1])

    def test_common_and_differential_hips_do_not_modify_root(self):
        _, clean, faulty = self.small()
        for a, b in zip(clean, faulty):
            if "/fixed_bias/hips_" not in a["segment"]:
                continue
            self.assertEqual(a["joints"][19], b["joints"][19])
            left = b["joints"][11][1]-a["joints"][11][1]
            right = b["joints"][12][1]-a["joints"][12][1]
            self.assertAlmostEqual(left, .02)
            self.assertAlmostEqual(right, -.02 if a["segment"].endswith("differential") else .02)

    def test_forward_bend_has_identical_fixed_foot_inputs(self):
        _, clean, faulty = self.small()
        rows = [(a, b) for a, b in zip(clean, faulty) if a["segment"].startswith("forward_bend/")]
        for a, b in rows:
            for k in (15, 16, 20, 21, 22, 23, 24, 25):
                self.assertEqual(a["joints"][k], clean[0]["joints"][k])
                self.assertEqual(a["joints"][k], b["joints"][k])
        self.assertNotEqual(rows[0][0]["joints"][18], rows[5][0]["joints"][18])

    def test_view_bias_varies_with_heading_but_fixed_bias_does_not(self):
        _, clean, faulty = self.small()
        values = {}
        for a, b in zip(clean, faulty):
            if a["segment"].endswith("/center"):
                values.setdefault(a["segment"], []).append(b["joints"][19][1]-a["joints"][19][1])
        fixed = values["static/fixed_bias/center"]
        view = values["turn/view_bias/center"]
        self.assertEqual(set(fixed), {.02})
        self.assertEqual(view[0], 0)
        self.assertGreater(max(view)-min(view), .01)

    def test_bad_fixture_parameters_rejected(self):
        for kwargs in ({"amplitude_mm": -1}, {"amplitude_mm": float("nan")}, {"amplitude_mm": True},
                       {"hz": True}, {"hz": 0}, {"seconds": 1}, {"selected": []},
                       {"selected": [cases()[0], cases()[0]]}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                dataset(**kwargs)

    def test_pair_rejects_non_target_changes_or_relabelled_data(self):
        for mutation in ("knee", "score", "missing", "time", "seq_type", "stage", "measured"):
            h, a, b = self.small()
            if mutation == "knee": b[0]["joints"][13][0] += .01
            if mutation == "score": b[0]["joints"][19][3] = .2
            if mutation == "missing": b[0]["joints"][19] = None
            if mutation == "time": b[0]["t_ns"] += 1
            if mutation == "seq_type": a[0]["seq"] = b[0]["seq"] = False
            if mutation == "stage": h["input_stage"] = "post_ik"
            if mutation == "measured": h["time_basis"] = "measured_capture_group"
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                validate_pair(h, a, b)


class DifferenceTests(unittest.TestCase):
    def test_constant_bias_is_not_hidden_by_zero_jitter(self):
        result = position_difference([[0, 0, 0]]*4, [[0, .02, 0]]*4, .02)
        self.assertAlmostEqual(result["rms_mm"], 20)
        self.assertAlmostEqual(result["gain"], 1)
        self.assertEqual(result["centered_delta_rms_mm"], 0)
        self.assertEqual(result["mean_delta_mm"], [0, 20, 0])

    def test_missing_pairs_remain_null_and_in_denominator(self):
        result = position_difference([None, [0, 0, 0]], [[0, 0, 0], None], .02)
        self.assertEqual(result["eligible"], 2)
        self.assertEqual(result["pairs"], 0)
        self.assertEqual(result["fault_available"], 1)
        self.assertIsNone(result["rms_mm"])
        self.assertIsNone(result["gain"])
        self.assertIsNone(position_difference([[0, 0, 0]], [[0, 0, 0]], 0)["gain"])
        with self.assertRaises(ValueError): position_difference([[0, 0]], [[0, 0]], 1)

    def tracker(self, angle=0, confidence=1):
        return {"position_m": [0, 0, 0], "quat_wxyz": [math.cos(angle/2), 0, 0, math.sin(angle/2)],
                "roll_confidence": confidence, "swing_confidence": 1}

    def test_supported_twist_and_unknown_twist_are_separate(self):
        result = tracker_difference([self.tracker(), self.tracker(confidence=0)],
                                    [self.tracker(math.pi/3)]*2, .02)["rotation"]
        self.assertAlmostEqual(result["supported_twist_delta_p95_deg"], 60)
        self.assertAlmostEqual(result["swing_delta_p95_deg"], 0)
        self.assertEqual(result["twist_supported_pairs"], 1)
        self.assertEqual(result["twist_unobservable_pairs"], 1)
        unknown = tracker_difference([self.tracker()], [self.tracker(1, 0)], .02)["rotation"]
        self.assertIsNone(unknown["supported_twist_delta_p95_deg"])

    def test_quaternion_sign_does_not_create_a_rotation(self):
        t = self.tracker(.4)
        opposite = {**t, "quat_wxyz": [-v for v in t["quat_wxyz"]]}
        result = tracker_difference([t], [opposite], .02)["rotation"]
        self.assertAlmostEqual(result["quaternion_delta_p95_deg"], 0)
        self.assertAlmostEqual(result["supported_twist_delta_p95_deg"], 0)


@unittest.skipUnless(os.environ.get("FITRA_TEMPORAL_REPLAY"), "C++ replay required")
class DiagnosticReplayTests(unittest.TestCase):
    def replay(self, header, rows, diagnostic=True):
        with tempfile.TemporaryDirectory() as directory:
            source, out = Path(directory)/"input.jsonl", Path(directory)/"out.jsonl"
            source.write_text("\n".join(json.dumps(r) for r in [header, *rows])+"\n")
            command = [os.environ["FITRA_TEMPORAL_REPLAY"], str(source), str(out)]
            if diagnostic: command.append("--diagnostics")
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            oh, output = read_jsonl(out)
            verify_input_binding(source, oh)
            for row in output: row.pop("processing_ms")
            return oh, output

    def small(self, amplitude=20):
        selected = [c for c in cases() if c["id"] == "static/fixed_bias/hips_common"]
        return dataset(amplitude, hz=20, seconds=2, selected=selected)

    def test_instrumentation_does_not_change_candidates_and_is_prefix_causal(self):
        h, _, rows = self.small()
        plain_header, plain = self.replay(h, rows, False)
        diag_header, diag = self.replay(h, rows)
        self.assertEqual({k: v for k, v in diag_header.items() if k != "diagnostics_schema"}, plain_header)
        self.assertEqual([{k: v for k, v in r.items() if k != "diagnostics"} for r in diag], plain)
        self.assertEqual(self.replay(h, rows[:15])[1], diag[:15])

    def test_boundary_has_no_stale_diagnostics(self):
        h, clean, _ = self.small()
        clean[15]["state"], clean[15]["joints"] = "boundary", None
        _, out = self.replay(h, clean)
        self.assertIsNone(out[15]["diagnostics"])
        self.assertTrue(out[16]["reset"])
        _, isolated = self.replay(h, clean[16:])
        self.assertEqual(out[16], isolated[0])

    def test_known_hip_fault_propagates_through_ik_without_changing_raw_knees(self):
        h, clean, faulty = self.small()
        ch, co = self.replay(h, clean)
        fh, fo = self.replay(h, faulty)
        report = score_pair(h, clean, faulty, ch, co, fh, fo)
        json.dumps(report, allow_nan=False)
        self.assertEqual(len(report["details"]), 3)
        minimal = report["details"][0]
        self.assertEqual(minimal["candidate"], "minimal")
        self.assertEqual(minimal["joints"]["raw"]["13"]["rms_mm"], 0)
        self.assertEqual(minimal["joints"]["post_kalman"]["13"]["rms_mm"], 0)
        self.assertGreater(minimal["joints"]["post_ik"]["13"]["rms_mm"], .1)
        for mutation in ("lineage", "count", "reset", "stage", "config"):
            bh, bad = copy.deepcopy((fh, fo))
            if mutation == "lineage": bad[0]["seq"] += 1
            if mutation == "count": bad.pop()
            if mutation == "reset": bad[0]["reset"] = False
            if mutation == "stage": del bad[0]["diagnostics"]["minimal"]["post_ik"]
            if mutation == "config": bh["subject_height_prior_m"] = 1.8
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                score_pair(h, clean, faulty, ch, co, bh, bad)

    def test_zero_fault_has_zero_difference_in_every_stage(self):
        h, clean, faulty = self.small(0)
        ch, co = self.replay(h, clean)
        fh, fo = self.replay(h, faulty)
        report = score_pair(h, clean, faulty, ch, co, fh, fo)
        for d in report["details"]:
            for joints in d["joints"].values():
                for k in BODY:
                    self.assertEqual(joints[str(k)]["rms_mm"], 0)
                    self.assertIsNone(joints[str(k)]["gain"])
            for trackers in d["trackers"].values():
                for t in trackers.values():
                    if t["pairs"]:
                        self.assertEqual(t["rms_mm"], 0)
                        self.assertLess(t["rotation"]["quaternion_delta_p95_deg"], 1e-4)


if __name__ == "__main__":
    unittest.main()
