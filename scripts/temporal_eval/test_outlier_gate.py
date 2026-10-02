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
from hip_sensitivity import DISTANCE
from outlier_controls import (FLAGS, GUARDED, MARKER, NAMES, cases, dataset,
                              score_controls, validate_gate, verify_baselines)
from synthetic import pose, template_profile


@unittest.skipUnless(os.environ.get("FITRA_TEMPORAL_REPLAY"), "C++ replay required")
class OutlierGateTests(unittest.TestCase):
    def source(self, offsets):
        h, _ = dataset()
        rows = []
        for i, offset in enumerate(offsets):
            p = pose()
            for j in p:
                if j is not None: j[0] += offset
            rows.append({"seq": i, "t_ns": round(i*1e9/60), "segment": "test", "state": "fresh",
                         "phase": "motion", "evaluate": True,
                         "joints": p, "reference_joints": copy.deepcopy(p)})
        return h, rows

    def replay(self, header, rows, guarded=True, diagnostic=True):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, output, profile = root/"input.jsonl", root/"output.jsonl", root/"profile.json"
            source.write_text("\n".join(json.dumps(r, allow_nan=False) for r in [header, *rows])+"\n")
            frozen_input = source.read_bytes()
            profile.write_text(json.dumps(template_profile()))
            command = [os.environ["FITRA_TEMPORAL_REPLAY"], str(source), str(output), str(profile)]
            command += [f for f in FLAGS if diagnostic or f != "--diagnostics"]
            if guarded: command.append("--outlier-experiments")
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(source.read_bytes(), frozen_input, "raw input was overwritten")
            h, out = read_jsonl(output)
            verify_input_binding(source, h)
            return h, out

    def gate(self, row, k): return row["diagnostics"][GUARDED]["joint_gate"][k]

    def point(self, row, k): return row["diagnostics"][GUARDED]["post_gate"][k]

    def test_optional_candidate_preserves_all_ten_baselines_and_schema(self):
        h, rows = dataset(hz=20)
        ch, control = self.replay(h, rows, guarded=False)
        oh, out = self.replay(h, rows)
        self.assertEqual(tuple(oh["candidates"]), NAMES)
        self.assertEqual(oh["outlier_experiment"], MARKER)
        verify_baselines(ch, control, oh, out)
        for row in out:
            for n in DISTANCE:
                self.assertEqual(set(row["diagnostics"][n]), {"post_kalman", "post_ik", "pre_smoothing"})
            validate_gate(row["diagnostics"][GUARDED])

    def test_one_and_two_report_high_score_spikes_do_not_poison_history(self):
        for k in (13, 15, 20):
            for length in (1, 2):
                with self.subTest(joint=k, length=length):
                    h, rows = self.source([0]*12)
                    for r in rows[4:4+length]: r["joints"][k][1] += .3
                    _, out = self.replay(h, rows)
                    for i in range(4, 4+length):
                        state = self.gate(out[i], k)
                        self.assertEqual(state["state"], "predicted")
                        self.assertTrue(state["rejected"])
                        self.assertFalse(state["reacquired"])
                        self.assertEqual(self.point(out[i], k)[3], 0)
                        self.assertLess(norm(sub(self.point(out[i], k)[:3], pose()[k][:3])), 1e-7)
                        self.assertEqual(rows[i]["joints"][k][3], .95)
                    state = self.gate(out[4+length], k)
                    self.assertEqual(state["state"], "accepted")
                    self.assertFalse(state["reacquired"])
                    self.assertLess(norm(sub(self.point(out[4+length], k)[:3], pose()[k][:3])), 1e-7)
                    self.assertTrue(all(self.gate(r, k)["state"] == "accepted" for r in out[4+length:]))
                    # Unaffected observations pass immediately, before any spatial solve.
                    for i in range(4, 4+length):
                        for other in (11, 14, 16, 21):
                            if other != k:
                                self.assertEqual(self.gate(out[i], other)["state"], "accepted")
                                self.assertLess(norm(sub(self.point(out[i], other)[:3], rows[i]["joints"][other][:3])), 1e-7)

    def test_velocity_uses_accepted_history_and_short_prediction_is_causal(self):
        h, rows = self.source([i*.01 for i in range(12)])
        for i in (4, 5): rows[i]["joints"][13][1] += .5
        _, out = self.replay(h, rows)
        for i in (4, 5):
            self.assertEqual(self.gate(out[i], 13)["state"], "predicted")
            self.assertLess(norm(sub(self.point(out[i], 13)[:3], rows[i]["reference_joints"][13][:3])), 2e-7)
        self.assertEqual(self.gate(out[6], 13)["state"], "accepted")
        self.assertFalse(self.gate(out[6], 13)["reacquired"])

    def test_two_swapped_knee_reports_are_rejected_without_relabeling_raw(self):
        h, rows = self.source([0]*10)
        for i in (4, 5): rows[i]["joints"][13], rows[i]["joints"][14] = rows[i]["joints"][14], rows[i]["joints"][13]
        preserved = copy.deepcopy(rows)
        _, out = self.replay(h, rows)
        self.assertEqual(rows, preserved)
        for i in (4, 5):
            for k in (13, 14):
                self.assertTrue(self.gate(out[i], k)["rejected"])
                self.assertEqual(self.gate(out[i], k)["state"], "predicted")
                self.assertLess(norm(sub(self.point(out[i], k)[:3], pose()[k][:3])), 1e-7)
        for k in (13, 14): self.assertEqual(self.gate(out[6], k)["state"], "accepted")

    def test_recovery_across_rejection_does_not_seed_velocity(self):
        h, rows = self.source([0, 0, 0, .04, .04, .045, .045, .055, .055, .065, .075, .075])
        for i in (4, 6, 8, 11): rows[i]["joints"][13] = None
        _, out = self.replay(h, rows)
        for i in (6, 8):
            self.assertEqual(self.gate(out[i], 13)["state"], "predicted")
            self.assertLess(norm(sub(self.point(out[i], 13)[:3], rows[i-1]["joints"][13][:3])), 2e-7)
        expected = list(rows[10]["joints"][13][:3])
        expected[0] += .01
        self.assertLess(norm(sub(self.point(out[11], 13)[:3], expected)), 2e-7)

    def test_sustained_plausible_shift_reacquires_on_third_report(self):
        h, rows = self.source([0]*4+[.3]*8)
        _, out = self.replay(h, rows)
        for i in (4, 5):
            self.assertEqual(self.gate(out[i], 13)["state"], "predicted")
            self.assertFalse(self.gate(out[i], 13)["reacquired"])
        self.assertEqual(self.gate(out[6], 13)["state"], "accepted")
        self.assertTrue(self.gate(out[6], 13)["reacquired"])
        self.assertLess(norm(sub(self.point(out[6], 13)[:3], rows[6]["joints"][13][:3])), 1e-7)
        self.assertEqual(self.gate(out[7], 13)["state"], "accepted")
        self.assertFalse(self.gate(out[7], 13)["reacquired"])

    def test_missing_observation_interrupts_pending_reacquisition(self):
        h, rows = self.source([0]*3+[.3]*8)
        rows[4]["joints"][13] = None
        _, out = self.replay(h, rows)
        for i in (3, 5, 6): self.assertFalse(self.gate(out[i], 13)["reacquired"])
        self.assertEqual(self.gate(out[7], 13)["state"], "accepted")
        self.assertTrue(self.gate(out[7], 13)["reacquired"])

    def test_deadline_exactly_50ms_and_foot_fk_cannot_extend_it(self):
        h, rows = self.source([0, .02, .12, .122, .16, .16])
        for i, t in enumerate((0, 10_000_000, 60_000_000, 61_000_000, 80_000_000, 90_000_000)):
            rows[i]["t_ns"] = t
        for i in (2, 3, 4):
            for k in (15, 20, 22, 24): rows[i]["joints"][k] = None
        _, out = self.replay(h, rows)
        state = self.gate(out[2], 15)
        self.assertEqual(state["state"], "predicted")
        self.assertAlmostEqual(state["age_s"], .05, places=8)
        self.assertFalse(state["rejected"])
        self.assertAlmostEqual(self.point(out[2], 15)[0], .22, places=6)
        for i in (3, 4):
            self.assertEqual(self.gate(out[i], 15)["state"], "unavailable")
            self.assertIsNone(self.point(out[i], 15))
            self.assertIsNone(out[i]["candidates"][GUARDED][8], "expired ankle revived by foot FK")
            self.assertIsNotNone(out[i]["candidates"]["distance_5mm"][8], "control did not exercise seeded foot FK")

    def test_toe_only_expiry_disables_foot_even_with_measured_ankle(self):
        h, rows = self.source([0]*12)
        for i in range(3, 11): rows[i]["joints"][20] = None
        _, out = self.replay(h, rows)
        for i in range(7, 11):
            self.assertEqual(self.gate(out[i], 15)["state"], "accepted")
            self.assertEqual(self.gate(out[i], 20)["state"], "unavailable")
            self.assertIsNone(out[i]["candidates"][GUARDED][8])
            self.assertIsNotNone(out[i]["candidates"]["distance_5mm"][8])
        self.assertIsNotNone(out[11]["candidates"][GUARDED][8])

    def test_repeated_inconsistent_spikes_expire_and_return_to_old_trajectory(self):
        h, rows = self.source([0]*17)
        for i in range(4, 14): rows[i]["joints"][13][1] += .4 if i % 2 == 0 else -.4
        _, out = self.replay(h, rows)
        for i in range(8, 14):
            self.assertEqual(self.gate(out[i], 13)["state"], "unavailable")
            self.assertTrue(self.gate(out[i], 13)["rejected"])
            self.assertFalse(self.gate(out[i], 13)["reacquired"])
            self.assertIsNone(out[i]["candidates"][GUARDED][4])
        self.assertEqual(self.gate(out[14], 13)["state"], "accepted")

    def test_normal_start_stop_and_high_speed_do_not_require_confirmation(self):
        h, rows = dataset()
        selected = [r for r in rows if r["segment"] in ("normal_start_stop", "normal_high_speed")]
        _, out = self.replay(h, selected)
        for s, o in zip(selected, out):
            for k in (13, 15, 20):
                self.assertEqual(self.gate(o, k)["state"], "accepted")
                self.assertFalse(self.gate(o, k)["reacquired"])
                self.assertFalse(self.gate(o, k)["rejected"])
                self.assertLess(norm(sub(self.point(o, k)[:3], s["joints"][k][:3])), 2e-7)

    def test_small_real_reversal_is_not_rejected_by_noisy_forecast(self):
        h, rows = self.source([0, .05, 0, .04, 0])
        _, out = self.replay(h, rows)
        for i, row in enumerate(out):
            for k in (13, 15, 20):
                self.assertEqual(self.gate(row, k)["state"], "accepted")
                self.assertFalse(self.gate(row, k)["rejected"])
                self.assertLess(norm(sub(self.point(row, k)[:3], rows[i]["joints"][k][:3])), 2e-7)

    def test_prefix_teacher_independence_and_boundary_reset(self):
        h, rows = self.source([0]*4+[.3]*8)
        rows[7]["state"] = "boundary"
        rows[7]["joints"] = rows[7]["reference_joints"] = None
        _, out = self.replay(h, rows)
        _, prefix = self.replay(h, rows[:6])
        def numerical(row): return {k: v for k, v in row.items() if k != "processing_ms"}
        self.assertEqual([numerical(r) for r in prefix], [numerical(r) for r in out[:6]])
        self.assertEqual(numerical(self.replay(h, rows[8:])[1][0]), numerical(out[8]))
        self.assertIsNone(out[7]["diagnostics"])
        self.assertTrue(all(t is None for t in out[7]["candidates"][GUARDED]))
        self.assertEqual(self.gate(out[8], 13)["state"], "accepted")
        self.assertFalse(self.gate(out[8], 13)["reacquired"])
        changed = copy.deepcopy(rows)
        for r in changed:
            if r["reference_joints"]:
                for j in r["reference_joints"]:
                    if j is not None: j[0] += 1
        _, teacher = self.replay(h, changed)
        for a, b in zip(out, teacher):
            for key in ("candidates", "diagnostics", "constraint_metrics"):
                self.assertEqual(a[key], b[key])

    def test_gap_reset_and_diagnostics_on_off_preserve_outputs(self):
        h, rows = self.source([0]*4+[.3]*4)
        rows[4]["t_ns"] += 700_000_000
        for i in range(5, len(rows)): rows[i]["t_ns"] += 700_000_000
        _, out = self.replay(h, rows)
        self.assertEqual(self.gate(out[4], 13)["state"], "accepted")
        _, plain = self.replay(h, rows, diagnostic=False)
        for a, b in zip(out, plain):
            self.assertEqual({k: v for k, v in a.items() if k not in ("processing_ms", "diagnostics")},
                             {k: v for k, v in b.items() if k != "processing_ms"})

    def test_score_keeps_missing_step_pair_denominators(self):
        h, rows = dataset(hz=20)
        oh, out = self.replay(h, rows)
        report = score_controls(h, rows, oh, out)
        d = next(x for x in report["details"] if x["segment"] == "missing_left_foot"
                 and x["candidate"] == GUARDED and x["role"] == "left_foot")
        self.assertEqual(d["eligible_reports"], 40)
        self.assertEqual(d["eligible_transitions"], 39)
        self.assertLess(d["valid_transition_pairs"], d["eligible_transitions"])
        self.assertLess(d["valid_reports"], d["eligible_reports"])
        gates = [x for x in report["joint_states"] if x["segment"] == "missing_left_foot" and x["joint"] == 15]
        self.assertEqual(sum(gates[0][n] for n in ("accepted", "predicted", "unavailable")), 40)

    def test_runner_produces_bound_artifacts_and_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root/"run"
            command = [sys.executable, str(Path(__file__).with_name("outlier_controls.py")),
                       os.environ["FITRA_TEMPORAL_REPLAY"], str(output)]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            complete = json.loads((output/"complete.json").read_text())
            self.assertEqual(complete["reports"], 3*60*len(cases()))
            self.assertTrue(complete["existing_ten_candidates_unchanged"])
            self.assertFalse(complete["runtime_applied"])
            for filename in ("input.jsonl", "control.jsonl", "replay.jsonl", "report.json", "report.md", "manifest.json"):
                self.assertIn(filename, complete["artifacts"])
            frozen = (output/"complete.json").read_bytes()
            again = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(again.returncode, 2)
            self.assertEqual((output/"complete.json").read_bytes(), frozen)

    def test_external_raw_input_is_copied_byte_exact_without_synthetic_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, output = root/"raw input.jsonl", root/"external-run"
            header, rows = self.source([0]*4)
            header["reference_kind"] = "none"
            for r in rows: r.pop("reference_joints")
            original = "\n".join(json.dumps(r, indent=None) for r in [header, *rows])+"\n"
            source.write_text(original)
            result = subprocess.run([sys.executable, str(Path(__file__).with_name("outlier_controls.py")),
                os.environ["FITRA_TEMPORAL_REPLAY"], str(output), "--input", str(source)],
                capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(source.read_bytes(), (output/"input.jsonl").read_bytes())
            self.assertEqual(source.read_text(), original)
            self.assertFalse((output/"synthetic-profile.json").exists())
            manifest = json.loads((output/"manifest.json").read_text())
            self.assertEqual(manifest["source_kind"], "external_raw_input")
            self.assertNotIn("synthetic_profile", manifest["assets"])
            report = json.loads((output/"report.json").read_text())
            self.assertEqual(report["reference_kind"], "none")
            self.assertIn("保存済みraw入力", (output/"report.md").read_text())
            self.assertTrue(all(d["reference_pairs"] == 0 and d["same_time_reference_rmse_mm"] is None
                                for d in report["details"]))

    def test_cli_rejects_missing_prerequisites_and_duplicate_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for flags in (("--outlier-experiments",),
                          ("--ik-experiments", "--isolate-head", "--outlier-experiments"),
                          (*FLAGS, "--outlier-experiments", "--outlier-experiments")):
                result = subprocess.run([os.environ["FITRA_TEMPORAL_REPLAY"], str(root/"unused"), str(root/"out"), *flags],
                                        capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("--outlier-experiments", result.stderr)
                self.assertFalse((root/"out").exists())


class OutlierControlFixtureTests(unittest.TestCase):
    def test_fixture_is_deterministic_and_preserves_teacher(self):
        header, rows = dataset(hz=20)
        self.assertEqual((header, rows), dataset(hz=20))
        self.assertEqual(len(rows), len(cases())*60)
        for row in rows:
            if row["segment"].startswith(("knee_", "ankle_", "toe_")) or row["segment"] in ("swapped_knees", "persistent_inconsistent_spikes", "missing_left_foot"):
                self.assertEqual(row["reference_joints"], pose())
        for hz in (True, 0, 19, 121, math.nan):
            with self.assertRaises(ValueError): dataset(hz)

    def test_diagnostic_validator_rejects_lost_provenance_and_expired_predictions(self):
        p = pose()
        state = [{"state": "accepted" if j is not None else "unavailable", "rejected": False,
                  "reacquired": False, "age_s": 0 if j is not None else None} for j in p]
        diagnostic = {"post_kalman": p, "post_ik": p, "pre_smoothing": [None]*10,
                      "post_gate": p, "joint_gate": state}
        validate_gate(diagnostic)
        for field, value in (("state", "unknown"), ("age_s", .1), ("rejected", 1)):
            bad = copy.deepcopy(diagnostic)
            bad["joint_gate"][13][field] = value
            with self.assertRaises(ValueError): validate_gate(bad)
        bad = copy.deepcopy(diagnostic)
        bad["joint_gate"][13].update({"state": "predicted", "age_s": .02})
        with self.assertRaises(ValueError): validate_gate(bad)  # score remains a measured .95
        bad["post_gate"][13][3] = 0
        validate_gate(bad)
        bad["joint_gate"][13]["age_s"] = .051
        with self.assertRaises(ValueError): validate_gate(bad)


if __name__ == "__main__": unittest.main()
