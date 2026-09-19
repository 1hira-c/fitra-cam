import json
from pathlib import Path
import tempfile
import unittest

from run_recording import apply_labels, artifact, recording_info
from import_dump import convert


class RecordingTests(unittest.TestCase):
    def test_simcc_score_is_not_a_probability(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp) / "raw.jsonl"
            row = {"frame": 0, "kp_format": "halpe26", "persons_3d": [
                {"joints": [[0, 0, 1, 1.0332, True]] * 26}]}
            p.write_text(json.dumps(row) + "\n")
            _, rows = convert(p, 60, 1.7, "test", "unlabelled")
            self.assertEqual(rows[0]["joints"][0][3], 1.0332)
            for invalid in (-0.1, float("nan"), float("inf"), True):
                row["persons_3d"][0]["joints"][0][3] = invalid
                p.write_text(json.dumps(row) + "\n")
                with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                    convert(p, 60, 1.7, "test", "unlabelled")

    def test_camera_order_is_not_metadata_order(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)
            cameras = []
            for cam in (2, 1, 0):
                name = f"raw_cam{cam}.avi"
                (p / name).write_bytes(b"fixture-not-a-video")
                cameras.append({"file": name, "frames": 120, "fps_written": 58.81})
            (p / "meta.json").write_text(json.dumps({"cameras": cameras}))
            info = recording_info(p)
            self.assertEqual([c["id"] for c in info["selected_cameras"]], ["cam0", "cam1"])
            self.assertEqual(info["nominal_hz"], 58.81)
            self.assertEqual(info["recorded_camera_count"], 3)
            self.assertIn("not_measured_sync", info["pairing"])
            (p / "frames.csv").write_text("time")
            with self.assertRaises(ValueError):
                recording_info(p)

    def labels(self):
        return {"schema": "fitra_temporal_labels_v1", "evidence": "manual video review",
                "intervals": [{"start_frame": 1, "end_frame": 3, "phase": "static"},
                              {"start_frame": 3, "end_frame": 4, "phase": "motion", "evaluate": False}]}

    def rows(self):
        return [{"seq": i, "segment": "original", "phase": "unlabelled"} for i in range(5)]

    def test_labels_preserve_continuity_and_default_unknown(self):
        rows = self.rows()
        apply_labels(rows, self.labels())
        self.assertEqual([r["phase"] for r in rows], ["unlabelled", "static", "static", "motion", "unlabelled"])
        self.assertTrue(all(r["segment"] == "original" for r in rows))
        self.assertFalse(rows[3]["evaluate"])

    def test_reject_invalid_intervals(self):
        for changes in ({"start_frame": -1}, {"end_frame": 6}, {"end_frame": 1},
                        {"start_frame": True}, {"phase": "guess"}, {"evaluate": "false"}):
            with self.subTest(changes=changes):
                labels = self.labels()
                labels["intervals"][0].update(changes)
                with self.assertRaises(ValueError):
                    apply_labels(self.rows(), labels)
        labels = self.labels()
        labels["intervals"][1]["start_frame"] = 2
        with self.assertRaises(ValueError):
            apply_labels(self.rows(), labels)

    def test_labels_require_evidence(self):
        labels = self.labels()
        del labels["evidence"]
        with self.assertRaises(ValueError):
            apply_labels(self.rows(), labels)

    def test_streamed_artifact_hash_detects_change(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp) / "data"
            p.write_bytes(b"a")
            old = artifact(p)
            p.write_bytes(b"b")
            self.assertNotEqual(old["sha256"], artifact(p)["sha256"])


if __name__ == "__main__":
    unittest.main()
