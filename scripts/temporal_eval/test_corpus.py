import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from capture_local import command
from inspect_corpus import load_session, group_frames, summary, uint


class CorpusTests(unittest.TestCase):
    def fixture(self, root):
        session = root / "ready" / "session"
        segment = session / "segments" / "000000"
        segment.mkdir(parents=True)
        rows = []
        for cam in range(3):
            (segment / f"cam{cam}.mjpeg").write_bytes(b"abcdefgh")
            for i in range(2):
                t = 2**54 + i*16_666_667 + cam*2_000_000
                rows.append({"camera": cam, "seq": str(i), "v4l2_sequence": (2**32-1+i) % 2**32,
                             "payload": f"cam{cam}.mjpeg", "offset": str(i*4), "size": 4,
                             "captured_mono_ns": str(t+1_000_000), "v4l2_mono_ns": str(t),
                             "v4l2_semantics": "monotonic_eof"})
        (segment / "frames.jsonl").write_text("".join(json.dumps(r)+"\n" for r in rows))
        sm = {"schema": "fitra_event_corpus_v1", "segment_index": 0, "frame_counts": [2,2,2]}
        (segment / "segment.json").write_text(json.dumps(sm))
        self.checksums(segment)
        m = {"schema": sm["schema"], "session_id": "session", "close_reason": "post_roll_elapsed",
             "recorder_drops": {"queue": 0, "storage": 0},
             "cameras": [{"index": c, "config_slot": c, "raw_encoding": "jpeg", "frame_count": 2} for c in range(3)]}
        (session / "manifest.json").write_text(json.dumps(m))
        return session, segment

    def checksums(self, segment):
        (segment / "checksums.sha256").write_text("".join(
            hashlib.sha256(p.read_bytes()).hexdigest()+"  "+p.name+"\n"
            for p in sorted(segment.iterdir()) if p.name != "checksums.sha256"))

    def test_verified_three_view_index_and_wrap(self):
        with tempfile.TemporaryDirectory() as temp:
            s, _ = self.fixture(Path(temp))
            m, frames, _ = load_session(s)
            groups, semantics = group_frames(frames, 5)
            self.assertEqual(len(groups), 2)
            self.assertEqual(groups[0]["frames"][0]["v4l2_mono_ns"], 2**54)
            r = summary(m, frames, groups, semantics, 5)
            self.assertEqual(r["skew_max_ms"], 4)
            self.assertEqual([c["kernel_sequence_gaps"] for c in r["cameras"]], [0,0,0])
            self.assertEqual([c["group_adoption_ratio"] for c in r["cameras"]], [1,1,1])

    def test_corruption_and_checksum_coverage(self):
        with tempfile.TemporaryDirectory() as temp:
            s, seg = self.fixture(Path(temp))
            (seg / "cam0.mjpeg").write_bytes(b"corrupt")
            with self.assertRaisesRegex(ValueError, "checksum mismatch"):
                load_session(s)
        with tempfile.TemporaryDirectory() as temp:
            s, seg = self.fixture(Path(temp))
            (seg / "unindexed.bin").write_bytes(b"x")
            with self.assertRaisesRegex(ValueError, "coverage"):
                load_session(s)

    def test_semantic_validation_even_with_valid_checksums(self):
        for field, value in (("offset", "1"), ("size", 999), ("payload", "../escape"),
                             ("seq", "0"), ("v4l2_mono_ns", "1"), ("v4l2_sequence", 5)):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as temp:
                s, seg = self.fixture(Path(temp))
                rows = [json.loads(line) for line in (seg / "frames.jsonl").read_text().splitlines()]
                # A backwards jump in the second kernel sequence, not a normal wrap.
                if field == "v4l2_sequence":
                    rows[0][field] = 20
                rows[1][field] = value
                (seg / "frames.jsonl").write_text("".join(json.dumps(r)+"\n" for r in rows))
                self.checksums(seg)
                with self.assertRaises(ValueError):
                    load_session(s)

    def test_partial_and_manifest_count_mismatch(self):
        with tempfile.TemporaryDirectory() as temp:
            s, seg = self.fixture(Path(temp))
            (s / "RECOVERED_INCOMPLETE.txt").write_text("incomplete")
            with self.assertRaises(ValueError):
                load_session(s)

    def test_manifest_counts_and_segment_index(self):
        with tempfile.TemporaryDirectory() as temp:
            s, _ = self.fixture(Path(temp))
            m = json.loads((s / "manifest.json").read_text())
            m["cameras"][0]["frame_count"] = 3
            (s / "manifest.json").write_text(json.dumps(m))
            with self.assertRaisesRegex(ValueError, "session frame count"):
                load_session(s)
        with tempfile.TemporaryDirectory() as temp:
            s, seg = self.fixture(Path(temp))
            m = json.loads((seg / "segment.json").read_text())
            m["segment_index"] = 1
            (seg / "segment.json").write_text(json.dumps(m))
            self.checksums(seg)
            with self.assertRaisesRegex(ValueError, "segment schema/index"):
                load_session(s)
        with tempfile.TemporaryDirectory() as temp:
            s, seg = self.fixture(Path(temp))
            (seg.parent / "000001.partial").mkdir()
            with self.assertRaises(ValueError):
                load_session(s)

    def test_missing_timestamp_never_uses_host_clock(self):
        with tempfile.TemporaryDirectory() as temp:
            s, _ = self.fixture(Path(temp))
            _, frames, _ = load_session(s)
            for fs in frames:
                for f in fs:
                    f["v4l2_mono_ns"] = None
                    f["v4l2_semantics"] = "unavailable"
            self.assertEqual(group_frames(frames, 15), ([], "unavailable"))

    def test_grouping_no_reuse_and_no_out_of_window_pairs(self):
        with tempfile.TemporaryDirectory() as temp:
            s, _ = self.fixture(Path(temp))
            _, frames, _ = load_session(s)
            self.assertEqual(group_frames(frames, 1)[0], [])
            groups, _ = group_frames(frames, 15)
            self.assertEqual(len({g["frames"][0]["seq"] for g in groups}), len(groups))
            frames[1][0]["v4l2_semantics"] = "monotonic_soe"
            with self.assertRaisesRegex(ValueError, "mixed"):
                group_frames(frames, 15)

    def test_uint_and_local_only_launch(self):
        self.assertEqual(uint(str(2**63)), 2**63)
        for value in (True, -1, 1.5, "1.0", "-1", str(2**64)):
            with self.assertRaises(ValueError):
                uint(value)
        cmd = command(Path("/main"), Path("/config"), Path("/new"))
        self.assertEqual(cmd[cmd.index("--corpus-remote")+1], "")
        self.assertIn("--no-vmt-out", cmd)
        self.assertIn("--no-vmt-discovery", cmd)


if __name__ == "__main__":
    unittest.main()
