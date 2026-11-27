"""Exercise the model setup messages without installing the optional GPU backend."""

import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


def run_cli(*args, trial=None):
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    env.pop("ROMA_TRIAL", None)
    if trial is not None:
        env["ROMA_TRIAL"] = str(trial)
    script = Path(__file__).with_name("run_roma.py")
    return subprocess.run(
        [sys.executable, "-S", str(script), *args],
        env=env,
        capture_output=True,
        text=True,
    )


class ModelSetupTest(unittest.TestCase):
    def test_help_exposes_terms_before_model_setup(self):
        result = run_cli("--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("DINOv3", result.stdout)
        self.assertIn("THIRD_PARTY_NOTICES.md#roma-v2", result.stdout)

    def test_missing_local_assets_report_the_required_input(self):
        unset = run_cli("frames")
        self.assertEqual(unset.returncode, 2, unset.stderr)
        self.assertIn("set ROMA_TRIAL", unset.stderr)
        with tempfile.TemporaryDirectory() as trial:
            checkpoint = Path(trial) / "hub/checkpoints/romav2.0.1.pt"
            missing = run_cli("frames", trial=trial)
            self.assertEqual(missing.returncode, 2, missing.stderr)
            self.assertIn(str(checkpoint), missing.stderr)
            checkpoint.parent.mkdir(parents=True)
            checkpoint.touch()
            missing = run_cli("frames", trial=trial)
            self.assertEqual(missing.returncode, 2, missing.stderr)
            self.assertIn("hubconf.py", missing.stderr)


if __name__ == "__main__":
    unittest.main()
