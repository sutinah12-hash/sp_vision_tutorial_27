#!/usr/bin/env python3
"""Black-box tests of the compiled C++ simulator, not a Python simulation."""
import argparse
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / "build/aim_simulator"


class SimulatorTests(unittest.TestCase):
    def invoke(self, *arguments):
        return subprocess.run([str(BINARY), "--config", str(ROOT / "sim/camera.yaml"), *arguments],
                              cwd=ROOT, capture_output=True, text=True, timeout=120, check=False)

    def test_independent_collision_scorer_and_pnp_roundtrip(self):
        process = self.invoke("--self-test")
        self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
        self.assertIn('"known_yaw_offset_misses": true', process.stdout)
        self.assertIn('"stationary_front_plate_hit": true', process.stdout)
        self.assertIn('"integration_step_refinement": true', process.stdout)
        self.assertIn('"outpost_top_panel_rejected": true', process.stdout)

    def test_seed_reproducibility_and_real_artifacts(self):
        with tempfile.TemporaryDirectory(prefix="aim-simulator-tests-") as temporary:
            outputs = []
            for name in ("first", "second"):
                folder = Path(temporary) / name
                process = self.invoke("--scenario", "follow", "--duration", "3.5", "--seed", "71",
                                      "--noise", "0.25", "--dropout", "0.02", "--output", str(folder))
                self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
                metrics = json.loads((folder / "metrics.json").read_text(encoding="utf-8"))
                self.assertTrue(metrics["passed"])
                self.assertEqual(metrics["fire_pulses"], 0)
                self.assertEqual((folder / "tracking.png").read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
                self.assertGreater((folder / "final.png").stat().st_size, 5000)
                outputs.append((metrics, (folder / "telemetry.csv").read_bytes(),
                                (folder / "final.png").read_bytes()))
            self.assertEqual(outputs[0], outputs[1], "Fixed-seed C++ execution must be deterministic")

    def test_nonfinite_noise_is_rejected(self):
        process = self.invoke("--noise", "nan")
        self.assertNotEqual(process.returncode, 0)
        self.assertIn("Invalid duration/noise/dropout/speed", process.stderr)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", default="build/aim_simulator")
    options = parser.parse_args()
    BINARY = (ROOT / options.binary).resolve()
    if not BINARY.is_file():
        parser.error(f"Compile the simulator first: {BINARY}")
    unittest.main(argv=[sys.argv[0]], verbosity=2)
