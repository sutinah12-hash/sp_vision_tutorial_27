#!/usr/bin/env python3
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("summary", ROOT / "tools/summarize_log.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class LogSummaryTests(unittest.TestCase):
    def test_old_and_new_logs_and_partial_write(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "telemetry.jsonl"
            rows = [
                {"event": "metadata", "enemy_color": "red"},
                {"event": "frame", "reason": "no_valid_observation", "detections": 1, "valid_poses": 0},
                {"event": "frame", "reason": "follow_only", "diagnostic": "follow_only", "control": 1,
                 "valid_poses": 1, "enemy_valid_poses": 1, "frame_age_ms": 25},
                {"event": "camera_timeout", "reason": "camera_timeout", "control": 0},
            ]
            path.write_text("\n".join(json.dumps(row) for row in rows) + '\n{"partial":', encoding="utf-8")
            report = MODULE.summarize(MODULE.select_log(temporary))
            self.assertEqual(report["frames"], 2)
            self.assertEqual(report["events"], 3)
            self.assertEqual(report["malformed_lines"], 1)
            self.assertEqual(report["metrics"]["valid_poses"]["positive"], 1)
            self.assertEqual(report["metrics"]["valid_poses"]["mean"], .5)
            self.assertEqual(report["metrics"]["enemy_valid_poses"]["samples"], 1)
            self.assertEqual(report["diagnostics"]["not_recorded"], 2)

    def test_missing_log_does_not_succeed(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(ValueError):
                MODULE.select_log(temporary)

    def test_red_profiles_preserve_original_calibration(self):
        # Compare every non-comment line; no YAML dependency needed in a source checkout.
        def settings(name):
            return [line.strip() for line in (ROOT / "configs" / name).read_text(encoding="utf-8").splitlines()
                    if line.strip() and not line.lstrip().startswith("#")]
        for profile in ("standard", "sentry"):
            original = settings(profile + ".yaml")
            red = settings(profile + "_red.yaml")
            expected = ['enemy_color: "red"' if line.startswith("enemy_color:") else line for line in original]
            self.assertEqual(red, expected)


if __name__ == "__main__":
    unittest.main()
