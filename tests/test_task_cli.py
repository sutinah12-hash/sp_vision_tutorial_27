#!/usr/bin/env python3
"""Exercise fail-closed CLI paths only; never run a valid --live invocation."""

import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--build-dir", type=Path, default=Path("build"))
    parser.add_argument("--report", type=Path, default=Path("artifacts/tests/task_cli.json"))
    args = parser.parse_args()
    root = args.project_root.resolve()
    build = args.build_dir if args.build_dir.is_absolute() else root / args.build_dir
    report = args.report if args.report.is_absolute() else root / args.report
    config = root / "configs/offline_cpu.yaml"
    if not config.is_file():
        parser.error(f"CPU-only configuration is absent: {config}")
    for task in (1, 2, 3):
        if not (build / f"task_{task}").is_file():
            parser.error(f"Build task_{task} before running this test")

    records = []
    with tempfile.TemporaryDirectory(prefix="aim-cli-test-") as temporary:
        folder = Path(temporary)
        empty = folder / "empty video.avi"
        empty.write_bytes(b"")
        missing = folder / "definitely_missing_configuration.yaml"
        output = folder / "run-output"
        for task in (1, 2, 3):
            executable = str(build / f"task_{task}")
            cases = [
                ("no_arguments_help", [], 0, "Usage:"),
                ("explicit_help", ["--help"], 0, "safe default"),
                ("missing_mode", [str(config)], 1, "Choose exactly one"),
                # Even if the mode parser regresses, this nonexistent YAML prevents hardware construction.
                ("conflicting_modes", [str(missing), "--live", "--video", str(empty)], 1, "Choose exactly one"),
                ("offline_fire_forbidden", [str(config), "--video", str(empty), "--allow-fire"], 1, "--allow-fire"),
                ("missing_config_argument", ["--video", str(empty)], 1, "configuration path is required"),
                ("missing_config_file", [str(missing), "--video", str(empty)], 1, "failed safely"),
                ("empty_video", [str(config), "--video", str(empty), "--output", str(output)], 1, "Cannot decode video"),
                ("zero_frames", [str(config), "--video", str(empty), "--frames", "0"], 1, "positive integer"),
                ("negative_frames", [str(config), "--video", str(empty), "--frames", "-2"], 1, "positive integer"),
                ("nonnumeric_frame_suffix", [str(config), "--video", str(empty), "--frames", "90junk"], 1, "positive integer"),
                ("unknown_flag", [str(config), "--not-a-real-option"], 1, "Unknown option"),
            ]
            if task == 1:
                cases.append(("task1_can_never_arm", [str(missing), "--live", "--allow-fire"], 1, "--allow-fire"))
            for name, flags, expected_code, expected_text in cases:
                start = time.monotonic()
                try:
                    result = subprocess.run(
                        [executable, *flags], cwd=root, capture_output=True, text=True,
                        encoding="utf-8", errors="replace", timeout=30, check=False,
                    )
                    combined = result.stdout + result.stderr
                    passed = (
                        result.returncode == expected_code and expected_text in combined
                        and "[Gimbal]" not in combined and "HikRobot" not in combined
                    )
                    record = {
                        "task": task, "case": name, "passed": passed,
                        "returncode": result.returncode, "expected_returncode": expected_code,
                        "expected_text": expected_text, "seconds": time.monotonic() - start,
                        "output": combined[-2400:],
                    }
                except subprocess.TimeoutExpired as error:
                    record = {
                        "task": task, "case": name, "passed": False,
                        "seconds": time.monotonic() - start, "error": str(error),
                    }
                records.append(record)
                print(f"{'PASS' if record['passed'] else 'FAIL'} task_{task}: {name}", flush=True)

    passed = sum(record["passed"] for record in records)
    summary = {
        "suite": "offline_cli_fail_closed", "passed": passed, "total": len(records),
        "hardware_open_requested": False,
        "scope": "Help, invalid argument combinations, nonexistent YAML and empty local video only",
        "cases": records,
    }
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"CLI tests: {passed}/{len(records)}; report={report}")
    return 0 if passed == len(records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
