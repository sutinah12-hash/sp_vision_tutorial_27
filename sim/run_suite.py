#!/usr/bin/env python3
"""Run the compiled C++ simulator; Python does not implement the controller.

All subprocess output and failing metrics are retained. Exit 0 means every
requested synthetic test passed, never that a hardware assessment was passed.
"""
import argparse
import csv
import hashlib
import json
import platform
import subprocess
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_digest(path):
    # Git canonicalizes tracked text to LF; hash that exact UTF-8 text while
    # keeping executable hashing byte-for-byte (never normalize a binary).
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", default="build/aim_simulator")
    parser.add_argument("--output", default="artifacts/simulation_suite")
    parser.add_argument("--seeds", default="1,7,29")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--noise", type=float, default=0.25)
    parser.add_argument("--dropout", type=float, default=0.02)
    parser.add_argument("--skip-stress", action="store_true")
    parser.add_argument("--follow-turn-only", action="store_true",
                        help="Separate full-360-degree task1 tests; use --seeds 7 for nominal + stress")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    binary = (root / args.binary).resolve()
    output = (root / args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    if not binary.is_file():
        parser.error(f"Compile the C++ aim_simulator first: {binary}")
    seeds = [int(value) for value in args.seeds.split(",")]
    cases = [("follow", 0.0, args.duration)]
    for sign in (1.0, -1.0):
        cases.extend(("low", sign * speed, args.duration) for speed in (3.0, 4.0, 5.0))
        cases.extend(("medium", sign * speed, args.duration) for speed in (5.0, 7.0, 9.0))
        cases.append(("outpost", sign * 2.5, 35.0))
        cases.append(("low_to_medium", sign * 4.0, 32.0))
    if args.follow_turn_only:
        cases = [("follow_turn", 0.0, 16.0)]
    runs = []

    def execute(scenario, speed, duration, seed, stress=False):
        name = f"{scenario}_{speed:+.1f}_seed{seed}" + ("_stress" if stress else "")
        folder = output / name
        folder.mkdir(parents=True, exist_ok=True)
        command = [str(binary), "--scenario", scenario, "--speed", str(speed),
                   "--duration", str(duration), "--seed", str(seed),
                   "--noise", str(args.noise), "--dropout", str(args.dropout),
                   "--config", str(root / "sim/camera.yaml"), "--output", str(folder)]
        if stress:
            command.append("--stress")
        record = {"name": name, "command": command, "stress": stress, "passed": False}
        with (folder / "stdout.log").open("w", encoding="utf-8") as log:
            try:
                proc = subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT,
                                      timeout=240, check=False)
                record["returncode"] = proc.returncode
            except subprocess.TimeoutExpired:
                record["error"] = "timeout after 240 seconds"
        metrics = folder / "metrics.json"
        if metrics.exists():
            record["metrics"] = json.loads(metrics.read_text(encoding="utf-8"))
            measured = record["metrics"]
            expected_shots = 20 if scenario == "low_to_medium" else 10
            quota_exercised = scenario not in ("low", "medium", "low_to_medium") or (
                measured["fire_pulses"] == expected_shots and measured["shots_confirmed"] == expected_shots)
            stage_specific_limit = True
            if scenario == "low_to_medium":
                with (folder / "telemetry.csv").open(newline="", encoding="utf-8") as stream:
                    for row in csv.DictReader(stream):
                        time = float(row["time"])
                        if time <= 2.0 or duration / 2.0 <= time < duration / 2.0 + 2.0:
                            continue
                        if row["state"] not in ("tracking", "acquiring", "temporary_loss"):
                            continue
                        limit = 0.6 if time < duration / 2.0 else 1.0
                        error = abs(float(row["estimated_omega"]) - float(row["truth_omega"]))
                        stage_specific_limit = stage_specific_limit and error <= limit
            record["suite_checks"] = {"full_task2_shot_allotment_exercised": quota_exercised,
                                      "individual_stage_speed_limit": stage_specific_limit}
            record["passed"] = (record.get("returncode") == 0 and measured["passed"] and
                                all(record["suite_checks"].values()))
        runs.append(record)
        print(f"{'PASS' if record['passed'] else 'FAIL'} {name}", flush=True)

    for seed in seeds:
        for scenario, speed, duration in cases:
            execute(scenario, speed, duration, seed)
    if not args.skip_stress:
        stress_cases = (("follow", 0.0), ("low", 4.0), ("low", -4.0),
                                ("medium", 7.0), ("medium", -7.0),
                                ("outpost", 2.5), ("outpost", -2.5))
        if args.follow_turn_only:
            stress_cases = (("follow_turn", 0.0),)
        for scenario, speed in stress_cases:
            duration = 35.0 if scenario == "outpost" else 16.0 if scenario == "follow_turn" else args.duration
            execute(scenario, speed, duration,
                    seeds[0], True)
    source_files = ["sim/aim_simulator.cpp", "sim/camera.yaml", "sim/run_suite.py",
                    "tasks/auto_aim/controller.cpp", "tasks/auto_aim/controller.hpp",
                    "tasks/auto_aim/solver.cpp", "tasks/auto_aim/target.cpp"]
    report = {
        "evidence": "SYNTHETIC INPUT / NOT HARDWARE RESULT",
        "passed": all(run["passed"] for run in runs),
        "runs_count": len(runs), "passed_count": sum(run["passed"] for run in runs),
        "host": platform.platform(), "binary_sha256": digest(binary),
        "source_sha256": {name: source_digest(root / name) for name in source_files if (root / name).exists()},
        "source_digest_encoding": "UTF-8 source bytes with CRLF canonicalized to LF; binary SHA-256 is unmodified bytes",
        "method": "C++ Solver reprojection -> pixel noise/dropout -> Solver PnP -> Controller -> delayed gimbal -> independent moving-plate ballistic collision",
        "limitations": ["No real YOLO inference", "Ideal calibrated camera and known rigid geometry",
                        "No real communication, projectile drag, motor backlash, or exposure blur",
                        "Synthetic success does not establish real-robot completion"],
        "runs": runs,
    }
    (output / "suite_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{report['passed_count']}/{len(runs)} synthetic tests passed")
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
