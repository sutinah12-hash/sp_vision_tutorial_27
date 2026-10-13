#!/usr/bin/env python3
"""Receive real simulator Plotter UDP JSON on loopback; never open robot hardware."""

import argparse
from datetime import datetime, timezone
import errno
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time


ROOT = Path(__file__).resolve().parents[1]
ADDRESS = ("127.0.0.1", 9870)
COMMAND_FIELDS = ("cmd_yaw", "cmd_pitch")
ESTIMATE_FIELDS = ("estimated_omega",)
TRUTH_FIELDS = ("truth_omega",)
FEEDBACK_FIELDS = ("simulated_feedback_yaw", "simulated_feedback_pitch")
REQUIRED_FIELDS = ("time",) + COMMAND_FIELDS + ESTIMATE_FIELDS + TRUTH_FIELDS + FEEDBACK_FIELDS


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def reject_nonfinite_json(value):
    raise ValueError(f"Non-standard JSON constant: {value}")


def receive_plotter(binary):
    """Bind before launching our child; never reuse a port or terminate another process."""
    samples = []
    ignored_other_streams = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            receiver.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        try:
            receiver.bind(ADDRESS)
        except OSError as error:
            if error.errno in (errno.EADDRINUSE, errno.EACCES) or getattr(error, "winerror", None) in (10013, 10048):
                raise RuntimeError(
                    "Cannot exclusively bind UDP 127.0.0.1:9870: the port is occupied or denied. "
                    "No existing process was stopped; no simulator was launched."
                ) from error
            raise RuntimeError(f"Cannot bind UDP 127.0.0.1:9870: {error}") from error
        receiver.settimeout(0.05)
        with tempfile.TemporaryDirectory(prefix="aim-plotter-test-") as temporary:
            command = [str(binary), "--config", str(ROOT / "sim/camera.yaml"),
                       "--scenario", "follow", "--duration", "3", "--seed", "71",
                       "--plot", "--output", str(Path(temporary) / "sim")]
            with tempfile.TemporaryFile(mode="w+b") as output:
                process = subprocess.Popen(command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT)
                deadline = time.monotonic() + 60.0
                finished = None
                try:
                    while True:
                        now = time.monotonic()
                        require(now < deadline, "Our simulator child exceeded the 60 s test timeout")
                        if process.poll() is not None and finished is None:
                            finished = now
                        if finished is not None and now - finished >= 0.25:
                            break
                        try:
                            payload, sender = receiver.recvfrom(65535)
                        except socket.timeout:
                            continue
                        require(sender[0] == "127.0.0.1", "Received a non-loopback datagram")
                        sample = json.loads(payload.decode("utf-8"), parse_constant=reject_nonfinite_json)
                        require(isinstance(sample, dict), "Each UDP payload must be a JSON object")
                        # Runtime replay telemetry shares this port but has no synthetic-truth
                        # or simulated-feedback labels. Ignore only that separate stream.
                        # A partially labelled simulator packet still fails the strict checks.
                        if not set(TRUTH_FIELDS + FEEDBACK_FIELDS).intersection(sample):
                            ignored_other_streams += 1
                            continue
                        for field in REQUIRED_FIELDS:
                            require(field in sample, f"Missing telemetry field: {field}; received keys: {sorted(sample)}")
                            value = sample[field]
                            require(type(value) in (int, float) and math.isfinite(value),
                                    f"Telemetry field {field} must be a finite JSON number")
                        require(not samples or sample["time"] > samples[-1]["time"],
                                "Simulation timestamps must increase, not repeat or go backwards")
                        require(0.0 <= sample["time"] <= 3.0 + 1e-8,
                                "Unexpected timestamp from another sender or run")
                        samples.append(sample)
                finally:
                    # Only this Popen child may be stopped, and only on test failure/timeout.
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait(timeout=5)
                output.seek(0)
                log = output.read().decode("utf-8", errors="replace")
                require(process.returncode == 0,
                        f"Simulator child failed with exit {process.returncode}:\n{log}")
            metrics = json.loads((Path(temporary) / "sim/metrics.json").read_text(encoding="utf-8"))
            require(metrics.get("passed") is True, "Simulator's own checks did not pass")
            require(metrics.get("fire_pulses") == 0, "The follow scenario must never emit fire pulses")
    require(len(samples) >= 20, f"Expected a telemetry stream, received only {len(samples)} datagrams")
    require(samples[-1]["time"] - samples[0]["time"] >= 2.0,
            "Received telemetry does not cover at least 2 s of the 3 s simulation")
    require(max(abs(sample["cmd_yaw"]) + abs(sample["cmd_pitch"]) for sample in samples) > 1e-3,
            "Command stream stayed at zero; the follow simulation was not exercised")
    require(set(TRUTH_FIELDS).isdisjoint(COMMAND_FIELDS + ESTIMATE_FIELDS + FEEDBACK_FIELDS),
            "Synthetic ground truth must remain explicitly separate from estimates and commands")
    return {
        "schema": "aim-plotter-loopback-test-v1", "passed": True,
        "evidence": "Real UDP reception from compiled simulator; SYNTHETIC INPUT / NOT HARDWARE RESULT",
        "listener": "127.0.0.1:9870/UDP", "scenario": "follow", "duration_s": 3.0,
        "datagrams_received": len(samples), "all_required_numbers_finite": True,
        "ignored_other_streams": ignored_other_streams,
        "timestamps_strictly_increasing": True, "simulator_checks_passed": True,
        "fire_pulses": metrics["fire_pulses"],
        "command_fields": list(COMMAND_FIELDS), "estimator_fields": list(ESTIMATE_FIELDS),
        "synthetic_truth_fields": list(TRUTH_FIELDS), "simulated_feedback_fields": list(FEEDBACK_FIELDS),
        "truth_label_separate_from_estimate": True,
        "first_sample": samples[0], "last_sample": samples[-1],
        "not_tested": ["PlotJuggler GUI rendering", "real robot telemetry", "physical actuator readiness"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", default="build/aim_simulator")
    parser.add_argument("--report", help="Optional path for the actual test-result JSON")
    options = parser.parse_args()
    binary = (ROOT / options.binary).resolve()
    report = {"schema": "aim-plotter-loopback-test-v1", "passed": False}
    try:
        require(binary.is_file(), f"Compile the simulator first: {binary}")
        report = receive_plotter(binary)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        report["error"] = str(error)
    report["test_time_utc"] = datetime.now(timezone.utc).isoformat()
    encoded = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if options.report:
        destination = (ROOT / options.report).resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
