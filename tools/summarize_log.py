#!/usr/bin/env python3
"""Read a task log without contacting hardware. Accept a JSONL file or a run directory."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys


def select_log(path):
    path = Path(path)
    if path.is_file():
        return path
    candidates = list(path.glob("**/telemetry.jsonl")) if path.is_dir() else []
    if not candidates:
        raise ValueError(f"No telemetry.jsonl found under {path}")
    return max(candidates, key=lambda candidate: (candidate.stat().st_mtime_ns, str(candidate)))


def summarize(path):
    metadata = {}
    reasons, diagnostics, metrics = Counter(), Counter(), {}
    frames = events = malformed = 0
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError("not an object")
            except ValueError:
                malformed += 1
                continue
            if row.get("event") == "metadata":
                metadata = row
                continue
            events += 1
            frames += row.get("event", "frame") == "frame"
            reasons[str(row.get("reason", "missing"))] += 1
            diagnostics[str(row.get("diagnostic", "not_recorded"))] += 1
            for key in ("detections", "valid_poses", "enemy_valid_poses", "task_type_poses",
                        "control", "fire", "auto_mode", "imu_ok", "transport_fresh",
                        "frame_age_ms", "processing_ms"):
                value = row.get(key)
                if isinstance(value, (int, float)) and math.isfinite(value):
                    entry = metrics.setdefault(key, {"samples": 0, "positive": 0, "sum": 0.0, "max": value})
                    entry["samples"] += 1
                    entry["positive"] += value > 0
                    entry["sum"] += value
                    entry["max"] = max(entry["max"], value)
    for entry in metrics.values():
        entry["mean"] = entry.pop("sum") / entry["samples"]
    return {"log": str(Path(path).resolve()), "metadata": metadata, "frames": frames,
            "events": events, "malformed_lines": malformed, "reasons": dict(reasons),
            "diagnostics": dict(diagnostics), "metrics": metrics}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, help="File or directory; newest log selected in a directory")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    try:
        report = summarize(select_log(args.path))
    except (OSError, ValueError) as error:
        print(f"Log check failed: {error}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print(f"Log: {report['log']}")
        print("Metadata:", json.dumps(report["metadata"], ensure_ascii=False))
        print(f"Frames={report['frames']}; events={report['events']}; unreadable lines={report['malformed_lines']}")
        for field in ("reasons", "diagnostics"):
            print(field + ":")
            for name, count in sorted(report[field].items(), key=lambda item: (-item[1], item[0])):
                print(f"  {count:7d}  {name}")
        print("Metrics (positive/samples; mean; max):")
        for key, entry in report["metrics"].items():
            print(f"  {key}: {entry['positive']}/{entry['samples']}; {entry['mean']:.3f}; {entry['max']:.3f}")
        if report["diagnostics"].get("not_recorded"):
            print("Old log: missing diagnostics are UNKNOWN, not zero. Re-run the new version to collect them.")
        print("Logs show software observations/commands, not proof of physical tracking or hits.")
    return 0 if report["events"] and not report["malformed_lines"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
