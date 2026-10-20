#!/usr/bin/env python3
"""Optional desktop test: local video only, never a camera or serial port."""

import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--video", required=True, type=Path)
    parser.add_argument("--output", type=Path, default=Path("artifacts/preview_check"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(str(args.video.resolve()))
    ok, original = capture.read()
    capture.release()
    if not ok:
        raise RuntimeError("Cannot read the first video frame")
    reports = []
    for width, height in ((960, 600), (800, 500)):
        size = f"{width}x{height}"
        run_output = output / size
        with (output / f"{size}.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen([str(args.binary.resolve()), "configs/offline_cpu.yaml", "--video",
                str(args.video.resolve()), "--frames", "90", "--show", "--window-size", size,
                "--output", str(run_output)], cwd=root, stdout=log, stderr=subprocess.STDOUT,
                env={**os.environ, "GDK_BACKEND": "x11"})
            geometry = None
            try:
                deadline = time.monotonic() + 15
                while time.monotonic() < deadline and process.poll() is None:
                    result = subprocess.run(["xwininfo", "-name", "Final Project Aim"],
                        capture_output=True, text=True, timeout=2)
                    match = re.search(r"Width: (\d+)\s+Height: (\d+)", result.stdout)
                    if match:
                        geometry = list(map(int, match.groups()))
                        if geometry == [width, height]:
                            break
                    time.sleep(.1)
                code = process.wait(timeout=30)
                if code != 0 or geometry != [width, height]:
                    raise RuntimeError(f"{size}: exit={code}, window={geometry}; see {log.name}")
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
        snapshots = sorted(run_output.glob("task_1_*/frame_000000.png"))
        if not snapshots:
            raise RuntimeError("No first-frame PNG was saved")
        canvas = cv2.imread(str(snapshots[-1]))
        records = [json.loads(line) for line in (snapshots[-1].parent / "telemetry.jsonl").read_text().splitlines()]
        if sum(row.get("event") == "frame" for row in records) != 90:
            raise RuntimeError("Preview stopped early instead of processing all 90 frames")
        h, w = original.shape[:2]
        if canvas is None or canvas.shape != (h + 166, w, 3):
            raise RuntimeError("Preview canvas does not contain the full original image dimensions")
        image = canvas[166:]
        corners = [np.array_equal(image[y:y+4, x:x+4], original[y:y+4, x:x+4])
                   for x, y in ((0, 0), (w-4, 0), (0, h-4), (w-4, h-4))]
        if not all(corners):
            raise RuntimeError("Image corners changed; inspect potential overlay/crop before approving")
        reports.append({"requested_window": [width, height], "actual_x11_window": geometry,
                        "original_image": [w, h], "canvas": [w, h+166],
                        "four_corners_pixel_equal": corners, "preview_png": str(snapshots[-1]),
                        "exit_code": code, "input": "offline recording; not hardware"})
    report = {"passed": True, "display": os.environ.get("DISPLAY"), "backend": "GTK3/X11", "runs": reports}
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
