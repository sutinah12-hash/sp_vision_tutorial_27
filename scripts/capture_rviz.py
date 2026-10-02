#!/usr/bin/env python3
"""Capture an actual visible X11 RViz window; never render or invent a UI image."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window-id', required=True, help='RViz id shown by wmctrl -l')
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    if args.out.exists():
        parser.error('Choose a new output filename to preserve existing evidence.')
    windows = subprocess.check_output(['wmctrl', '-l'], text=True)
    match = [line for line in windows.splitlines()
             if int(line.split()[0], 16) == int(args.window_id, 0)]
    if not match or 'rviz' not in match[0].lower():
        parser.error('The requested id is not an RViz window.')
    subprocess.run(['wmctrl', '-ia', args.window_id], check=True)
    time.sleep(.4)
    geometry = subprocess.check_output(['xdotool', 'getwindowgeometry', '--shell', args.window_id], text=True)
    values = dict(line.split('=', 1) for line in geometry.splitlines() if '=' in line)
    x, y, w, h = [int(values[key]) for key in ['X', 'Y', 'WIDTH', 'HEIGHT']]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    # Root-window XGetImage fails on Wayland/Xwayland. Read this specific X11
    # window instead; ffmpeg decodes the actual XWD pixels into a PNG.
    with tempfile.TemporaryDirectory(prefix='nav_rviz_capture_') as temporary:
        dump=Path(temporary)/'window.xwd'
        subprocess.run(['xwd', '-silent', '-id', args.window_id, '-out', str(dump)], check=True)
        subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-n', '-i', str(dump),
                        '-frames:v', '1', str(args.out)], check=True)
    args.out.with_suffix('.json').write_text(json.dumps({
        'captured_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'window': match[0], 'geometry': values,
        'method': 'Native XWD capture of RViz window, lossless PNG conversion; no synthetic content',
    }, indent=2) + '\n')
    print(args.out.resolve())


if __name__ == '__main__':
    main()
