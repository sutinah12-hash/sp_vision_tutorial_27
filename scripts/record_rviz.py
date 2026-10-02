#!/usr/bin/env python3
"""Record the actual X11 RViz window, including on a Wayland/Xwayland desktop."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window-id', required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--duration', type=int, default=240)
    args=parser.parse_args()
    if args.out.exists():
        parser.error('Choose a new filename; recordings are never overwritten.')
    lines=subprocess.check_output(['wmctrl','-l'],text=True).splitlines()
    windows=[line for line in lines if int(line.split()[0],16)==int(args.window_id,0)]
    if not windows or 'rviz' not in windows[0].lower():
        parser.error('The window id must identify RViz.')
    args.out.parent.mkdir(parents=True,exist_ok=True)
    subprocess.run(['wmctrl','-ia',args.window_id],check=True)
    command=['ffmpeg','-hide_banner','-loglevel','warning','-n','-f','x11grab',
             '-window_id',str(int(args.window_id,0)),'-framerate','10','-draw_mouse','1',
             '-i',os.environ.get('DISPLAY',':0'),'-t',str(args.duration),
             '-vf','pad=ceil(iw/2)*2:ceil(ih/2)*2','-c:v','libx264','-preset','ultrafast',
             '-crf','23','-threads','1','-pix_fmt','yuv420p','-movflags','+faststart',str(args.out)]
    args.out.with_suffix('.json').write_text(json.dumps({
        'started_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
        'window':windows[0],'command':command,
        'note':'Continuous native RViz window capture at 10 fps, no synthetic frames or speed-up. '
               'Capture ends when the window closes or duration is reached. No audio.',
    },indent=2)+'\n')
    subprocess.run(command,check=True)


if __name__=='__main__':
    main()
