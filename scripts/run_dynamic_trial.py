#!/usr/bin/env python3
"""Record a full dynamic trial by clicking the real RViz fixed-goal button once."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def windows():
    listing = subprocess.run(['wmctrl', '-l'], text=True, capture_output=True)
    if listing.returncode and listing.stderr.strip():
        raise RuntimeError(listing.stderr.strip())
    lines = listing.stdout.splitlines()
    rviz = [line.split()[0] for line in lines if ' - RViz' in line]
    sim = [line.split()[0] for line in lines if 'SP Nav Sim' in line]
    return rviz, sim


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', choices=['mpc','pid','sampling'], default='mpc')
    parser.add_argument('--fast-mpc', action='store_true')
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--timeout', type=float, default=360.)
    args = parser.parse_args()
    if args.out.exists() or any(windows()):
        parser.error('Use a new output directory and stop the previous experiment first')
    root = Path(__file__).resolve().parents[1]
    command = [sys.executable, str(root/'scripts/run_trial.py'), '--controller', args.controller,
               '--dynamic-obstacles', '--out', str(args.out), '--timeout', str(args.timeout)]
    if args.fast_mpc:
        if args.controller != 'mpc':
            parser.error('Fast MPC requires the MPC controller')
        command += ['--fast-mpc', '--mpc-max-acceleration', '2.0',
                    '--mpc-braking-acceleration', '1.2', '--mpc-lateral-acceleration', '1.0']
    trial = subprocess.Popen(command, start_new_session=True)
    recording = None
    video_log = None
    try:
        deadline = time.monotonic()+60
        while time.monotonic() < deadline:
            rviz, sim = windows()
            if trial.poll() is not None:
                raise RuntimeError('Trial exited before the GUI became ready')
            recorder_ready = args.out/'recorder.log'
            obstacle_ready = args.out/'obstacles.log'
            if (len(rviz) == len(sim) == 1 and recorder_ready.exists() and obstacle_ready.exists()
                and 'Recorder ready' in recorder_ready.read_text()
                and 'recorder ready' in obstacle_ready.read_text()):
                break
            time.sleep(.5)
        else:
            raise RuntimeError('GUI/recorders not ready within 60 seconds')
        subprocess.run(['wmctrl','-ir',rviz[0],'-b','remove,maximized_vert,maximized_horz'],check=True)
        subprocess.run(['wmctrl','-ir',rviz[0],'-e','0,0,40,1600,1016'],check=True)
        # Its right edge may be off-screen; all R2/R3 target pixels remain visible.
        subprocess.run(['wmctrl','-ir',sim[0],'-e','0,1610,40,1600,1000'],check=True)
        time.sleep(2)
        subprocess.run(['wmctrl','-ia',rviz[0]],check=True)
        # Reset is RViz's display-cache reset, not a simulator reset.
        subprocess.run(['xdotool','mousemove','--window',rviz[0],'25','1002','click','1'],check=True)
        time.sleep(2)
        subprocess.run([sys.executable,str(root/'scripts/capture_rviz.py'),
                        '--window-id',rviz[0],'--out',str(args.out/'before_goal.png')],check=True)
        video_log = (args.out/'video_capture.log').open('w')
        record_start = time.monotonic()
        recording = subprocess.Popen([sys.executable,str(root/'scripts/record_rviz.py'),
            '--window-id',rviz[0],'--out',str(args.out/'rviz_one_click.mp4'),
            '--duration',str(int(args.timeout+60))],stdout=video_log,stderr=subprocess.STDOUT)
        time.sleep(2)
        if recording.poll() is not None:
            raise RuntimeError('Native video capture failed before the click')
        clicked = time.monotonic()
        subprocess.run(['xdotool','mousemove','--window',rviz[0],'270','878','click','1'],check=True)
        (args.out/'gui_trigger.json').write_text(json.dumps({
            'trigger': 'one xdotool click on the actual RViz FixedGoal panel button',
            'rviz_window':rviz[0], 'sim_window':sim[0], 'button_pixel_xy':[270,878],
            'approx_click_offset_in_video_s':clicked-record_start,
            'layout':'RViz 1600x1016 and original simulator 1600x1000 on non-overlapping areas',
            'recording':'continuous native window capture; no time stretching or frame substitution',
        },indent=2)+'\n')
        result = trial.wait(timeout=args.timeout+90)
        recording.wait(timeout=20)
        print(f'Trial exited {result}; raw video and obstacle traces: {args.out}',flush=True)
        return result
    finally:
        if trial.poll() is None:
            os.killpg(trial.pid, signal.SIGINT)
            trial.wait(timeout=40)
        if recording is not None and recording.poll() is None:
            recording.terminate()
            recording.wait(timeout=10)
        if video_log is not None:
            video_log.close()


if __name__ == '__main__':
    sys.exit(main())
