#!/usr/bin/env python3
"""Launch a fresh official simulator, record one trial, then stop only our child processes."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', choices=['mpc', 'pid'], default='mpc')
    parser.add_argument('--no-smoothing', action='store_true')
    parser.add_argument('--out', required=True)
    parser.add_argument('--timeout', type=float, default=240)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--auto-goal', action='store_true')
    args = parser.parse_args()
    out = Path(args.out).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)
    workspace = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = env.get('ROS_DOMAIN_ID', '27')
    if args.headless:
        env['SDL_VIDEODRIVER'] = 'dummy'
        env['QT_QPA_PLATFORM'] = 'offscreen'
    else:
        env['QT_QPA_PLATFORM'] = 'xcb'
    launch_cmd = ['ros2', 'launch', 'sp_nav_bringup', 'project.launch.py',
                  'controller:=' + args.controller,
                  'smoothing:=' + ('false' if args.no_smoothing else 'true'),
                  'rviz:=' + ('false' if args.headless else 'true')]
    label = args.controller.upper() + (' with raw A*' if args.no_smoothing else ' with smoothed A*')
    recorder_cmd = [sys.executable, str(workspace / 'scripts/evaluate_run.py'),
                    '--out', str(out / 'data'), '--label', label, '--timeout', str(args.timeout)]
    if args.auto_goal:
        recorder_cmd.append('--send-goal')
    with (out / 'launch.log').open('w') as launch_log, (out / 'recorder.log').open('w') as recorder_log:
        launch = subprocess.Popen(launch_cmd, env=env, stdout=launch_log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        recorder = subprocess.Popen(recorder_cmd, env=env, stdout=recorder_log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        try:
            deadline = time.monotonic() + args.timeout + 90
            while recorder.poll() is None:
                if launch.poll() is not None:
                    raise RuntimeError('Navigation launch exited; inspect launch.log')
                if 'process has died' in (out / 'launch.log').read_text(errors='replace'):
                    raise RuntimeError('A navigation node died; inspect launch.log')
                if time.monotonic() > deadline:
                    raise RuntimeError('Trial deadline exceeded; inspect launch.log and recorder.log')
                time.sleep(1)
            result = recorder.returncode
        finally:
            stop(recorder)
            stop(launch)
    metrics = out / 'data' / 'metrics.json'
    print(metrics.read_text() if metrics.exists() else (out / 'recorder.log').read_text())
    return result


if __name__ == '__main__':
    sys.exit(main())
