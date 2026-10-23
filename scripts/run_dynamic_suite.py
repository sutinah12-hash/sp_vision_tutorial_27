#!/usr/bin/env python3
"""Run and record all four controller configurations sequentially in one scene."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix',type=Path,required=True,
                        help='New output prefix, e.g. results/moving')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    variants = [('mpc','mpc',[]),('pid','pid',[]),('sampling','sampling',[]),
                ('fast_mpc','mpc',['--fast-mpc'])]
    outputs = [(name,controller,extra,Path(f'{args.prefix}_{name}_rviz_01'))
               for name,controller,extra in variants]
    if any(out.exists() or Path(str(out)+'.stdout.log').exists() for _,_,_,out in outputs):
        parser.error('Choose a new prefix; previous experiments are not overwritten')
    args.prefix.parent.mkdir(parents=True,exist_ok=True)
    summary = []
    for name,controller,extra,out in outputs:
        command = [sys.executable,str(root/'scripts/run_dynamic_trial.py'),
                   '--controller',controller,'--out',str(out),*extra]
        print(f'Starting {name}: {out}',flush=True)
        with Path(str(out)+'.stdout.log').open('x') as log:
            result = subprocess.run(command,cwd=root,stdout=log,stderr=subprocess.STDOUT)
        if result.returncode not in (0,2):
            raise RuntimeError(f'{name}: automation failed ({result.returncode}); inspect {out}.stdout.log')
        metrics = json.loads((out/'data/metrics.json').read_text())
        obstacle = json.loads((out/'obstacles/obstacle_metrics.json').read_text())
        row = {'controller':name,'directory':str(out),'navigation':metrics['result'],
               'action_time_s':metrics['action_time_s'],'scene_complete':obstacle['scenario_completed']}
        summary.append(row)
        Path(f'{args.prefix}_suite_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(row),flush=True)
        time.sleep(3)


if __name__ == '__main__':
    main()
