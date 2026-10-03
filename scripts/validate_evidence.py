#!/usr/bin/env python3
"""Check committed evidence against raw samples, not just the report's prose."""
import ast
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def main():
    names=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],
                                  cwd=ROOT).decode('utf-8').split('\0')
    assert all(name.isascii() for name in names), 'Non-ASCII filename found'
    for path in [ROOT/'README.md', *list((ROOT/'docs').rglob('*.md'))]:
        assert '\ufffd' not in path.read_text(encoding='utf-8'), f'Replacement character in {path}'
    for base in [ROOT/'scripts', ROOT/'src']:
        for path in base.rglob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    for path in (ROOT/'src').rglob('*.xml'):
        ET.parse(path)
    results={}
    for file in sorted((ROOT/'docs/evidence').glob('*/data/metrics.json')):
        directory=file.parent
        metrics=json.loads(file.read_text())
        provenance=json.loads((directory.parent/'provenance.json').read_text())
        assert provenance['git_status']=='', f'Dirty tested source: {file}'
        assert metrics['goal_messages']==1
        assert np.linalg.norm(np.array(metrics['start_xy'])-[.9,.9])<.03
        assert np.linalg.norm(np.array(metrics['goal_xy'])-[14.1,14.1])<1e-6
        data=np.genfromtxt(directory/'samples.csv',delimiter=',',names=True)
        assert len(data)==metrics['sample_count']
        final=np.array([data['x_m'][-1],data['y_m'][-1]])
        assert np.linalg.norm(final-np.array(metrics['final_xy']))<1e-9
        assert abs(np.linalg.norm(final-[14.1,14.1])-metrics['final_error_m'])<1e-9
        command_limit=2.0 if provenance.get('arguments',{}).get('fast_mpc',False) else 1.25
        assert np.max(np.hypot(data['cmd_x_mps'],data['cmd_y_mps']))<=command_limit+1e-6
        if metrics['result']=='PASS':
            assert metrics['action_status']==4 and metrics['final_error_m']<.05
            assert metrics['final_speed_mps']<.08 and metrics['action_time_s']>0
        reference=np.loadtxt(directory/'reference_path.csv',delimiter=',',skiprows=1)
        a=reference[:-1]; delta=np.diff(reference,axis=0)
        denominator=np.maximum(np.sum(delta*delta,axis=1),1e-12)
        valid=np.isfinite(data['initial_path_error_m'])
        errors=[]
        for x,y,stored in zip(data['x_m'][valid],data['y_m'][valid],data['initial_path_error_m'][valid]):
            point=np.array([x,y]); t=np.sum((point-a)*delta,axis=1)/denominator
            projection=a+np.clip(t,0,1)[:,None]*delta
            error=float(np.min(np.linalg.norm(projection-point,axis=1)))
            assert abs(error-stored)<1e-8
            errors.append(error)
        assert abs(np.sqrt(np.mean(np.square(errors)))-metrics['tracking_rmse_m'])<1e-8
        results[directory.parent.name]=metrics['result']
    expected_trials={
        'final_visible_mpc_rviz_01',
        'final_visible_pid_rviz_02',
        'final_visible_sampling_rviz_01',
        'final_visible_fast_mpc_rviz_01',
    }
    assert set(results)==expected_trials, (
        f'Expected exactly the four final RViz trials, got {sorted(results)}'
    )
    assert set(results.values())=={'PASS'}, 'Every retained final RViz trial must pass'
    print(json.dumps({'result':'PASS','evidence_trials':results,
                      'checks':['ASCII filenames','UTF-8 documentation','Python AST','XML parse',
                                'clean tested revisions','one goal and fixed endpoints','command limits',
                                'arrival criteria','raw trajectory and RMSE recomputation']},indent=2))


if __name__=='__main__':
    main()
