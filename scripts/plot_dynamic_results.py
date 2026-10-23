#!/usr/bin/env python3
"""Recompute moving-obstacle evidence and plot stationary/moving comparisons."""
import argparse
import json
import math
import re
from pathlib import Path

import numpy as np
from PIL import Image


def path_distance(points, path):
    a, delta = path[:-1], np.diff(path, axis=0)
    denominator = np.maximum(np.sum(delta*delta, axis=1),1e-12)
    result = []
    for point in points:
        t = np.clip(np.sum((point-a)*delta,axis=1)/denominator,0,1)
        result.append(np.min(np.linalg.norm(a+t[:,None]*delta-point,axis=1)))
    return np.array(result)


def verify_dynamic(directory):
    directory = Path(directory)
    base = directory/'obstacles'
    metrics = json.loads((base/'obstacle_metrics.json').read_text())
    scenario = json.loads((base/'scenario.json').read_text())
    events = json.loads((base/'obstacle_events.json').read_text())
    data = np.genfromtxt(base/'obstacle_samples.csv',delimiter=',',names=True)
    reference = np.loadtxt(base/'obstacle_initial_path.csv',delimiter=',',skiprows=1)
    assert len(data) == metrics['sample_count'] and len(data)>2
    assert metrics['goal_messages'] == 1 and metrics['error'] is None
    assert np.all(np.diff(data['time_s'])>0)
    assert np.all(np.diff(data['shared_tf_stamp_ns'])>0), 'Unmatched or duplicate TF frame'
    assert metrics['scenario_id'] == scenario['id']
    assert max(np.max(data[f'r{k}_tf_age_s']) for k in (1,2,3)) <= scenario['maximum_tf_age_s']
    p1 = np.column_stack([data['r1_x_m'],data['r1_y_m']])
    assert np.linalg.norm(p1[0]-[.9,.9])<.05
    for k in (2,3):
        xy = np.column_stack([data[f'r{k}_x_m'],data[f'r{k}_y_m']])
        distances = np.linalg.norm(xy-p1,axis=1)
        np.testing.assert_allclose(distances,data[f'r1_r{k}_center_distance_m'],atol=1e-9)
        speed = np.r_[0.,np.linalg.norm(np.diff(xy,axis=0),axis=1)/np.diff(data['time_s'])]
        moving = speed>scenario['movement_speed_threshold_mps']
        crossing = moving & (path_distance(xy,reference)<=scenario['route_incursion_distance_m'])
        encounter = crossing & (distances<=scenario['encounter_distance_m'])
        row = metrics['robots'][f'R{k}']
        np.testing.assert_allclose(row['minimum_r1_center_distance_m'],distances.min(),atol=1e-9)
        np.testing.assert_allclose(row['travel_distance_m'],
                                   np.linalg.norm(np.diff(xy,axis=0),axis=1).sum(),atol=1e-9)
        np.testing.assert_allclose(row['max_displacement_m'],np.linalg.norm(xy-xy[0],axis=1).max(),atol=1e-9)
        assert row['moving_route_incursion_samples'] == int(crossing.sum())
        assert row['near_r1_moving_incursion_samples'] == int(encounter.sum())
        assert row['final_phase'] == int(data[f'r{k}_phase'][-1])
        phase1 = [e for e in events if e['robot']==k and e['phase']==1]
        assert len(phase1)<=1
        if phase1:
            x,y = phase1[0]['r1_xy']; b = scenario[f'R{k}']['trigger_box']
            assert b[0]<=x<=b[1] and b[2]<=y<=b[3]
        else:
            assert row['final_phase']==0 and row['max_displacement_m']<1e-6
        for event in [e for e in events if e['robot']==k]:
            assert np.linalg.norm(np.array(event['target_pixel_quantized_m'])-
                                  event['target_requested_m'])<.011
    completed = all(row['max_displacement_m']>=1 and row['near_r1_moving_incursion_samples']>=1
                    and row['final_phase']==(4 if name=='R2' else 3)
                    for name,row in metrics['robots'].items())
    assert metrics['scenario_completed'] == completed
    return metrics,scenario,data,reference


def plot_trial(directory, workspace, label):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    obstacle,scenario,data,reference = verify_dynamic(directory)
    nav = json.loads((directory/'data/metrics.json').read_text())
    launch_log = (directory/'launch.log').read_text(errors='replace')
    recorder_log = (directory/'recorder.log').read_text(errors='replace')
    goal = re.search(r'\[([0-9]+\.[0-9]+)\].*Goal received:',recorder_log)
    aborted = re.search(r'\[([0-9]+\.[0-9]+)\].*\[NavToPose\] Action ABORTED',launch_log)
    diagnostics = {
        'action_status':nav['action_status'],
        'abort_time_after_goal_s':float(aborted[1])-float(goal[1]) if aborted and goal else None,
        'blocked_start_log_count':launch_log.count('Start is in blocked cell (cost=100)'),
        'navigation_failure_logged':'Navigation task failed.' in launch_log,
        'note':'Abort timing is derived from ROS log timestamps, not the recording timeout.',
    }
    (directory/'navigation_diagnostics.json').write_text(json.dumps(diagnostics,indent=2)+'\n')
    image = np.flipud(np.array(Image.open(workspace/'src/sp_nav_bringup/map/maze_map.pgm')))
    xy = {k:np.column_stack([data[f'r{k}_x_m'],data[f'r{k}_y_m']]) for k in (1,2,3)}
    colors = {1:'#0072b2',2:'#009e73',3:'#d55e00'}
    figure,axes = plt.subplots(2,2,figsize=(12,10))
    def draw_map(axis, limits, title):
        axis.imshow(image,cmap='gray',origin='lower',extent=[0,15,0,15],vmin=0,vmax=255)
        axis.plot(reference[:,0],reference[:,1],'--',color='#999999',lw=1,label='Initial R1 reference')
        for k in (1,2,3):
            axis.plot(xy[k][:,0],xy[k][:,1],color=colors[k],lw=1.6,label=f'R{k} measured TF')
            axis.plot(*xy[k][0],marker='o',color=colors[k],ms=5)
            axis.plot(*xy[k][-1],marker='s',color=colors[k],ms=5)
        axis.set(xlim=limits[:2],ylim=limits[2:],xlabel='x (m)',ylabel='y (m)',title=title)
        axis.set_aspect('equal')
    draw_map(axes[0,0],(0,15,0,15),'Full route and moving obstacle tracks')
    axes[0,0].legend(fontsize=8,loc='upper left')
    for k in (2,3):
        row = obstacle['robots'][f'R{k}']
        axes[0,1].plot(data['time_s'],data[f'r1_r{k}_center_distance_m'],color=colors[k],label=f'R1-R{k}')
        axes[0,1].plot(row['closest_time_s'],row['minimum_r1_center_distance_m'],'o',color=colors[k])
    axes[0,1].axhline(math.sqrt(.5),color='#666666',ls='--',label='0.707 m conservative body bound')
    axes[0,1].set(xlabel='Wall-clock time after goal (s)',ylabel='Center distance (m)',
                  title='Sampled robot separation (not a collision counter)',ylim=(0,7))
    axes[0,1].legend(fontsize=8,loc='upper right')
    axes[0,1].grid(alpha=.2)
    draw_map(axes[1,0],(8.2,10.4,3.4,7.8),'R3 oncoming / departure')
    r2_title = ('R2 crossing / return via clearance waypoint' if obstacle['robots']['R2']['final_phase']
                else 'R2 not triggered: R1 did not reach this encounter')
    draw_map(axes[1,1],(3.3,6.6,5.3,8.4),r2_title)
    for k,axis in ((3,axes[1,0]),(2,axes[1,1])):
        if obstacle['robots'][f'R{k}']['final_phase']==0:
            axis.text(.02,.98,'Not triggered\nNo obstacle motion',transform=axis.transAxes,
                      va='top',fontsize=9,bbox={'facecolor':'white','alpha':.8})
            continue
        j = np.argmin(data[f'r1_r{k}_center_distance_m'])
        pair = np.array([xy[1][j],xy[k][j]])
        axis.plot(pair[:,0],pair[:,1],':o',color='#aa00aa',lw=1,ms=4)
        axis.text(.02,.98,f'Min. center separation:\n{np.linalg.norm(pair[0]-pair[1]):.3f} m',
                  transform=axis.transAxes,va='top',fontsize=9,bbox={'facecolor':'white','alpha':.8})
    figure.suptitle(f'{label} | navigation {nav["result"]} | dynamic scene complete: {obstacle["scenario_completed"]}')
    figure.tight_layout(rect=(0,.025,1,.96))
    figure.text(.5,.01,'Dots: start; squares: final. Recorded TF positions; no synthetic movement. '
                'Paths can overlap at different times.',ha='center',fontsize=9)
    figure.savefig(directory/'dynamic_trajectories.png',dpi=150)
    plt.close(figure)
    return nav,obstacle,scenario


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--trial',action='append',nargs=3,metavar=('LABEL','STATIC_DIR','DYNAMIC_DIR'),required=True)
    parser.add_argument('--out',type=Path,required=True)
    args = parser.parse_args()
    rows = []
    scenario = None
    for label,stationary,dynamic in args.trial:
        static = json.loads((Path(stationary)/'data/metrics.json').read_text())
        nav,obstacle,current_scene = plot_trial(Path(dynamic),args.workspace,label)
        if scenario is None:
            scenario = current_scene
        assert scenario==current_scene, 'Dynamic trials must use an identical scenario'
        provenance=json.loads((Path(dynamic)/'provenance.json').read_text())
        diagnostics=json.loads((Path(dynamic)/'navigation_diagnostics.json').read_text())
        row = {'label':label,'stationary_directory':stationary,'dynamic_directory':dynamic,
               'stationary':static,'moving':nav,'obstacles':obstacle,'diagnostics':diagnostics,
               'observation_timeout_s':provenance['arguments']['timeout']}
        rows.append(row)
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
    x = np.arange(len(rows)); width = .35
    fig,axes = plt.subplots(2,2,figsize=(12,8))
    for axis,key,scale,title in [
        (axes[0,0],'action_time_s',1,'Arrival time / observation timeout (s)'),
        (axes[0,1],'final_error_m',100,'Final position error (cm, log scale)'),
        (axes[1,0],'tracking_rmse_m',100,'Distance to initial route: RMSE (cm)'),
    ]:
        for offset,group,color,label in [(-width/2,'stationary','#a9bbc9','R2/R3 stationary'),
                                         (width/2,'moving','#258c99','R2/R3 moving')]:
            values = [row[group][key]*scale if row[group][key] is not None
                      else row['observation_timeout_s'] for row in rows]
            colors = [color if row[group]['result']=='PASS' else '#c0504d' for row in rows]
            bars = axis.bar(x+offset,values,width,color=colors,label=label)
            for bar,row,value in zip(bars,rows,values):
                text = f'{value:.2f}' if row[group]['result']=='PASS' else f'{value:.2f}\nFAIL'
                axis.text(bar.get_x()+bar.get_width()/2,bar.get_height(),text,ha='center',va='bottom',fontsize=8)
        axis.set_title(title)
        axis.set_xticks(x,[row['label'] for row in rows])
        axis.grid(axis='y',alpha=.2)
        axis.margins(y=.18)
    axes[0,0].legend(fontsize=8)
    axes[0,1].set_yscale('log')
    axis=axes[1,1]
    for k,offset,color in [(2,-width/2,'#009e73'),(3,width/2,'#d55e00')]:
        values=[row['obstacles']['robots'][f'R{k}']['minimum_r1_center_distance_m']
                if row['obstacles']['robots'][f'R{k}']['final_phase'] else 0. for row in rows]
        bars=axis.bar(x+offset,values,width,color=color,label=f'R1-R{k}')
        for bar,value in zip(bars,values):
            axis.text(bar.get_x()+bar.get_width()/2,value,f'{value:.3f}' if value else 'Not triggered',
                      ha='center',va='bottom',fontsize=8,rotation=0 if value else 90)
    axis.axhline(math.sqrt(.5),ls='--',color='#666666',label='Conservative body bound')
    axis.set(title='Moving trials: minimum center separation (m)',xticks=x,xticklabels=[r['label'] for r in rows])
    axis.legend(fontsize=8)
    axis.grid(axis='y',alpha=.2)
    axis.margins(y=.2)
    axis.set_ylim(0,1.12)
    fig.suptitle('Stationary-obstacle baseline vs. moving-obstacle experiments')
    fig.tight_layout(rect=(0,.06,1,.96))
    fig.text(.5,.025,'FAIL time bar is the observation limit, not arrival time; incomplete-run RMSE is not a full-route score.\n'
             'VM/GUI load affects wall time. Below 0.707 m, center distances alone cannot certify no body contact.',
             ha='center',fontsize=9)
    fig.savefig(args.out/'comparison.png',dpi=160)
    plt.close(fig)
    print(json.dumps({'validated_trials':len(rows),'scenario':scenario['id']},indent=2))


if __name__ == '__main__':
    main()
