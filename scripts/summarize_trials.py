#!/usr/bin/env python3
"""Summarize recorded trials, including failures; never synthesize trajectory data."""
import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np


def path_metrics(path):
    points = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
    edges = np.diff(points, axis=0)
    edges = edges[np.linalg.norm(edges, axis=1) > 1e-8]
    lengths = np.linalg.norm(edges, axis=1)
    if len(edges) < 2:
        return {'length_m': float(lengths.sum()), 'sum_squared_turn_rad2': 0.0}
    tangents = edges / lengths[:, None]
    angles = np.arccos(np.clip(np.sum(tangents[:-1] * tangents[1:], axis=1), -1, 1))
    return {'length_m': float(lengths.sum()), 'sum_squared_turn_rad2': float((angles**2).sum()),
            'max_turn_rad': float(angles.max()), 'point_count': len(points)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials', nargs='+', type=Path)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    groups = defaultdict(list)
    trials = []
    for directory in args.trials:
        metrics = json.loads((directory / 'data/metrics.json').read_text())
        provenance = json.loads((directory / 'provenance.json').read_text())
        record = dict(name=directory.name, provenance=provenance, metrics=metrics)
        for kind in ['reference_path', 'raw_path']:
            path = directory / 'data' / (kind + '.csv')
            if path.exists():
                record[kind + '_shape'] = path_metrics(path)
        trials.append(record)
        groups[metrics['label']].append(metrics)
    fields = ['action_time_s', 'final_error_m', 'tracking_rmse_m', 'controller_p95_ms',
              'minimum_wall_clearance_m', 'odom_observed_hz']
    aggregates = {}
    for label, runs in groups.items():
        entry = {'trials': len(runs), 'passed': sum(r['result'] == 'PASS' for r in runs)}
        for field in fields:
            values = [r[field] for r in runs if r['result'] == 'PASS' and r[field] is not None]
            entry[field] = {'mean': float(np.mean(values)), 'std': float(np.std(values)),
                            'min': float(np.min(values)), 'max': float(np.max(values))} if values else None
        aggregates[label] = entry
    result = {'groups': aggregates, 'trials': trials,
              'note': 'Statistics include successful trials only; pass counts include every supplied trial. '
                      'Small sample; VM scheduling varies. Never treat these as universal benchmarks.'}
    (args.out / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    labels = list(aggregates)
    ticks = [label.replace(' with ', '\n') +
             f"\nPASS {aggregates[label]['passed']}/{aggregates[label]['trials']}" for label in labels]
    for ax, field, title, unit, factor in zip(axes, fields[:1]+[fields[2], fields[3]],
            ['Goal-to-action time', 'Path tracking RMSE', 'Control compute P95'],
            ['Time [s]', 'Distance [cm]', 'Compute time [ms]'], [1, 100, 1]):
        for index, label in enumerate(labels):
            stats = aggregates[label][field]
            if stats is None:
                continue
            mean, std = stats['mean']*factor, stats['std']*factor
            ax.bar(index, mean, color='#2276a5', alpha=.8)
            ax.errorbar(index, mean, yerr=std, color='#333333', capsize=4, fmt='none')
            ax.annotate(f'{mean:.3g}', (index, mean+std), xytext=(0, 5),
                        textcoords='offset points', ha='center', fontsize=9)
        ax.set_xticks(range(len(labels)), ticks, fontsize=8)
        ax.set(title=title, ylabel=unit, xlabel='Controller / smoothing setting')
        ax.set_ylim(bottom=0, top=ax.get_ylim()[1]*1.18)
        ax.grid(axis='y', alpha=.2)
    fig.suptitle('Recorded ROS2 trials: mean and population standard deviation (successful runs)')
    fig.text(.5, .015, 'Source: per-trial metrics.json and raw samples, 2026-10-02, VMware Ubuntu 22.04. '
             'Same fixed start/goal and official simulation parameters.', ha='center', fontsize=8)
    fig.tight_layout(rect=(0, .055, 1, .95))
    fig.savefig(args.out / 'comparison.png', dpi=160)
    plt.close(fig)
    print(json.dumps(aggregates, indent=2))


if __name__ == '__main__':
    main()
