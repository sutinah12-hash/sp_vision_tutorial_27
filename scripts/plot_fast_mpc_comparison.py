#!/usr/bin/env python3
"""Plot a fair headless comparison between formal and speed-oriented MPC."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load(directory):
    return json.loads((Path(directory) / 'data' / 'metrics.json').read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', nargs='+', required=True)
    parser.add_argument('--fast', nargs='+', required=True)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    groups = {'Formal MPC': [load(x) for x in args.baseline],
              'Fast MPC': [load(x) for x in args.fast]}
    summary = {}
    for label, rows in groups.items():
        summary[label] = {
            'trials': len(rows),
            'passed': sum(row['result'] == 'PASS' for row in rows),
            'action_time_mean_s': float(np.mean([row['action_time_s'] for row in rows])),
            'action_time_values_s': [row['action_time_s'] for row in rows],
            'final_error_mean_cm': float(100*np.mean([row['final_error_m'] for row in rows])),
            'tracking_rmse_mean_cm': float(100*np.mean([row['tracking_rmse_m'] for row in rows])),
            'minimum_clearance_mean_m': float(np.mean([row['minimum_wall_clearance_m'] for row in rows])),
            'max_speed_mean_mps': float(np.mean([row['max_speed_mps'] for row in rows])),
        }
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    labels = list(summary)
    panels = [
        ('Arrival time (s)', 'action_time_mean_s', '#4472c4'),
        ('Tracking RMSE (cm)', 'tracking_rmse_mean_cm', '#ed7d31'),
        ('Final error (cm)', 'final_error_mean_cm', '#70ad47'),
        ('Minimum clearance (m)', 'minimum_clearance_mean_m', '#a05a9f'),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    for ax, (title, key, color) in zip(axes.flat, panels):
        values = [summary[label][key] for label in labels]
        bars = ax.bar(labels, values, color=[color, '#5b9bd5'])
        ax.set_title(title)
        ax.grid(axis='y', alpha=.25)
        for bar, value in zip(bars, values):
            ax.text(bar.get_x()+bar.get_width()/2, value, f'{value:.2f}',
                    ha='center', va='bottom', fontsize=9)
    fig.suptitle('MPC profiles — same headless simulator, map, start and goal')
    fig.tight_layout()
    fig.savefig(args.out/'comparison.png', dpi=180)


if __name__ == '__main__':
    main()
