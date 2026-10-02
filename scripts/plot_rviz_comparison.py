#!/usr/bin/env python3
"""Build the comparison figure from recorded RViz trial metrics."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--trial', action='append', nargs=2, metavar=('LABEL', 'DIRECTORY'), required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()

    rows = []
    for label, directory in args.trial:
        directory = Path(directory)
        metrics = json.loads((directory / 'data' / 'metrics.json').read_text())
        provenance = json.loads((directory / 'provenance.json').read_text())
        rows.append((label, metrics, provenance))

    args.out.mkdir(parents=True, exist_ok=True)
    summary = {
        label: {
            'result': metrics['result'],
            'trigger': metrics['trigger'],
            'action_time_s': metrics['action_time_s'],
            'time_or_timeout_s': (
                metrics['action_time_s']
                if metrics['action_time_s'] is not None
                else provenance['arguments']['timeout']
            ),
            'final_error_cm': 100.0 * metrics['final_error_m'],
            'tracking_rmse_cm': 100.0 * metrics['tracking_rmse_m'],
            'minimum_wall_clearance_m': metrics['minimum_wall_clearance_m'],
            'max_speed_mps': metrics['max_speed_mps'],
        }
        for label, metrics, provenance in rows
    }
    (args.out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')

    labels = [label for label, _, _ in rows]
    panels = [
        ('Arrival time or timeout (s)', 'time_or_timeout_s', False),
        ('Tracking RMSE (cm)', 'tracking_rmse_cm', False),
        ('Final error (cm, log scale)', 'final_error_cm', True),
        ('Minimum wall clearance (m)', 'minimum_wall_clearance_m', False),
    ]
    figure, axes = plt.subplots(2, 2, figsize=(11, 7.2))
    colors = ['#c0504d' if summary[label]['result'] != 'PASS' else '#4472c4' for label in labels]
    for axis, (title, key, logarithmic) in zip(axes.flat, panels):
        values = [summary[label][key] for label in labels]
        bars = axis.bar(labels, values, color=colors)
        axis.set_title(title)
        if logarithmic:
            axis.set_yscale('log')
        axis.grid(axis='y', alpha=0.25)
        axis.tick_params(axis='x', rotation=12)
        for bar, label, value in zip(bars, labels, values):
            suffix = ' (FAIL)' if summary[label]['result'] != 'PASS' else ''
            axis.text(bar.get_x() + bar.get_width() / 2, value,
                      f'{value:.2f}{suffix}', ha='center', va='bottom', fontsize=9)
    figure.suptitle('Organizer-aligned RViz trials: same map, robots, start and goal')
    figure.tight_layout()
    figure.savefig(args.out / 'comparison.png', dpi=180)


if __name__ == '__main__':
    main()
