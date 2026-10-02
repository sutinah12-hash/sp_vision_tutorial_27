#!/usr/bin/env python3
"""Identify the unmodified official velocity plant offline; never drive a ROS robot."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from scipy.optimize import curve_fit

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/sp_nav_sim'))
from sp_nav_sim.sim._dyn import VelocityFilter

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--out', required=True)
args = parser.parse_args()
out = Path(args.out)
out.mkdir(parents=True, exist_ok=False)
plant = VelocityFilter(1 / 30)
t = np.arange(1, 121) / 30
y = np.array([plant.step(0.5, 0, 0, 0, 0, 2, 2)[0] for _ in t])
def model(t, delay, tau):
    return 0.5 * (1 - np.exp(-np.maximum(t - delay, 0) / tau))
fit, _ = curve_fit(model, t, y, p0=[0.18, 0.28], bounds=([0, 0.01], [1, 1]))
metrics = dict(delay_s=float(fit[0]), tau_s=float(fit[1]),
               fit_rmse_mps=float(np.sqrt(np.mean((model(t, *fit) - y) ** 2))),
               command_mps=0.5, sample_rate_hz=30,
               note='Offline step-response fit, fixed yaw. Full rotating-base navigation is tested separately.')
(out / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
np.savetxt(out / 'step_response.csv', np.c_[t, y, model(t, *fit)], delimiter=',',
           header='time_s,actual_mps,fit_mps', comments='')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.plot(t, y, label='Unmodified simulator plant')
plt.plot(t, model(t, *fit), '--', label='Delay + first-order fit')
plt.xlabel('Time [s]'); plt.ylabel('Speed [m/s]'); plt.legend(); plt.grid(alpha=.2)
plt.tight_layout(); plt.savefig(out / 'step_response.png', dpi=150)
print(json.dumps(metrics, indent=2))
