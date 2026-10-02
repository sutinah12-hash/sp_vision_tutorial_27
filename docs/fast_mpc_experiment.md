# Fast MPC experiment

The formal controller remains configured with a 1.25 m/s command limit. This experiment adds a launch-time override which raises only `MpcController.max_speed` to 2.0 m/s. The simulator physical limit is already 2.0 m/s; simulator parameters, map, dynamics, start, goal, planner, prediction horizon and control frequency are unchanged.

Run it in a fresh output directory:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/run_trial.py --controller mpc --fast-mpc \
  --out results/fast_mpc_01
```

The override is applied only at launch time, so it does not modify the formal `nav_params.yaml`. Compare `action_time_s`, `final_error_m`, `tracking_rmse_m`, `minimum_wall_clearance_m`, `max_speed_mps`, and the result status with the formal MPC trials. A shorter time alone is not sufficient: a run with a larger tracking error, unsafe clearance, collision, or failure should not replace the baseline.

The 2.0 m/s limit is an upper bound, not a requested constant speed. The MPC still slows for curvature, acceleration limits, delay compensation and terminal braking. The overlay should therefore be treated as a speed-limit sensitivity test rather than a guaranteed 2x speed-up.

## Parameter scan

The simulator remained unchanged (`v_max=2.0 m/s`, `a_max=2.0 m/s²`). Controller-only tests produced the following result:

- `fast_mpc_all2_01`: speed, acceleration, braking and lateral acceleration all set to 2.0. The robot stopped progressing around `(7.94, 10.49)` and the run was terminated. This combination is not used.
- `fast_mpc_refined_01`: `2.0 / 2.0 / 1.0 / 0.9` for speed, acceleration, braking and lateral acceleration. PASS in 94.11 s; final error 0.76 cm, tracking RMSE 11.62 cm, minimum clearance 0.45 m.
- `fast_mpc_balanced_01` and `_02`: `2.0 / 2.0 / 1.2 / 1.0`. Both PASS in 90.76 s and 89.24 s. Mean time 90.00 s, mean final error 1.36 cm, mean tracking RMSE 12.34 cm, minimum clearance 0.50 m in both runs.

The balanced setting is the current speed-oriented profile. It improves time relative to the formal headless MPC mean of 91.91 s, while the formal 1.25 m/s profile retains lower tracking RMSE (about 9.25 cm). Both profiles remain available because they optimize different priorities.

Reproduce the balanced profile with:

```bash
python3 scripts/run_trial.py --controller mpc --fast-mpc \
  --mpc-max-acceleration 2.0 \
  --mpc-braking-acceleration 1.2 \
  --mpc-lateral-acceleration 1.0 \
  --headless --auto-goal --out results/fast_mpc_balanced_new
```
