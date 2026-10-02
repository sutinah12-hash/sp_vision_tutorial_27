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
