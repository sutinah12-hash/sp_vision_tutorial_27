#!/usr/bin/env python3
"""Read-only timing of the official costmap sampling function; no ROS messages."""
import time
from types import SimpleNamespace
from pathlib import Path
import numpy as np
from sp_nav_sim.sim_robot_node import SimRobotNode
from sp_nav_sim.sim.sim_map import load_sim_map

root = Path(__file__).resolve().parents[1]
_, occ, meta, _ = load_sim_map(str(root / 'src/sp_nav_bringup/map/maze_map.yaml'))
fixture = SimpleNamespace(map_res=meta.resolution, map_meta=meta,
                          map_h=occ.shape[0], map_w=occ.shape[1], map_occ=occ)
print('numpy:', np.__version__, 'timer_budget_ms:', 1000 / 30)
for size in [58, 80]:
    samples = []
    for _ in range(15):
        begin = time.perf_counter()
        SimRobotNode._sample_obstacle_mask(fixture, 0.0, 0.0, size, size)
        samples.append((time.perf_counter() - begin) * 1000)
    print('grid_size:', size, 'median_ms:', float(np.median(samples)),
          'sampling_ms:', samples)
