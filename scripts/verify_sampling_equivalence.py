#!/usr/bin/env python3
"""Compare optimized map sampling with the exact upstream implementation."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace

import numpy as np
from sp_nav_sim.sim_robot_node import SimRobotNode
from sp_nav_sim.sim.sim_map import load_sim_map, SimMapMeta

BASELINE = '05c33fc8aa6695e76bcff310b42f8fdfe276e1cb'
SIM_FILE = 'src/sp_nav_sim/sp_nav_sim/sim_robot_node.py'
ROOT = Path(__file__).resolve().parents[1]


def official_sampler():
    source = subprocess.check_output(['git', 'show', f'{BASELINE}:{SIM_FILE}'], cwd=ROOT, text=True)
    tree = ast.parse(source)
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SimRobotNode')
    function = next(n for n in node.body if isinstance(n, ast.FunctionDef)
                    and n.name == '_sample_obstacle_mask')
    module = ast.Module(body=[function], type_ignores=[])
    namespace = {'np': np}
    exec(compile(ast.fix_missing_locations(module), '<upstream_sampler>', 'exec'), namespace)
    return namespace['_sample_obstacle_mask']


def timed(function, fixture, args):
    samples = []
    for _ in range(11):
        begin = time.perf_counter()
        function(fixture, *args)
        samples.append((time.perf_counter() - begin) * 1000)
    return float(np.median(samples))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    original = official_sampler()
    optimized = SimRobotNode._sample_obstacle_mask
    _, occ, meta, _ = load_sim_map(str(ROOT / 'src/sp_nav_bringup/map/maze_map.yaml'))
    fixture = SimpleNamespace(map_res=meta.resolution, map_meta=meta, map_occ=occ,
                              map_h=occ.shape[0], map_w=occ.shape[1], robot_radius=.25,
                              margin=.05, d_safe=.70, w_pen=12.0)
    rng = np.random.default_rng(2704)
    cases = [(0., 0., 58, 58), (0., 0., 80, 80), (11., 11., 80, 80),
             (-2., -2., 80, 80), (14.8, 14.8, 80, 80), (0., 0., 0, 0)]
    cases += [(float(rng.uniform(-4, 18)), float(rng.uniform(-4, 18)),
               int(rng.integers(1, 90)), int(rng.integers(1, 90))) for _ in range(100)]
    # Boundary and half-cell rounding cases, including origins offset by one ULP.
    for origin in [0., .05, .10, .15, .90, 2.10, 10.05, 14.95]:
        for value in [origin, np.nextafter(origin, -np.inf), np.nextafter(origin, np.inf)]:
            cases.append((float(value), float(value), 80, 80))
    cells = 0
    for index, case in enumerate(cases):
        before, after = original(fixture, *case), optimized(fixture, *case)
        if not np.array_equal(before, after):
            raise AssertionError(f'Occupancy mismatch: case {index}, {case}')
        cells += before.size
        if before.size:
            # Dynamic obstacle stamping and cost conversion are unchanged; verify outputs too.
            for position in [np.array([3.90, 7.20]), np.array([9.34, 3.92])]:
                SimRobotNode._stamp_robot_disk(fixture, before, *case, position)
                SimRobotNode._stamp_robot_disk(fixture, after, *case, position)
            cost_a = SimRobotNode._obstacle_mask_to_costs(fixture, before)
            cost_b = SimRobotNode._obstacle_mask_to_costs(fixture, after)
            assert np.array_equal(cost_a, cost_b), f'Cost mismatch: case {index}'
    # The coordinate conversion must also preserve nonzero map origins and resolutions.
    for res in [.025, .1, .2]:
        fixture.map_res = res
        fixture.map_meta = SimMapMeta(res, np.array([-1.5, 2.3, 0.]), 0, .65, .196)
        for case in cases[:20]:
            before, after = original(fixture, *case), optimized(fixture, *case)
            assert np.array_equal(before, after), f'Origin/resolution mismatch: {res}, {case}'
            cells += before.size
    fixture.map_res, fixture.map_meta = meta.resolution, meta
    timing = {}
    for size in [58, 80]:
        case = (0., 0., size, size)
        old = timed(original, fixture, case)
        new = timed(optimized, fixture, case)
        timing[str(size)] = dict(original_ms=old, optimized_ms=new, speedup=old / new)
    protected = subprocess.check_output(['git', 'ls-tree', '-r', '--name-only', BASELINE,
                                        'src/sp_nav_sim', 'src/sp_nav_bringup/map',
                                        'src/sp_controller_server/include/sp_controller_server/controller_plugin.hpp'],
                                       cwd=ROOT, text=True).splitlines()
    hashes = {}
    for name in protected:
        if name == SIM_FILE:
            continue
        before = subprocess.check_output(['git', 'show', f'{BASELINE}:{name}'], cwd=ROOT)
        after = (ROOT / name).read_bytes()
        assert before == after, f'Protected file changed: {name}'
        hashes[name] = hashlib.sha256(after).hexdigest()
    report = dict(result='PASS', baseline=BASELINE, cases=len(cases) + 60,
                  cells_compared=cells, timing=timing, unchanged_file_sha256=hashes,
                  note='Only static occupancy sampling is vectorized. No caching or callback scheduling changes. Simulator parameters and dynamics are byte-identical to upstream.')
    (out / 'verification.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
