#!/usr/bin/env python3
"""Move R2/R3 with the simulator's existing right-click target UI; record TF truth.

No simulator parameter, R1 command, R1 target, or physics state is written here.
The same position-triggered scenario is used for all four controller configurations.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as RosPath
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

SCENARIO = {
    'id': 'two_position_triggered_encounters_v2',
    'R3': {'trigger_box': [10.0, 12.2, 6.8, 7.9],
           'outbound': [8.85, 6.15], 'return': [9.4, 3.95],
           'return_when': 'R3.y >= 6.0'},
    'R2': {'trigger_box': [5.15, 6.5, 5.5, 6.7],
           'outbound': [5.8, 7.3], 'return': [3.9, 7.2],
           'clearance_waypoint': [4.8, 7.55],
           'return_when': 'R2.x >= 5.65; use clearance waypoint before returning'},
    'sample_hz': 10, 'maximum_tf_age_s': 0.5,
    'movement_speed_threshold_mps': 0.05,
    'route_incursion_distance_m': 0.35, 'encounter_distance_m': 2.0,
    'source': 'Existing SimGui right-button target; fixed official 30 Hz dynamics',
}


def in_box(xy, box):
    return box[0] <= xy[0] <= box[1] and box[2] <= xy[1] <= box[3]


def distance_to_path(point, path):
    if path is None or len(path) < 2:
        return float('nan')
    a, delta = path[:-1], np.diff(path, axis=0)
    t = np.clip(np.sum((point-a)*delta, axis=1) /
                np.maximum(np.sum(delta*delta, axis=1), 1e-12), 0, 1)
    return float(np.min(np.linalg.norm(a+t[:, None]*delta-point, axis=1)))


def find_simulator():
    windows = subprocess.check_output(['wmctrl', '-l'], text=True)
    matches = [line.split()[0] for line in windows.splitlines() if 'SP Nav Sim' in line]
    if len(matches) != 1:
        raise RuntimeError(f'Expected one simulator window, found {len(matches)}')
    return matches[0]


class Scenario(Node):
    def __init__(self, out):
        super().__init__('dynamic_obstacle_experiment')
        self.out = out
        self.out.mkdir(parents=True, exist_ok=True)
        self.start = None
        self.goal_count = 0
        self.poses = {}
        self.ages = {}
        self.phases = {2: 0, 3: 0}
        self.events = []
        self.rows = []
        self.reference = None
        self.trails = {2: [], 3: []}
        self.error = None
        self.window = None
        self.create_subscription(TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.path_sub = self.create_subscription(RosPath, '/global_path', self.on_path, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dynamic_obstacles/markers', 10)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.show_trails)
        (out/'scenario.json').write_text(json.dumps(SCENARIO, indent=2)+'\n')
        self.get_logger().info('Dynamic obstacle recorder ready; waiting for one RViz goal.')

    def on_goal(self, message):
        self.goal_count += 1
        if self.start is None:
            self.start = time.monotonic()

    def on_path(self, message):
        if self.start is not None and self.reference is None and message.poses:
            self.reference = np.array([[p.pose.position.x, p.pose.position.y] for p in message.poses])
            # The frozen path is needed only once; avoid deserializing all later replans.
            self.destroy_subscription(self.path_sub)

    def on_tf(self, message):
        now = time.monotonic()
        for transform in message.transforms:
            child = transform.child_frame_id
            if child not in ('base_link', 'base_link_2', 'base_link_3'):
                continue
            robot = 1 if child == 'base_link' else int(child[-1])
            expected = 'lidar_odom' if robot == 1 else 'map'
            if transform.header.frame_id != expected:
                self.error = f'Unexpected TF parent for {child}: {transform.header.frame_id}'
                continue
            p = transform.transform.translation
            offset = .9 if robot == 1 else 0.0
            self.poses[robot] = np.array([p.x+offset, p.y+offset])
            self.ages[robot] = now

    def set_target(self, robot, target, phase):
        if self.window is None:
            self.window = find_simulator()
        geometry = subprocess.check_output(
            ['xdotool', 'getwindowgeometry', '--shell', self.window], text=True)
        values = dict(line.split('=', 1) for line in geometry.splitlines() if '=' in line)
        if (int(values['WIDTH']), int(values['HEIGHT'])) != (1600, 1000):
            raise RuntimeError('Simulator GUI must retain its official 1600x1000 drawing area')
        nearest = min((2, 3), key=lambda k: np.linalg.norm(self.poses[k]-target))
        if nearest != robot:
            raise RuntimeError(f'Target {target} would select R{nearest}, not R{robot}')
        # Exact inverse of the original SimGui mapping (300x300, 0.05 m/cell).
        px = round(300 + target[0]/.05*(1000/300))
        py = round((299-target[1]/.05)*(1000/300))
        started = time.monotonic()
        subprocess.run(['xdotool', 'mousemove', '--window', self.window, str(px), str(py),
                        'sleep', '0.12', 'click', '3'], check=True, timeout=3)
        self.phases[robot] = phase
        event = {'time_s': started-self.start, 'robot': robot, 'phase': phase,
                 'target_requested_m': target,
                 'target_pixel_quantized_m': [(px-300)*.05/(1000/300),
                                              (299-py/(1000/300))*.05],
                 'r1_xy': self.poses[1].tolist(), 'robot_xy': self.poses[robot].tolist(),
                 'method': 'xdotool physical right click in existing simulator GUI',
                 'window_id': self.window, 'pixel_xy': [px, py]}
        self.events.append(event)
        (self.out/'obstacle_events.json').write_text(json.dumps(self.events, indent=2)+'\n')
        self.get_logger().info(f'R{robot} phase {phase}, target {target}, t={event["time_s"]:.2f}s')

    def tick(self):
        if self.start is None or len(self.poses) != 3 or self.error:
            return
        now = time.monotonic()
        ages = [now-self.ages[k] for k in (1, 2, 3)]
        if max(ages) > SCENARIO['maximum_tf_age_s']:
            return
        p1, p2, p3 = [self.poses[k] for k in (1, 2, 3)]
        self.rows.append([now-self.start, *p1, *p2, *p3,
                          np.linalg.norm(p1-p2), np.linalg.norm(p1-p3),
                          self.phases[2], self.phases[3], *ages])
        for k in (2, 3):
            if not self.trails[k] or np.linalg.norm(self.poses[k]-self.trails[k][-1]) > .02:
                self.trails[k].append(self.poses[k].copy())
        try:
            if self.phases[3] == 0 and in_box(p1, SCENARIO['R3']['trigger_box']):
                self.set_target(3, SCENARIO['R3']['outbound'], 1)
            elif self.phases[3] == 1 and p3[1] >= 6.0:
                self.set_target(3, SCENARIO['R3']['return'], 2)
            elif self.phases[3] == 2 and np.linalg.norm(p3-SCENARIO['R3']['return']) < .12:
                self.phases[3] = 3
            if self.phases[2] == 0 and self.phases[3] > 0 and in_box(p1, SCENARIO['R2']['trigger_box']):
                self.set_target(2, SCENARIO['R2']['outbound'], 1)
            elif self.phases[2] == 1 and p2[0] >= 5.65:
                self.set_target(2, SCENARIO['R2']['clearance_waypoint'], 2)
            elif self.phases[2] == 2 and np.linalg.norm(p2-SCENARIO['R2']['clearance_waypoint']) < .15:
                self.set_target(2, SCENARIO['R2']['return'], 3)
            elif self.phases[2] == 3 and np.linalg.norm(p2-SCENARIO['R2']['return']) < .12:
                self.phases[2] = 4
        except Exception as exc:
            self.error = str(exc)
            self.get_logger().error(self.error)

    def show_trails(self):
        if not rclpy.ok() or self.start is None:
            return
        markers = MarkerArray()
        for k, rgb in [(2, (.1, 1., .25)), (3, (1., .25, .15))]:
            if len(self.trails[k]) < 2:
                continue
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'experiment_obstacle_tracks'
            marker.id = k
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.
            marker.scale.x = .045
            marker.color.r, marker.color.g, marker.color.b = rgb
            marker.color.a = 1.
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=.12) for p in self.trails[k]]
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def save(self):
        header = ['time_s','r1_x_m','r1_y_m','r2_x_m','r2_y_m','r3_x_m','r3_y_m',
                  'r1_r2_center_distance_m','r1_r3_center_distance_m','r2_phase','r3_phase',
                  'r1_tf_age_s','r2_tf_age_s','r3_tf_age_s']
        with (self.out/'obstacle_samples.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(self.rows)
        if self.reference is not None:
            np.savetxt(self.out/'obstacle_initial_path.csv', self.reference, delimiter=',',
                       header='x_m,y_m', comments='')
        data = np.array(self.rows)
        result = {'scenario_id': SCENARIO['id'], 'goal_messages': self.goal_count,
                  'error': self.error, 'sample_count': len(self.rows), 'robots': {},
                  'position_source': 'Simulator TF (no Gaussian noise); R1 translated to map by (0.9,0.9)',
                  'separation_note': 'Sampled center distances, not collision counts. '
                    'Square chassis is 0.50 m per side; 0.707107 m is the orientation-independent '
                    'non-overlap bound. A smaller distance alone does not establish a collision.'}
        if len(data) >= 2:
            dt = np.diff(data[:, 0])
            result['max_sample_gap_s'] = float(np.max(dt))
            result['max_tf_age_s'] = float(np.max(data[:, 11:14]))
            for k in (2, 3):
                xy = data[:, 2*k-1:2*k+1]
                distance = data[:, k+5]
                speed = np.r_[0., np.linalg.norm(np.diff(xy, axis=0), axis=1)/dt]
                path_distance = np.array([distance_to_path(p, self.reference) for p in xy])
                moving = speed > SCENARIO['movement_speed_threshold_mps']
                crossing = moving & (path_distance <= SCENARIO['route_incursion_distance_m'])
                encounter = crossing & (distance <= SCENARIO['encounter_distance_m'])
                closest = int(np.argmin(distance))
                result['robots'][f'R{k}'] = {
                    'start_xy': xy[0].tolist(), 'final_xy': xy[-1].tolist(),
                    'travel_distance_m': float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()),
                    'max_displacement_m': float(np.linalg.norm(xy-xy[0], axis=1).max()),
                    'minimum_r1_center_distance_m': float(distance[closest]),
                    'minimum_moving_r1_center_distance_m': float(distance[moving].min()) if moving.any() else None,
                    'closest_time_s': float(data[closest, 0]),
                    'moving_route_incursion_samples': int(crossing.sum()),
                    'near_r1_moving_incursion_samples': int(encounter.sum()),
                    'first_near_r1_incursion_time_s': float(data[encounter, 0][0]) if encounter.any() else None,
                    'final_phase': self.phases[k],
                }
        result['scenario_completed'] = (self.error is None and self.goal_count == 1 and
            len(result['robots']) == 2 and all(
                row['max_displacement_m'] >= 1.0 and row['near_r1_moving_incursion_samples'] >= 1 and
                row['final_phase'] == (4 if name == 'R2' else 3)
                for name, row in result['robots'].items()))
        (self.out/'obstacle_metrics.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Scenario(args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
