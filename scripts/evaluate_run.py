#!/usr/bin/env python3
"""Record a real one-goal ROS2 trial. Never publishes cmd_vel or changes the simulator."""
import argparse
import csv
import json
import math
from pathlib import Path
import time
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from action_msgs.msg import GoalStatusArray
from PIL import Image
from scipy.ndimage import distance_transform_edt
from ament_index_python.packages import get_package_share_directory


def path_array(message):
    return np.array([[p.pose.position.x, p.pose.position.y] for p in message.poses], dtype=float)


def distance_to_path(point, path):
    if path is None or len(path) == 0:
        return float('nan')
    if len(path) == 1:
        return float(np.linalg.norm(point - path[0]))
    a, b = path[:-1], path[1:]
    delta = b - a
    t = np.sum((point - a) * delta, axis=1) / np.maximum(np.sum(delta * delta, axis=1), 1e-12)
    projection = a + np.clip(t, 0, 1)[:, None] * delta
    return float(np.min(np.linalg.norm(projection - point, axis=1)))


class Recorder(Node):
    def __init__(self, args):
        super().__init__('nav_trial_recorder')
        self.args = args
        self.start = None
        self.start_ros = None
        self.goal_count = 0
        self.goal = np.array([14.1, 14.1])
        self.initial_xy = None
        self.xy = None
        self.yaw = 0.0
        self.cmd = np.zeros(2)
        self.ref = self.raw_ref = self.latest_ref = None
        self.rows = []
        self.action_success_time = None
        self.action_status = 0
        self.finished = False
        self.success = False
        self.auto_sent = False
        self.born = time.monotonic()
        self.last_progress = self.born
        self.sent_trajectory = RosPath()
        self.sent_trajectory.header.frame_id = 'map'
        self.trajectory_pub = self.create_publisher(RosPath, '/executed_path', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10) if args.send_goal else None
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(RosPath, '/global_path', self.on_path, 10)
        self.create_subscription(RosPath, '/global_path_raw', self.on_raw, 10)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, 10)
        self.create_subscription(Twist, '/sentry/cmd_vel', self.on_command, 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_status, qos)
        map_file = Path(get_package_share_directory('sp_nav_bringup')) / 'map' / 'maze_map.pgm'
        self.map_image = np.flipud(np.array(Image.open(map_file)))
        self.clearance = distance_transform_edt(self.map_image > 128) * 0.05
        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info('Recorder ready. Click the fixed-goal button in RViz ONCE.')

    def on_goal(self, message):
        self.goal_count += 1
        if self.start is not None:
            self.get_logger().error('More than one goal received: trial will be marked invalid.')
            return
        self.start = time.monotonic()
        self.start_ros = self.get_clock().now().nanoseconds
        self.goal = np.array([message.pose.position.x, message.pose.position.y])
        self.initial_xy = None if self.xy is None else self.xy.copy()
        self.get_logger().info(f'Goal received: {self.goal.tolist()}, initial pose: {self.initial_xy}')

    def on_path(self, message):
        if not message.poses:
            return
        self.latest_ref = path_array(message)
        if self.start is not None and self.ref is None:
            self.ref = self.latest_ref.copy()

    def on_raw(self, message):
        if self.start is not None and self.raw_ref is None and message.poses:
            self.raw_ref = path_array(message)

    def on_command(self, message):
        # cmd_vel is in rotating base_link; express it in map coordinates for logging.
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.cmd = np.array([c * message.linear.x - s * message.linear.y,
                             s * message.linear.x + c * message.linear.y])

    def on_odom(self, message):
        # The official simulator fixes map -> lidar_odom translation at (0.9, 0.9).
        self.xy = np.array([message.pose.pose.position.x + 0.9, message.pose.pose.position.y + 0.9])
        q = message.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.start is None or self.finished:
            return
        elapsed = time.monotonic() - self.start
        vel = np.array([message.twist.twist.linear.x, message.twist.twist.linear.y])
        ix, iy = np.floor(self.xy / 0.05).astype(int)
        clearance = float(self.clearance[iy, ix]) if 0 <= ix < 300 and 0 <= iy < 300 else 0.0
        self.rows.append([elapsed, *self.xy, *vel, *self.cmd,
                          float(np.linalg.norm(self.xy - self.goal)),
                          distance_to_path(self.xy, self.ref),
                          distance_to_path(self.xy, self.raw_ref),
                          distance_to_path(self.xy, self.latest_ref), clearance])
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = message.header.stamp
        pose.pose = message.pose.pose
        pose.pose.position.x = float(self.xy[0])
        pose.pose.position.y = float(self.xy[1])
        self.sent_trajectory.poses.append(pose)
        if len(self.rows) % 5 == 0:
            self.sent_trajectory.header.stamp = message.header.stamp
            self.trajectory_pub.publish(self.sent_trajectory)

    def on_status(self, message):
        if self.start is None:
            return
        for status in message.status_list:
            stamp = status.goal_info.stamp.sec * 1000000000 + status.goal_info.stamp.nanosec
            if stamp + 100000000 < self.start_ros:
                continue
            self.action_status = status.status
            if status.status == 4 and self.action_success_time is None:
                self.action_success_time = time.monotonic() - self.start
                self.get_logger().info(f'Action SUCCEEDED after {self.action_success_time:.3f} seconds.')

    def tick(self):
        if self.finished:
            return
        if self.args.send_goal and not self.auto_sent and self.xy is not None:
            if time.monotonic() - self.born > 5 and self.goal_pub.get_subscription_count() >= 2:
                if np.linalg.norm(self.xy - [0.9, 0.9]) > 0.03:
                    raise RuntimeError('Auto test refused: robot is not at the fixed start.')
                msg = PoseStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.position.x = msg.pose.position.y = 14.1
                msg.pose.orientation.w = 1.0
                self.goal_pub.publish(msg)
                self.auto_sent = True
        if self.start is None:
            return
        elapsed = time.monotonic() - self.start
        if time.monotonic() - self.last_progress > 10:
            self.last_progress = time.monotonic()
            error = float(np.linalg.norm(self.xy - self.goal)) if self.xy is not None else -1
            self.get_logger().info(f'Progress: t={elapsed:.1f}s xy={self.xy} goal_error={error:.3f}m')
        if self.action_success_time is not None and elapsed > self.action_success_time + 4.0:
            self.finish('action_completed')
        elif elapsed > self.args.timeout:
            self.finish('timeout')

    def finish(self, reason):
        if self.finished:
            return
        self.finished = True
        out = Path(self.args.out).expanduser()
        out.mkdir(parents=True, exist_ok=False)
        columns = ['time_s', 'x_m', 'y_m', 'vx_mps', 'vy_mps', 'cmd_x_mps', 'cmd_y_mps',
                   'goal_error_m', 'initial_path_error_m', 'initial_raw_path_error_m',
                   'current_path_error_m', 'wall_clearance_m']
        with (out / 'samples.csv').open('w', newline='') as stream:
            writer = csv.writer(stream); writer.writerow(columns); writer.writerows(self.rows)
        data = np.asarray(self.rows)
        if len(data) == 0:
            raise RuntimeError('No odometry samples were recorded.')
        for name, value in [('reference_path', self.ref), ('raw_path', self.raw_ref)]:
            if value is not None:
                np.savetxt(out / f'{name}.csv', value, delimiter=',', header='x_m,y_m', comments='')
        valid_start = self.initial_xy is not None and np.linalg.norm(self.initial_xy - [0.9, 0.9]) < 0.03
        valid_goal = np.linalg.norm(self.goal - [14.1, 14.1]) < 1e-6
        end_error = float(data[-1, 7])
        final_speed = float(np.linalg.norm(data[-1, 3:5]))
        self.success = bool(self.action_status == 4 and self.goal_count == 1 and valid_start and
                            valid_goal and end_error < 0.05 and final_speed < 0.08)
        tracking = data[:, 8]
        summary = {
            'label': self.args.label, 'result': 'PASS' if self.success else 'FAIL', 'reason': reason,
            'trigger': 'single_topic_message_test' if self.args.send_goal else 'rviz_click',
            'goal_messages': self.goal_count, 'action_status': self.action_status,
            'start_xy': self.initial_xy.tolist() if self.initial_xy is not None else None,
            'goal_xy': self.goal.tolist(), 'final_xy': data[-1, 1:3].tolist(),
            'action_time_s': self.action_success_time, 'final_error_m': end_error,
            'final_speed_mps': final_speed, 'sample_count': len(data),
            'odom_observed_hz': float((len(data) - 1) / (data[-1, 0] - data[0, 0])),
            'tracking_rmse_m': float(np.sqrt(np.nanmean(tracking ** 2))),
            'tracking_mean_m': float(np.nanmean(tracking)),
            'tracking_p95_m': float(np.nanpercentile(tracking, 95)),
            'tracking_max_m': float(np.nanmax(tracking)),
            'raw_path_rmse_m': float(np.sqrt(np.nanmean(data[:, 9] ** 2))),
            'minimum_wall_clearance_m': float(data[:, 11].min()),
            'max_speed_mps': float(np.linalg.norm(data[:, 3:5], axis=1).max()),
            'distance_travelled_m': float(np.linalg.norm(np.diff(data[:, 1:3], axis=0), axis=1).sum()),
            'notes': 'Tracking error is point-to-segment distance to the FIRST global path, frozen at goal planning. Wall clearance is centre-to-occupied-pixel distance, not a simulator collision event.',
        }
        (out / 'metrics.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
        self.plot(out, data, summary)
        print(json.dumps(summary, indent=2), flush=True)

    def plot(self, out, data, summary):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        ax = axes[0]
        ax.imshow(self.map_image, origin='lower', extent=[0, 15, 0, 15], cmap='gray', vmin=0, vmax=255)
        if self.raw_ref is not None:
            ax.plot(*self.raw_ref.T, color='#e89a26', linewidth=1.3, label='Original A*')
        if self.ref is not None:
            ax.plot(*self.ref.T, color='#13a26b', linewidth=2, label='Initial reference')
        ax.plot(data[:, 1], data[:, 2], color='#177ee5', linewidth=1.4, label='Measured trajectory')
        ax.scatter([0.9, 14.1], [0.9, 14.1], c=['#13a26b', '#ed4545'], s=50, zorder=4)
        ax.set(xlabel='x [m]', ylabel='y [m]', title=self.args.label + ' - real simulation trace')
        ax.legend(loc='lower right', fontsize=8)
        ax = axes[1]
        ax.plot(data[:, 0], data[:, 8], label='Initial reference error [m]')
        ax.plot(data[:, 0], np.linalg.norm(data[:, 3:5], axis=1), label='Measured speed [m/s]', alpha=.8)
        ax.set(xlabel='Wall-clock time after goal [s]', title='Tracking error and speed')
        ax.grid(alpha=.2); ax.legend()
        action_time = summary['action_time_s']
        action_text = f'{action_time:.2f}' if action_time is not None else 'not reached'
        fig.suptitle(f"{summary['result']} | final error {summary['final_error_m']:.4f} m | "
                     f"tracking RMSE {summary['tracking_rmse_m']:.4f} m | "
                     f"action time {action_text} s")
        fig.tight_layout()
        fig.savefig(out / 'trajectory.png', dpi=160)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--label', default='MPC with smoothed A*')
    parser.add_argument('--timeout', type=float, default=240)
    parser.add_argument('--send-goal', action='store_true', help='Automated one-message integration test; final RViz test uses no flag')
    args = parser.parse_args()
    if Path(args.out).expanduser().exists():
        parser.error('Output directory already exists; choose a new trial name to preserve evidence.')
    rclpy.init()
    node = Recorder(args)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        if node.rows:
            node.finish('interrupted')
    finally:
        node.destroy_node(); rclpy.shutdown()
    return 0 if node.success else 2


if __name__ == '__main__':
    sys.exit(main())
