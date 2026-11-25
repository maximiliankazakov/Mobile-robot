"""Independent monitor of live ROS estimates and Webots reference. No publishers."""
from collections import Counter
from functools import partial
import json
import math
import os
from pathlib import Path
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data

OUT = HANDOFF = Path(os.environ['G1_RUN_DIR'])
ROUTE = json.loads(Path('/opt/g1/data/route.json').read_text())
TX, TY, YAW = ROUTE['candidate_map_from_world_xy_yaw']
assert YAW == 0, 'The registered transform for this demonstration is a translation'


def stamp(t):
    return t.sec + t.nanosec / 1e9


class Observe(Node):
    def __init__(self):
        super().__init__('g1_live_evaluator')
        self.estimates = {'A': [], 'B': [], 'reference': []}
        self.reasons = {'A': Counter(), 'B': Counter()}
        self.diagnostics = {'A': [], 'B': []}
        self.clock = None
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/g1/a/pose', partial(self.pose, 'A'), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/pose', partial(self.pose, 'B'), 10)
        self.create_subscription(Odometry, '/evaluation/ground_truth', partial(self.pose, 'reference'), 10)
        self.create_subscription(DiagnosticArray, '/g1/a/diagnostics', partial(self.diag, 'A'), 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', partial(self.diag, 'B'), 10)

    def on_clock(self, message):
        self.clock = stamp(message.clock)

    def pose(self, method, message):
        p = message.pose.pose
        if method != 'reference' and (message.header.frame_id != 'map' or not all(math.isfinite(v) for v in message.pose.covariance)):
            raise ValueError(f'invalid pose or covariance from {method}')
        offset = (TX, TY) if method == 'reference' else (0, 0)
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                         1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        position = [stamp(message.header.stamp), p.position.x + offset[0], p.position.y + offset[1], yaw]
        if not all(math.isfinite(v) for v in position):
            raise ValueError(f'nonfinite pose from {method}')
        rows = self.estimates[method]
        if not rows or position[0] > rows[-1][0]:
            rows.append(position)

    def diag(self, method, message):
        for status in message.status:
            if status.name == 'g1_localization':
                self.reasons[method][status.message] += 1
                self.diagnostics[method].append([stamp(message.header.stamp), status.message])

    def status(self):
        result = {'sim_s': self.clock, 'methods': {}}
        for method in ('A', 'B'):
            rows = self.estimates[method]
            result['methods'][method] = {
                'pose': rows[-1][1:] if rows else None,
                'accepted': self.reasons[method]['accepted'],
                'fallback': self.reasons[method]['accepted_fixed_fallback'],
            }
        temporary = OUT / '.status.next'
        temporary.write_text(json.dumps(result), encoding='utf-8')
        temporary.replace(OUT / 'status.json')



rclpy.init()
node = Observe()
try:
    start = update = time.monotonic()
    while time.monotonic() - start < 170:
        rclpy.spin_once(node, timeout_sec=.025)
        if time.monotonic() - update >= .15:
            node.status()
            update = time.monotonic()
        if (HANDOFF / 'finished').exists():
            node.status()
            break
finally:
    result = {'pose_columns': ['sim_s', 'x_map_m', 'y_map_m', 'yaw_rad'],
              'tracks': node.estimates, 'icp': node.reasons, 'diagnostics': node.diagnostics}
    (OUT / 'live-poses.json').write_text(json.dumps(result) + '\n')
    node.destroy_node()
    rclpy.try_shutdown()
