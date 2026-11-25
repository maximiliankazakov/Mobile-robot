"""Waypoint fixture driven by the live G1 map pose in Webots. The G4 monitor owns /cmd_vel."""
import json
import math
import os
from pathlib import Path
import signal
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener

MAX_V = .15
MAX_W = .35
ROUTE_TOL = .18
RUN = Path(os.environ["G1_RUN_DIR"])


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def stamp(msg):
    return msg.sec + msg.nanosec * 1e-9


def require(ok, why):
    if not ok:
        raise RuntimeError(why)


def route_distance(x, y, route):
    return min(math.hypot(max(0, x-max(x0,x1), min(x0,x1)-x),
                          max(0, y-max(y0,y1), min(y0,y1)-y))
               for (x0,y0), (x1,y1) in zip(route, route[1:]))


class Drive(Node):
    def __init__(self, route):
        super().__init__("g1_capture_waypoints")
        self.route = route
        self.goal = 1
        self.turn = True
        self.cancel = False
        self.latest = {}
        self.clock = None
        self.pose = None
        self.ranges = None
        self.safety = None
        self.error = None
        self.localized = False
        self.timing = {"first_clock": None, "first_nonzero_command": None}
        self.trace = []
        self.applied = []
        self.reached = []
        self.nearest_scan = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.command = self.create_publisher(TwistStamped, "/control/cmd_vel_raw", 1)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(Imu, "/sensors/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/sensors/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "/cmd_vel", self.on_applied, 5)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diagnostics, 10)

    def sample(self, name, t):
        self.latest[name] = time.monotonic(), stamp(t)

    def on_clock(self, msg):
        if self.clock is not None and stamp(msg.clock) < stamp(self.clock):
            self.error = "clock rewind"
        self.clock = msg.clock
        if self.timing["first_clock"] is None and stamp(msg.clock) > 0:
            self.timing["first_clock"] = {"wall_s": time.time(), "sim_s": stamp(msg.clock)}
        self.sample("clock", msg.clock)

    def on_pose(self, msg):
        if msg.header.frame_id != "map":
            self.error = "incorrect G1 pose frame"
            return
        pose = msg.pose.pose
        q = pose.orientation
        self.pose = (pose.position.x, pose.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        if not all(map(math.isfinite, self.pose)):
            self.error = "nonfinite G1 pose"
        self.sample("odom", msg.header.stamp)

    def on_imu(self, msg):
        if msg.header.frame_id == "imu_link" and math.isfinite(msg.angular_velocity.z):
            self.sample("imu", msg.header.stamp)

    def on_scan(self, msg):
        if msg.header.frame_id != "lidar_link":
            self.error = "wrong LiDAR frame"
            return
        rays = [(angle(msg.angle_min+i*msg.angle_increment), r) for i,r in enumerate(msg.ranges)
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        front = [dist for yaw,dist in rays if abs(yaw) <= math.pi/4]
        if front:
            near_angle, near_range = min(rays, key=lambda ray: ray[1])
            self.ranges = (near_range, min(front))
            self.nearest_scan = {"sim_s": stamp(msg.header.stamp), "range_m": near_range, "angle_rad": near_angle}
            self.sample("scan", msg.header.stamp)

    def on_applied(self, msg):
        if msg.header.frame_id == "base_link":
            v, w = msg.twist.linear.x, msg.twist.angular.z
            if not (0 <= v <= MAX_V and abs(w) <= MAX_W):
                self.error = "G4 output exceeds guarded fixture velocity"
            self.applied.append((stamp(msg.header.stamp), v, w))
            if self.timing["first_nonzero_command"] is None and (abs(v) > 1e-6 or abs(w) > 1e-6):
                self.timing["first_nonzero_command"] = {"wall_s": time.time(), "sim_s": stamp(msg.header.stamp)}
            self.sample("applied", msg.header.stamp)

    def on_diagnostics(self, msg):
        for status in msg.status:
            if status.name == "g1_localization" and status.message == "accepted":
                self.localized = True
            if status.name == "capstone_safety/safety_monitor":
                self.safety = status.message
                self.sample("safety", msg.header.stamp)
                if self.safety in {"COLLISION_RISK", "LIMIT", "NONFINITE", "TIME_JUMP"}:
                    self.error = f"G4: {self.safety}"

    def send(self, v=0., w=0.):
        msg = TwistStamped()
        if self.clock is not None:
            msg.header.stamp = self.clock
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.command.publish(msg)

    def ready(self, now, sim):
        sources = {"clock": .35, "odom": .4, "imu": .4, "scan": .4, "applied": .4, "safety": 2.5}
        for key, age in sources.items():
            wall, source_sim = self.latest.get(key, (0, -1e9))
            if now-wall > age or abs(sim-source_sim) > age:
                return False
        return (self.localized and self.command.get_subscription_count() == 1
                and self.buffer.can_transform("map", "odom", Time())
                and self.buffer.can_transform("base_link", "lidar_link", Time()))

    def setpoint(self, x, y, yaw, sim):
        if self.goal == len(self.route):
            return 0., 0., True
        gx, gy = self.route[self.goal]
        distance = math.hypot(gx-x, gy-y)
        if distance <= .055:
            self.reached.append((self.goal, sim, x, y))
            print(f"Waypoint {self.goal} reached at t={sim:.3f}s", flush=True)
            self.goal += 1
            self.turn = True
            return 0., 0., False
        delta = angle(math.atan2(gy-y, gx-x)-yaw)
        if self.turn and abs(delta) > .08:
            return 0., max(-MAX_W, min(MAX_W, 1.4*delta)), False
        self.turn = False
        if abs(delta) > .25:
            self.turn = True
            return 0., 0., False
        return min(MAX_V, .95*distance), max(-.12, min(.12, 1.3*delta)), False

    def run(self):
        start = None
        started_at = time.monotonic()
        deadline = started_at + 72
        next_tick = started_at
        settled_at = None
        safety_stopped = None
        while time.monotonic() < deadline and not self.cancel:
            rclpy.spin_once(self, timeout_sec=.015)
            now = time.monotonic()
            if now < next_tick:
                continue
            next_tick = now + .048
            require(self.error is None, self.error)
            publishers = [p.node_name for p in self.get_publishers_info_by_topic("/cmd_vel")]
            require(not publishers or publishers == ["safety_monitor"], f"/cmd_vel writers {publishers}")
            require(self.count_publishers("/clock") <= 1, "competing clocks")
            if start is None and self.clock is not None:
                self.send()
            if self.clock is None or not self.ready(now, stamp(self.clock)):
                require(start is None and now-started_at < 35, "missing or stale control input")
                continue
            sim = stamp(self.clock)
            if start is None:
                if self.safety != "OK" or not publishers:
                    require(now-started_at < 35, "G4 failed to acknowledge initial stop")
                    continue
                require(math.hypot(*self.pose[:2]) < .09 and abs(self.pose[2]) < .08, "unexpected start pose")
                start = sim
                self.timing["ready"] = {"wall_s": time.time(), "sim_s": sim}
                (RUN / "mission-started").touch()
                print(f"Live route started using G1 map pose at t={sim:.3f}s", flush=True)
            require(sim-start <= 35, "route simulation deadline")
            x, y, yaw = self.pose
            require(route_distance(x, y, self.route) <= ROUTE_TOL, "outside route corridor")
            near, front = self.ranges
            require(near >= .45, f"near field obstacle: {self.nearest_scan}")
            if self.safety != "OK":
                self.send()
                safety_stopped = now if safety_stopped is None else safety_stopped
                require(now-safety_stopped < 1.25, f"G4 did not resume: {self.safety}")
                continue
            safety_stopped = None
            v, w, finished = self.setpoint(x, y, yaw, sim)
            require(not v or front >= .65, "frontal laser clearance")
            self.send(v, w)
            self.trace.append((sim, x, y, yaw, self.goal, v, w, near, front))
            if finished:
                settled_at = sim if settled_at is None else settled_at
                if sim-settled_at >= 1.5:
                    self.timing["settled"] = {"wall_s": time.time(), "sim_s": sim}
                    return
            else:
                settled_at = None
        raise RuntimeError("route cancelled or wall-time limit")


def main():
    waypoints = json.loads(Path("/opt/g1/data/route.json").read_text())["reference_route_world_xy"]
    ox, oy = waypoints[0]
    route = [(x-ox, y-oy) for x, y in waypoints]
    require(len(route) == 3 and route[0] != route[-1], "two distinct capture legs required")
    rclpy.init()
    node = Drive(route)
    complete = False
    reason = None
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: setattr(node, "cancel", True))
    try:
        node.run()
        complete = True
    except Exception as exc:
        reason = str(exc)
        raise
    finally:
        for _ in range(9):
            node.send()
            time.sleep(.05)
        (RUN / "drive.json").write_text(json.dumps(dict(complete=complete, failure=reason, local_route=route,
            columns=["sim_s","x","y","yaw","goal","raw_v","raw_w","nearest_range","front_range"],
            reached=node.reached, samples=node.trace, applied=node.applied, timing=node.timing, last_scan=node.nearest_scan))+"\n")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
