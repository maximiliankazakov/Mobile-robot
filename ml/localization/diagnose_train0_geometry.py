"""Compare sealed train_0 startup rays to world landmarks and recorded SLAM faces.

Requires ROS 2 Jazzy Python. Reads SQLite and CDR offline, without ROS playback.
The world anchors and world-to-map transform are specific to this commissioning
capture. No estimated pose, ground truth, or fit is used in this check.
"""
import argparse
from contextlib import closing
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
import statistics

from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


ROOT = Path(__file__).resolve().parent
STARTUP_NS = (22_700_000_000, 22_800_000_000, 22_900_000_000, 23_000_000_000)
# Fixed exterior strips known from the commissioning visibility check.
WEST_WALL_BANDS_M = ((-3.45, -2.60), (2.60, 3.45))
RACK_POST_M = 0.08  # WarehouseRack.proto
EDGE_GUARD_M = 0.008
MAP_FACE_SEARCH_M = 0.22


def check_sha(path, digest):
    if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
        raise ValueError(f"SHA-256 mismatch: {path}")


def msg_stamp(msg):
    stamp = msg.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))


@dataclass(frozen=True)
class Face:
    name: str
    axis: int
    coordinate: float
    low: float
    high: float
    normal: int


def landmarks(world):
    floor = re.search(r"^  floorSize ([\d.]+) ([\d.]+)$", world, re.M)
    spawn = re.search(r"^TiagoBase \{\n  translation ([-\d.]+) ([-\d.]+) [-\d.]+", world, re.M)
    rack = next(block for block in re.findall(r"^WarehouseRack \{\n.*?^\}", world, re.M | re.S)
                if 'name "rack 2"' in block)
    x, y, _ = map(float, re.search(r"translation ([-\d.]+) ([-\d.]+) ([-\d.]+)", rack).groups())
    length, depth, _ = map(float, re.search(r"size ([\d.]+) ([\d.]+) ([\d.]+)", rack).groups())
    rot = float(re.search(r"rotation 0 0 1 ([\d.]+)", rack).group(1))
    if not math.isclose(rot, math.pi / 2, abs_tol=1e-5):
        raise ValueError("Expected near-axis-aligned rack 2 in sealed world")
    x0, y0 = map(float, spawn.groups())
    west_wall_x = -float(floor.group(1)) / 2 - x0
    faces = [Face(f"west_wall_{side}", 0, west_wall_x, low, high, 1)
             for side, (low, high) in zip(("south", "north"), WEST_WALL_BANDS_M)]
    # The west posts' face x values use the actual PROTO geometry. The
    # world writes yaw as 1.5708; snapping it to pi/2 moves these faces
    # by under 5 micrometres, far below the edge guard and grid resolution.
    front = x - x0 - depth / 2
    for group, a, b in (("south", y - y0 - length / 2, y - y0 - length / 2 + RACK_POST_M),
                        ("north", y - y0 + length / 2 - RACK_POST_M, y - y0 + length / 2)):
        faces.extend([
            Face(f"rack2_{group}_west_face", 0, front, a, b, -1),
            Face(f"rack2_{group}_south_face", 1, a, front, front + RACK_POST_M, -1),
            Face(f"rack2_{group}_north_face", 1, b, front, front + RACK_POST_M, 1),
        ])
    return faces


def first_intersection(faces, origin, direction):
    hits = []
    for face in faces:
        ray_normal = direction[face.axis]
        if ray_normal * face.normal >= 0:
            continue
        other = 1 - face.axis
        distance = (face.coordinate - origin[face.axis]) / ray_normal
        tangent = origin[other] + distance * direction[other]
        if distance > 0 and face.low <= tangent <= face.high:
            hits.append((distance, tangent, face))
    if not hits:
        return None
    closest = min(hits, key=lambda hit: hit[0])
    distance, tangent, face = closest
    if min(tangent - face.low, face.high - tangent) < EDGE_GUARD_M:
        return None
    return closest


def grid_faces(grid, expected_hash):
    if grid.header.frame_id != "map" or abs(yaw(grid.info.origin.orientation)) > 1e-9:
        raise ValueError("Expected an axis-aligned grid in map")
    if hashlib.sha256(bytes(x & 255 for x in grid.data)).hexdigest() != expected_hash:
        raise ValueError("Recorded grid differs from frozen occupancy cells")
    width, height, res = grid.info.width, grid.info.height, grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    cells = grid.data
    result = []
    for y in range(height):
        for x in range(width):
            if cells[y * width + x] < 65:
                continue
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nx, ny = x + dx, y + dy
                if not (0 <= nx < width and 0 <= ny < height and 0 <= cells[ny * width + nx] <= 25):
                    continue
                if dx:
                    result.append(Face("", 0, ox + res * (x + (dx > 0)),
                                       oy + res * y, oy + res * (y + 1), dx))
                else:
                    result.append(Face("", 1, oy + res * (y + (dy > 0)),
                                       ox + res * x, ox + res * (x + 1), dy))
    return result


def nearest_parallel(grid, reference, tangent):
    candidates = (face for face in grid if face.axis == reference.axis and
                  face.normal == reference.normal and
                  abs(face.coordinate - reference.coordinate) <= MAP_FACE_SEARCH_M and
                  face.low - 1e-8 <= tangent <= face.high + 1e-8)
    return min(candidates, default=None, key=lambda face: abs(face.coordinate - reference.coordinate))


def load_inputs(db):
    types = {"/map": OccupancyGrid, "/tf_static": TFMessage,
             "/sensors/scan": LaserScan, "/sensors/wheel_odom": Odometry}
    grids, sensors, scans, speeds = [], [], {}, {}
    uri = db.resolve().as_uri() + "?mode=ro&immutable=1"
    with closing(sqlite3.connect(uri, uri=True)) as connection:
        for topic, data in connection.execute("""
                SELECT topics.name, messages.data FROM messages
                JOIN topics ON topics.id = messages.topic_id
                WHERE topics.name IN ('/map','/tf_static','/sensors/scan','/sensors/wheel_odom')
                ORDER BY messages.timestamp, messages.id"""):
            msg = deserialize_message(data, types[topic])
            if topic == "/map":
                grids.append(msg)
            elif topic == "/tf_static":
                for tf in msg.transforms:
                    if (tf.header.frame_id, tf.child_frame_id) == ("base_link", "lidar_link"):
                        p, q = tf.transform.translation, tf.transform.rotation
                        sensors.append((p.x, p.y, yaw(q)))
            elif msg_stamp(msg) in STARTUP_NS:
                stream = scans if topic == "/sensors/scan" else speeds
                key = msg_stamp(msg)
                if key in stream:
                    raise ValueError(f"Duplicate startup sample {topic} {key}")
                stream[key] = msg
    if not grids or not sensors or any(tf != sensors[0] for tf in sensors):
        raise ValueError("Expected frozen /map and fixed base_link -> lidar_link")
    if tuple(scans) != STARTUP_NS or set(speeds) != set(STARTUP_NS):
        raise ValueError("Expected the four startup scans and wheel ticks")
    for stamp, msg in speeds.items():
        if (msg.header.frame_id, msg.child_frame_id) != ("odom", "base_link"):
            raise ValueError(f"Invalid wheel frames at {stamp}")
        if max(abs(msg.twist.twist.linear.x), abs(msg.twist.twist.angular.z)) > 0.01:
            raise ValueError(f"Robot was not stationary at {stamp}")
    return grids, sensors[0], scans


def sample_rays(faces, grid, lidar, scans):
    origin, angle = lidar[:2], lidar[2]
    for stamp, msg in scans.items():
        if msg.header.frame_id != "lidar_link":
            raise ValueError("Expected scan in lidar_link")
        for idx, observed in enumerate(msg.ranges):
            theta = angle + msg.angle_min + idx * msg.angle_increment
            direction = (math.cos(theta), math.sin(theta))
            hit = first_intersection(faces, origin, direction)
            if hit is None:
                continue
            distance, tangent, face = hit
            if not msg.range_min <= distance <= msg.range_max:
                continue
            valid = math.isfinite(observed) and msg.range_min <= observed <= msg.range_max
            parallel = nearest_parallel(grid, face, tangent)
            scan_error = face.normal * direction[face.axis] * (observed - distance) if valid else None
            map_error = face.normal * (parallel.coordinate - face.coordinate) if parallel else None
            yield dict(stamp_ns=stamp, ray_index=idx, landmark=face.name, normal_axis=face.axis,
                       normal_sign=face.normal, expected_range_m=distance,
                       observed_range_m=observed if valid else None,
                       scan_minus_world_normal_m=scan_error, map_minus_world_normal_m=map_error)


def describe(values):
    values = [v for v in values if v is not None]
    if not values:
        return {"count": 0}
    p95 = sorted(map(abs, values))[math.ceil(0.95 * len(values)) - 1]
    return {"count": len(values), "median_m": statistics.median(values), "abs_p95_m": p95}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("train0_world", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    seal = json.loads((ROOT / "train0_sealed_scoring_contract.json").read_text())
    commissioning = json.loads((ROOT / "train0_commissioning.json").read_text())
    for filename in ("metadata.yaml", "inputs_0.db3"):
        check_sha(args.bag_dir / filename, seal["sealed_capture"]["sha256"][f"inputs/{filename}"])
    check_sha(args.train0_world, commissioning["hashes"]["train_0.wbt"])
    grids, lidar, scans = load_inputs(args.bag_dir / "inputs_0.db3")
    faces = landmarks(args.train0_world.read_text())
    grid_hash = seal["inference"]["frozen_occupancy_grid_sha256"]
    grid = grid_faces(grids[0], grid_hash)
    for msg in grids[1:]:
        if msg.info != grids[0].info or msg.header.frame_id != "map" or msg.data != grids[0].data:
            raise ValueError("Map changed within the input bag")
    rows = list(sample_rays(faces, grid, lidar, scans))
    if not rows:
        raise ValueError("No landmark rays selected from the sealed inputs")
    csv_path = args.output / "landmark_rays.csv"
    json_path = args.output / "geometry.json"
    if csv_path.exists() or json_path.exists():
        raise ValueError(f"Use a fresh output directory: {args.output}")
    with csv_path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    results = {}
    for name in sorted({face.name for face in faces}):
        subset = [r for r in rows if r["landmark"] == name]
        key = "scan_minus_world_normal_m"
        map_key = "map_minus_world_normal_m"
        results[name] = {
            "predicted_ray_samples": len(subset),
            "distinct_ray_indices": len({r["ray_index"] for r in subset}),
            "raw_scan_vs_world": describe(r[key] for r in subset),
            "parallel_slam_face_vs_world": describe(r[map_key] for r in subset),
            "parallel_slam_face_vs_scan": describe(
                r[map_key] - r[key] for r in subset if r[map_key] is not None and r[key] is not None),
        }
    report = {"input": seal["sealed_capture"]["id"], "frozen_grid_sha256": grid_hash,
              "world_sha256": commissioning["hashes"]["train_0.wbt"],
              "lidar_xy_yaw_in_base": lidar, "startup_scan_stamps_ns": STARTUP_NS,
              "anchors": [vars(face) for face in faces], "landmarks": results}
    with json_path.open("x") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
