"""Generate G1 world candidates with a static spawn and route clearance check.

Does not start Webots. The reference paths are short bring-up candidates, not
the full capture matrix or permission to move.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ml/perception"))
from generate_layout import build_layout, generate, load_splits, top_level_nodes

FOOTPRINT_RADIUS_M = 0.272
CLEARANCE_MARGIN_M = 0.35
SERIALIZATION_MARGIN_M = 0.005
MIN_CLEARANCE_M = FOOTPRINT_RADIUS_M + CLEARANCE_MARGIN_M
SPAWN_GRID_M = 0.1
MAX_OFFSET_STEPS = 30
# Keep train_0's documented route. Give each other layout its own reference path.
REFERENCE_LEGS_M = {
    "train_0": (0.40, 0.32),
    "train_1": (0.50, 0.24),
    "train_2": (0.30, 0.42),
    "val_0": (0.46, 0.36),
    "test_0": (0.42, 0.30),
    "test_1": (0.34, 0.46),
}


def route_clearance(points, footprints, half_x, half_y):
    """Exact clearance of the piecewise axis-aligned route to rectangle bounds."""
    distances = []
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x0 != x1 and y0 != y1:
            raise ValueError("Reference legs must be axis aligned")
        left, right = sorted((x0, x1))
        bottom, top = sorted((y0, y1))
        distances.append(min(half_x - max(abs(left), abs(right)),
                             half_y - max(abs(bottom), abs(top))))
        for xmin, ymin, xmax, ymax in footprints:
            dx = max(xmin - right, left - xmax, 0.0)
            dy = max(ymin - top, bottom - ymax, 0.0)
            distances.append(math.hypot(dx, dy))
    return min(distances) - SERIALIZATION_MARGIN_M


def choose_spawn(nominal, legs, footprints, half_x, half_y):
    axis = range(-MAX_OFFSET_STEPS, MAX_OFFSET_STEPS + 1)
    offsets = sorted(((i, j) for i in axis for j in axis),
                     key=lambda p: (p[0] ** 2 + p[1] ** 2, p[0], p[1]))
    dx, dy = legs
    for i, j in offsets:
        x = round(nominal[0] + i * SPAWN_GRID_M, 4)
        y = round(nominal[1] + j * SPAWN_GRID_M, 4)
        route = [(x, y), (round(x + dx, 4), y), (round(x + dx, 4), round(y + dy, 4))]
        clearance = route_clearance(route, footprints, half_x, half_y)
        if clearance >= MIN_CLEARANCE_M:
            return route, clearance
    raise ValueError("No spawn and short reference route with enough static clearance")


def prepare(template, protos):
    nodes = dict(top_level_nodes(template)[1])
    arena, tiago = nodes["RectangleArena"], nodes["TiagoBase"]
    width, depth = map(float, re.search(r"floorSize ([\d.]+) ([\d.]+)", arena).groups())
    wall = float(re.search(r"wallThickness ([\d.]+)", arena).group(1))
    # Use the inset used by the layout generator as the conservative free boundary.
    half_x, half_y = width / 2 - wall / 2 - 0.05, depth / 2 - wall / 2 - 0.05
    if re.search(r"^  rotation ", tiago, re.M):
        raise ValueError("Expected the template start with yaw zero")
    origin = re.search(r"^  translation ([-\d.]+) ([-\d.]+) ([-\d.]+)$", tiago, re.M)
    nominal = tuple(map(float, origin.groups()[:2]))
    height = origin.group(3)
    generated = {}
    for split, layouts in load_splits().items():
        for name, seed in layouts.items():
            footprints = build_layout(seed, half_x, half_y).rects
            route, clearance = choose_spawn(nominal, REFERENCE_LEGS_M[name], footprints, half_x, half_y)
            x, y = route[0]
            world = generate(template, name, seed, protos)
            if (x, y) != nominal:
                new_tiago = tiago.replace(origin.group(0), f"  translation {x:.4f} {y:.4f} {height}", 1)
                if world.count(tiago) != 1:
                    raise ValueError("Expected exactly one retained TiagoBase")
                world = world.replace(tiago, new_tiago, 1)
            generated[name] = (world, {
                "layout_id": name, "layout_seed": seed,
                "split": "validation" if split == "val" else split,
                "world_sha256": hashlib.sha256(world.encode()).hexdigest(),
                "footprints_sha256": hashlib.sha256(json.dumps(sorted(footprints)).encode()).hexdigest(),
                "spawn_world_xy_yaw": [x, y, 0.0],
                "candidate_map_from_world_xy_yaw": [-x, -y, 0.0],
                "reference_route_world_xy": route,
                "clearance_lower_bound_m": clearance,
                "status": "static_candidate_pending_map_and_runtime_checks",
            })
    return generated


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="new directory for the six candidates")
    parser.add_argument("--protos-dir", default="../protos",
                        help="WarehouseRack PROTO path as seen from the generated world")
    args = parser.parse_args()
    template = (ROOT / "simulation/worlds/warehouse.wbt").read_text()
    generated = prepare(template, args.protos_dir)
    args.output.mkdir(parents=True, exist_ok=False)
    for name, (world, _) in generated.items():
        (args.output / f"{name}.wbt").write_text(world)
    (args.output / "candidate-worlds.json").write_text(json.dumps({
        "schema": "g1-capture-world-candidates-v1",
        "minimum_clearance_m": MIN_CLEARANCE_M,
        "scope": "static only; maps, routes and timestamp contract still require verification",
        "worlds": [record for _, record in generated.values()],
    }, indent=2) + "\n")


if __name__ == "__main__":
    main()
