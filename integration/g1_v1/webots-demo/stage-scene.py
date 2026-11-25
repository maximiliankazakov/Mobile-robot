"""Copy the selected held-out world and its verified map and model."""
import hashlib
import json
from pathlib import Path
import re
import sys

CASE = 'g1-test_0-nominal-06'


def verified(source, expected):
    blob = source.read_bytes()
    if hashlib.sha256(blob).hexdigest() != expected:
        raise ValueError(f'Unexpected SHA-256 for {source.name}')
    return blob


def main(bundle, destination, urdf):
    index = json.loads((bundle / 'inputs/index.json').read_text())
    case = next(entry for entry in index['cases'] if entry['id'] == CASE)
    provenance = bundle / 'provenance' / CASE
    params = json.loads((provenance / 'route.json').read_text())
    destination.mkdir(parents=True, exist_ok=True)
    world = verified(provenance / 'world.wbt', params['world_sha256']).decode()
    world = world.replace('  basicTimeStep 4\n', '  basicTimeStep 4\n  FPS 20\n', 1)
    camera = 'Viewpoint {\n  orientation -0.59720087 0.30447033 0.74205723 2.41823227\n  position -6.19500000 1.13500000 3.66000000\n}'
    world, replacements = re.subn(r'Viewpoint \{[^}]+\}', camera, world, count=1)
    if replacements != 1:
        raise ValueError('The selected world must have a camera viewpoint')
    (destination / 'g1-demo.wbt').write_text(world)
    route = {key: params[key] for key in ('capture_id','world_sha256','reference_route_world_xy','candidate_map_from_world_xy_yaw')}
    (destination / 'route.json').write_text(json.dumps(route, indent=2) + '\n')
    (destination / 'map.txt').write_bytes(verified(bundle / 'inputs' / CASE / 'observed/map.txt', case['variants']['observed']['hashes']['map.txt']))
    (destination / 'model.txt').write_bytes(verified(bundle / 'models/B/model.txt', index['model_hashes']['B']))
    robot, count = re.subn(r'    <plugin type="capstone_sim_rgbd::RgbdCamera">[\s\S]*?</plugin>\n', '', urdf.read_text(), count=1)
    if count != 1:
        raise ValueError('Expected the RGB-D plugin in the stock robot description')
    (destination / 'tiago_base.urdf').write_text(robot)


if __name__ == '__main__':
    main(*map(Path, sys.argv[1:]))
