#!/usr/bin/env python3
"""Remove one sensor over a fixed simulation-time window from a G1 input bag."""

import argparse
from collections import Counter
from decimal import Decimal, InvalidOperation
import json
from pathlib import Path
import tempfile

from export_bag import has_sealed_pilot_storage, inventory, marked_pilot


TOPIC_TYPES = {
    '/clock': 'rosgraph_msgs/msg/Clock',
    '/map': 'nav_msgs/msg/OccupancyGrid',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
    '/sensors/wheel_odom': 'nav_msgs/msg/Odometry',
    '/sensors/imu': 'sensor_msgs/msg/Imu',
    '/sensors/scan': 'sensor_msgs/msg/LaserScan',
}
TARGETS = {'imu': '/sensors/imu', 'lidar': '/sensors/scan'}
NS = 1_000_000_000


def nanoseconds(value):
    try:
        seconds = Decimal(value)
        ns = seconds * NS
        if seconds.is_finite() and 0 <= ns == ns.to_integral_value():
            return int(ns)
    except InvalidOperation:
        pass
    raise argparse.ArgumentTypeError('Use non-negative simulation seconds with nanosecond precision')


def make_dropout(source, destination, sensor, start_ns, end_ns):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    if not 0 <= start_ns < end_ns:
        raise ValueError('The dropout window must have positive duration')
    if destination.exists():
        raise FileExistsError(destination)
    source_hashes = inventory(source)
    if marked_pilot(source) or has_sealed_pilot_storage(source_hashes):
        raise ValueError('Commissioning bags cannot produce experiment inputs')
    target = TARGETS[sensor]
    target_type = get_message(TOPIC_TYPES[target])
    options = rosbag2_py.ConverterOptions('cdr', 'cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source), storage_id='sqlite3'), options)
    topics = reader.get_all_topics_and_types()
    if len(topics) != len(TOPIC_TYPES) or {t.name: t.type for t in topics} != TOPIC_TYPES:
        raise ValueError('Expected a G1 input-only bag with exactly the six input topics')

    with tempfile.TemporaryDirectory(prefix=f'.{destination.name}.', dir=destination.parent) as stage:
        output = Path(stage) / 'bag'
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='sqlite3'), options)
        for topic in topics:
            writer.create_topic(topic)
        kept = Counter()
        removed = []
        first_stamp = last_stamp = None
        try:
            while reader.has_next():
                name, payload, received_ns = reader.read_next()
                if name == target:
                    stamp = deserialize_message(payload, target_type).header.stamp
                    stamp_ns = stamp.sec * NS + stamp.nanosec
                    if last_stamp is not None and stamp_ns <= last_stamp:
                        raise ValueError(f'Non-increasing header stamps on {target}')
                    if first_stamp is None:
                        first_stamp = stamp_ns
                    last_stamp = stamp_ns
                    if start_ns <= stamp_ns < end_ns:
                        removed.append(stamp_ns)
                        continue
                writer.write(name, payload, received_ns)
                kept[name] += 1
        finally:
            writer.close()

        if not removed or first_stamp >= start_ns or last_stamp < end_ns:
            raise ValueError('The input must contain target messages before, inside and after the window')
        if kept.keys() != TOPIC_TYPES.keys():
            raise ValueError('The output must preserve every input stream')
        provenance = {
            'scenario': f'{sensor}_dropout',
            'topic': target,
            'window_sim_ns': [start_ns, end_ns],
            'window_end_exclusive': True,
            'removed_count': len(removed),
            'first_removed_stamp_ns': removed[0],
            'last_removed_stamp_ns': removed[-1],
            'retained_message_counts': dict(sorted(kept.items())),
            'source_bag_sha256': source_hashes,
            'output_bag_sha256': inventory(output),
        }
        (output / 'dropout.json').write_text(json.dumps(provenance, indent=2) + '\n')
        # Reserve the name so concurrent runs cannot replace each other's output.
        destination.mkdir()
        try:
            output.rename(destination)
        except OSError:
            destination.rmdir()
            raise
    return provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--sensor', choices=TARGETS, required=True)
    parser.add_argument('--start', type=nanoseconds, required=True, help='First simulation second to exclude')
    parser.add_argument('--end', type=nanoseconds, required=True, help='First simulation second to keep again')
    args = parser.parse_args()
    print(json.dumps(make_dropout(args.source, args.destination, args.sensor, args.start, args.end), indent=2))


if __name__ == '__main__':
    main()
