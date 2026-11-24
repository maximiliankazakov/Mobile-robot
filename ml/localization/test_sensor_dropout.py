"""Exercise G1 dropout filtering against real SQLite bags in ROS 2 Jazzy."""

import json
from pathlib import Path
import tempfile
import unittest

import rosbag2_py
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

from make_sensor_dropout import NS, TOPIC_TYPES, make_dropout


def make_input(path, extra_truth=False):
    types = dict(TOPIC_TYPES)
    if extra_truth:
        types['/evaluation/ground_truth'] = 'nav_msgs/msg/Odometry'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for i, (name, msg_type) in enumerate(types.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(i, name, msg_type, 'cdr'))
    for header_ns in (NS - 1, NS, 2 * NS - 1, 2 * NS):
        for i, (name, msg_type) in enumerate(types.items()):
            msg = get_message(msg_type)()
            if hasattr(msg, 'header'):
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(header_ns, NS)
            # Keep bag receive times distinct from message header times.
            writer.write(name, serialize_message(msg), 100 * header_ns + i)
    writer.close()


def contents(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    records = []
    while reader.has_next():
        records.append(reader.read_next())
    return records


class SensorDropoutTest(unittest.TestCase):
    def test_only_target_messages_in_half_open_window_are_removed(self):
        for sensor, target in (('imu', '/sensors/imu'), ('lidar', '/sensors/scan')):
            with self.subTest(sensor=sensor), tempfile.TemporaryDirectory() as directory:
                source, destination = (Path(directory) / name for name in ('source', 'output'))
                make_input(source)
                original = contents(source)
                removed = [record for record in original
                           if record[0] == target and NS <= record[2] // 100 < 2 * NS]
                info = make_dropout(source, destination, sensor, NS, 2 * NS)
                self.assertEqual(info['removed_count'], 2)
                self.assertEqual(contents(destination), [record for record in original if record not in removed])
                self.assertEqual(json.loads((destination / 'dropout.json').read_text()), info)
                with self.assertRaises(FileExistsError):
                    make_dropout(source, destination, sensor, NS, 2 * NS)

    def test_incomplete_window_and_embedded_ground_truth_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            source, mixed, destination = (Path(directory) / name for name in ('source', 'mixed', 'output'))
            make_input(source)
            make_input(mixed, extra_truth=True)
            with self.assertRaisesRegex(ValueError, 'before, inside and after'):
                make_dropout(source, destination, 'imu', NS, 3 * NS)
            with self.assertRaisesRegex(ValueError, 'input-only'):
                make_dropout(mixed, destination, 'imu', NS, 2 * NS)
            self.assertFalse(destination.exists())


if __name__ == '__main__':
    unittest.main()
