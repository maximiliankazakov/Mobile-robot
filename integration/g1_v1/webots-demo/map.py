"""Publish the frozen map of the demonstration as a transient-local ROS map."""
from pathlib import Path
import math

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    with Path('/opt/g1/data/map.txt').open() as source:
        width, height, resolution, x, y, yaw = source.readline().split()
        cells = list(map(int, source.read().split()))
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.width, message.info.height = int(width), int(height)
    message.info.resolution = float(resolution)
    message.info.origin.position.x, message.info.origin.position.y = float(x), float(y)
    message.info.origin.orientation.z = math.sin(float(yaw) / 2)
    message.info.origin.orientation.w = math.cos(float(yaw) / 2)
    if len(cells) != message.info.width * message.info.height:
        raise ValueError('The frozen map has an invalid cell count')
    message.data = cells
    rclpy.init()
    node = rclpy.create_node('g1_frozen_map')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(OccupancyGrid, '/map', qos)
    publisher.publish(message)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
