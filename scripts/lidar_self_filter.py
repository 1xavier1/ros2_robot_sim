#!/usr/bin/env python3
"""Filter vehicle body returns and out-of-range points from LiDAR clouds."""

from math import cos, sin, sqrt
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import yaml


WORKSPACE_DIR = Path(__file__).resolve().parents[1]
VEHICLE_GEOMETRY_PATH = WORKSPACE_DIR / "config" / "vehicle_geometry.yaml"
SENSOR_MOUNT_PATH = WORKSPACE_DIR / "config" / "sensor_mount.yaml"

RAW_TOPIC = "/sensing/lidar/points_raw"
FILTERED_TOPIC = "/sensing/lidar/points_filtered"


def load_yaml(path):
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def rotation_matrix_from_rpy(roll, pitch, yaw):
    cr = cos(roll)
    sr = sin(roll)
    cp = cos(pitch)
    sp = sin(pitch)
    cy = cos(yaw)
    sy = sin(yaw)

    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def transform_point(point, translation, rotation):
    x, y, z = point
    return (
        rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z + translation[0],
        rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z + translation[1],
        rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z + translation[2],
    )


def inside_box(point, box_min, box_max):
    return all(box_min[index] <= point[index] <= box_max[index] for index in range(3))


class LidarSelfFilter(Node):
    def __init__(self):
        super().__init__("lidar_self_filter")
        vehicle_geometry = load_yaml(VEHICLE_GEOMETRY_PATH)
        sensor_mount = load_yaml(SENSOR_MOUNT_PATH)

        self_filter = vehicle_geometry["vehicle_geometry"]["self_filter"]
        lidar_mount = sensor_mount["lidar"]

        self.box_min = tuple(float(value) for value in self_filter["box_min"])
        self.box_max = tuple(float(value) for value in self_filter["box_max"])
        self.min_range = float(lidar_mount["min_range"])
        self.max_range = float(lidar_mount["max_range"])
        self.translation = tuple(float(value) for value in lidar_mount["xyz"])
        self.rotation = rotation_matrix_from_rpy(
            *(float(value) for value in lidar_mount["rpy"])
        )

        self.publisher = self.create_publisher(PointCloud2, FILTERED_TOPIC, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            RAW_TOPIC,
            self.filter_cloud,
            10,
        )

    def filter_cloud(self, cloud):
        filtered_points = []
        for point in point_cloud2.read_points(cloud, skip_nans=True):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            distance = sqrt(x * x + y * y + z * z)
            point_in_base = transform_point((x, y, z), self.translation, self.rotation)

            if distance < self.min_range or distance > self.max_range:
                continue
            if inside_box(point_in_base, self.box_min, self.box_max):
                continue
            filtered_points.append(point)

        filtered_cloud = point_cloud2.create_cloud(
            cloud.header,
            cloud.fields,
            filtered_points,
        )
        self.publisher.publish(filtered_cloud)


def main():
    rclpy.init()
    node = LidarSelfFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
