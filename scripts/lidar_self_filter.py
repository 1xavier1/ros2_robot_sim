#!/usr/bin/env python3
"""Filter vehicle body returns and out-of-range points from LiDAR clouds."""

from math import atan2, cos, sin, sqrt
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import yaml


CONFIG_DIR = Path(get_package_share_directory("robot_description")) / "config"

RAW_TOPIC = "/sensing/lidar/points_raw"
FILTERED_TOPIC = "/sensing/lidar/points_filtered"

FAST_LIO_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
    PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
]


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


def estimate_ring(point, vertical_fov, scan_lines):
    if scan_lines <= 1 or vertical_fov <= 0.0:
        return 0

    x, y, z = point
    horizontal_range = sqrt(x * x + y * y)
    vertical_angle = atan2(z, horizontal_range)
    normalized = (vertical_angle + vertical_fov / 2.0) / vertical_fov
    ring = round(normalized * (scan_lines - 1))
    return max(0, min(scan_lines - 1, int(ring)))


class LidarSelfFilter(Node):
    def __init__(self):
        super().__init__("lidar_self_filter")
        vehicle_geometry = load_yaml(CONFIG_DIR / "vehicle_geometry.yaml")
        sensor_mount = load_yaml(CONFIG_DIR / "sensor_mount.yaml")

        self_filter = vehicle_geometry["vehicle_geometry"]["self_filter"]
        lidar_mount = sensor_mount["lidar"]

        self.box_min = tuple(float(value) for value in self_filter["box_min"])
        self.box_max = tuple(float(value) for value in self_filter["box_max"])
        self.min_range = float(lidar_mount["min_range"])
        self.max_range = float(lidar_mount["max_range"])
        self.vertical_fov = float(lidar_mount["vertical_fov"])
        self.scan_lines = int(lidar_mount["scan_lines"])
        self.scan_rate = float(lidar_mount["scan_rate"])
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
        point_count = max(int(cloud.width) * max(int(cloud.height), 1), 1)
        scan_period_us = 1_000_000.0 / self.scan_rate if self.scan_rate > 0 else 0.0
        for point_index, point in enumerate(point_cloud2.read_points(cloud, skip_nans=True)):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            intensity = float(point[3]) if len(point) > 3 else 0.0
            distance = sqrt(x * x + y * y + z * z)
            point_in_base = transform_point((x, y, z), self.translation, self.rotation)

            if distance < self.min_range or distance > self.max_range:
                continue
            if inside_box(point_in_base, self.box_min, self.box_max):
                continue
            ring = estimate_ring((x, y, z), self.vertical_fov, self.scan_lines)
            time_offset_us = (point_index / max(point_count - 1, 1)) * scan_period_us
            filtered_points.append((x, y, z, intensity, ring, time_offset_us))

        filtered_cloud = point_cloud2.create_cloud(
            cloud.header,
            FAST_LIO_FIELDS,
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
