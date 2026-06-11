#!/usr/bin/env python3
"""Export raw lidar points projected by wheel odom, plus FAST-LIO/odom diagnostics."""

import argparse
import json
import math
import signal
import sys
from collections import defaultdict
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def transform_body_points(points, pose):
    x0, y0, yaw = pose
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    transformed = []
    for x, y, z in points:
        transformed.append(
            (
                x0 + cos_yaw * x - sin_yaw * y,
                y0 + sin_yaw * x + cos_yaw * y,
                z,
            )
        )
    return transformed


def transform_lidar_points_to_base(points, lidar_xyz, lidar_yaw):
    cos_yaw = math.cos(lidar_yaw)
    sin_yaw = math.sin(lidar_yaw)
    tx, ty, tz = lidar_xyz
    transformed = []
    for x, y, z in points:
        transformed.append(
            (
                tx + cos_yaw * x - sin_yaw * y,
                ty + sin_yaw * x + cos_yaw * y,
                tz + z,
            )
        )
    return transformed


def transform_lidar_points_to_odom(points, odom_pose, lidar_xyz, lidar_yaw):
    base_points = transform_lidar_points_to_base(points, lidar_xyz, lidar_yaw)
    return transform_body_points(base_points, odom_pose)


def raytrace_cells(start, end):
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    cells = []
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return cells
        err2 = 2 * err
        if err2 >= dy:
            err += dy
            x0 += sx
        if err2 <= dx:
            err += dx
            y0 += sy


def filter_lidar_points_by_range(points, min_range, max_range):
    filtered = []
    for x, y, z in points:
        planar_range = math.hypot(x, y)
        if min_range <= planar_range <= max_range:
            filtered.append((x, y, z))
    return filtered


def filter_points_in_box(points, box_min, box_max):
    min_x, min_y, min_z = box_min
    max_x, max_y, max_z = box_max
    filtered = []
    for x, y, z in points:
        inside = (
            min_x <= x <= max_x
            and min_y <= y <= max_y
            and min_z <= z <= max_z
        )
        if not inside:
            filtered.append((x, y, z))
    return filtered


def filter_points_by_height(points, min_z, max_z):
    stats = {
        "height_low_points": 0,
        "height_points": 0,
        "height_high_points": 0,
    }
    filtered = []
    for x, y, z in points:
        if z < min_z:
            stats["height_low_points"] += 1
        elif z > max_z:
            stats["height_high_points"] += 1
        else:
            stats["height_points"] += 1
            filtered.append((x, y, z))
    return filtered, stats


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def nearest_pose_by_stamp(pose_buffer, target_stamp, max_age):
    if not pose_buffer:
        return None
    stamp, pose = min(pose_buffer, key=lambda item: abs(item[0] - target_stamp))
    if abs(stamp - target_stamp) > max_age:
        return None
    return pose


def classify_cells(occupied_counts, free_counts, min_hits, min_free_hits, occupied_free_ratio):
    occupied_cells = set()
    for cell, occupied_hits in occupied_counts.items():
        free_hits = free_counts.get(cell, 0)
        if occupied_hits >= min_hits and occupied_hits >= free_hits * occupied_free_ratio:
            occupied_cells.add(cell)
    free_cells = {
        cell
        for cell, free_hits in free_counts.items()
        if free_hits >= min_free_hits and cell not in occupied_cells
    }
    return occupied_cells, free_cells


def make_occupancy_grid(occupied_cells, free_cells, width, height):
    grid = np.full((height, width), np.uint8(205), dtype=np.uint8)
    for x, y in free_cells:
        if 0 <= x < width and 0 <= y < height:
            grid[height - 1 - y, x] = np.uint8(254)
    for x, y in occupied_cells:
        if 0 <= x < width and 0 <= y < height:
            grid[height - 1 - y, x] = np.uint8(0)
    return grid


def load_lidar_mount(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    lidar = data["lidar"]
    xyz = tuple(float(value) for value in lidar["xyz"])
    rpy = tuple(float(value) for value in lidar["rpy"])
    return xyz, rpy[2]


def load_self_filter_box(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    self_filter = data["vehicle_geometry"]["self_filter"]
    box_min = tuple(float(value) for value in self_filter["box_min"])
    box_max = tuple(float(value) for value in self_filter["box_max"])
    return box_min, box_max


class OdomProjectedMap(Node):
    def __init__(self, args):
        super().__init__("odom_projected_map_exporter")
        self.args = args
        mount_path = Path(args.sensor_mount)
        if not mount_path.is_absolute():
            mount_path = Path.cwd() / mount_path
        self.lidar_xyz, self.lidar_yaw = load_lidar_mount(mount_path)
        geometry_path = Path(args.vehicle_geometry)
        if not geometry_path.is_absolute():
            geometry_path = Path.cwd() / geometry_path
        self.self_box_min, self.self_box_max = load_self_filter_box(geometry_path)
        self.occupied_counts = defaultdict(int)
        self.free_counts = defaultdict(int)
        self.latest_odom = None
        self.pose_buffer = deque(maxlen=args.pose_buffer_size)
        self.latest_lio = None
        self.latest_reference_odom = None
        self.first_pair = None
        self.latest_pair = None
        self.frames = 0
        self.skipped_clouds_no_pose = 0
        self.filter_stats = defaultdict(int)
        self.create_subscription(PointCloud2, args.cloud_topic, self.on_cloud, 10)
        self.create_subscription(Odometry, args.pose_topic, self.on_odom, 10)
        self.create_subscription(Odometry, args.lio_odom_topic, self.on_lio, 10)
        if args.reference_topic:
            self.create_subscription(
                Odometry,
                args.reference_topic,
                self.on_reference_odom,
                10,
            )

    def on_odom(self, msg):
        self.latest_odom = msg
        self.pose_buffer.append((stamp_to_sec(msg.header.stamp), self.pose_tuple(msg)))
        self.record_pair()

    def on_lio(self, msg):
        self.latest_lio = msg
        self.record_pair()

    def on_reference_odom(self, msg):
        self.latest_reference_odom = msg

    def record_pair(self):
        if self.latest_odom is None or self.latest_lio is None:
            return
        pair = (self.pose_tuple(self.latest_odom), self.pose_tuple(self.latest_lio))
        if self.first_pair is None:
            self.first_pair = pair
        self.latest_pair = pair

    def pose_tuple(self, msg):
        p = msg.pose.pose.position
        return (float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation))

    def on_cloud(self, msg):
        cloud_stamp = stamp_to_sec(msg.header.stamp)
        odom_pose = nearest_pose_by_stamp(
            self.pose_buffer,
            cloud_stamp,
            max_age=self.args.max_pose_age,
        )
        if odom_pose is None:
            self.skipped_clouds_no_pose += 1
            return
        lidar_points = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            lidar_points.append((x, y, z))
        self.filter_stats["raw_points"] += len(lidar_points)
        lidar_points = filter_lidar_points_by_range(
            lidar_points,
            min_range=self.args.min_range,
            max_range=self.args.max_range,
        )
        self.filter_stats["range_points"] += len(lidar_points)
        base_points = transform_lidar_points_to_base(lidar_points, self.lidar_xyz, self.lidar_yaw)
        if not self.args.disable_self_filter:
            base_points = filter_points_in_box(
                base_points,
                box_min=self.self_box_min,
                box_max=self.self_box_max,
            )
        self.filter_stats["self_filtered_points"] += len(base_points)
        endpoint_points, height_stats = filter_points_by_height(
            base_points,
            min_z=self.args.min_z,
            max_z=self.args.max_z,
        )
        for key, value in height_stats.items():
            self.filter_stats[key] += value
        odom_points = transform_body_points(endpoint_points, odom_pose)
        sensor_x, sensor_y, _sensor_z = transform_lidar_points_to_odom(
            [(0.0, 0.0, 0.0)], odom_pose, self.lidar_xyz, self.lidar_yaw
        )[0]
        sensor_cell = self.metric_cell(sensor_x, sensor_y)
        for x, y, _z in odom_points:
            endpoint_cell = self.metric_cell(x, y)
            ray_cells = raytrace_cells(sensor_cell, endpoint_cell)
            for free_cell in ray_cells[:-1]:
                self.free_counts[free_cell] += 1
            self.occupied_counts[endpoint_cell] += 1
        self.frames += 1
        if self.frames % 50 == 0:
            self.get_logger().info(
                f"frames: {self.frames}, occupied: {len(self.occupied_counts)}, "
                f"free: {len(self.free_counts)}"
            )

    def metric_cell(self, x, y):
        return (
            int(math.floor(x / self.args.resolution)),
            int(math.floor(y / self.args.resolution)),
        )

    def export(self):
        occupied_cells, free_cells = classify_cells(
            self.occupied_counts,
            self.free_counts,
            min_hits=self.args.min_hits,
            min_free_hits=self.args.min_free_hits,
            occupied_free_ratio=self.args.occupied_free_ratio,
        )
        all_cells = occupied_cells | free_cells
        if not occupied_cells or not all_cells:
            raise RuntimeError("no odom-projected points accumulated")
        output_base = Path(self.args.output)
        output_base.parent.mkdir(parents=True, exist_ok=True)

        padding_cells = int(math.ceil(self.args.padding / self.args.resolution))
        min_cell_x = min(cell[0] for cell in all_cells) - padding_cells
        max_cell_x = max(cell[0] for cell in all_cells) + padding_cells
        min_cell_y = min(cell[1] for cell in all_cells) - padding_cells
        max_cell_y = max(cell[1] for cell in all_cells) + padding_cells
        width = max_cell_x - min_cell_x + 1
        height = max_cell_y - min_cell_y + 1
        local_occupied = {
            (cell[0] - min_cell_x, cell[1] - min_cell_y) for cell in occupied_cells
        }
        local_free = {
            (cell[0] - min_cell_x, cell[1] - min_cell_y) for cell in free_cells
        }
        pgm_grid = make_occupancy_grid(local_occupied, local_free, width, height)
        origin_x = min_cell_x * self.args.resolution
        origin_y = min_cell_y * self.args.resolution
        self.write_pgm(output_base.with_suffix(".pgm"), pgm_grid)
        self.write_yaml(output_base.with_suffix(".yaml"), output_base.with_suffix(".pgm").name, origin_x, origin_y)
        self.write_diagnostics(output_base.with_suffix(".json"), width, height, len(occupied_cells))
        self.get_logger().info(
            f"map: {width}x{height}, occupied: {len(occupied_cells)}, free: {len(free_cells)}, "
            f"frames: {self.frames}, "
            f"diagnostics: {output_base.with_suffix('.json')}"
        )

    def write_pgm(self, path, grid):
        with path.open("wb") as stream:
            stream.write(f"P5\n{grid.shape[1]} {grid.shape[0]}\n255\n".encode("ascii"))
            stream.write(grid.tobytes())

    def write_yaml(self, path, image_name, min_x, min_y):
        path.write_text(
            f"image: {image_name}\n"
            "mode: trinary\n"
            f"resolution: {self.args.resolution:.6f}\n"
            f"origin: [{min_x:.6f}, {min_y:.6f}, 0.000000]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.25\n",
            encoding="utf-8",
        )

    def write_diagnostics(self, path, width, height, point_count):
        data = {
            "frames": self.frames,
            "occupied_points": point_count,
            "map_width": width,
            "map_height": height,
            "pose_topic": self.args.pose_topic,
            "odom_topic": self.args.odom_topic,
            "lio_odom_topic": self.args.lio_odom_topic,
            "reference_metrics": {
                "reference_topic": self.args.reference_topic,
                "available": self.latest_reference_odom is not None,
            },
            "filter_stats": dict(self.filter_stats),
            "skipped_clouds_no_pose": self.skipped_clouds_no_pose,
            "max_pose_age": self.args.max_pose_age,
            "diagnostics": {},
        }
        if self.first_pair is not None and self.latest_pair is not None:
            first_odom, first_lio = self.first_pair
            latest_odom, latest_lio = self.latest_pair
            odom_delta = delta_pose(first_odom, latest_odom)
            lio_delta = delta_pose(first_lio, latest_lio)
            data["diagnostics"] = {
                "odom_delta": pose_dict(odom_delta),
                "lio_delta": pose_dict(lio_delta),
                "delta_error": pose_dict(
                    (
                        lio_delta[0] - odom_delta[0],
                        lio_delta[1] - odom_delta[1],
                        wrap_angle(lio_delta[2] - odom_delta[2]),
                    )
                ),
            }
        path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def delta_pose(start, end):
    return (end[0] - start[0], end[1] - start[1], wrap_angle(end[2] - start[2]))


def pose_dict(pose):
    return {"x": pose[0], "y": pose[1], "yaw": pose[2]}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cloud-topic", default="/sensing/lidar/points")
    parser.add_argument("--pose-topic", default="/localization/wheel_lio_odom")
    parser.add_argument("--odom-topic", default=None)
    parser.add_argument("--reference-topic", default="/robot/ground_truth/odom")
    parser.add_argument("--lio-odom-topic", default="/mapping/lio/odom")
    parser.add_argument("--sensor-mount", default="config/sensor_mount.yaml")
    parser.add_argument("--vehicle-geometry", default="config/vehicle_geometry.yaml")
    parser.add_argument("--disable-self-filter", action="store_true")
    parser.add_argument("--output", default="maps/odom_projected_map")
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--max-pose-age", type=float, default=0.12)
    parser.add_argument("--pose-buffer-size", type=int, default=300)
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--voxel-size", type=float, default=0.15)
    parser.add_argument("--min-hits", type=int, default=3)
    parser.add_argument("--min-free-hits", type=int, default=2)
    parser.add_argument("--occupied-free-ratio", type=float, default=0.5)
    parser.add_argument("--min-range", type=float, default=0.1)
    parser.add_argument("--max-range", type=float, default=15.0)
    parser.add_argument("--min-z", type=float, default=0.05)
    parser.add_argument("--max-z", type=float, default=1.6)
    parser.add_argument("--padding", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.odom_topic is not None:
        args.pose_topic = args.odom_topic
    rclpy.init()
    node = OdomProjectedMap(args)

    def shutdown(_sig, _frame):
        node.export()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=args.duration_sec)
        while rclpy.ok() and node.get_clock().now() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.export()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
