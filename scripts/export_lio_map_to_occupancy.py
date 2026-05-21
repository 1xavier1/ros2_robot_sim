#!/usr/bin/env python3
"""Export /mapping/lio/map_points to a Nav2-compatible occupancy map."""

import argparse
from math import ceil, floor
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


DEFAULT_TOPIC = "/mapping/lio/map_points"
DEFAULT_OUTPUT = "maps/lio_map"


class MapPointCapture(Node):
    def __init__(self, topic):
        super().__init__("lio_map_point_capture")
        self.cloud = None
        self.subscription = self.create_subscription(
            PointCloud2,
            topic,
            self.capture_cloud,
            10,
        )

    def capture_cloud(self, cloud):
        self.cloud = cloud


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--padding", type=float, default=1.0)
    parser.add_argument("--min-z", type=float, default=0.05)
    parser.add_argument("--max-z", type=float, default=2.0)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    args, _ = parser.parse_known_args()
    return args


def collect_points(cloud, min_z, max_z):
    points = []
    for point in point_cloud2.read_points(
        cloud,
        field_names=("x", "y", "z"),
        skip_nans=True,
    ):
        x = float(point[0])
        y = float(point[1])
        z = float(point[2])
        if min_z <= z <= max_z:
            points.append((x, y))
    return points


def point_bounds(points, padding):
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return (
        floor((min(xs) - padding) * 1000.0) / 1000.0,
        floor((min(ys) - padding) * 1000.0) / 1000.0,
        ceil((max(xs) + padding) * 1000.0) / 1000.0,
        ceil((max(ys) + padding) * 1000.0) / 1000.0,
    )


def make_grid(points, resolution, padding):
    min_x, min_y, max_x, max_y = point_bounds(points, padding)
    width = max(1, int(ceil((max_x - min_x) / resolution)))
    height = max(1, int(ceil((max_y - min_y) / resolution)))
    grid = [[205 for _ in range(width)] for _ in range(height)]

    for x, y in points:
        col = int((x - min_x) / resolution)
        row = int((y - min_y) / resolution)
        if 0 <= col < width and 0 <= row < height:
            grid[height - 1 - row][col] = 0

    return grid, (min_x, min_y, 0.0)


def write_pgm(path, grid):
    height = len(grid)
    width = len(grid[0]) if height else 0
    with path.open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        for row in grid:
            stream.write(bytes(row))


def write_yaml(path, image_name, resolution, origin):
    yaml_text = (
        f"image: {image_name}\n"
        "mode: trinary\n"
        f"resolution: {resolution:.6f}\n"
        f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    path.write_text(yaml_text, encoding="utf-8")


def wait_for_cloud(topic, timeout_sec):
    rclpy.init()
    node = MapPointCapture(topic)
    try:
        deadline = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and node.cloud is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds > deadline:
                raise TimeoutError(f"timed out waiting for {topic}")
        return node.cloud
    finally:
        node.destroy_node()
        rclpy.shutdown()


def export_cloud(args):
    cloud = wait_for_cloud(args.topic, args.timeout_sec)
    points = collect_points(cloud, args.min_z, args.max_z)
    if not points:
        raise RuntimeError("no map points inside configured height range")

    output_base = Path(args.output)
    if not output_base.is_absolute():
        output_base = Path.cwd() / output_base
    output_base.parent.mkdir(parents=True, exist_ok=True)

    pgm_path = output_base.with_suffix(".pgm")
    yaml_path = output_base.with_suffix(".yaml")
    grid, origin = make_grid(points, args.resolution, args.padding)
    write_pgm(pgm_path, grid)
    write_yaml(yaml_path, pgm_path.name, args.resolution, origin)
    return yaml_path, pgm_path, len(points)


def main():
    args = parse_args()
    yaml_path, pgm_path, point_count = export_cloud(args)
    print(f"exported Nav2 occupancy map: {yaml_path}")
    print(f"image: {pgm_path}")
    print(f"points: {point_count}")


if __name__ == "__main__":
    main()
