#!/usr/bin/env python3
"""Accumulate FAST-LIO registered point clouds into a global map and export occupancy grid."""

import argparse
import math
import signal
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class MapAccumulator(Node):
    def __init__(self, voxel_size):
        super().__init__("map_accumulator")
        self.voxel_size = voxel_size
        self.counts = defaultdict(int)
        self.points = defaultdict(list)
        self.cloud_sub = self.create_subscription(
            PointCloud2, "/mapping/lio/map_points", self.on_cloud, 10
        )
        self.frame_count = 0
        self.total_points = 0

    def voxel_key(self, x, y, z):
        vx = int(x / self.voxel_size)
        vy = int(y / self.voxel_size)
        vz = int(z / self.voxel_size)
        return (vx, vy, vz)

    def on_cloud(self, msg):
        self.frame_count += 1
        for pt in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            key = self.voxel_key(pt[0], pt[1], pt[2])
            if self.counts[key] == 0:
                self.points[key] = (pt[0], pt[1], pt[2])
            self.counts[key] += 1
        self.total_points += len(msg.data) // msg.point_step
        if self.frame_count % 50 == 0:
            self.get_logger().info(f"frames: {self.frame_count}, voxels: {len(self.counts)}")

    def export(self, output_base, resolution, min_z, max_z, min_hits):
        if not self.points:
            self.get_logger().error("no voxels accumulated")
            return
        filtered = []
        for key, (px, py, pz) in self.points.items():
            if min_z <= pz <= max_z and self.counts[key] >= min_hits:
                filtered.append((px, py))
        if len(filtered) == 0:
            self.get_logger().error("no points after filtering")
            return
        pts = np.array(filtered)
        xs, ys = pts[:, 0], pts[:, 1]
        min_x, max_x = xs.min(), xs.max()
        min_y, max_y = ys.min(), ys.max()
        margin_x = (max_x - min_x) * 0.1 + 1.0
        margin_y = (max_y - min_y) * 0.1 + 1.0
        min_x -= margin_x
        max_x += margin_x
        min_y -= margin_y
        max_y += margin_y
        width = int(math.ceil((max_x - min_x) / resolution)) + 1
        height = int(math.ceil((max_y - min_y) / resolution)) + 1
        if width > 10000 or height > 10000:
            self.get_logger().error(f"map too large: {width}x{height}")
            return
        grid = np.zeros((height, width), dtype=np.int32)
        for px, py in zip(xs, ys):
            gx = int((px - min_x) / resolution)
            gy = int((py - min_y) / resolution)
            if 0 <= gx < width and 0 <= gy < height:
                grid[gy, gx] = 1
        occ_mask = grid > 0
        occupied = occ_mask.sum()
        total = grid.size
        self.get_logger().info(
            f"map: {width}x{height} ({width*resolution:.1f}m x {height*resolution:.1f}m), "
            f"occupied: {occupied}/{total} ({100*occupied/total:.1f}%), "
            f"filtered points: {len(filtered)}, voxels: {len(self.counts)}"
        )
        pgm_grid = np.where(occ_mask, np.uint8(255), np.uint8(0))
        self._save_pgm(pgm_grid, output_base, resolution, min_x, min_y, height, width)

    def _save_pgm(self, grid, output_base, resolution, min_x, min_y, height, width):
        pgm_path = Path(f"{output_base}.pgm")
        with open(pgm_path, "wb") as f:
            f.write(f"P5\n{width} {height}\n255\n".encode())
            f.write(grid.tobytes())
        origin_y = min_y + height * resolution
        content = (
            f"image: {pgm_path.name}\n"
            f"mode: trinary\n"
            f"resolution: {resolution:.6f}\n"
            f"origin: [{min_x:.6f}, {origin_y:.6f}, 0.000000]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.25\n"
        )
        Path(f"{output_base}.yaml").write_text(content)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="maps/lio_accumulated_map")
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--min-z", type=float, default=-0.5)
    parser.add_argument("--max-z", type=float, default=3.0)
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parser.add_argument("--voxel-size", type=float, default=0.15)
    parser.add_argument("--min-hits", type=int, default=3)
    args = parser.parse_args()

    rclpy.init()
    node = MapAccumulator(voxel_size=args.voxel_size)

    def shutdown(sig, frame):
        node.export(args.output, args.resolution, args.min_z, args.max_z, args.min_hits)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=args.duration_sec)
        while rclpy.ok() and node.get_clock().now() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.export(args.output, args.resolution, args.min_z, args.max_z, args.min_hits)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
