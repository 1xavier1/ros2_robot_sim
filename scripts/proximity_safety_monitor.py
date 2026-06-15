#!/usr/bin/env python3
"""Rear/front mmWave near-field e-stop. Front sensor is gated off in push mode."""

import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool

from proximity_core import MMWAVE_KEYS, decide_estop, gated_map, min_xy_distance


class ProximitySafetyMonitor(Node):
    def __init__(self):
        super().__init__("proximity_safety_monitor")
        self.declare_parameter("config", "")
        cfg_path = self.get_parameter("config").value or os.path.join(
            get_package_share_directory("robot_description"),
            "config",
            "sensor_mount.yaml",
        )
        with open(cfg_path) as handle:
            cfg = yaml.safe_load(handle)

        self.stop_distance = float(cfg.get("safety", {}).get("stop_distance", 0.6))
        self.gated_in_push = gated_map(cfg)
        self.sensor_min = {key: None for key in self.gated_in_push}
        self.push_active = False

        self.estop_pub = self.create_publisher(Bool, "/safety/estop", 10)
        self.cmd_pub = self.create_publisher(Twist, "/robot/cmd_vel", 10)
        for key in self.gated_in_push:
            topic = "/sensing/" + cfg[key]["topic"]
            self.create_subscription(PointCloud2, topic, self._make_cb(key), 10)
        self.create_subscription(Bool, "/push/active", self.on_push, 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"proximity monitor: stop={self.stop_distance}m gated={self.gated_in_push}"
        )

    def _make_cb(self, name):
        def cb(msg):
            points = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
            self.sensor_min[name] = min_xy_distance(points)

        return cb

    def on_push(self, msg):
        self.push_active = msg.data

    def tick(self):
        stop = decide_estop(
            self.sensor_min, self.push_active, self.stop_distance, self.gated_in_push
        )
        self.estop_pub.publish(Bool(data=stop))
        if stop:
            self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = ProximitySafetyMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
