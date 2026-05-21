#!/usr/bin/env python3
"""Publish localization mode, gated GPS, and fusion weight hints."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
import yaml


CONFIG_DIR = Path(get_package_share_directory("robot_description")) / "config"


def load_modes():
    with (CONFIG_DIR / "localization_modes.yaml").open(
        "r",
        encoding="utf-8",
    ) as stream:
        return yaml.safe_load(stream)["localization_modes"]


def max_position_covariance(msg):
    covariance = msg.position_covariance
    return max(covariance[0], covariance[4], covariance[8])


class LocalizationModeManager(Node):
    def __init__(self):
        super().__init__("localization_mode_manager")
        self.modes = load_modes()
        self.mode_pub = self.create_publisher(String, "/localization/mode", 10)
        self.weights_pub = self.create_publisher(
            String,
            "/localization/fusion_weights",
            10,
        )
        self.gps_pub = self.create_publisher(
            NavSatFix,
            "/localization/gps/gated",
            10,
        )
        self.subscription = self.create_subscription(
            NavSatFix,
            "/sensing/gps/fix",
            self.on_gps,
            10,
        )

    def on_gps(self, msg):
        good_status = msg.status.status >= NavSatStatus.STATUS_FIX
        good_covariance = (
            max_position_covariance(msg)
            <= self.modes["gps_covariance_threshold"]
        )
        mode = "OUTDOOR" if good_status and good_covariance else "BARN"
        self.mode_pub.publish(String(data=mode))
        weights = self.modes[mode]
        self.weights_pub.publish(String(data=str(weights)))
        if mode == "OUTDOOR":
            self.gps_pub.publish(msg)


def main():
    rclpy.init()
    node = LocalizationModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
