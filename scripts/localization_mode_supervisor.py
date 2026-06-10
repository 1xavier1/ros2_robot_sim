#!/usr/bin/env python3
"""Publish route-aware localization mode for P0 task navigation."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from task_map_core import load_task_map, region_for_pose


SUPPORTED_MODES = ("OUTDOOR", "TRANSITION", "INDOOR", "DEGRADED")


class LocalizationModeSupervisor(Node):
    def __init__(self):
        super().__init__("localization_mode_supervisor")
        self.declare_parameter("task_map", "config/task_map.example.yaml")
        self.task_map = load_task_map(self.get_parameter("task_map").value)
        self.latest_wheel_lio_status = ""

        self.mode_pub = self.create_publisher(
            String,
            "/localization/supervised_mode",
            10,
        )
        self.create_subscription(
            Odometry,
            "/localization/global_odom",
            self.on_pose,
            10,
        )
        self.create_subscription(
            String,
            "/localization/wheel_lio_status",
            self.on_wheel_lio_status,
            10,
        )

    def on_wheel_lio_status(self, msg):
        self.latest_wheel_lio_status = msg.data

    def on_pose(self, msg):
        if "state=degraded" in self.latest_wheel_lio_status:
            self.mode_pub.publish(String(data="DEGRADED; reason=wheel_lio_degraded"))
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        region = region_for_pose(self.task_map, x, y)
        if region is None:
            self.mode_pub.publish(String(data="DEGRADED; reason=outside_regions"))
            return
        mode = region.get("localization_mode", "INDOOR")
        if mode not in SUPPORTED_MODES:
            self.mode_pub.publish(
                String(data=f"DEGRADED; reason=unknown_region_mode:{mode}")
            )
            return
        self.mode_pub.publish(String(data=f"{mode}; region={region['id']}"))


def main():
    rclpy.init()
    node = LocalizationModeSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
