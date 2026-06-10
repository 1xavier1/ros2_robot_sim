#!/usr/bin/env python3
"""Record taught waypoints and routes into a task_map.yaml file."""

import copy
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from task_map_core import (
    direction_from_linear_velocity,
    load_task_map,
    should_append_route_sample,
    yaw_from_quaternion,
)


class RouteRecorder(Node):
    def __init__(self):
        super().__init__("route_recorder")
        self.declare_parameter("task_map_template", "config/task_map.example.yaml")
        self.declare_parameter("task_map_output", "maps/task_map.yaml")
        self.declare_parameter("min_sample_distance", 0.3)
        self.declare_parameter("min_yaw_change", 0.25)

        self.latest_pose = None
        self.latest_direction = "forward"
        self.recording_route_id = None
        self.samples = []
        self.task_map = load_task_map(self.get_parameter("task_map_template").value)

        self.status_pub = self.create_publisher(String, "/teach/status", 10)
        self.create_subscription(String, "/teach/command", self.on_command, 10)
        self.create_subscription(
            Odometry, "/localization/global_odom", self.on_pose, 10
        )
        self.create_subscription(
            Odometry, "/localization/wheel_lio_odom", self.on_pose, 10
        )
        self.create_subscription(Odometry, "/robot/odom", self.on_wheel_odom, 10)

    def on_pose(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        ]
        if self.recording_route_id:
            self.append_sample_if_needed()

    def on_wheel_odom(self, msg):
        self.latest_direction = direction_from_linear_velocity(msg.twist.twist.linear.x)

    def on_command(self, msg):
        parts = msg.data.split()
        if not parts:
            return
        command = parts[0]
        args = dict(part.split("=", 1) for part in parts[1:] if "=" in part)
        if command == "mark_waypoint":
            self.mark_waypoint(args.get("id", "waypoint"), args.get("role", "taught"))
        elif command == "start_recording":
            self.start_recording(args.get("id", "taught_route"))
        elif command == "stop_recording":
            self.stop_recording()
        elif command == "save_task_map":
            self.save_task_map()

    def mark_waypoint(self, waypoint_id, role):
        if self.latest_pose is None:
            self.publish_status("teach=blocked; reason=no_pose")
            return
        self.task_map["waypoints"].append({
            "id": waypoint_id,
            "pose": list(self.latest_pose),
            "role": role,
            "source": "taught",
        })
        self.publish_status(f"teach=marked; waypoint={waypoint_id}")

    def start_recording(self, route_id):
        self.recording_route_id = route_id
        self.samples = []
        self.publish_status(f"teach=recording; route={route_id}; samples=0")

    def append_sample_if_needed(self):
        min_distance = float(self.get_parameter("min_sample_distance").value)
        min_yaw = float(self.get_parameter("min_yaw_change").value)
        if should_append_route_sample(self.samples, self.latest_pose, min_distance, min_yaw):
            self.samples.append({
                "pose": list(self.latest_pose),
                "direction": self.latest_direction,
            })

    def stop_recording(self):
        if not self.recording_route_id:
            return
        route = {
            "id": self.recording_route_id,
            "motion_profile": "forward_only_safe",
            "source": "taught",
            "sample_policy": {
                "min_distance": float(self.get_parameter("min_sample_distance").value),
                "min_yaw_change": float(self.get_parameter("min_yaw_change").value),
            },
            "path": copy.deepcopy(self.samples),
        }
        self.task_map["recorded_routes"].append(route)
        self.publish_status(
            f"teach=stopped; route={self.recording_route_id}; samples={len(self.samples)}"
        )
        self.recording_route_id = None

    def save_task_map(self):
        output = Path(self.get_parameter("task_map_output").value)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(self.task_map, stream, allow_unicode=False, sort_keys=False)
        self.publish_status(f"teach=saved; path={output}")

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = RouteRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
