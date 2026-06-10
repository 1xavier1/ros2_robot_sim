#!/usr/bin/env python3
"""Execute taught task_map routes through Nav2."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from task_map_core import goal_poses_for_task, load_task_map, motion_profiles_by_id


class TaskExecutor(Node):
    def __init__(self):
        super().__init__("task_executor")
        self.declare_parameter("task_map", "config/task_map.example.yaml")
        self.declare_parameter("default_task_id", "daily_patrol")
        self.task_map = load_task_map(self.get_parameter("task_map").value)
        self.state = "IDLE"
        self.active_task_id = None

        self.status_pub = self.create_publisher(String, "/task/status", 10)
        self.goal_pub = self.create_publisher(String, "/task/current_goal", 10)
        self.create_subscription(String, "/task/command", self.on_command, 10)
        self.nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            "navigate_through_poses",
        )
        self.publish_status("IDLE")

    def on_command(self, msg):
        parts = msg.data.split()
        if not parts:
            return
        command = parts[0]
        args = dict(part.split("=", 1) for part in parts[1:] if "=" in part)
        if command == "start_task":
            self.start_task(args.get("id", self.get_parameter("default_task_id").value))
        elif command == "pause_task":
            self.state = "PAUSED"
            self.publish_status("PAUSED")
        elif command == "resume_task":
            self.state = "RUNNING"
            self.publish_status("RUNNING")
        elif command == "cancel_task":
            self.state = "CANCELLED"
            self.publish_status("CANCELLED")
        elif command == "return_home":
            self.state = "RETURNING_HOME"
            self.publish_status("RETURNING_HOME")

    def start_task(self, task_id):
        try:
            poses = goal_poses_for_task(self.task_map, task_id)
            self.ensure_reverse_policy(task_id)
        except ValueError as exc:
            self.state = "BLOCKED"
            self.publish_status(f"BLOCKED; reason={exc}")
            return
        self.active_task_id = task_id
        self.state = "RUNNING"
        self.publish_status(f"RUNNING; task={task_id}; goals={len(poses)}")
        self.send_nav_goal(poses)

    def ensure_reverse_policy(self, task_id):
        task = next(item for item in self.task_map["tasks"] if item["id"] == task_id)
        route = next(
            item for item in self.task_map["recorded_routes"] if item["id"] == task["route"]
        )
        profile = motion_profiles_by_id(self.task_map)[route["motion_profile"]]
        if not profile.get("allow_reverse", False):
            self.get_logger().info("allow_reverse=false; executing forward-only route")

    def send_nav_goal(self, poses):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [self.pose_stamped(x, y, yaw) for x, y, yaw in poses]
        if goal_msg.poses:
            first = goal_msg.poses[0].pose.position
            self.goal_pub.publish(String(data=f"x={first.x:.3f}; y={first.y:.3f}"))
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.state = "BLOCKED"
            self.publish_status("BLOCKED; reason=nav2_action_unavailable")
            return
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.on_goal_response)

    def pose_stamped(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = self.task_map["site"]["map_frame"]
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        return msg

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = "BLOCKED"
            self.publish_status("BLOCKED; reason=nav2_goal_rejected")
            return
        goal_handle.get_result_async().add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        result = future.result()
        if result.status == 4:
            self.state = "COMPLETED"
            self.publish_status("COMPLETED")
        else:
            self.state = "BLOCKED"
            self.publish_status(f"BLOCKED; reason=nav2_status_{result.status}")

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = TaskExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
