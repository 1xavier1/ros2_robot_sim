#!/usr/bin/env python3
"""Execute taught task_map routes through Nav2."""

import math

from action_msgs.msg import GoalStatus
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String

from task_map_core import (
    adapt_start_pose_for_ackermann,
    executable_goal_poses_for_task,
    item_by_id,
    load_task_map,
    motion_profiles_by_id,
    point_in_polygon,
    yaw_from_quaternion,
)


class TaskExecutor(Node):
    def __init__(self):
        super().__init__("task_executor")
        self.declare_parameter("task_map", "config/task_map.example.yaml")
        self.declare_parameter("default_task_id", "daily_patrol")
        self.task_map = load_task_map(self.get_parameter("task_map").value)
        self.state = "IDLE"
        self.active_task_id = None
        self.active_goal_handle = None
        self.nav_goal_pending = False
        self.remaining_poses = []
        self.current_pose = None
        self.pending_start_task_id = None
        self.last_status = "IDLE"
        self.push_active = False

        self.status_pub = self.create_publisher(String, "/task/status", 10)
        self.goal_pub = self.create_publisher(String, "/task/current_goal", 10)
        self.active_path_pub = self.create_publisher(Path, "/task/active_path", 10)
        self.push_pub = self.create_publisher(Bool, "/push/active", 10)
        self.create_subscription(String, "/task/command", self.on_command, 10)
        self.create_subscription(Odometry, "/localization/global_odom", self.on_pose, 10)
        self.nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            "navigate_through_poses",
        )
        self.create_timer(1.0, self.republish_status)
        self.publish_status("IDLE")
        self.set_push(False)

    def reload_task_map(self):
        self.task_map = load_task_map(self.get_parameter("task_map").value)

    def on_command(self, msg):
        parts = msg.data.split()
        if not parts:
            return
        command = parts[0]
        args = dict(part.split("=", 1) for part in parts[1:] if "=" in part)
        self.get_logger().info(f"task command received: {msg.data}")
        if command == "start_task":
            self.start_task(args.get("id", self.get_parameter("default_task_id").value))
        elif command in ("cancel_task", "manual_override"):
            self.cancel_active_goal("MANUAL_OVERRIDE" if command == "manual_override" else "CANCELED")
        elif command == "pause_task":
            self.pause_task()
        elif command == "resume_task":
            self.resume_task()
        elif command == "return_home":
            self.return_home()

    def on_pose(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation),
        )
        if self.pending_start_task_id and not self.has_active_or_pending_goal():
            task_id = self.pending_start_task_id
            self.pending_start_task_id = None
            self.start_task(task_id)

    def start_task(self, task_id):
        if self.has_active_or_pending_goal():
            self.publish_status("BLOCKED; reason=task_already_running")
            return
        if self.current_pose is None:
            self.pending_start_task_id = task_id
            self.state = "WAITING_FOR_LOCALIZATION"
            self.publish_status(
                f"WAITING_FOR_LOCALIZATION; task={task_id}; reason=no_current_pose"
            )
            return
        try:
            self.reload_task_map()
            poses, warnings = executable_goal_poses_for_task(self.task_map, task_id)
            poses = self.prepare_ackermann_goal_poses(task_id, poses)
            self.ensure_reverse_policy(task_id)
        except ValueError as exc:
            self.state = "BLOCKED"
            self.publish_status(f"BLOCKED; reason={exc}")
            return
        self.active_task_id = task_id
        self.set_push(self.task_is_push(task_id, poses))
        self.remaining_poses = list(poses)
        self.state = "RUNNING"
        warning_text = "" if not warnings else "; warning=" + ",".join(warnings)
        self.publish_status(f"RUNNING; task={task_id}; goals={len(poses)}{warning_text}")
        self.send_nav_goal(poses)

    def has_active_or_pending_goal(self):
        return self.nav_goal_pending or self.active_goal_handle is not None

    def set_push(self, active):
        self.push_active = bool(active)
        self.push_pub.publish(Bool(data=self.push_active))

    def task_is_push(self, task_id, poses=None):
        try:
            task = item_by_id(self.task_map["tasks"], task_id, "task")
        except ValueError:
            return False
        if task.get("push", False):
            return True
        push_zones = self.task_map.get("map_overlays", {}).get("push_zones", [])
        for pose in poses or []:
            for zone in push_zones:
                polygon = zone.get("polygon") or []
                if len(polygon) >= 3 and point_in_polygon(pose[0], pose[1], polygon):
                    return True
        return False

    def ensure_reverse_policy(self, task_id):
        task = next(item for item in self.task_map["tasks"] if item["id"] == task_id)
        if task.get("type") == "waypoint_sequence":
            return
        route = next(
            item for item in self.task_map["recorded_routes"] if item["id"] == task["route"]
        )
        profile = motion_profiles_by_id(self.task_map)[route["motion_profile"]]
        if not profile.get("allow_reverse", False):
            self.get_logger().info("allow_reverse=false; executing forward-only route")

    def task_type(self, task_id):
        task = next(item for item in self.task_map["tasks"] if item["id"] == task_id)
        return task.get("type")

    def prepare_ackermann_goal_poses(self, task_id, poses):
        poses = adapt_start_pose_for_ackermann(poses, self.current_pose)
        if self.task_type(task_id) != "taught_route":
            return poses
        return self.thin_taught_route_goals(poses, min_spacing=2.5)

    def thin_taught_route_goals(self, poses, min_spacing):
        if len(poses) <= 2:
            return poses
        thinned = [poses[0]]
        for pose in poses[1:-1]:
            previous = thinned[-1]
            if math.hypot(pose[0] - previous[0], pose[1] - previous[1]) >= min_spacing:
                thinned.append(pose)
        if thinned[-1] != poses[-1]:
            thinned.append(poses[-1])
        return thinned

    def send_nav_goal(self, poses):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [self.pose_stamped(x, y, yaw) for x, y, yaw in poses]
        self.remaining_poses = list(poses)
        self.publish_active_path(goal_msg.poses)
        if goal_msg.poses:
            first = goal_msg.poses[0].pose.position
            self.goal_pub.publish(String(data=f"x={first.x:.3f}; y={first.y:.3f}"))
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.nav_goal_pending = False
            self.state = "BLOCKED"
            self.publish_status("BLOCKED; reason=nav2_action_unavailable")
            return
        self.nav_goal_pending = True
        self.get_logger().info(f"sending Nav2 goal with {len(goal_msg.poses)} poses")
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

    def publish_active_path(self, poses):
        path = Path()
        path.header.frame_id = self.task_map["site"]["map_frame"]
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses
        self.active_path_pub.publish(path)

    def on_goal_response(self, future):
        self.nav_goal_pending = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.active_goal_handle = None
            self.state = "BLOCKED"
            self.publish_status("BLOCKED; reason=nav2_goal_rejected")
            return
        self.get_logger().info("Nav2 goal accepted")
        self.active_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        if self.active_goal_handle is None:
            return
        result = future.result()
        self.active_goal_handle = None
        self.nav_goal_pending = False
        self.set_push(False)
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.state = "COMPLETED"
            self.publish_status("COMPLETED")
        else:
            self.state = "BLOCKED"
            self.publish_status(f"BLOCKED; reason=nav2_status_{result.status}")

    def cancel_active_goal(self, state):
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None
        self.nav_goal_pending = False
        if state in ("CANCELED", "MANUAL_OVERRIDE"):
            self.active_task_id = None
            self.remaining_poses = []
        self.state = state
        self.set_push(False)
        self.publish_active_path([])
        self.publish_status(state)

    def pause_task(self):
        if not self.has_active_or_pending_goal():
            self.publish_status("PAUSED; reason=no_active_goal")
            self.state = "PAUSED"
            return
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None
        self.nav_goal_pending = False
        self.state = "PAUSED"
        self.set_push(False)
        self.publish_active_path([])
        self.publish_status(f"PAUSED; task={self.active_task_id or ''}")

    def resume_task(self):
        if not self.active_task_id:
            self.publish_status("BLOCKED; reason=no_paused_task")
            self.state = "BLOCKED"
            return
        try:
            self.reload_task_map()
            poses, _warnings = executable_goal_poses_for_task(
                self.task_map,
                self.active_task_id,
            )
            poses = self.prepare_ackermann_goal_poses(self.active_task_id, poses)
        except ValueError as exc:
            self.state = "BLOCKED"
            self.publish_status(f"BLOCKED; reason={exc}")
            return
        self.set_push(self.task_is_push(self.active_task_id))
        self.state = "RUNNING"
        self.publish_status(f"RUNNING; task={self.active_task_id}; goals={len(poses)}")
        self.send_nav_goal(poses)

    def return_home(self):
        if self.has_active_or_pending_goal():
            self.cancel_active_goal("CANCELED")
        try:
            self.reload_task_map()
            home = item_by_id(self.task_map["waypoints"], "home", "waypoint")
            pose = home.get("pose")
            if not isinstance(pose, (list, tuple)) or len(pose) != 3:
                raise ValueError("waypoint home pose must be 3 numeric values")
        except ValueError as exc:
            self.state = "BLOCKED"
            self.publish_status(f"BLOCKED; reason={exc}")
            return
        if self.current_pose is not None:
            distance = math.hypot(
                float(pose[0]) - self.current_pose[0],
                float(pose[1]) - self.current_pose[1],
            )
            if distance <= 0.25:
                self.active_task_id = None
                self.remaining_poses = []
                self.state = "HOME_REACHED"
                self.publish_active_path([])
                self.publish_status(
                    f"HOME_REACHED; reason=home_already_within_tolerance; "
                    f"distance={distance:.3f}"
                )
                return
        self.active_task_id = "return_home"
        self.set_push(False)
        self.state = "RUNNING"
        self.publish_status("RUNNING; task=return_home; goals=1")
        self.send_nav_goal([tuple(pose)])

    def publish_unsupported(self, command):
        if self.has_active_or_pending_goal():
            self.state = "RUNNING"
            self.publish_status(f"RUNNING; warning=unsupported_command:{command}")
            return
        self.state = "BLOCKED"
        self.publish_status(f"BLOCKED; reason=unsupported_command:{command}")

    def publish_status(self, text):
        self.last_status = text
        self.status_pub.publish(String(data=text))

    def republish_status(self):
        self.status_pub.publish(String(data=self.last_status))


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
