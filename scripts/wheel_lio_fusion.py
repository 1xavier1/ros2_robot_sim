#!/usr/bin/env python3
"""Fuse wheel translation, FAST-LIO yaw, and optional gated GPS anchoring."""

import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


EARTH_RADIUS_M = 6378137.0


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def pose_tuple(msg):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    return (float(position.x), float(position.y), yaw_from_quat(orientation))


def gps_to_local_xy(gps, origin):
    origin_lat, origin_lon = origin
    lat_rad = math.radians(origin_lat)
    dx = (
        math.radians(gps.longitude - origin_lon)
        * EARTH_RADIUS_M
        * math.cos(lat_rad)
    )
    dy = math.radians(gps.latitude - origin_lat) * EARTH_RADIUS_M
    return (dx, dy)


def compose_wheel_lio_pose(
    lio_anchor,
    wheel_anchor,
    wheel_current,
    lio_current,
    use_lio_yaw=True,
):
    wheel_dx = wheel_current[0] - wheel_anchor[0]
    wheel_dy = wheel_current[1] - wheel_anchor[1]
    anchor_yaw_delta = lio_anchor[2] - wheel_anchor[2]
    cos_yaw = math.cos(anchor_yaw_delta)
    sin_yaw = math.sin(anchor_yaw_delta)

    x = lio_anchor[0] + cos_yaw * wheel_dx - sin_yaw * wheel_dy
    y = lio_anchor[1] + sin_yaw * wheel_dx + cos_yaw * wheel_dy
    if use_lio_yaw:
        yaw = lio_current[2]
    else:
        yaw = lio_anchor[2] + wrap_angle(wheel_current[2] - wheel_anchor[2])
    return (x, y, wrap_angle(yaw))


class WheelLioFusion(Node):
    def __init__(self):
        super().__init__("wheel_lio_fusion")
        self.declare_parameter("lio_timeout_sec", 0.5)
        self.declare_parameter("wheel_timeout_sec", 0.25)
        self.declare_parameter("gps_timeout_sec", 1.0)
        self.declare_parameter("gps_anchor_blend_weight", 0.0)
        self.declare_parameter("max_lio_translation_error", 1.0)

        self.lio_anchor = None
        self.wheel_anchor = None
        self.latest_lio = None
        self.latest_wheel = None
        self.latest_gps = None
        self.latest_lio_stamp = None
        self.latest_wheel_stamp = None
        self.latest_gps_stamp = None
        self.gps_origin = None
        self.fused_origin_xy = None
        self.global_offset = (0.0, 0.0)

        self.odom_pub = self.create_publisher(
            Odometry,
            "/localization/wheel_lio_odom",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/localization/wheel_lio_status",
            10,
        )
        self.create_subscription(
            Odometry,
            "/mapping/lio/odom",
            self.on_lio,
            10,
        )
        self.create_subscription(
            Odometry,
            "/robot/odom",
            self.on_wheel,
            50,
        )
        self.create_subscription(
            NavSatFix,
            "/localization/gps/gated",
            self.on_gps,
            10,
        )

    def on_lio(self, msg):
        self.latest_lio = msg
        self.latest_lio_stamp = self.get_clock().now()
        self.publish_if_ready()

    def on_wheel(self, msg):
        self.latest_wheel = msg
        self.latest_wheel_stamp = self.get_clock().now()
        self.publish_if_ready()

    def on_gps(self, msg):
        self.latest_gps = msg
        self.latest_gps_stamp = self.get_clock().now()
        if self.gps_origin is None:
            self.gps_origin = (msg.latitude, msg.longitude)

    def is_fresh(self, stamp, timeout_parameter):
        if stamp is None:
            return False
        timeout = float(self.get_parameter(timeout_parameter).value)
        return self.get_clock().now() - stamp <= Duration(seconds=timeout)

    def publish_if_ready(self):
        if self.latest_lio is None or self.latest_wheel is None:
            return
        if not self.is_fresh(self.latest_wheel_stamp, "wheel_timeout_sec"):
            self.publish_status("wheel=stale")
            return

        lio_pose = pose_tuple(self.latest_lio)
        wheel_pose = pose_tuple(self.latest_wheel)
        if self.lio_anchor is None or self.wheel_anchor is None:
            self.lio_anchor = lio_pose
            self.wheel_anchor = wheel_pose

        use_lio_yaw = self.is_fresh(self.latest_lio_stamp, "lio_timeout_sec")
        fused_pose = compose_wheel_lio_pose(
            self.lio_anchor,
            self.wheel_anchor,
            wheel_pose,
            lio_pose,
            use_lio_yaw=use_lio_yaw,
        )
        self.refresh_anchor_if_lio_drift_exceeds_limit(fused_pose, lio_pose, wheel_pose)

        fused = copy.deepcopy(self.latest_lio)
        fused.header.stamp = self.latest_lio.header.stamp
        fused.header.frame_id = "map"
        fused.child_frame_id = "base_link"
        fused.pose.pose.position.x = fused_pose[0] + self.global_offset[0]
        fused.pose.pose.position.y = fused_pose[1] + self.global_offset[1]
        fused.pose.pose.position.z = self.latest_lio.pose.pose.position.z
        qx, qy, qz, qw = quat_from_yaw(fused_pose[2])
        fused.pose.pose.orientation.x = qx
        fused.pose.pose.orientation.y = qy
        fused.pose.pose.orientation.z = qz
        fused.pose.pose.orientation.w = qw
        fused.twist = self.latest_wheel.twist

        self.apply_gps_anchor(fused)
        self.odom_pub.publish(fused)
        lio_state = "lio_yaw=fresh" if use_lio_yaw else "lio_yaw=fallback"
        self.publish_status(f"{lio_state}; wheel=fresh")

    def refresh_anchor_if_lio_drift_exceeds_limit(
        self,
        fused_pose,
        lio_pose,
        wheel_pose,
    ):
        max_error = float(self.get_parameter("max_lio_translation_error").value)
        if max_error <= 0.0:
            return
        error = math.hypot(fused_pose[0] - lio_pose[0], fused_pose[1] - lio_pose[1])
        if error <= max_error:
            return
        self.lio_anchor = lio_pose
        self.wheel_anchor = wheel_pose

    def apply_gps_anchor(self, fused):
        if self.latest_gps is None or self.gps_origin is None:
            return
        if not self.is_fresh(self.latest_gps_stamp, "gps_timeout_sec"):
            return
        weight = float(self.get_parameter("gps_anchor_blend_weight").value)
        weight = max(0.0, min(1.0, weight))
        if weight <= 0.0:
            return

        if self.fused_origin_xy is None:
            self.fused_origin_xy = (
                fused.pose.pose.position.x,
                fused.pose.pose.position.y,
            )
        gps_dx, gps_dy = gps_to_local_xy(self.latest_gps, self.gps_origin)
        gps_x = self.fused_origin_xy[0] + gps_dx
        gps_y = self.fused_origin_xy[1] + gps_dy
        residual_x = gps_x - fused.pose.pose.position.x
        residual_y = gps_y - fused.pose.pose.position.y
        self.global_offset = (
            self.global_offset[0] + weight * residual_x,
            self.global_offset[1] + weight * residual_y,
        )
        fused.pose.pose.position.x += weight * residual_x
        fused.pose.pose.position.y += weight * residual_y

    def gps_state(self):
        if self.latest_gps is None:
            return "gps=none"
        if self.is_fresh(self.latest_gps_stamp, "gps_timeout_sec"):
            return "gps=fresh"
        return "gps=stale"

    def publish_status(self, prefix):
        self.status_pub.publish(String(data=f"{prefix}; {self.gps_state()}"))


def main():
    rclpy.init()
    node = WheelLioFusion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
