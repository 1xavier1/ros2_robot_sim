#!/usr/bin/env python3
"""Publish a global localization layer for Nav2.

This node is intentionally conservative. FAST-LIO remains the mapping front end;
this backend consumes the downstream fused odom, applies bounded GPS anchor
offsets when gated GPS is fresh, and can apply a small odom-proximity loop
correction when explicitly enabled. It is not a scan-matching pose graph.
"""

import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String


EARTH_RADIUS_M = 6378137.0


class GlobalLocalizationBackend(Node):
    def __init__(self):
        super().__init__("global_localization_backend")
        self.declare_parameter("gps_timeout_sec", 1.0)
        self.declare_parameter("wheel_timeout_sec", 0.25)
        self.declare_parameter("imu_timeout_sec", 0.25)
        self.declare_parameter("gps_anchor_blend_weight", 0.02)
        self.declare_parameter("enable_loop_closure", False)
        self.declare_parameter("loop_keyframe_distance", 1.0)
        self.declare_parameter("loop_candidate_radius", 0.75)
        self.declare_parameter("loop_min_travel_distance", 8.0)
        self.declare_parameter("loop_correction_gain", 0.15)
        self.declare_parameter("max_loop_correction_step", 0.20)

        self.latest_gps = None
        self.latest_gps_stamp = None
        self.latest_wheel_stamp = None
        self.latest_imu_stamp = None
        self.gps_origin = None
        self.odom_origin_xy = None
        self.global_offset_x = 0.0
        self.global_offset_y = 0.0
        self.keyframes = []
        self.path_distance = 0.0
        self.last_keyframe_xy = None
        self.pending_loop_correction = None
        self.loop_status = "loop_closure=disabled"

        self.odom_pub = self.create_publisher(
            Odometry,
            "/localization/global_odom",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/localization/backend_status",
            10,
        )
        self.loop_pub = self.create_publisher(
            String,
            "/localization/loop_closure_status",
            10,
        )

        self.create_subscription(
            Odometry,
            "/localization/fused_odom",
            self.on_fused_odom,
            10,
        )
        self.create_subscription(
            Odometry,
            "/robot/odom",
            self.on_wheel_odom,
            50,
        )
        self.create_subscription(
            NavSatFix,
            "/localization/gps/gated",
            self.on_gps,
            10,
        )
        self.create_subscription(
            Imu,
            "/sensing/imu/data",
            self.on_imu,
            50,
        )

    def on_wheel_odom(self, _msg):
        self.latest_wheel_stamp = self.get_clock().now()

    def on_imu(self, _msg):
        self.latest_imu_stamp = self.get_clock().now()

    def on_gps(self, msg):
        self.latest_gps = msg
        self.latest_gps_stamp = self.get_clock().now()
        if self.gps_origin is None:
            self.gps_origin = (msg.latitude, msg.longitude)

    def on_fused_odom(self, msg):
        global_odom = copy.deepcopy(msg)
        global_odom.header.frame_id = "map"
        global_odom.child_frame_id = "base_link"

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.odom_origin_xy is None:
            self.odom_origin_xy = (x, y)

        gps = self.latest_fresh_gps()
        if gps is not None:
            self.update_gps_anchor_offset(x, y, gps)

        self.update_loop_status(x + self.global_offset_x, y + self.global_offset_y)
        self.apply_loop_correction()

        global_odom.pose.pose.position.x = x + self.global_offset_x
        global_odom.pose.pose.position.y = y + self.global_offset_y

        self.odom_pub.publish(global_odom)
        self.publish_status(gps)

    def latest_fresh_gps(self):
        if self.latest_gps is None or self.latest_gps_stamp is None:
            return None
        timeout = self.get_parameter("gps_timeout_sec").value
        if self.get_clock().now() - self.latest_gps_stamp > Duration(seconds=timeout):
            return None
        return self.latest_gps

    def is_stamp_fresh(self, stamp, timeout_param):
        if stamp is None:
            return False
        timeout = self.get_parameter(timeout_param).value
        return self.get_clock().now() - stamp <= Duration(seconds=timeout)

    def gps_to_local_xy(self, gps):
        if self.gps_origin is None or self.odom_origin_xy is None:
            return None
        origin_lat, origin_lon = self.gps_origin
        lat_rad = math.radians(origin_lat)
        dx = (
            math.radians(gps.longitude - origin_lon)
            * EARTH_RADIUS_M
            * math.cos(lat_rad)
        )
        dy = math.radians(gps.latitude - origin_lat) * EARTH_RADIUS_M
        return (
            self.odom_origin_xy[0] + dx,
            self.odom_origin_xy[1] + dy,
        )

    def update_gps_anchor_offset(self, current_x, current_y, gps):
        gps_xy = self.gps_to_local_xy(gps)
        if gps_xy is None:
            return
        weight = float(self.get_parameter("gps_anchor_blend_weight").value)
        weight = max(0.0, min(1.0, weight))
        residual_x = gps_xy[0] - (current_x + self.global_offset_x)
        residual_y = gps_xy[1] - (current_y + self.global_offset_y)
        self.global_offset_x += weight * residual_x
        self.global_offset_y += weight * residual_y

    def update_loop_status(self, x, y):
        if not bool(self.get_parameter("enable_loop_closure").value):
            self.loop_status = "loop_closure=disabled"
            return

        keyframe_distance = float(self.get_parameter("loop_keyframe_distance").value)
        if self.last_keyframe_xy is None:
            self.last_keyframe_xy = (x, y)
            self.keyframes.append((x, y, self.path_distance))
            self.loop_status = "loop_closure=searching"
            return

        step = math.hypot(x - self.last_keyframe_xy[0], y - self.last_keyframe_xy[1])
        if step < keyframe_distance:
            self.loop_status = "loop_closure=searching"
            return

        self.path_distance += step
        self.last_keyframe_xy = (x, y)
        candidate_radius = float(self.get_parameter("loop_candidate_radius").value)
        min_travel = float(self.get_parameter("loop_min_travel_distance").value)
        for kx, ky, kdist in self.keyframes[:-3]:
            if self.path_distance - kdist < min_travel:
                continue
            distance_to_candidate = math.hypot(x - kx, y - ky)
            if distance_to_candidate <= candidate_radius:
                self.pending_loop_correction = (kx - x, ky - y)
                self.loop_status = "loop_closure=candidate_odom_proximity"
                self.keyframes.append((x, y, self.path_distance))
                return
        self.keyframes.append((x, y, self.path_distance))
        self.loop_status = "loop_closure=searching"

    def apply_loop_correction(self):
        if self.pending_loop_correction is None:
            return
        correction_x, correction_y = self.pending_loop_correction
        gain = float(self.get_parameter("loop_correction_gain").value)
        gain = max(0.0, min(1.0, gain))
        max_step = max(0.0, float(self.get_parameter("max_loop_correction_step").value))
        step_x = correction_x * gain
        step_y = correction_y * gain
        step_norm = math.hypot(step_x, step_y)
        if max_step > 0.0 and step_norm > max_step:
            scale = max_step / step_norm
            step_x *= scale
            step_y *= scale
        self.global_offset_x += step_x
        self.global_offset_y += step_y

        remaining_x = correction_x - step_x
        remaining_y = correction_y - step_y
        if math.hypot(remaining_x, remaining_y) <= 0.05:
            self.pending_loop_correction = None
        else:
            self.pending_loop_correction = (remaining_x, remaining_y)
        self.loop_status = "loop_closure=applied_odom_proximity"

    def publish_status(self, gps):
        gps_state = "gps_anchor=fresh" if gps is not None else "gps_anchor=stale"
        wheel_state = (
            "wheel=fresh"
            if self.is_stamp_fresh(self.latest_wheel_stamp, "wheel_timeout_sec")
            else "wheel=stale"
        )
        imu_state = (
            "imu=fresh"
            if self.is_stamp_fresh(self.latest_imu_stamp, "imu_timeout_sec")
            else "imu=stale"
        )
        offset = f"offset=({self.global_offset_x:.3f},{self.global_offset_y:.3f})"
        status = f"{gps_state}; {wheel_state}; {imu_state}; {self.loop_status}; {offset}"
        self.status_pub.publish(String(data=status))
        self.loop_pub.publish(String(data=self.loop_status))


def main():
    rclpy.init()
    node = GlobalLocalizationBackend()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
