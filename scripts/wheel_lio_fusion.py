#!/usr/bin/env python3
"""Fuse wheel translation, FAST-LIO yaw, and optional gated GPS anchoring."""

import math
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


EARTH_RADIUS_M = 6378137.0


@dataclass(frozen=True)
class MotionDelta:
    dx: float
    dy: float
    distance: float
    heading: float
    yaw_delta: float
    speed: float


@dataclass(frozen=True)
class MotionComparison:
    distance_diff: float
    direction_diff: float
    yaw_diff: float
    wheel_lio_speed_ratio: float
    lio_wheel_speed_ratio: float


@dataclass(frozen=True)
class FusionThresholds:
    motion_window_min_distance: float = 0.05
    wheel_lio_distance_warn: float = 0.15
    wheel_lio_distance_error: float = 0.35
    wheel_lio_speed_ratio_warn: float = 1.8
    wheel_lio_speed_ratio_error: float = 3.0
    yaw_delta_warn: float = 0.25
    yaw_delta_error: float = 0.60
    turning_yaw_rate_threshold: float = 0.25
    max_consecutive_bad_frames: int = 5


@dataclass(frozen=True)
class FusionDecision:
    state: str
    reason: str


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_motion_delta(previous_pose, current_pose, dt):
    dx = current_pose[0] - previous_pose[0]
    dy = current_pose[1] - previous_pose[1]
    distance = math.hypot(dx, dy)
    heading = math.atan2(dy, dx) if distance > 1e-9 else previous_pose[2]
    yaw_delta = wrap_angle(current_pose[2] - previous_pose[2])
    speed = distance / dt if dt > 1e-6 else 0.0
    return MotionDelta(
        dx=dx,
        dy=dy,
        distance=distance,
        heading=heading,
        yaw_delta=yaw_delta,
        speed=speed,
    )


def speed_ratio(high, low):
    if low <= 1e-6:
        return float("inf") if high > 1e-6 else 1.0
    return high / low


def compare_wheel_lio_motion(wheel_delta, lio_delta):
    return MotionComparison(
        distance_diff=abs(wheel_delta.distance - lio_delta.distance),
        direction_diff=abs(wrap_angle(wheel_delta.heading - lio_delta.heading)),
        yaw_diff=abs(wrap_angle(wheel_delta.yaw_delta - lio_delta.yaw_delta)),
        wheel_lio_speed_ratio=speed_ratio(wheel_delta.speed, lio_delta.speed),
        lio_wheel_speed_ratio=speed_ratio(lio_delta.speed, wheel_delta.speed),
    )


def classify_fusion_state(
    wheel_delta,
    lio_delta,
    thresholds,
    consecutive_bad_frames,
):
    comparison = compare_wheel_lio_motion(wheel_delta, lio_delta)
    if consecutive_bad_frames >= thresholds.max_consecutive_bad_frames:
        return FusionDecision("degraded", "consecutive_bad_frames")
    if comparison.yaw_diff >= thresholds.yaw_delta_error:
        return FusionDecision("degraded", "yaw_delta_error")
    if abs(wheel_delta.yaw_delta) >= thresholds.turning_yaw_rate_threshold:
        return FusionDecision("turning_caution", "yaw_rate_high")
    if (
        max(wheel_delta.distance, lio_delta.distance)
        < thresholds.motion_window_min_distance
    ):
        return FusionDecision("normal", "insufficient_motion")
    if (
        wheel_delta.distance > lio_delta.distance
        and comparison.distance_diff >= thresholds.wheel_lio_distance_error
    ) or comparison.wheel_lio_speed_ratio >= thresholds.wheel_lio_speed_ratio_error:
        return FusionDecision("wheel_suspect", "wheel_distance_high")
    if (
        lio_delta.distance > wheel_delta.distance
        and comparison.distance_diff >= thresholds.wheel_lio_distance_error
    ) or comparison.lio_wheel_speed_ratio >= thresholds.wheel_lio_speed_ratio_error:
        return FusionDecision("lio_suspect", "lio_distance_high")
    if comparison.distance_diff >= thresholds.wheel_lio_distance_warn:
        return FusionDecision("turning_caution", "distance_warn")
    if comparison.direction_diff >= thresholds.yaw_delta_warn:
        return FusionDecision("turning_caution", "direction_warn")
    if comparison.yaw_diff >= thresholds.yaw_delta_warn:
        return FusionDecision("turning_caution", "yaw_delta_warn")
    return FusionDecision("normal", "motion_consistent")


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


def should_refresh_lio_anchor(fused_pose, lio_pose, max_error):
    if max_error <= 0.0:
        return False
    translation_error = math.hypot(
        fused_pose[0] - lio_pose[0],
        fused_pose[1] - lio_pose[1],
    )
    return translation_error > max_error


def select_output_stamp(lio_stamp, current_stamp, use_lio_yaw):
    if use_lio_yaw:
        return lio_stamp
    return current_stamp


def can_initialize_anchor(use_lio_yaw):
    return use_lio_yaw


def maybe_refresh_anchor_and_pose(
    lio_anchor,
    wheel_anchor,
    wheel_pose,
    lio_pose,
    use_lio_yaw,
    max_error,
):
    fused_pose = compose_wheel_lio_pose(
        lio_anchor,
        wheel_anchor,
        wheel_pose,
        lio_pose,
        use_lio_yaw=use_lio_yaw,
    )
    if use_lio_yaw and should_refresh_lio_anchor(fused_pose, lio_pose, max_error):
        lio_anchor = lio_pose
        wheel_anchor = wheel_pose
        fused_pose = compose_wheel_lio_pose(
            lio_anchor,
            wheel_anchor,
            wheel_pose,
            lio_pose,
            use_lio_yaw=use_lio_yaw,
        )
    return lio_anchor, wheel_anchor, fused_pose


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

        use_lio_yaw = self.is_fresh(self.latest_lio_stamp, "lio_timeout_sec")
        if self.lio_anchor is None or self.wheel_anchor is None:
            if not can_initialize_anchor(use_lio_yaw):
                self.publish_status("lio=stale_waiting_anchor; wheel=fresh")
                return

        lio_pose = pose_tuple(self.latest_lio)
        wheel_pose = pose_tuple(self.latest_wheel)
        if self.lio_anchor is None or self.wheel_anchor is None:
            self.lio_anchor = lio_pose
            self.wheel_anchor = wheel_pose

        max_error = float(self.get_parameter("max_lio_translation_error").value)
        self.lio_anchor, self.wheel_anchor, fused_pose = maybe_refresh_anchor_and_pose(
            self.lio_anchor,
            self.wheel_anchor,
            wheel_pose,
            lio_pose,
            use_lio_yaw,
            max_error,
        )

        fused = Odometry()
        fused.header.stamp = select_output_stamp(
            self.latest_lio.header.stamp,
            self.get_clock().now().to_msg(),
            use_lio_yaw,
        )
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
