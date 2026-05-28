#!/usr/bin/env python3
"""Publish FAST-LIO, wheel odometry, and gated GPS fused localization output."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


EARTH_RADIUS_M = 6378137.0


class LioWheelFusion(Node):
    def __init__(self):
        super().__init__("lio_wheel_fusion")
        self.declare_parameter("wheel_timeout_sec", 0.25)
        self.declare_parameter("gps_timeout_sec", 1.0)
        self.declare_parameter("gps_blend_weight", 0.05)
        self.latest_wheel_odom = None
        self.latest_wheel_stamp = None
        self.latest_gps = None
        self.latest_gps_stamp = None
        self.gps_origin = None
        self.lio_origin_xy = None
        self.publisher = self.create_publisher(
            Odometry,
            "/localization/fused_odom",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/localization/fusion_status",
            10,
        )
        self.create_subscription(
            Odometry,
            "/mapping/lio/odom",
            self.on_lio_odom,
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

    def on_wheel_odom(self, msg):
        self.latest_wheel_odom = msg
        self.latest_wheel_stamp = self.get_clock().now()

    def on_gps(self, msg):
        self.latest_gps = msg
        self.latest_gps_stamp = self.get_clock().now()
        if self.gps_origin is None:
            self.gps_origin = (msg.latitude, msg.longitude)

    def on_lio_odom(self, lio):
        fused = Odometry()
        fused.header.stamp = lio.header.stamp
        fused.header.frame_id = "map"
        fused.child_frame_id = "base_link"
        fused.pose = lio.pose
        if self.lio_origin_xy is None:
            self.lio_origin_xy = (
                lio.pose.pose.position.x,
                lio.pose.pose.position.y,
            )

        wheel = self.latest_fresh_wheel()
        if wheel is not None:
            fused.twist = wheel.twist
        else:
            fused.twist = lio.twist

        gps = self.latest_fresh_gps()
        if gps is not None:
            self.apply_gps_position_correction(fused, gps)

        self.publisher.publish(fused)
        self.publish_status(wheel, gps)

    def latest_fresh_wheel(self):
        if self.latest_wheel_odom is None or self.latest_wheel_stamp is None:
            return None
        timeout = self.get_parameter("wheel_timeout_sec").value
        if self.get_clock().now() - self.latest_wheel_stamp > Duration(seconds=timeout):
            return None
        wheel = self.latest_wheel_odom
        _ = wheel.twist.twist
        return wheel

    def latest_fresh_gps(self):
        if self.latest_gps is None or self.latest_gps_stamp is None:
            return None
        timeout = self.get_parameter("gps_timeout_sec").value
        if self.get_clock().now() - self.latest_gps_stamp > Duration(seconds=timeout):
            return None
        return self.latest_gps

    def gps_to_local_xy(self, gps):
        if self.gps_origin is None or self.lio_origin_xy is None:
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
            self.lio_origin_xy[0] + dx,
            self.lio_origin_xy[1] + dy,
        )

    def apply_gps_position_correction(self, fused, gps):
        gps_xy = self.gps_to_local_xy(gps)
        if gps_xy is None:
            return
        gps_blend_weight = float(self.get_parameter("gps_blend_weight").value)
        gps_blend_weight = max(0.0, min(1.0, gps_blend_weight))
        fused.pose.pose.position.x = (
            (1.0 - gps_blend_weight) * fused.pose.pose.position.x
            + gps_blend_weight * gps_xy[0]
        )
        fused.pose.pose.position.y = (
            (1.0 - gps_blend_weight) * fused.pose.pose.position.y
            + gps_blend_weight * gps_xy[1]
        )

    def publish_status(self, wheel, gps):
        wheel_state = "wheel=fresh" if wheel is not None else "wheel=stale"
        gps_state = "gps=gated" if gps is not None else "gps=none"
        self.status_pub.publish(String(data=f"{wheel_state}; {gps_state}"))


def main():
    rclpy.init()
    node = LioWheelFusion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
