#!/usr/bin/env python3
"""Bridge FAST-LIO map->base_link odom into the Nav2 TF tree.

Publishes map->odom by combining T_map_base (FAST-LIO) with inv(T_odom_basefootprint).
Nav2 receives: map -> odom -> base_footprint -> base_link
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def quat_inverse(qx, qy, qz, qw):
    return (-qx, -qy, -qz, qw)


def quat_multiply(q1x, q1y, q1z, q1w, q2x, q2y, q2z, q2w):
    return (
        q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y,
        q1w * q2y - q1x * q2z + q1y * q2w + q1z * q2x,
        q1w * q2z + q1x * q2y - q1y * q2x + q1z * q2w,
        q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z,
    )


def quat_rotate(qx, qy, qz, qw, vx, vy, vz):
    qvx, qvy, qvz, qvw = quat_multiply(
        qx, qy, qz, qw,
        vx, vy, vz, 0.0,
    )
    neg_qx, neg_qy, neg_qz, neg_qw = quat_inverse(qx, qy, qz, qw)
    rx, ry, rz, _ = quat_multiply(qvx, qvy, qvz, qvw, neg_qx, neg_qy, neg_qz, neg_qw)
    return rx, ry, rz


class LioTfAdapter(Node):
    def __init__(self):
        super().__init__("lio_tf_adapter")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_lio_odom = None
        self.latest_wheel_odom = None
        self.lio_sub = self.create_subscription(
            Odometry, "/mapping/lio/odom", self.on_lio_odom, 10
        )
        self.wheel_sub = self.create_subscription(
            Odometry, "/robot/odom", self.on_wheel_odom, 10
        )
        self.timer = self.create_timer(0.05, self.publish_tf)

    def on_lio_odom(self, msg):
        self.latest_lio_odom = msg

    def on_wheel_odom(self, msg):
        self.latest_wheel_odom = msg

    def publish_tf(self):
        if self.latest_lio_odom is None or self.latest_wheel_odom is None:
            return
        lio = self.latest_lio_odom
        wheel = self.latest_wheel_odom

        # T_map_base from FAST-LIO
        m_x = lio.pose.pose.position.x
        m_y = lio.pose.pose.position.y
        m_z = lio.pose.pose.position.z
        m_qx = lio.pose.pose.orientation.x
        m_qy = lio.pose.pose.orientation.y
        m_qz = lio.pose.pose.orientation.z
        m_qw = lio.pose.pose.orientation.w

        # T_odom_basefootprint from wheel odometry (inverse of what we need)
        o_x = wheel.pose.pose.position.x
        o_y = wheel.pose.pose.position.y
        o_z = wheel.pose.pose.position.z
        o_qx = wheel.pose.pose.orientation.x
        o_qy = wheel.pose.pose.orientation.y
        o_qz = wheel.pose.pose.orientation.z
        o_qw = wheel.pose.pose.orientation.w

        # T_map_odom = T_map_base * inv(T_odom_basefootprint)
        inv_o_qx, inv_o_qy, inv_o_qz, inv_o_qw = quat_inverse(o_qx, o_qy, o_qz, o_qw)
        inv_o_x, inv_o_y, inv_o_z = quat_rotate(
            inv_o_qx, inv_o_qy, inv_o_qz, inv_o_qw,
            -o_x, -o_y, -o_z,
        )

        # Compose: first rotate inv_o_pos by map quat, then add map pos
        map_odom_x = m_x + quat_rotate(m_qx, m_qy, m_qz, m_qw, inv_o_x, inv_o_y, inv_o_z)[0]
        map_odom_y = m_y + quat_rotate(m_qx, m_qy, m_qz, m_qw, inv_o_x, inv_o_y, inv_o_z)[1]
        map_odom_z = m_z + quat_rotate(m_qx, m_qy, m_qz, m_qw, inv_o_x, inv_o_y, inv_o_z)[2]
        map_odom_qx, map_odom_qy, map_odom_qz, map_odom_qw = quat_multiply(
            m_qx, m_qy, m_qz, m_qw,
            inv_o_qx, inv_o_qy, inv_o_qz, inv_o_qw,
        )

        t = TransformStamped()
        t.header.stamp = lio.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = map_odom_x
        t.transform.translation.y = map_odom_y
        t.transform.translation.z = map_odom_z
        t.transform.rotation.x = map_odom_qx
        t.transform.rotation.y = map_odom_qy
        t.transform.rotation.z = map_odom_qz
        t.transform.rotation.w = map_odom_qw
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = LioTfAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
