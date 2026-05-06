#!/usr/bin/env python3
"""转发 cmd_vel 到后轮驱动"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelForwarder(Node):
    def __init__(self):
        super().__init__('cmd_vel_forwarder')
        self.subscription = self.create_subscription(
            Twist,
            '/robot/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.publisher = self.create_publisher(
            Twist,
            '/robot/rear/cmd_vel',
            10
        )
        self.get_logger().info('CmdVel forwarder started')

    def cmd_vel_callback(self, msg):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    forwarder = CmdVelForwarder()
    rclpy.spin(forwarder)
    forwarder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
