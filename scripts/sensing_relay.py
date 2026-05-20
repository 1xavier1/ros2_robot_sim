#!/usr/bin/env python3
"""Relay one ROS 2 topic to another for a configured message type."""

import argparse
import importlib

import rclpy
from rclpy.node import Node


def load_message_class(message_type):
    package_name, namespace, class_name = message_type.split("/")
    if namespace != "msg":
        raise ValueError(f"unsupported ROS interface namespace: {namespace}")
    module = importlib.import_module(f"{package_name}.msg")
    return getattr(module, class_name)


class SensingRelay(Node):
    def __init__(self, input_topic, output_topic, message_type):
        super().__init__("sensing_relay")
        message_class = load_message_class(message_type)
        self.publisher = self.create_publisher(message_class, output_topic, 10)
        self.subscription = self.create_subscription(
            message_class,
            input_topic,
            self.publisher.publish,
            10,
        )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", required=True)
    parser.add_argument("--output-topic", required=True)
    parser.add_argument("--message-type", required=True)
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = SensingRelay(args.input_topic, args.output_topic, args.message_type)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
