#!/usr/bin/env python3
"""Diagnose FAST-LIO odometry drift against a selected reference trajectory.

Ground truth is accepted only through reference_topic for simulation evaluation.
It is not published or fed into production localization fusion.
"""

import argparse
import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pose_tuple(msg):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    return (float(position.x), float(position.y), yaw_from_quat(orientation))


def compute_delta_metrics(
    reference_start,
    reference_end,
    estimate_start,
    estimate_end,
):
    reference_dx = reference_end[0] - reference_start[0]
    reference_dy = reference_end[1] - reference_start[1]
    estimate_dx = estimate_end[0] - estimate_start[0]
    estimate_dy = estimate_end[1] - estimate_start[1]

    reference_distance = math.hypot(reference_dx, reference_dy)
    estimate_distance = math.hypot(estimate_dx, estimate_dy)
    translation_error = math.hypot(
        estimate_dx - reference_dx,
        estimate_dy - reference_dy,
    )
    yaw_error = wrap_angle(
        (estimate_end[2] - estimate_start[2])
        - (reference_end[2] - reference_start[2])
    )

    if reference_distance > 0.0:
        scale_ratio = estimate_distance / reference_distance
        drift_per_meter = translation_error / reference_distance
    else:
        scale_ratio = 0.0
        drift_per_meter = 0.0

    return {
        "reference_distance": reference_distance,
        "estimate_distance": estimate_distance,
        "scale_ratio": scale_ratio,
        "translation_error": translation_error,
        "drift_per_meter": drift_per_meter,
        "yaw_error": yaw_error,
    }


class FastLioDriftDiagnostic(Node):
    def __init__(self, args):
        super().__init__("fast_lio_drift_diagnostic")
        self.output = Path(args.output)
        self.reference_topic = args.reference_topic
        self.reference_start = None
        self.reference_latest = None
        self.estimate_starts = {}
        self.estimate_latest = {}

        self.create_subscription(
            Odometry,
            self.reference_topic,
            self.on_reference,
            10,
        )
        self.create_subscription(
            Odometry,
            "/mapping/lio/odom",
            lambda msg: self.on_estimate("/mapping/lio/odom", msg),
            10,
        )
        self.create_subscription(
            Odometry,
            "/robot/odom",
            lambda msg: self.on_estimate("/robot/odom", msg),
            10,
        )
        self.create_subscription(
            Odometry,
            "/localization/wheel_lio_odom",
            lambda msg: self.on_estimate("/localization/wheel_lio_odom", msg),
            10,
        )
        self.create_timer(args.report_period, self.write_report)

    def on_reference(self, msg):
        pose = pose_tuple(msg)
        if self.reference_start is None:
            self.reference_start = pose
        self.reference_latest = pose

    def on_estimate(self, topic, msg):
        pose = pose_tuple(msg)
        if topic not in self.estimate_starts:
            self.estimate_starts[topic] = pose
        self.estimate_latest[topic] = pose

    def build_report(self):
        report = {
            "reference_topic": self.reference_topic,
            "estimates": {},
        }
        if self.reference_start is None or self.reference_latest is None:
            return report

        for topic, estimate_start in self.estimate_starts.items():
            estimate_end = self.estimate_latest.get(topic)
            if estimate_end is None:
                continue
            report["estimates"][topic] = compute_delta_metrics(
                reference_start=self.reference_start,
                reference_end=self.reference_latest,
                estimate_start=estimate_start,
                estimate_end=estimate_end,
            )
        return report

    def write_report(self):
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(
            json.dumps(self.build_report(), indent=2, sort_keys=True),
            encoding="utf-8",
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Export FAST-LIO drift diagnostics to a .json file.",
    )
    parser.add_argument(
        "--reference-topic",
        dest="reference_topic",
        default="/robot/ground_truth/odom",
        help="Reference odometry topic for evaluation only.",
    )
    parser.add_argument(
        "--output",
        default="fast_lio_drift_diagnostic.json",
        help="Output .json diagnostics path.",
    )
    parser.add_argument(
        "--report-period",
        type=float,
        default=1.0,
        help="Seconds between JSON report writes.",
    )
    parser.add_argument(
        "--duration-sec",
        dest="duration_sec",
        type=float,
        default=30.0,
        help="Seconds to collect odometry before exporting and exiting.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = FastLioDriftDiagnostic(args)
    try:
        deadline = node.get_clock().now() + Duration(seconds=args.duration_sec)
        while rclpy.ok() and node.get_clock().now() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.write_report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
