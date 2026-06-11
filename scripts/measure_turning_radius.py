#!/usr/bin/env python3
"""Measure simulated turning radius from odometry while commanding max steering."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def fit_circle(points):
    """Fit x^2 + y^2 + d*x + e*y + f = 0 using normal equations."""
    if len(points) < 3:
        raise ValueError("at least 3 points are required")

    sums = {
        "x": 0.0,
        "y": 0.0,
        "xx": 0.0,
        "yy": 0.0,
        "xy": 0.0,
        "xxx": 0.0,
        "yyy": 0.0,
        "xxy": 0.0,
        "xyy": 0.0,
        "r2": 0.0,
    }
    for x, y in points:
        xx = x * x
        yy = y * y
        r2 = xx + yy
        sums["x"] += x
        sums["y"] += y
        sums["xx"] += xx
        sums["yy"] += yy
        sums["xy"] += x * y
        sums["xxx"] += xx * x
        sums["yyy"] += yy * y
        sums["xxy"] += xx * y
        sums["xyy"] += x * yy
        sums["r2"] += r2

    matrix = [
        [sums["xx"], sums["xy"], sums["x"]],
        [sums["xy"], sums["yy"], sums["y"]],
        [sums["x"], sums["y"], float(len(points))],
    ]
    vector = [
        -(sums["xxx"] + sums["xyy"]),
        -(sums["xxy"] + sums["yyy"]),
        -sums["r2"],
    ]
    d, e, f = solve_3x3(matrix, vector)
    center_x = -d * 0.5
    center_y = -e * 0.5
    radius_sq = center_x * center_x + center_y * center_y - f
    if radius_sq <= 0.0:
        raise ValueError("fitted radius is invalid")
    return center_x, center_y, math.sqrt(radius_sq)


def solve_3x3(matrix, vector):
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    for pivot_index in range(3):
        pivot_row = max(
            range(pivot_index, 3),
            key=lambda index: abs(augmented[index][pivot_index]),
        )
        if abs(augmented[pivot_row][pivot_index]) < 1e-9:
            raise ValueError("singular fit matrix")
        augmented[pivot_index], augmented[pivot_row] = (
            augmented[pivot_row],
            augmented[pivot_index],
        )
        pivot = augmented[pivot_index][pivot_index]
        augmented[pivot_index] = [value / pivot for value in augmented[pivot_index]]
        for row_index in range(3):
            if row_index == pivot_index:
                continue
            factor = augmented[row_index][pivot_index]
            augmented[row_index] = [
                current - factor * base
                for current, base in zip(augmented[row_index], augmented[pivot_index])
            ]
    return [augmented[index][3] for index in range(3)]


class TurningRadiusMeasurer(Node):
    def __init__(self, args):
        super().__init__("turning_radius_measurer")
        self.args = args
        self.samples = []
        self.publisher = self.create_publisher(Twist, args.cmd_topic, 10)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 50)

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.samples.append(
            (
                time.monotonic(),
                float(pose.position.x),
                float(pose.position.y),
                yaw_from_quaternion(pose.orientation),
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.angular.z),
            )
        )

    def publish_command(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

    def stop(self):
        for _ in range(10):
            self.publish_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def run_once(self, direction):
        angular = direction * abs(self.args.angular)
        start_time = time.monotonic()
        end_time = start_time + self.args.duration
        while time.monotonic() < end_time:
            self.publish_command(self.args.linear, angular)
            rclpy.spin_once(self, timeout_sec=1.0 / self.args.rate)
        self.stop()

        stable_samples = [
            sample
            for sample in self.samples
            if sample[0] - start_time >= self.args.settle_sec
            and sample[0] <= end_time
            and abs(sample[4]) >= self.args.min_speed
            and abs(sample[5]) >= self.args.min_yaw_rate
        ]
        if len(stable_samples) < self.args.min_samples:
            raise RuntimeError(
                f"only {len(stable_samples)} stable samples collected; "
                "check that the robot is moving and odom is published"
            )

        points = [(sample[1], sample[2]) for sample in stable_samples]
        _, _, fitted_radius = fit_circle(points)
        speed_radius_values = [
            abs(sample[4] / sample[5])
            for sample in stable_samples
            if abs(sample[5]) > 1e-6
        ]
        mean_speed_radius = sum(speed_radius_values) / len(speed_radius_values)
        distance = path_length(points)
        yaw_span = abs(wrap_angle(stable_samples[-1][3] - stable_samples[0][3]))
        return {
            "direction": "left" if direction > 0 else "right",
            "samples": len(stable_samples),
            "fitted_radius_m": fitted_radius,
            "speed_radius_m": mean_speed_radius,
            "path_length_m": distance,
            "yaw_span_rad": yaw_span,
        }


def path_length(points):
    return sum(
        math.hypot(current[0] - previous[0], current[1] - previous[1])
        for previous, current in zip(points, points[1:])
    )


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cmd-topic", default="/robot/cmd_vel")
    parser.add_argument("--odom-topic", default="/robot/odom")
    parser.add_argument("--linear", type=float, default=0.20)
    parser.add_argument("--angular", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=18.0)
    parser.add_argument("--settle-sec", type=float, default=3.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--min-speed", type=float, default=0.03)
    parser.add_argument("--min-yaw-rate", type=float, default=0.03)
    parser.add_argument("--min-samples", type=int, default=80)
    parser.add_argument(
        "--direction",
        choices=["left", "right"],
        default="left",
        help="Turn direction to measure.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = TurningRadiusMeasurer(args)
    try:
        direction = 1 if args.direction == "left" else -1
        result = node.run_once(direction)
        print(
            "direction={direction} samples={samples} "
            "fitted_radius_m={fitted_radius_m:.3f} "
            "speed_radius_m={speed_radius_m:.3f} "
            "path_length_m={path_length_m:.3f} yaw_span_rad={yaw_span_rad:.3f}".format(
                **result
            )
        )
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
