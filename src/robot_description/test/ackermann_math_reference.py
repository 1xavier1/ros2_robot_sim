import math


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def compute_ackermann_targets(
    linear_x,
    angular_z,
    wheelbase=0.45,
    track_width=0.35,
    wheel_radius=0.07,
    max_steering_angle=0.5236,
):
    if abs(linear_x) < 0.01:
        steer = clamp(angular_z * 0.5, -max_steering_angle, max_steering_angle)
        return steer, steer, 0.0, 0.0

    steer = math.atan(wheelbase * angular_z / linear_x)
    steer = clamp(steer, -max_steering_angle, max_steering_angle)

    if abs(steer) < 1e-6:
        wheel_velocity = linear_x / wheel_radius
        return 0.0, 0.0, wheel_velocity, wheel_velocity

    sign = 1.0 if steer > 0.0 else -1.0
    radius = wheelbase / math.tan(abs(steer))

    inner_angle = math.atan(wheelbase / (radius - track_width / 2.0))
    outer_angle = math.atan(wheelbase / (radius + track_width / 2.0))

    inner_rear_linear = abs(linear_x) * (radius - track_width / 2.0) / radius
    outer_rear_linear = abs(linear_x) * (radius + track_width / 2.0) / radius

    if linear_x < 0.0:
        inner_rear_linear = -inner_rear_linear
        outer_rear_linear = -outer_rear_linear

    if sign > 0.0:
        left_steer = inner_angle
        right_steer = outer_angle
        left_wheel = inner_rear_linear / wheel_radius
        right_wheel = outer_rear_linear / wheel_radius
    else:
        left_steer = -outer_angle
        right_steer = -inner_angle
        left_wheel = outer_rear_linear / wheel_radius
        right_wheel = inner_rear_linear / wheel_radius

    return left_steer, right_steer, left_wheel, right_wheel
