import pytest

from ackermann_math_reference import compute_ackermann_targets


def test_left_turn_has_larger_left_steering_angle_and_outer_rear_speed():
    left, right, rear_left, rear_right = compute_ackermann_targets(0.5, 0.3)

    assert left > right > 0.0
    assert left == pytest.approx(0.293, abs=0.02)
    assert right == pytest.approx(0.239, abs=0.02)
    assert rear_right > rear_left > 0.0


def test_right_turn_has_larger_right_steering_angle_and_outer_rear_speed():
    left, right, rear_left, rear_right = compute_ackermann_targets(0.5, -0.3)

    assert right < left < 0.0
    assert abs(right) > abs(left)
    assert rear_left > rear_right > 0.0


def test_reverse_positive_yaw_uses_opposite_steering_sign():
    left, right, rear_left, rear_right = compute_ackermann_targets(-0.5, 0.3)

    assert right < left < 0.0
    assert abs(right) > abs(left)
    assert rear_left < rear_right < 0.0


def test_reverse_negative_yaw_uses_opposite_steering_sign():
    left, right, rear_left, rear_right = compute_ackermann_targets(-0.5, -0.3)

    assert left > right > 0.0
    assert abs(left) > abs(right)
    assert rear_right < rear_left < 0.0


def test_zero_linear_velocity_does_not_drive_rear_wheels():
    left, right, rear_left, rear_right = compute_ackermann_targets(0.0, 0.3)

    assert left > 0.0
    assert right > 0.0
    assert rear_left == 0.0
    assert rear_right == 0.0
