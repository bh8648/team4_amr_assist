import math

from robot_bridge.pose_utils import (
    is_followable_pose,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


def test_quaternion_to_yaw():
    assert math.isclose(quaternion_to_yaw(0.0, 0.0, 0.0, 1.0), 0.0)


def test_yaw_to_quaternion_round_trip():
    yaw = -1.528
    z, w = yaw_to_quaternion(yaw)
    assert math.isclose(quaternion_to_yaw(0.0, 0.0, z, w), yaw)


def test_followable_pose_rejects_origin_and_zero_quaternion():
    assert not is_followable_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    assert not is_followable_pose(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert is_followable_pose(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)
