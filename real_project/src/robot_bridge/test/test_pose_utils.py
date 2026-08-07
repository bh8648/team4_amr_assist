import math

from robot_bridge.pose_utils import build_robot_status, is_valid_quaternion, quaternion_to_yaw


def test_quaternion_to_yaw_identity_is_zero():
    assert quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == 0.0


def test_quaternion_to_yaw_90_degrees():
    # z축 기준 90도(pi/2) 회전 쿼터니언
    yaw = quaternion_to_yaw(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-6)


def test_is_valid_quaternion_rejects_all_zero():
    assert is_valid_quaternion(0.0, 0.0, 0.0, 0.0) is False


def test_is_valid_quaternion_accepts_identity():
    assert is_valid_quaternion(0.0, 0.0, 0.0, 1.0) is True


def test_build_robot_status_fields():
    msg = build_robot_status('robot5', 87.5, 1.2, -3.4, 0.5)
    assert msg.robot_id == 'robot5'
    assert msg.battery == 87.5
    assert msg.x == 1.2
    assert msg.y == -3.4
    assert msg.yaw == 0.5
    assert msg.current_task_id == ''
