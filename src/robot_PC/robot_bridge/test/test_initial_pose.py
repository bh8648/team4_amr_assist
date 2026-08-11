"""Dock initial pose 저장·자동 발행 테스트."""

import math
from unittest.mock import Mock

from geometry_msgs.msg import PoseWithCovarianceStamped
import pytest
import rclpy

from robot_bridge.capture_initial_pose_node import CaptureInitialPoseNode
from robot_bridge.initial_pose_publisher_node import InitialPosePublisherNode
from robot_bridge.initial_pose_utils import load_initial_pose, save_initial_pose


def _amcl_pose(x=1.2, y=-3.4, yaw=0.7):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_pose_yaml_round_trip_preserves_other_robot(tmp_path):
    path = tmp_path / 'initial_poses.yaml'
    save_initial_pose(path, 'robot11', {'x': 1, 'y': 2, 'yaw': 0.3})
    save_initial_pose(path, 'robot5', {'x': -1, 'y': -2, 'yaw': -0.4})
    assert load_initial_pose(path, 'robot11')['x'] == 1.0
    assert load_initial_pose(path, 'robot5')['yaw'] == -0.4


def test_pose_yaml_rejects_non_finite_value(tmp_path):
    with pytest.raises(ValueError, match='NaN/inf'):
        save_initial_pose(
            tmp_path / 'initial_poses.yaml',
            'robot5',
            {'x': float('nan'), 'y': 0.0, 'yaw': 0.0},
        )


def test_capture_node_saves_first_map_pose(tmp_path, ros_context):
    path = tmp_path / 'initial_poses.yaml'
    node = CaptureInitialPoseNode(robot_id='robot5', pose_file=str(path))
    try:
        node.pose_callback(_amcl_pose())
        node.pose_callback(_amcl_pose(x=9.0))
        saved = load_initial_pose(path, 'robot5')
        assert saved['x'] == pytest.approx(1.2)
        assert saved['y'] == pytest.approx(-3.4)
        assert saved['yaw'] == pytest.approx(0.7)
    finally:
        node.destroy_node()


def test_publisher_reuses_saved_pose_and_stops_after_amcl(tmp_path, ros_context):
    path = tmp_path / 'initial_poses.yaml'
    save_initial_pose(path, 'robot5', {'x': 1.2, 'y': -3.4, 'yaw': 0.7})
    node = InitialPosePublisherNode(robot_id='robot5', pose_file=str(path))
    try:
        node.initial_pose_pub = Mock()
        node.publish_initial_pose()
        sent = node.initial_pose_pub.publish.call_args.args[0]
        assert sent.header.frame_id == 'map'
        assert sent.pose.pose.position.x == pytest.approx(1.2)
        assert sent.pose.pose.position.y == pytest.approx(-3.4)
        assert sent.pose.pose.orientation.z == pytest.approx(math.sin(0.35))
        assert sent.pose.pose.orientation.w == pytest.approx(math.cos(0.35))

        node.amcl_pose_callback(_amcl_pose(x=9.0, y=9.0, yaw=-2.0))
        assert not node.confirmed
        node.amcl_pose_callback(_amcl_pose())
        node.publish_initial_pose()
        node.initial_pose_pub.publish.assert_called_once()
        assert node.confirmed
    finally:
        node.destroy_node()


def test_robot5_without_capture_uses_dock_origin(tmp_path, ros_context):
    node = InitialPosePublisherNode(
        robot_id='robot5', pose_file=str(tmp_path / 'missing.yaml'))
    try:
        assert node.pose['x'] == 0.0
        assert node.pose['y'] == 0.0
        assert node.pose['yaw'] == 0.0
        assert node.start_timer is not None
    finally:
        node.destroy_node()


def test_robot11_without_capture_reloads_after_measurement(tmp_path, ros_context):
    path = tmp_path / 'missing.yaml'
    node = InitialPosePublisherNode(
        robot_id='robot11', pose_file=str(path))
    try:
        assert node.pose is None
        assert node.start_timer is None
        assert node.attempts == 0
        save_initial_pose(path, 'robot11', {'x': 1.2, 'y': -3.4, 'yaw': 0.7})
        node._try_reload_pose()
        assert node.pose['x'] == pytest.approx(1.2)
        assert node.start_timer is not None
    finally:
        node.destroy_node()
