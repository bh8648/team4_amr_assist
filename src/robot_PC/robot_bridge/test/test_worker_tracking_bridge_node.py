from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from robot_bridge.worker_tracking_bridge_node import WorkerTrackingBridgeNode


def _pose(x=1.0, y=2.0):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.w = 1.0
    return msg


@pytest.fixture
def node():
    rclpy.init()
    n = WorkerTrackingBridgeNode(robot_id='robot5')
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_disabled_ignores_raw_pose(node):
    node.target_pose_pub = Mock()
    node.worker_detected_pub = Mock()

    node.raw_pose_callback(_pose())

    node.target_pose_pub.publish.assert_not_called()
    node.worker_detected_pub.publish.assert_not_called()


def test_enabled_forwards_pose_and_sends_detected_once(node):
    node.target_pose_pub = Mock()
    node.worker_detected_pub = Mock()
    node.enable_callback(Bool(data=True))

    node.raw_pose_callback(_pose())
    node.target_pose_pub.publish.assert_called_once()
    node.worker_detected_pub.publish.assert_called_once()
    assert node.worker_detected_pub.publish.call_args.args[0].data is True

    node.raw_pose_callback(_pose(x=1.5))
    assert node.target_pose_pub.publish.call_count == 2
    node.worker_detected_pub.publish.assert_called_once()


def test_lost_timeout_fires_once(node):
    node.worker_lost_timeout = 1.0
    node.worker_lost_pub = Mock()
    node.enable_callback(Bool(data=True))
    node.last_pose_time_ns = node.get_clock().now().nanoseconds - int(2e9)

    node.check_worker_lost()
    node.worker_lost_pub.publish.assert_called_once()
    assert node.worker_lost_pub.publish.call_args.args[0].data is True

    node.check_worker_lost()
    node.worker_lost_pub.publish.assert_called_once()


def test_reenable_resets_detected_and_lost_latches(node):
    node.target_pose_pub = Mock()
    node.worker_detected_pub = Mock()
    node.worker_lost_pub = Mock()
    node.worker_lost_timeout = 1.0

    node.enable_callback(Bool(data=True))
    node.raw_pose_callback(_pose())
    node.last_pose_time_ns = node.get_clock().now().nanoseconds - int(2e9)
    node.check_worker_lost()
    node.worker_lost_pub.publish.assert_called_once()

    node.enable_callback(Bool(data=False))
    node.enable_callback(Bool(data=True))
    node.raw_pose_callback(_pose())

    assert node.worker_detected_pub.publish.call_count == 2


def test_invalid_robot_id_is_rejected():
    rclpy.init()
    try:
        with pytest.raises(ValueError, match='robot_id'):
            WorkerTrackingBridgeNode(robot_id='robot7')
    finally:
        rclpy.shutdown()
