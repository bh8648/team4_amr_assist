from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from robot_bridge.robot_bridge_node import RobotBridgeNode
from robot_status.msg import TaskState


def _valid_pose(x=1.0, y=2.0):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.w = 1.0
    return msg


@pytest.mark.parametrize('robot_id', ['robot5', 'robot11'])
def test_robot_specific_topics_and_status_identity(robot_id):
    rclpy.init()
    node = RobotBridgeNode(robot_id=robot_id)
    try:
        assert node.robot_id == robot_id
        assert node.amcl_sub.topic_name == f'/{robot_id}/amcl_pose'
        assert node.battery_sub.topic_name == f'/{robot_id}/battery_state'
        assert node.target_person_pose_sub.topic_name == f'/{robot_id}/target_person_pose'
        assert node.worker_tracking_enable_pub.topic_name == f'/{robot_id}/worker_tracking/enable'

        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position.x, pose.pose.pose.position.y = 1.5, -2.0
        pose.pose.pose.orientation.w = 1.0
        battery = BatteryState()
        battery.percentage = 0.75
        node.amcl_pose_callback(pose)
        node.battery_callback(battery)
        status = node.build_status_message()
        assert status.robot_id == robot_id
        assert status.battery == 75.0
        assert status.x == 1.5
        assert status.y == -2.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ('robot_id', 'other_robot_id'),
    [('robot5', 'robot11'), ('robot11', 'robot5')],
)
def test_task_state_isolated_by_robot_id(robot_id, other_robot_id):
    rclpy.init()
    node = RobotBridgeNode(robot_id=robot_id)
    try:
        other = TaskState()
        other.robot_id, other.state = other_robot_id, 'FOLLOWING'
        node.task_state_callback(other)
        assert node.current_task_state == ''

        own = TaskState()
        own.robot_id, own.task_id, own.state = robot_id, 'TASK-1', 'FOLLOWING'
        node.task_state_callback(own)
        assert node.current_task_state == 'FOLLOWING'
        assert node.current_task_id == 'TASK-1'
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('robot_id', ['robot5', 'robot11'])
def test_worker_detection_command_uses_configured_robot_id(robot_id):
    rclpy.init()
    node = RobotBridgeNode(robot_id=robot_id)
    try:
        node.worker_tracking_enable_pub = Mock()
        node.task_command_pub = Mock()
        state = TaskState()
        state.robot_id, state.task_id = robot_id, 'TASK-1'
        state.state, state.goal_type, state.goal_completed = 'ASSIGNED', 'TO_WORKER', True
        node.task_state_callback(state)
        node.worker_detected_callback(Bool(data=True))
        command = node.task_command_pub.publish.call_args.args[0]
        assert command.command == 'WORKER_DETECTED'
        assert command.robot_id == robot_id
        assert command.task_id == 'TASK-1'
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('robot_id', ['robot5', 'robot11'])
def test_following_pose_and_undock_are_independent(robot_id):
    rclpy.init()
    node = RobotBridgeNode(robot_id=robot_id)
    try:
        node.current_task_state = 'FOLLOWING'
        node.send_follow_goal = Mock()
        node.target_person_pose_callback(_valid_pose())
        node.send_follow_goal.assert_called_once()

        node.dock_client = Mock()
        node.undock_client = Mock()
        node.undock_client.wait_for_server.return_value = True
        node.undock_client.send_goal_async.return_value = Mock()
        node.dock_callback(Bool(data=False))
        node.undock_client.send_goal_async.assert_called_once()
        node.dock_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_invalid_robot_id_is_rejected():
    rclpy.init()
    try:
        with pytest.raises(ValueError, match='robot_id'):
            RobotBridgeNode(robot_id='robot7')
    finally:
        rclpy.shutdown()
