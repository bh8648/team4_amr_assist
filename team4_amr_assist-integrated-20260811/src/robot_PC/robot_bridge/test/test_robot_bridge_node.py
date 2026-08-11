import math
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
def test_first_tracking_pose_adapts_to_worker_detected_command(robot_id):
    """추종 노드 수정 없이 첫 target pose를 WORKER_DETECTED 명령으로 변환한다."""
    rclpy.init()
    node = RobotBridgeNode(robot_id=robot_id)
    try:
        node.current_task_state, node.current_task_id = 'ASSIGNED', 'TASK-1'
        node.worker_tracking_enabled = True
        node.task_command_pub = Mock()
        node.send_follow_goal = Mock()

        node.target_person_pose_callback(_valid_pose(x=2.0, y=0.0))
        node.target_person_pose_callback(_valid_pose(x=2.1, y=0.0))

        node.task_command_pub.publish.assert_called_once()
        command = node.task_command_pub.publish.call_args.args[0]
        assert command.command == 'WORKER_DETECTED'
        assert command.robot_id == robot_id
        assert command.task_id == 'TASK-1'
        # 중앙에서 FOLLOWING 상태를 회신하기 전에는 로봇을 추종 이동시키지 않는다.
        node.send_follow_goal.assert_not_called()
        assert node.pending_person_pose.pose.position.x == 2.1
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('robot_id', ['robot5', 'robot11'])
def test_following_state_uses_pose_saved_during_detection(robot_id):
    """중앙의 FOLLOWING 회신 즉시 저장한 사람 pose로 안전거리 추종을 시작한다."""
    rclpy.init()
    node = RobotBridgeNode(robot_id=robot_id)
    try:
        node.current_task_state, node.current_task_id = 'ASSIGNED', 'TASK-1'
        node.worker_tracking_enabled = True
        node.latest_x, node.latest_y, node.latest_yaw = 0.0, 0.0, 0.0
        node.task_command_pub = Mock()
        node.should_send_follow_goal = Mock(return_value=True)
        node.send_follow_goal = Mock()
        node.target_person_pose_callback(_valid_pose(x=2.0, y=0.0))

        state = TaskState()
        state.robot_id, state.task_id, state.state = robot_id, 'TASK-1', 'FOLLOWING'
        node.task_state_callback(state)

        node.send_follow_goal.assert_called_once()
        goal = node.send_follow_goal.call_args.args[0]
        assert math.isclose(goal.pose.position.x, 2.0 - node.follow_distance)
        assert node.pending_person_pose is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('robot_id', ['robot5', 'robot11'])
def test_following_pose_and_undock_are_independent(robot_id):
    rclpy.init()
    node = RobotBridgeNode(robot_id=robot_id)
    try:
        node.current_task_state = 'FOLLOWING'
        # 안전거리 goal 계산에는 로봇의 현재 AMCL 위치가 필요하다.
        node.latest_x, node.latest_y, node.latest_yaw = 0.0, 0.0, 0.0
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


@pytest.mark.parametrize('robot_id', ['robot5', 'robot11'])
def test_safe_follow_goal_stops_before_person_and_faces_person(robot_id):
    """사람 좌표가 아니라 사람에게서 follow_distance 떨어진 점을 goal로 사용한다."""
    rclpy.init()
    node = RobotBridgeNode(robot_id=robot_id)
    try:
        node.latest_x, node.latest_y, node.latest_yaw = 0.0, 0.0, 0.0
        goal = node.build_safe_follow_goal(_valid_pose(x=2.0, y=0.0))

        assert goal is not None
        assert math.isclose(goal.pose.position.x, 2.0 - node.follow_distance)
        assert math.isclose(goal.pose.position.y, 0.0)
        assert math.isclose(goal.pose.orientation.z, 0.0)
        assert math.isclose(goal.pose.orientation.w, 1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('robot_id', ['robot5', 'robot11'])
def test_safe_follow_goal_does_not_approach_person_inside_follow_distance(robot_id):
    """사람이 이미 가까우면 현재 위치를 유지하고 회전 방향만 사람에게 맞춘다."""
    rclpy.init()
    node = RobotBridgeNode(robot_id=robot_id)
    try:
        node.latest_x, node.latest_y, node.latest_yaw = 1.0, 1.0, 0.0
        goal = node.build_safe_follow_goal(_valid_pose(x=1.2, y=1.0))

        assert goal is not None
        assert math.isclose(goal.pose.position.x, 1.0)
        assert math.isclose(goal.pose.position.y, 1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('robot_id', ['robot5', 'robot11'])
def test_follow_goal_updates_when_only_person_direction_changes(robot_id):
    """근거리에서 위치가 같아도 사람이 옆으로 이동하면 회전 목표를 갱신한다."""
    rclpy.init()
    node = RobotBridgeNode(robot_id=robot_id)
    try:
        now_ns = node.get_clock().now().nanoseconds
        node.last_follow_goal_time_ns = now_ns - int(2.0 * 1e9)
        node.last_follow_goal = (1.0, 1.0, 0.0)
        changed_direction = _valid_pose(x=1.0, y=1.0)
        changed_direction.pose.orientation.z = math.sin(math.pi / 4.0)
        changed_direction.pose.orientation.w = math.cos(math.pi / 4.0)

        assert node.should_send_follow_goal(changed_direction)
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('robot_id', ['robot5', 'robot11'])
def test_tracking_watchdog_starts_nav2_spin_after_grace_period(robot_id):
    """좌표가 끊기면 직접 cmd_vel 대신 Nav2 Spin 탐색을 시작한다."""
    rclpy.init()
    node = RobotBridgeNode(robot_id=robot_id)
    try:
        node.current_task_state, node.current_task_id = 'FOLLOWING', 'TASK-1'
        now_ns = node.get_clock().now().nanoseconds
        node.last_person_pose_time_ns = now_ns - int((node.person_lost_grace_sec + 0.1) * 1e9)
        node.cancel_follow_goal = Mock()
        node.send_search_spin_goal = Mock()

        node.tracking_watchdog_callback()

        node.cancel_follow_goal.assert_called_once()
        node.send_search_spin_goal.assert_called_once()
        assert node.search_started_time_ns is not None
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('robot_id', ['robot5', 'robot11'])
def test_valid_person_pose_cancels_search_and_resumes_following(robot_id):
    """회전 탐색 중 사람을 다시 찾으면 Spin을 취소하고 안전거리 추종을 재개한다."""
    rclpy.init()
    node = RobotBridgeNode(robot_id=robot_id)
    try:
        node.current_task_state, node.current_task_id = 'FOLLOWING', 'TASK-1'
        node.latest_x, node.latest_y, node.latest_yaw = 0.0, 0.0, 0.0
        node.search_started_time_ns = node.get_clock().now().nanoseconds
        node.cancel_search_spin = Mock()
        node.should_send_follow_goal = Mock(return_value=True)
        node.send_follow_goal = Mock()

        node.target_person_pose_callback(_valid_pose(x=2.0, y=0.0))

        node.cancel_search_spin.assert_called_once()
        node.send_follow_goal.assert_called_once()
        sent_goal = node.send_follow_goal.call_args.args[0]
        assert math.isclose(sent_goal.pose.position.x, 2.0 - node.follow_distance)
        assert node.search_started_time_ns is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('robot_id', ['robot5', 'robot11'])
def test_tracking_timeout_publishes_worker_lost_only_once(robot_id):
    """재탐색 제한 시간이 지나도 같은 Task 오류를 중복 발행하지 않는다."""
    rclpy.init()
    node = RobotBridgeNode(robot_id=robot_id)
    try:
        node.current_task_state, node.current_task_id = 'FOLLOWING', 'TASK-1'
        now_ns = node.get_clock().now().nanoseconds
        node.last_person_pose_time_ns = now_ns - int(70.0 * 1e9)
        node.search_started_time_ns = now_ns - int((node.person_search_timeout_sec + 0.1) * 1e9)
        node.error_pub = Mock()
        node.cancel_search_spin = Mock()

        node.tracking_watchdog_callback()
        node.tracking_watchdog_callback()

        node.error_pub.publish.assert_called_once()
        error = node.error_pub.publish.call_args.args[0]
        assert error.error_code == 'WORKER_LOST'
        assert error.robot_id == robot_id
        assert error.task_id == 'TASK-1'
    finally:
        node.destroy_node()
        rclpy.shutdown()
