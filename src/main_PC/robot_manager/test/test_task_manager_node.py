import rclpy
from unittest.mock import Mock

import pytest

from robot_status.msg import (
    Destination,
    DestinationList,
    NavigationResult,
    RobotAssignment,
    RobotError,
    TaskCommand,
)

from robot_manager.task_manager_node import TaskManagerNode


def _assign(node, robot_id='robot11', x=1.0, y=2.0):
    msg = RobotAssignment()
    msg.assigned, msg.robot_id = True, robot_id
    msg.target_x, msg.target_y = x, y
    node.assignment_callback(msg)


def _error(robot_id, task_id, error_code):
    msg = RobotError()
    msg.robot_id, msg.task_id, msg.error_code = robot_id, task_id, error_code
    return msg


def _command(robot_id, task_id, command):
    msg = TaskCommand()
    msg.robot_id, msg.task_id, msg.command = robot_id, task_id, command
    return msg


def test_error_callback_ignores_invalid_transition_prefixed_code():
    rclpy.init()
    node = TaskManagerNode()
    try:
        _assign(node)
        task = node.tasks['robot11']
        assert task.state == 'ASSIGNED'

        node.error_callback(_error('robot11', task.task_id, 'INVALID_TRANSITION_ASSIGNED_START_TRANSPORT'))

        assert node.tasks['robot11'].state == 'ASSIGNED'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_error_callback_ignores_stale_task_command():
    rclpy.init()
    node = TaskManagerNode()
    try:
        _assign(node)
        task = node.tasks['robot11']

        node.error_callback(_error('robot11', task.task_id, 'STALE_TASK_COMMAND'))

        assert node.tasks['robot11'].state == 'ASSIGNED'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_error_callback_ignores_robot_already_has_task():
    rclpy.init()
    node = TaskManagerNode()
    try:
        _assign(node)
        task = node.tasks['robot11']

        node.error_callback(_error('robot11', task.task_id, 'ROBOT_ALREADY_HAS_TASK'))

        assert node.tasks['robot11'].state == 'ASSIGNED'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_error_callback_still_transitions_to_error_for_genuine_failure():
    rclpy.init()
    node = TaskManagerNode()
    try:
        _assign(node)
        task = node.tasks['robot11']
        node.stop_publishers['robot11'] = Mock()

        node.error_callback(_error('robot11', task.task_id, 'NAV_GOAL_REJECTED'))

        assert node.tasks['robot11'].state == 'ERROR'
        node.stop_publishers['robot11'].publish.assert_called_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_error_callback_does_nothing_when_no_task_exists():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.error_callback(_error('robot11', '', 'NAV_GOAL_REJECTED'))  # 예외 없이 통과해야 함
        assert 'robot11' not in node.tasks
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_follow_pause_resume_and_return_to_dock_flow():
    """인식 성공을 가정해 HMI 명령부터 실제 도킹 완료까지 상태 전이를 검증한다."""
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.dock_publishers['robot11'] = Mock()
        node.stop_publishers['robot11'] = Mock()
        node.send_navigation_goal = Mock()

        _assign(node)
        task = node.tasks['robot11']
        assert task.awaiting_undock is True
        assert node.dock_publishers['robot11'].publish.call_args.args[0].data is False
        node.send_navigation_goal.assert_not_called()

        undock = NavigationResult()
        undock.robot_id, undock.task_id = 'robot11', task.task_id
        undock.goal_type, undock.success = 'UNDOCK', True
        node.navigation_result_callback(undock)
        assert task.awaiting_undock is False
        node.send_navigation_goal.assert_called_once_with(task)

        node.handle_navigation_result('robot11', 'TO_WORKER', True, '')
        assert task.goal_completed is True

        node.command_callback(_command('robot11', task.task_id, 'WORKER_DETECTED'))
        assert task.state == 'FOLLOWING'

        node.command_callback(_command('robot11', task.task_id, 'PAUSE'))
        assert task.state == 'PAUSED'
        assert task.previous_state == 'FOLLOWING'
        assert node.stop_publishers['robot11'].publish.call_args.args[0].data is True

        node.command_callback(_command('robot11', task.task_id, 'RESUME'))
        assert task.state == 'FOLLOWING'
        assert node.stop_publishers['robot11'].publish.call_args.args[0].data is False

        node.command_callback(_command('robot11', task.task_id, 'PAUSE'))
        node.send_navigation_goal.reset_mock()
        node.command_callback(_command('robot11', task.task_id, 'RETURN_TO_DOCK'))
        assert task.state == 'RETURNING'
        assert task.goal_type == 'TO_DOCK'
        node.send_navigation_goal.assert_called_once_with(task, replace=True)

        node.handle_navigation_result('robot11', 'TO_DOCK', True, '')
        assert node.dock_publishers['robot11'].publish.call_args.args[0].data is True

        dock = NavigationResult()
        dock.robot_id, dock.task_id = 'robot11', task.task_id
        dock.goal_type, dock.success = 'DOCK', True
        node.navigation_result_callback(dock)
        assert task.state == 'DOCKED'
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('robot_id', ['robot5', 'robot11'])
def test_complete_delivery_flow_reaches_docked(robot_id):
    """Robot5/11 모두 배송지 선택부터 Nav2·실제 도킹 완료까지 검증한다."""
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.dock_publishers[robot_id] = Mock()
        node.stop_publishers[robot_id] = Mock()
        node.send_navigation_goal = Mock()

        destinations = DestinationList()
        destination = Destination()
        destination.destination_id = 'DEST-A'
        destination.destination_name = 'A 구역'
        destination.position_x, destination.position_y = 3.2, -1.4
        destinations.destinations.append(destination)
        node.destination_callback(destinations)

        _assign(node, robot_id=robot_id)
        task = node.tasks[robot_id]

        undock = NavigationResult()
        undock.robot_id, undock.task_id = robot_id, task.task_id
        undock.goal_type, undock.success = 'UNDOCK', True
        node.navigation_result_callback(undock)
        node.handle_navigation_result(robot_id, 'TO_WORKER', True, '')
        node.command_callback(_command(robot_id, task.task_id, 'WORKER_DETECTED'))
        assert task.state == 'FOLLOWING'

        start = _command(robot_id, task.task_id, 'START_TRANSPORT')
        start.destination_id = 'DEST-A'
        node.send_navigation_goal.reset_mock()
        node.command_callback(start)
        assert task.state == 'TRANSPORTING'
        assert task.goal_type == 'TO_DESTINATION'
        assert task.destination_id == 'DEST-A'
        assert task.target == (3.2, -1.4, 0.0)
        node.send_navigation_goal.assert_called_once_with(task, replace=True)

        node.handle_navigation_result(robot_id, 'TO_DESTINATION', True, '')
        assert task.goal_completed is True

        node.send_navigation_goal.reset_mock()
        node.command_callback(_command(robot_id, task.task_id, 'DELIVERY_CONFIRMED'))
        assert task.state == 'RETURNING'
        assert task.goal_type == 'TO_DOCK'
        node.send_navigation_goal.assert_called_once_with(task, replace=True)

        node.handle_navigation_result(robot_id, 'TO_DOCK', True, '')
        assert node.dock_publishers[robot_id].publish.call_args.args[0].data is True

        dock = NavigationResult()
        dock.robot_id, dock.task_id = robot_id, task.task_id
        dock.goal_type, dock.success = 'DOCK', True
        node.navigation_result_callback(dock)
        assert task.state == 'DOCKED'
        assert task.goal_completed is True
    finally:
        node.destroy_node()
        rclpy.shutdown()
