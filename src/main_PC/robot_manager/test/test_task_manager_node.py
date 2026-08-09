import rclpy
from unittest.mock import Mock

from robot_status.msg import RobotAssignment, RobotError

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
