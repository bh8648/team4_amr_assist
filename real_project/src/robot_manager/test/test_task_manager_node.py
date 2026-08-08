import rclpy
from unittest.mock import Mock

from rclpy.duration import Duration
from robot_status.msg import RobotAssignment, RobotError, RobotStatus

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


def _robot_status(robot_id='robot11', is_docked=False, dock_status_known=True):
    msg = RobotStatus()
    msg.robot_id = robot_id
    msg.is_docked, msg.dock_status_known = is_docked, dock_status_known
    return msg


def test_assignment_does_not_navigate_when_dock_status_unknown():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True

        _assign(node)

        node.nav_clients['robot11'].send_goal_async.assert_not_called()
        assert node.tasks['robot11'].awaiting_dock_check is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_assignment_navigates_immediately_when_known_undocked():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.robot_status_callback(_robot_status(is_docked=False, dock_status_known=True))
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True

        _assign(node)

        node.nav_clients['robot11'].send_goal_async.assert_called_once()
        assert node.tasks['robot11'].awaiting_dock_check is False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_assignment_requests_undock_once_when_docked():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.robot_status_callback(_robot_status(is_docked=True, dock_status_known=True))
        node.dock_publishers['robot11'] = Mock()
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True

        _assign(node)

        node.dock_publishers['robot11'].publish.assert_called_once()
        assert node.dock_publishers['robot11'].publish.call_args[0][0].data is False
        assert node.tasks['robot11'].undock_requested is True
        node.nav_clients['robot11'].send_goal_async.assert_not_called()

        # 같은 로봇 상태(is_docked=True)가 다시 와도 두 번째 언도킹 요청은 없어야 한다.
        node.robot_status_callback(_robot_status(is_docked=True, dock_status_known=True))
        node.dock_publishers['robot11'].publish.assert_called_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_undock_confirmed_then_navigation_starts():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.robot_status_callback(_robot_status(is_docked=True, dock_status_known=True))
        node.dock_publishers['robot11'] = Mock()
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        _assign(node)

        node.robot_status_callback(_robot_status(is_docked=False, dock_status_known=True))

        node.nav_clients['robot11'].send_goal_async.assert_called_once()
        assert node.tasks['robot11'].undock_requested is False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dock_wait_timeout_publishes_error_once():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.error_pub = Mock()
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        _assign(node)  # dock_status_known 없음 -> awaiting_dock_check=True

        task = node.tasks['robot11']
        task.dock_check_started_at = node.get_clock().now() - Duration(seconds=11.0)

        node.retry_navigation_goals()

        node.error_pub.publish.assert_called_once()
        assert node.error_pub.publish.call_args[0][0].error_code == 'DOCK_STATUS_UNKNOWN_TIMEOUT'

        # 다음 호출에서 같은 에러를 또 발행하지 않아야 한다(dock_check_started_at이 None으로 초기화됨).
        node.retry_navigation_goals()
        node.error_pub.publish.assert_called_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_worker_arrival_auto_transitions_to_following():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.robot_status_callback(_robot_status(is_docked=False, dock_status_known=True))
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        _assign(node)

        node.handle_navigation_result('robot11', 'TO_WORKER', True, '')

        assert node.tasks['robot11'].state == 'FOLLOWING'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_destination_arrival_auto_returns_and_sends_dock_goal():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.robot_status_callback(_robot_status(is_docked=False, dock_status_known=True))
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        _assign(node)
        task = node.tasks['robot11']
        task.state, task.goal_type = 'TRANSPORTING', 'TO_DESTINATION'

        node.handle_navigation_result('robot11', 'TO_DESTINATION', True, '')

        assert node.tasks['robot11'].state == 'RETURNING'
        assert node.tasks['robot11'].goal_type == 'TO_DOCK'
        assert node.nav_clients['robot11'].send_goal_async.call_count == 2  # 배정 이동 + 복귀 이동
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_worker_detected_command_no_longer_handled():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.robot_status_callback(_robot_status(is_docked=False, dock_status_known=True))
        node.error_pub = Mock()
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        _assign(node)

        from robot_status.msg import TaskCommand
        cmd = TaskCommand()
        cmd.command, cmd.robot_id, cmd.task_id = 'WORKER_DETECTED', 'robot11', node.tasks['robot11'].task_id
        node.command_callback(cmd)

        assert node.tasks['robot11'].state == 'ASSIGNED'  # 더 이상 이 커맨드로 전환되지 않음
        node.error_pub.publish.assert_called_once()
        assert 'INVALID_TRANSITION' in node.error_pub.publish.call_args[0][0].error_code
    finally:
        node.destroy_node()
        rclpy.shutdown()
