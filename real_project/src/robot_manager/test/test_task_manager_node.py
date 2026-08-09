import rclpy
from unittest.mock import Mock

from rclpy.duration import Duration
from rclpy.qos import ReliabilityPolicy
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


def test_robot_status_callback_does_not_navigate_when_task_paused():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        _assign(node)  # dock_status_known 없음 -> awaiting_dock_check=True
        node.tasks['robot11'].state = 'PAUSED'

        node.robot_status_callback(_robot_status(is_docked=False, dock_status_known=True))

        node.nav_clients['robot11'].send_goal_async.assert_not_called()
        assert node.tasks['robot11'].awaiting_dock_check is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_resume_task_does_not_navigate_while_awaiting_dock_check():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        node.stop_publishers['robot11'] = Mock()
        _assign(node)
        task = node.tasks['robot11']
        task.awaiting_dock_check, task.state, task.previous_state = True, 'PAUSED', 'ASSIGNED'

        node.resume_task('robot11')

        node.nav_clients['robot11'].send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cancel_does_not_navigate_while_awaiting_dock_check():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        _assign(node)  # dock_status_known 없음 -> awaiting_dock_check=True
        task = node.tasks['robot11']
        assert task.awaiting_dock_check is True

        from robot_status.msg import TaskCommand
        cmd = TaskCommand()
        cmd.command, cmd.robot_id, cmd.task_id = 'CANCEL', 'robot11', task.task_id
        node.command_callback(cmd)

        node.nav_clients['robot11'].send_goal_async.assert_not_called()
        assert node.tasks['robot11'].state == 'DOCKED'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cancel_during_dock_check_clears_flags_so_reassignment_works():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.error_pub = Mock()
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        _assign(node)
        task = node.tasks['robot11']

        from robot_status.msg import TaskCommand
        cmd = TaskCommand()
        cmd.command, cmd.robot_id, cmd.task_id = 'CANCEL', 'robot11', task.task_id
        node.command_callback(cmd)

        assert task.awaiting_dock_check is False
        assert task.undock_requested is False
        assert task.dock_check_started_at is None

        # 취소 후 10초가 지나도 타임아웃 에러가 발행되면 안 된다 (플래그가 이미 지워졌으므로).
        task.dock_check_started_at = None  # 이미 None이지만 명시적으로 확인
        node.retry_navigation_goals()
        node.error_pub.publish.assert_not_called()

        # 재배정도 정상적으로 받아들여져야 한다 (DOCKED 상태라 ROBOT_ALREADY_HAS_TASK가 아니어야 함).
        _assign(node)
        node.error_pub.publish.assert_not_called()
        assert node.tasks['robot11'].state == 'ASSIGNED'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_robot_status_subscription_uses_best_effort_qos():
    rclpy.init()
    node = TaskManagerNode()
    try:
        for sub in node.status_subscriptions:
            assert sub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stale_dock_state_is_treated_as_unknown():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.robot_status_callback(_robot_status(is_docked=True, dock_status_known=True))

        is_docked, dock_status_known, _received_at = node.robot_dock_states['robot11']
        node.robot_dock_states['robot11'] = (is_docked, dock_status_known, node.get_clock().now() - Duration(seconds=6.0))

        assert node.get_fresh_dock_state('robot11') == (False, False)
    finally:
        node.destroy_node()
        rclpy.shutdown()
