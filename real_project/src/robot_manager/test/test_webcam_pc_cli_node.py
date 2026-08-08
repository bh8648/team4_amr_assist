import rclpy
from unittest.mock import Mock

from robot_status.msg import RobotError, TaskState

from robot_manager.webcam_pc_cli import WebcamPcCliNode


def test_task_state_callback_caches_per_robot_id():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        msg = TaskState()
        msg.robot_id, msg.state, msg.task_id = 'robot11', 'FOLLOWING', 'TASK_1'
        node.task_state_callback(msg)
        assert node.task_cache['robot11'].state == 'FOLLOWING'
        assert node.task_cache['robot11'].task_id == 'TASK_1'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_error_callback_prints(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        msg = RobotError()
        msg.robot_id, msg.task_id, msg.error_code = 'robot11', 'TASK_1', 'NAV_GOAL_REJECTED'
        node.error_callback(msg)
        assert 'NAV_GOAL_REJECTED' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_status_prints_cached_states(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        state = TaskState()
        state.robot_id, state.state, state.task_id = 'robot11', 'FOLLOWING', 'TASK_1'
        node.task_state_callback(state)

        node.cmd_status([])

        assert 'robot11' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_run_cli_dispatches_status_then_quits(monkeypatch, capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        inputs = iter(['상태', '종료'])
        monkeypatch.setattr('builtins.input', lambda _prompt='': next(inputs))

        node.run_cli()

        assert '캐싱된 로봇 상태 없음' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_run_cli_unknown_command_prints_error(monkeypatch, capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        inputs = iter(['이상한명령', '종료'])
        monkeypatch.setattr('builtins.input', lambda _prompt='': next(inputs))

        node.run_cli()

        assert '알 수 없는 명령' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_follow_start_creates_timer():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.cmd_follow_start([])
        assert node.following_timer is not None
        assert node.following_index == 0
    finally:
        node.cmd_follow_stop([])
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_follow_start_warns_when_robot11_not_following(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.cmd_follow_start([])
        assert '[경고]' in capsys.readouterr().out
        assert node.following_timer is not None
    finally:
        node.cmd_follow_stop([])
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_follow_start_no_warning_when_robot11_following(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        state = TaskState()
        state.robot_id, state.state, state.task_id = 'robot11', 'FOLLOWING', 'TASK_1'
        node.task_state_callback(state)

        node.cmd_follow_start([])

        assert '[경고]' not in capsys.readouterr().out
        assert node.following_timer is not None
    finally:
        node.cmd_follow_stop([])
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_follow_start_rejects_invalid_interval():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.cmd_follow_start(['-1'])
        assert node.following_timer is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_follow_start_rejects_duplicate_start(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.cmd_follow_start(['100'])
        first_timer = node.following_timer

        node.cmd_follow_start(['100'])

        assert node.following_timer is first_timer
        assert '이미 진행 중' in capsys.readouterr().out
    finally:
        node.cmd_follow_stop([])
        node.destroy_node()
        rclpy.shutdown()


def test_publish_next_following_pose_publishes_correct_pose_and_increments():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.target_pose_pub = Mock()
        node.following_index = 0

        node._publish_next_following_pose()

        node.target_pose_pub.publish.assert_called_once()
        sent = node.target_pose_pub.publish.call_args[0][0]
        assert sent.header.frame_id == 'map'
        assert sent.pose.position.x == -1.5
        assert sent.pose.position.y == 0.5
        assert node.following_index == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_publish_next_following_pose_stops_after_ten_points():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.target_pose_pub = Mock()
        node._stop_following_timer = Mock()
        node.following_index = 10

        node._publish_next_following_pose()

        node.target_pose_pub.publish.assert_not_called()
        node._stop_following_timer.assert_called_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_follow_stop_cancels_timer():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.cmd_follow_start(['100'])
        assert node.following_timer is not None

        node.cmd_follow_stop([])

        assert node.following_timer is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_follow_stop_noop_when_not_running():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.cmd_follow_stop([])  # 예외 없이 통과해야 함
        assert node.following_timer is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
