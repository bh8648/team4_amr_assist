import os
import sqlite3
import tempfile
from unittest.mock import Mock

import rclpy

from robot_status.msg import RobotAssignment, RobotError, TaskState

from robot_manager.webcam_pc_cli import WebcamPcCliNode


def _make_temp_db():
    fd, path = tempfile.mkstemp(suffix='.db')
    os.close(fd)
    conn = sqlite3.connect(path)
    conn.execute('''
        CREATE TABLE destinations (
            destination_id   TEXT PRIMARY KEY,
            destination_name TEXT NOT NULL,
            position_x       REAL NOT NULL,
            position_y       REAL NOT NULL,
            orientation_yaw  REAL NOT NULL
        )
    ''')
    conn.commit()
    conn.close()
    return path


def test_assignment_callback_prints_success(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        msg = RobotAssignment()
        msg.assigned, msg.robot_id, msg.target_x, msg.target_y = True, 'robot11', -1.0, 0.0
        node.assignment_callback(msg)
        assert '배정 성공' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_assignment_callback_prints_failure(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        msg = RobotAssignment()
        msg.assigned = False
        node.assignment_callback(msg)
        assert '배정 실패' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()


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


def test_fetch_destinations_reads_rows_from_db():
    rclpy.init()
    node = WebcamPcCliNode()
    db_path = _make_temp_db()
    try:
        conn = sqlite3.connect(db_path)
        conn.execute("INSERT INTO destinations VALUES ('DEST_A', '목적지 A', -0.5, -2.0, 3.14159)")
        conn.commit()
        conn.close()
        node.db_path = db_path

        destinations = node.fetch_destinations()

        assert len(destinations) == 1
        assert destinations[0]['destination_id'] == 'DEST_A'
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.remove(db_path)


def test_fetch_destinations_empty_when_table_empty():
    rclpy.init()
    node = WebcamPcCliNode()
    db_path = _make_temp_db()
    try:
        node.db_path = db_path
        assert node.fetch_destinations() == []
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.remove(db_path)


def test_cmd_call_publishes_assignment_goal():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.assignment_goal_pub = Mock()
        node.cmd_call(['-1', '0'])
        node.assignment_goal_pub.publish.assert_called_once()
        sent = node.assignment_goal_pub.publish.call_args[0][0]
        assert sent.x == -1.0
        assert sent.y == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_call_invalid_args_does_not_publish():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.assignment_goal_pub = Mock()
        node.cmd_call(['only-one'])
        node.assignment_goal_pub.publish.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_list_destinations_prints_rows(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    db_path = _make_temp_db()
    try:
        conn = sqlite3.connect(db_path)
        conn.execute("INSERT INTO destinations VALUES ('DEST_A', '목적지 A', -0.5, -2.0, 3.14159)")
        conn.commit()
        conn.close()
        node.db_path = db_path

        node.cmd_list_destinations([])

        assert 'DEST_A' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.remove(db_path)


def test_cmd_worker_detected_publishes_with_active_robot():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.task_command_pub = Mock()
        state = TaskState()
        state.robot_id, state.state, state.task_id = 'robot11', 'ASSIGNED', 'TASK_1'
        node.task_state_callback(state)

        node.cmd_worker_detected([])

        node.task_command_pub.publish.assert_called_once()
        sent = node.task_command_pub.publish.call_args[0][0]
        assert sent.command == 'WORKER_DETECTED'
        assert sent.robot_id == 'robot11'
        assert sent.task_id == 'TASK_1'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_worker_detected_no_active_robot_does_not_publish():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.task_command_pub = Mock()
        node.cmd_worker_detected([])
        node.task_command_pub.publish.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_deliver_publishes_with_destination_coords():
    rclpy.init()
    node = WebcamPcCliNode()
    db_path = _make_temp_db()
    try:
        conn = sqlite3.connect(db_path)
        conn.execute("INSERT INTO destinations VALUES ('DEST_A', '목적지 A', -0.5, -2.0, 3.14159)")
        conn.commit()
        conn.close()
        node.db_path = db_path
        node.task_command_pub = Mock()
        state = TaskState()
        state.robot_id, state.state, state.task_id = 'robot11', 'FOLLOWING', 'TASK_1'
        node.task_state_callback(state)

        node.cmd_deliver(['DEST_A'])

        sent = node.task_command_pub.publish.call_args[0][0]
        assert sent.command == 'START_TRANSPORT'
        assert sent.target_x == -0.5
        assert sent.target_y == -2.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.remove(db_path)


def test_cmd_deliver_unknown_destination_does_not_publish():
    rclpy.init()
    node = WebcamPcCliNode()
    db_path = _make_temp_db()
    try:
        node.db_path = db_path
        node.task_command_pub = Mock()
        state = TaskState()
        state.robot_id, state.state, state.task_id = 'robot11', 'FOLLOWING', 'TASK_1'
        node.task_state_callback(state)

        node.cmd_deliver(['DEST_Z'])

        node.task_command_pub.publish.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.remove(db_path)


def test_cmd_confirm_publishes():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.task_command_pub = Mock()
        state = TaskState()
        state.robot_id, state.state, state.task_id = 'robot11', 'TRANSPORTING', 'TASK_1'
        node.task_state_callback(state)

        node.cmd_confirm([])

        sent = node.task_command_pub.publish.call_args[0][0]
        assert sent.command == 'DELIVERY_CONFIRMED'
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
