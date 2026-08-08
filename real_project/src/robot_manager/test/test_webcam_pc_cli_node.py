import os
import sqlite3
import tempfile

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
