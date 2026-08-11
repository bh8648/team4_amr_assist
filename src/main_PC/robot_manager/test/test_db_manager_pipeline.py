import sqlite3

import rclpy

from robot_manager.db_manager_node import DbManagerNode
from robot_status.msg import RobotAssignment, RobotError, RobotStatus, TaskState


SCHEMA = """
CREATE TABLE destinations (
    destination_id TEXT PRIMARY KEY,
    destination_name TEXT NOT NULL,
    position_x REAL NOT NULL,
    position_y REAL NOT NULL
);
CREATE TABLE tasks (
    task_id TEXT PRIMARY KEY,
    assigned_robot_id TEXT NOT NULL,
    destination_id TEXT,
    state TEXT NOT NULL,
    result TEXT,
    created_at TEXT NOT NULL DEFAULT (datetime('now', 'localtime')),
    completed_at TEXT,
    duration_seconds INTEGER
);
CREATE TABLE robot_status_logs (
    status_id INTEGER PRIMARY KEY AUTOINCREMENT,
    robot_id TEXT NOT NULL,
    online TEXT NOT NULL,
    state TEXT NOT NULL,
    current_task_id TEXT,
    battery REAL NOT NULL,
    position_x REAL NOT NULL,
    position_y REAL NOT NULL,
    orientation_yaw REAL NOT NULL,
    recorded_at TEXT NOT NULL DEFAULT (datetime('now', 'localtime'))
);
CREATE TABLE error_logs (
    error_id INTEGER PRIMARY KEY AUTOINCREMENT,
    robot_id TEXT NOT NULL,
    task_id TEXT,
    error_code TEXT NOT NULL,
    occurred_at TEXT NOT NULL DEFAULT (datetime('now', 'localtime'))
);
"""


def _state(robot_id, task_id, state, destination_id=''):
    msg = TaskState()
    msg.robot_id, msg.task_id, msg.state = robot_id, task_id, state
    msg.destination_id = destination_id
    return msg


def test_assignment_state_status_and_error_are_persisted(tmp_path):
    """실제 운영 DB를 건드리지 않고 전체 토픽의 SQLite 기록을 검증한다."""
    db_path = tmp_path / 'pipeline.db'
    with sqlite3.connect(db_path) as conn:
        conn.executescript(SCHEMA)
        conn.execute(
            'INSERT INTO destinations VALUES (?, ?, ?, ?)',
            ('DEST-A', 'A 구역', 3.2, -1.4),
        )

    rclpy.init(args=['--ros-args', '-p', f'db_path:={db_path}'])
    node = DbManagerNode()
    try:
        assignment = RobotAssignment()
        assignment.assigned = True
        assignment.assigned_at.sec = 100
        assignment.robot_id = 'robot5'
        assignment.target_x, assignment.target_y = 1.0, 2.0
        node.assignment_callback(assignment)
        task_id = 'TASK_100_0'

        node.task_state_callback(_state('robot5', task_id, 'FOLLOWING'))
        node.task_state_callback(_state('robot5', task_id, 'TRANSPORTING', 'DEST-A'))
        node.task_state_callback(_state('robot5', task_id, 'DOCKED', 'DEST-A'))

        status = RobotStatus()
        status.robot_id, status.battery = 'robot5', 82.0
        status.x, status.y, status.yaw = 3.2, -1.4, 0.5
        node.status_callback('robot5', status)
        node.save_status_to_db()

        error = RobotError()
        error.robot_id, error.task_id = 'robot5', task_id
        error.error_code = 'TEST_PIPELINE_ERROR'
        node.error_callback(error)

        task = node.conn.execute(
            'SELECT assigned_robot_id, destination_id, state, result FROM tasks WHERE task_id=?',
            (task_id,),
        ).fetchone()
        assert task == ('robot5', 'DEST-A', 'COMPLETED', 'SUCCESS')

        saved_status = node.conn.execute(
            'SELECT online, state, current_task_id, battery, position_x, position_y '
            'FROM robot_status_logs ORDER BY status_id DESC LIMIT 1'
        ).fetchone()
        assert saved_status == ('ONLINE', 'DOCKED', None, 82.0, 3.2, -1.4)

        saved_error = node.conn.execute(
            'SELECT robot_id, task_id, error_code FROM error_logs ORDER BY error_id DESC LIMIT 1'
        ).fetchone()
        assert saved_error == ('robot5', task_id, 'TEST_PIPELINE_ERROR')
    finally:
        node.destroy_node()
        rclpy.shutdown()
