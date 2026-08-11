"""관리자 HMI 백엔드의 DB·지도·안전 제어 계약 테스트."""

import sqlite3
from unittest.mock import Mock

from fastapi import HTTPException
from irobot_create_msgs.msg import DockStatus
import pytest
import rclpy

from robot_manager import hmi_backend_node as backend
from robot_manager.hmi_backend_node import HmiBackendNode
from robot_status.msg import NavigationResult, TaskState

from .test_db_manager_pipeline import SCHEMA


def _make_node(tmp_path):
    db_path = tmp_path / 'hmi.db'
    with sqlite3.connect(db_path) as conn:
        conn.executescript(SCHEMA)
        conn.execute('INSERT INTO destinations VALUES (?, ?, ?, ?)',
                     ('DEST-A', 'A 구역', 3.2, -1.4))
    rclpy.init(args=['--ros-args', '-p', f'db_path:={db_path}'])
    return HmiBackendNode()


def test_admin_hmi_loads_installed_map_and_database(tmp_path):
    """설치된 지도와 운영 DB가 관리자 화면 형식으로 정상 변환되는지 확인한다."""
    node = _make_node(tmp_path)
    try:
        map_data = node.fetch_map()
        assert map_data['width'] > 0 and map_data['height'] > 0
        assert len(map_data['cells']) == map_data['width'] * map_data['height']
        assert map_data['resolution'] > 0
        assert node.fetch_latest_robot_status() == {}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_admin_hmi_destination_api_returns_map_labels(tmp_path, monkeypatch):
    node = _make_node(tmp_path)
    monkeypatch.setattr(backend, 'ros_node', node)
    try:
        destinations = backend.get_hmi_destinations()
        assert destinations == [{
            'destination_id': 'DEST-A',
            'destination_name': 'A 구역',
            'position_x': 3.2,
            'position_y': -1.4,
        }]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_manual_control_is_blocked_until_cancel_or_error(tmp_path):
    """정상 작업 중 텔레옵·Dock을 차단하고 취소 뒤에만 허용한다."""
    node = _make_node(tmp_path)
    try:
        node.task_command_publisher = Mock()
        node.teleop_publishers['robot5'] = Mock()

        state = TaskState()
        state.robot_id, state.task_id, state.state = 'robot5', 'TASK-1', 'FOLLOWING'
        node.task_state_callback(state)
        assert node.set_dock('robot5', True) is False
        assert node.set_teleop_mode('robot5', True) is False

        state.state = 'CANCELED'
        node.task_state_callback(state)
        assert node.set_teleop_mode('robot5', True) is True
        assert node.publish_teleop('robot5', 0.1, 0.2) is True
        assert node.publish_teleop('robot5', 9.0, 0.0) is False
        dock_status = DockStatus()
        dock_status.is_docked = False
        node.dock_status_callback('robot5', dock_status)
        assert node.set_dock('robot5', True) is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_actual_dock_status_remains_authoritative_in_error_state(tmp_path):
    """ERROR 같은 작업 상태 추정값 대신 Create3 is_docked로 버튼 상태를 결정한다."""
    node = _make_node(tmp_path)
    try:
        control = node.control_states['robot11']
        assert control['dock_status_known'] == 0

        status = DockStatus()
        status.is_docked = False
        node.dock_status_callback('robot11', status)

        task = TaskState()
        task.robot_id, task.task_id, task.state = 'robot11', 'TASK-ERROR', 'ERROR'
        node.task_state_callback(task)
        assert control['dock_status_known'] == 1
        assert control['docked'] == 0

        status.is_docked = True
        node.dock_status_callback('robot11', status)
        assert control['docked'] == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_error_dock_success_shows_docked_without_hiding_assigned_state(tmp_path):
    node = _make_node(tmp_path)
    try:
        with sqlite3.connect(node.db_path) as conn:
            conn.execute(
                "INSERT INTO tasks (task_id, assigned_robot_id, state, result) "
                "VALUES ('TASK-ERROR', 'robot11', 'ERROR', 'FAILED')")
            conn.execute(
                "INSERT INTO robot_status_logs "
                "(robot_id, online, state, current_task_id, battery, position_x, position_y, orientation_yaw) "
                "VALUES ('robot11', 'ONLINE', 'ERROR', 'TASK-ERROR', 80, 1, 2, 0)")

        state = TaskState()
        state.robot_id, state.task_id, state.state = 'robot11', 'TASK-ERROR', 'ERROR'
        node.task_state_callback(state)
        dock_status = DockStatus()
        dock_status.is_docked = False
        node.dock_status_callback('robot11', dock_status)

        result = NavigationResult()
        result.robot_id, result.task_id = 'robot11', 'TASK-ERROR'
        result.goal_type, result.success = 'DOCK', True
        node.navigation_result_callback(result)
        assert node.fetch_hmi_robots()[0]['mode'] == 'DOCKED'

        with sqlite3.connect(node.db_path) as conn:
            conn.execute(
                "UPDATE tasks SET task_id='TASK-ASSIGNED', state='ASSIGNED', result=NULL "
                "WHERE assigned_robot_id='robot11'")
        state.task_id, state.state = 'TASK-ASSIGNED', 'ASSIGNED'
        node.task_state_callback(state)
        assert node.fetch_hmi_robots()[0]['mode'] == 'ASSIGNED'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_admin_api_rejects_unknown_robot_and_unauthorized_manual_state(monkeypatch):
    fake = Mock()
    fake.control_states = {'robot5': {}}
    fake.set_dock.return_value = False
    monkeypatch.setattr(backend, 'ros_node', fake)

    with pytest.raises(HTTPException) as unknown:
        backend.set_robot_dock('robot99', backend.DockRequest(dock=True))
    assert unknown.value.status_code == 404

    with pytest.raises(HTTPException) as conflict:
        backend.set_robot_dock('robot5', backend.DockRequest(dock=True))
    assert conflict.value.status_code == 409


def test_login_rejects_invalid_password():
    with pytest.raises(HTTPException) as error:
        backend.login(backend.LoginRequest(username='invalid', password='invalid'))
    assert error.value.status_code == 401
