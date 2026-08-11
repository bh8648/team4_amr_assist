"""관리자 HMI 백엔드의 DB·지도·안전 제어 계약 테스트."""

import sqlite3
from unittest.mock import Mock

from fastapi import HTTPException
import pytest
import rclpy

from robot_manager import hmi_backend_node as backend
from robot_manager.hmi_backend_node import HmiBackendNode
from robot_status.msg import TaskState

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
        assert node.set_dock('robot5', True) is True
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
