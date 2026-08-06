#!/usr/bin/env python3
import os
import secrets
import sqlite3
import threading
import uvicorn
from pathlib import Path
from typing import Optional
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# ROS 2 표준/커스텀 서비스 import
from std_srvs.srv import Trigger  # 필요 시 커스텀 srv로 변경 가능 (예: robot_status.srv.CancelTask)
from robot_status.msg import AssignmentGoal


# =========================================================
# 1. Pydantic 요청 Body 모델
# =========================================================
class CancelTaskRequest(BaseModel):
    robot_id: int
    task_id: Optional[str] = None

class EmergencyStopRequest(BaseModel):
    robot_id: int
    stop_state: bool = True  # True: 비상정지 실행, False: 해제


class AssignmentGoalRequest(BaseModel):
    x: float
    y: float


class ActiveStateRequest(BaseModel):
    active: bool


class DockRequest(BaseModel):
    dock: bool


class LoginRequest(BaseModel):
    username: str
    password: str


# =========================================================
# 2. HMI 백엔드 ROS 2 노드
# =========================================================
class HmiBackendNode(Node):
    def __init__(self):
        super().__init__('hmi_backend_node')

        # DbManagerNode와 같은 DB를 사용한다. 실행 위치가 달라도 환경 변수로
        # 명시할 수 있으며, 기본값은 현재 작업 디렉터리의 amr.db이다.
        default_db_path = os.path.abspath('amr.db')
        self.declare_parameter(
            'db_path', os.environ.get('AMR_DB_PATH', default_db_path)
        )
        self.db_path = os.path.abspath(
            os.path.expanduser(self.get_parameter('db_path').value)
        )
        default_map_path = os.path.abspath(
            'amr_delivery_ui/backend/maps/turtle5_map.yaml'
        )
        self.declare_parameter(
            'map_yaml_path', os.environ.get('AMR_MAP_YAML', default_map_path)
        )
        self.map_yaml_path = Path(
            os.path.expanduser(self.get_parameter('map_yaml_path').value)
        ).resolve()

        # 중앙 노드 통신용 ROS 2 서비스 클라이언트 생성
        self.cancel_task_client = self.create_client(Trigger, '/cancel_task')
        self.estop_client = self.create_client(Trigger, '/emergency_stop')
        self.assignment_goal_publisher = self.create_publisher(
            AssignmentGoal, '/assignment_goal', 10
        )
        self.control_states = {}
        self.estop_publishers = {}
        self.dock_publishers = {}
        for robot_id in ('5', '11'):
            self.estop_publishers[robot_id] = self.create_publisher(
                Bool, f'/amr{robot_id}/emergency_stop/request', 10
            )
            self.dock_publishers[robot_id] = self.create_publisher(
                Bool, f'/amr{robot_id}/dock/request', 10
            )
            self.control_states[robot_id] = {'estopped': 0, 'docked': 0}

        self.get_logger().info(f'HMI Backend Node가 시작되었습니다. (DB 경로: {self.db_path})')

    def fetch_error_logs(self, limit: int = 50):
        """DB에서 최신 오류 로그 조회 (SELECT)"""
        if not os.path.exists(self.db_path):
            return []

        try:
            conn = sqlite3.connect(self.db_path)
            conn.row_factory = sqlite3.Row  # 결과를 딕셔너리 형태로 반환
            cursor = conn.cursor()

            query = """
                SELECT error_id, robot_id, task_id, error_code, occurred_at, resolved_at
                FROM error_logs
                ORDER BY error_id DESC
                LIMIT ?
            """
            cursor.execute(query, (limit,))
            rows = cursor.fetchall()
            conn.close()

            return [dict(row) for row in rows]
        except sqlite3.Error as e:
            self.get_logger().error(f'Error logs DB 조회 실패: {e}')
            return []

    def fetch_latest_robot_status(self):
        """DB에서 각 로봇별 가장 최근 상태 1개씩 가져와 robot_id를 Key로 하는 딕셔너리로 반환"""
        if not os.path.exists(self.db_path):
            return {}

        try:
            conn = sqlite3.connect(self.db_path)
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()

            # 각 robot_id별 가장 마지막(MAX) status_id 행만 추출
            query = """
                SELECT * FROM robot_status_logs 
                WHERE status_id IN (
                    SELECT MAX(status_id) FROM robot_status_logs GROUP BY robot_id
                )
            """
            cursor.execute(query)
            rows = cursor.fetchall()
            conn.close()

            # 결과를 { robot_id: {상태 데이터...} } 딕셔너리 구조로 맵핑
            result = {}
            for row in rows:
                data = dict(row)
                # data['robot_id']가 문자열/정수여도 딕셔너리 키로 사용
                result[data['robot_id']] = data

            return result
        except sqlite3.Error as e:
            self.get_logger().error(f'Robot status DB 조회 실패: {e}')
            return {}

    @staticmethod
    def to_hmi_robot(row):
        """robot_status_logs 행을 HMI 화면이 사용하는 데이터 형식으로 변환."""
        data = dict(row)
        online = str(data.get('online', '')).upper()
        is_online = online == 'ONLINE'
        return {
            'robot_id': str(data['robot_id']),
            'mode': data.get('state') or 'IDLE',
            'battery': float(data.get('battery') or 0),
            'pose_x': float(data.get('position_x') or 0),
            'pose_y': float(data.get('position_y') or 0),
            'pose_yaw': float(data.get('orientation_yaw') or 0),
            'current_task_id': data.get('current_task_id'),
            'estopped': 0,
            'online': 'ONLINE' if is_online else 'OFFLINE',
            'last_update': data.get('recorded_at'),
            'sensors': {
                'lidar': 'ONLINE' if is_online else 'OFFLINE',
                'rgbd': 'ONLINE' if is_online else 'OFFLINE',
                'odom': 'ONLINE' if is_online else 'OFFLINE',
                'nav2': 'ONLINE' if is_online else 'OFFLINE',
            },
            'payload_loaded': 0,
            'payload_weight': 0,
            'worker_distance': None,
            'emergency_stopped': 0,
        }

    def fetch_hmi_robots(self):
        latest = self.fetch_latest_robot_status()
        robots = []
        with sqlite3.connect(self.db_path) as conn:
            conn.row_factory = sqlite3.Row
            for _, row in sorted(latest.items(), key=lambda item: str(item[0])):
                robot = self.to_hmi_robot(row)
                task = conn.execute(
                    """
                    SELECT * FROM tasks
                    WHERE assigned_robot_id = ?
                    ORDER BY created_at DESC LIMIT 1
                    """,
                    (robot['robot_id'],)
                ).fetchone()
                robot['current_task'] = dict(task) if task else None
                if task and task['state'] not in ('COMPLETED', 'CANCELLED', 'FAILED'):
                    robot['mode'] = task['state']
                    robot['current_task_id'] = task['task_id']
                control = self.control_states.get(
                    robot['robot_id'], {'estopped': 0, 'docked': 0}
                )
                robot['estopped'] = control['estopped']
                robot['emergency_stopped'] = control['estopped']
                robot['docked'] = control['docked']
                robots.append(robot)
        return robots

    def fetch_hmi_robot(self, robot_id: str):
        robot_id = str(robot_id)
        for robot in self.fetch_hmi_robots():
            if robot['robot_id'] == robot_id:
                return robot
        return None

    def fetch_destinations(self):
        with sqlite3.connect(self.db_path) as conn:
            conn.row_factory = sqlite3.Row
            rows = conn.execute(
                'SELECT * FROM destinations ORDER BY destination_name'
            ).fetchall()
        return [dict(row) for row in rows]

    def publish_assignment_goal(self, x: float, y: float):
        goal = AssignmentGoal()
        goal.x = float(x)
        goal.y = float(y)
        self.assignment_goal_publisher.publish(goal)

    def set_emergency_stop(self, robot_id: str, active: bool):
        robot_id = str(robot_id)
        publisher = self.estop_publishers.get(robot_id)
        if publisher is None:
            return False
        publisher.publish(Bool(data=bool(active)))
        self.control_states[robot_id]['estopped'] = int(active)
        self.get_logger().warn(
            f'AMR {robot_id} 일시정지 요청: {"ON" if active else "OFF"}'
        )
        return True

    def set_dock(self, robot_id: str, dock: bool):
        robot_id = str(robot_id)
        publisher = self.dock_publishers.get(robot_id)
        if publisher is None:
            return False
        publisher.publish(Bool(data=bool(dock)))
        self.control_states[robot_id]['docked'] = int(dock)
        self.get_logger().info(
            f'AMR {robot_id} 도킹 요청: {"DOCK" if dock else "UNDOCK"}'
        )
        return True

    @staticmethod
    def _read_pgm(path: Path):
        data = path.read_bytes()
        offset = 0

        def token():
            nonlocal offset
            while offset < len(data):
                if data[offset] == 35:
                    while offset < len(data) and data[offset] != 10:
                        offset += 1
                elif data[offset] <= 32:
                    offset += 1
                else:
                    break
            start = offset
            while offset < len(data) and data[offset] > 32:
                offset += 1
            return data[start:offset].decode('ascii')

        if token() != 'P5':
            raise ValueError('P5 형식의 PGM 지도만 지원합니다.')
        width, height, maximum = int(token()), int(token()), int(token())
        if data[offset:offset + 2] == b'\r\n':
            offset += 2
        elif offset < len(data) and data[offset] <= 32:
            offset += 1
        pixels = data[offset:offset + width * height]
        if maximum != 255 or len(pixels) != width * height:
            raise ValueError('PGM 지도 데이터가 올바르지 않습니다.')
        cells = [0] * (width * height)
        for row in range(height):
            for x in range(width):
                value = pixels[row * width + x]
                cells[(height - 1 - row) * width + x] = (
                    1 if value < 90 else 0 if value > 191 else -1
                )
        return width, height, cells

    def fetch_map(self):
        if not self.map_yaml_path.exists():
            raise FileNotFoundError(str(self.map_yaml_path))
        meta = {}
        for raw_line in self.map_yaml_path.read_text(encoding='utf-8').splitlines():
            line = raw_line.split('#', 1)[0].strip()
            if not line or ':' not in line:
                continue
            key, value = line.split(':', 1)
            meta[key.strip()] = value.strip()
        image_path = (self.map_yaml_path.parent / meta['image']).resolve()
        origin = [float(value.strip()) for value in meta['origin'].strip('[]').split(',')]
        width, height, cells = self._read_pgm(image_path)
        return {
            'width': width,
            'height': height,
            'resolution': float(meta['resolution']),
            'origin': origin,
            'cells': cells,
        }

    def call_cancel_task_service(self, robot_id: int, task_id: Optional[str]):
        """중앙 제어 노드로 작업 강제 종료 서비스 요청"""
        if not self.cancel_task_client.wait_for_service(timeout_sec=1.5):
            return False, "작업 취소 서비스(/cancel_task) 응답 없음"

        req = Trigger.Request()
        future = self.cancel_task_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result() is not None:
            return future.result().success, future.result().message
        return False, "작업 취소 서비스 요청 시간 초과"

    def call_emergency_stop_service(self, robot_id: int, stop_state: bool):
        """중앙 제어 노드로 비상정지 서비스 요청"""
        if not self.estop_client.wait_for_service(timeout_sec=1.5):
            return False, "비상정지 서비스(/emergency_stop) 응답 없음"

        req = Trigger.Request()
        future = self.estop_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result() is not None:
            return future.result().success, future.result().message
        return False, "비상정지 서비스 요청 시간 초과"


# =========================================================
# 3. FastAPI REST 서버 구성
# =========================================================
app = FastAPI(title="AMR HMI Backend Server API")

# React(localhost:3000 등)와의 CORS 허용 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ros_node: Optional[HmiBackendNode] = None
auth_tokens = set()
HMI_USERNAME = os.environ.get('HMI_USERNAME', 'rokey')
HMI_PASSWORD = os.environ.get('HMI_PASSWORD', '1234')
HMI_DB_TABLES = {
    'destinations': 'destination_id',
    'tasks': 'created_at',
    'error_logs': 'error_id',
    'robot_status_logs': 'status_id',
}


@app.middleware("http")
async def require_authentication(request: Request, call_next):
    """로그인 API 이외의 모든 /api 요청에 세션 토큰을 요구한다."""
    if request.method == 'OPTIONS' or request.url.path == '/api/auth/login':
        return await call_next(request)
    if request.url.path.startswith('/api/'):
        header = request.headers.get('Authorization', '')
        token = header.removeprefix('Bearer ').strip()
        if not token or token not in auth_tokens:
            return JSONResponse(status_code=401, content={'detail': '로그인이 필요합니다.'})
    return await call_next(request)


@app.post('/api/auth/login')
def login(data: LoginRequest):
    username_ok = secrets.compare_digest(data.username, HMI_USERNAME)
    password_ok = secrets.compare_digest(data.password, HMI_PASSWORD)
    if not username_ok or not password_ok:
        raise HTTPException(status_code=401, detail='아이디 또는 비밀번호가 올바르지 않습니다.')
    token = secrets.token_urlsafe(32)
    auth_tokens.add(token)
    return {'token': token, 'username': data.username}


@app.get('/api/database/tables')
def get_database_tables():
    """HMI에서 조회할 수 있는 DB 테이블과 행 개수."""
    if not ros_node:
        raise HTTPException(status_code=503, detail='ROS 노드가 준비되지 않았습니다.')
    result = []
    with sqlite3.connect(ros_node.db_path) as conn:
        existing = {
            row[0] for row in conn.execute(
                "SELECT name FROM sqlite_master WHERE type='table'"
            ).fetchall()
        }
        for table in HMI_DB_TABLES:
            if table in existing:
                count = conn.execute(f'SELECT COUNT(*) FROM {table}').fetchone()[0]
                result.append({'name': table, 'count': count})
    return result


@app.get('/api/database/table/{table_name}')
def get_database_table(table_name: str, limit: int = 100):
    """화이트리스트에 등록된 테이블의 최근 데이터 조회."""
    if not ros_node:
        raise HTTPException(status_code=503, detail='ROS 노드가 준비되지 않았습니다.')
    order_column = HMI_DB_TABLES.get(table_name)
    if order_column is None:
        raise HTTPException(status_code=404, detail='조회가 허용되지 않은 테이블입니다.')
    safe_limit = max(1, min(limit, 500))
    with sqlite3.connect(ros_node.db_path) as conn:
        conn.row_factory = sqlite3.Row
        cursor = conn.execute(
            f'SELECT * FROM {table_name} ORDER BY {order_column} DESC LIMIT ?',
            (safe_limit,)
        )
        rows = cursor.fetchall()
        columns = [description[0] for description in cursor.description]
    return {'table': table_name, 'columns': columns, 'rows': [dict(row) for row in rows]}


@app.get("/api/errors")
def get_errors(limit: int = 50):
    """6. 오류 코드 리스트 조회 API"""
    if not ros_node:
        raise HTTPException(status_code=500, detail="ROS 노드가 준비되지 않았습니다.")
    return ros_node.fetch_error_logs(limit)


@app.get("/api/robots/latest")
def get_latest_robots():
    """DB에 저장된 로봇별 최신 상태 정보 조회 API"""
    if not ros_node:
        raise HTTPException(status_code=500, detail="ROS 노드가 준비되지 않았습니다.")
    return ros_node.fetch_latest_robot_status()


@app.get("/api/robots")
def get_hmi_robots():
    """HMI 대시보드용 로봇 상태 목록."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    return ros_node.fetch_hmi_robots()


@app.get("/api/robot/{robot_id}/state")
def get_hmi_robot_state(robot_id: str):
    """HMI 상세 화면용 단일 로봇 최신 상태."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    robot = ros_node.fetch_hmi_robot(robot_id)
    if robot is None:
        raise HTTPException(status_code=404, detail=f"로봇 {robot_id}의 상태가 없습니다.")
    return robot


@app.get("/api/destinations")
def get_destinations():
    """amr.db에 등록된 목적지 목록."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    return ros_node.fetch_destinations()


@app.get("/api/map")
def get_map():
    """Nav2 YAML/PGM 지도를 HMI 렌더러 형식으로 반환."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    try:
        return ros_node.fetch_map()
    except (OSError, KeyError, ValueError) as error:
        raise HTTPException(status_code=500, detail=f"지도 로드 실패: {error}") from error


@app.post("/api/assignment/goal")
def request_robot_assignment(data: AssignmentGoalRequest):
    """HMI 목표를 AMR 자동 배정 노드에 전달."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    ros_node.publish_assignment_goal(data.x, data.y)
    return {"accepted": True, "target_x": data.x, "target_y": data.y}


@app.get("/api/robot/{robot_id}/task")
def get_robot_task(robot_id: str):
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    robot = ros_node.fetch_hmi_robot(robot_id)
    if robot is None:
        raise HTTPException(status_code=404, detail="로봇 상태가 없습니다.")
    return robot.get('current_task')


@app.post("/api/robot/{robot_id}/estop")
def set_robot_estop(robot_id: str, data: ActiveStateRequest):
    """AMR별 소프트웨어 일시정지/해제 요청 토픽 발행."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    if not ros_node.set_emergency_stop(robot_id, data.active):
        raise HTTPException(status_code=404, detail=f"지원하지 않는 AMR ID: {robot_id}")
    return {"accepted": True, "robot_id": robot_id, "estopped": data.active}


@app.post("/api/robot/{robot_id}/dock")
def set_robot_dock(robot_id: str, data: DockRequest):
    """AMR별 도킹/언도킹 요청 토픽 발행."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    if not ros_node.set_dock(robot_id, data.dock):
        raise HTTPException(status_code=404, detail=f"지원하지 않는 AMR ID: {robot_id}")
    return {"accepted": True, "robot_id": robot_id, "docked": data.dock}


@app.post("/api/task/cancel")
def cancel_task(data: CancelTaskRequest):
    """2. 작업 강제 종료 요청 API"""
    if not ros_node:
        raise HTTPException(status_code=500, detail="ROS 노드가 준비되지 않았습니다.")

    success, message = ros_node.call_cancel_task_service(data.robot_id, data.task_id)
    return {"success": success, "message": message, "robot_id": data.robot_id}


@app.post("/api/emergency-stop")
def emergency_stop(data: EmergencyStopRequest):
    """5. 비상정지 요청 API"""
    if not ros_node:
        raise HTTPException(status_code=500, detail="ROS 노드가 준비되지 않았습니다.")

    success, message = ros_node.call_emergency_stop_service(data.robot_id, data.stop_state)
    return {"success": success, "message": message, "robot_id": data.robot_id}


# =========================================================
# 4. 메인 실행부 (Spin 스레드 + FastAPI Uvicorn)
# =========================================================
def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = HmiBackendNode()

    # ROS 2 spin을 데몬 스레드로 실행하여 FastAPI와 병렬 구동
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    # Uvicorn 웹 서버 실행 (포트 8000)
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
