#!/usr/bin/env python3
import os
import math
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
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import DockStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_status.msg import NavigationResult, TaskCommand, TaskState

# =========================================================
# 1. Pydantic 요청 Body 모델
# =========================================================
class ActiveStateRequest(BaseModel):    # 로봇 일시정지 요청
    active: bool


class DockRequest(BaseModel):       # 도킹/언도킹 요청
    dock: bool


class TeleopRequest(BaseModel):
    linear: float = 0.0
    angular: float = 0.0


class TeleopModeRequest(BaseModel):
    # HMI의 단일 토글 버튼에서 전달하는 텔레옵 활성화 상태
    active: bool


class LoginRequest(BaseModel):      # HMI 로그인 요청
    username: str
    password: str


# =========================================================
# 2. HMI 백엔드 ROS 2 노드
# =========================================================
class HmiBackendNode(Node):
    # 도킹/언도킹과 텔레옵은 작업이 취소되었거나 오류가 난 로봇에서만 허용한다.
    MANUAL_CONTROL_STATES = {'CANCELED', 'ERROR'}
    MAX_TELEOP_LINEAR = 0.25
    MAX_TELEOP_ANGULAR = 1.0

    def __init__(self):
        super().__init__('hmi_backend_node')

        default_db_path = os.path.abspath('amr.db') # 데이터베이스 경로
        self.declare_parameter('db_path', os.environ.get('AMR_DB_PATH', default_db_path))

        self.db_path = os.path.abspath(os.path.expanduser(self.get_parameter('db_path').value)) # 지도의 경로
        package_share = Path(get_package_share_directory('robot_manager')).resolve()
        default_map_path = str(package_share / 'maps/map2.yaml')

        self.declare_parameter('map_yaml_path', os.environ.get('AMR_MAP_YAML', default_map_path))   # 지도 설정값 경로
        configured_map_path = Path(os.path.expanduser(self.get_parameter('map_yaml_path').value))
        self.map_yaml_path = (configured_map_path if configured_map_path.is_absolute() else package_share / configured_map_path).resolve()
        if not self.map_yaml_path.is_file():
            self.map_yaml_path = Path(default_map_path).resolve()

        self.task_command_publisher = self.create_publisher(TaskCommand, '/task/command', 10)   # 정지, 도킹, 언도킹 등의 명령을 보낼 퍼블리셔
        self.teleop_publishers = {robot_id: self.create_publisher(Twist, f'/{robot_id}/cmd_vel', 10) for robot_id in ('robot5', 'robot11')}
        self.task_state_subscription = self.create_subscription(TaskState, '/task/state', self.task_state_callback, 10)
        # Create3가 주기적으로 발행하는 실제 도킹 센서값을 HMI 버튼의 유일한 기준으로 사용한다.
        self.dock_status_subscriptions = [
            self.create_subscription(
                DockStatus, f'/{robot_id}/dock_status',
                lambda msg, selected_robot=robot_id: self.dock_status_callback(selected_robot, msg),
                qos_profile_sensor_data,
            )
            for robot_id in ('robot5', 'robot11')
        ]
        # 브릿지의 실제 Dock/Undock 액션 성공 결과로만 HMI 도킹 표시를 확정한다.
        self.navigation_result_subscription = self.create_subscription(
            NavigationResult, '/navigation/result', self.navigation_result_callback, 10)

        self.control_states = {}

        for robot_id in ('robot5', 'robot11'):  # 로봇마다 비상정지, 도킹 상태를 관리하기 위한 딕셔너리 초기화
            self.control_states[robot_id] = {
                'estopped': 0, 'docked': 0, 'dock_status_known': 0, 'task_state': 'DOCKED',
                'canceled': 0, 'teleop_enabled': 0, 'error_dock_recovered': 0,
            }

        self.get_logger().info('HMI Backend Node시작')

    def task_state_callback(self, msg):
        control = self.control_states.get(str(msg.robot_id))
        if control is None:
            return
        previous_state = control['task_state']
        control['task_state'] = str(msg.state).upper()
        if msg.state == 'DOCKED' and previous_state == 'ERROR':
            control['error_dock_recovered'] = 1
        elif msg.state in ('ASSIGNED', 'ERROR'):
            control['error_dock_recovered'] = 0
        if (msg.state == 'CANCELED'
                or (msg.state == 'RETURNING' and msg.detail == '작업 취소 후 복귀')):
            control['canceled'] = 1
        # 새 작업이 시작되면 이전 작업의 취소/수동조작 허가를 반드시 초기화한다.
        if msg.state == 'ASSIGNED':
            control['canceled'] = 0
            control['teleop_enabled'] = 0

    def dock_status_callback(self, robot_id: str, msg: DockStatus):
        """Create3의 실제 도킹 센서값으로 HMI 제어 상태를 계속 갱신한다."""
        control = self.control_states.get(str(robot_id))
        if control is None:
            return
        control['docked'] = int(bool(msg.is_docked))
        control['dock_status_known'] = 1
        if not msg.is_docked:
            control['error_dock_recovered'] = 0

    def navigation_result_callback(self, msg):
        """DockStatus 갱신 전에도 성공한 실제 액션 결과를 즉시 반영한다."""
        control = self.control_states.get(str(msg.robot_id))
        if control is None or not msg.success:
            return
        if msg.goal_type == 'DOCK':
            was_error = control['task_state'] == 'ERROR'
            if not was_error:
                state, result = self.latest_task_state(str(msg.robot_id))
                was_error = state == 'ERROR' or result == 'FAILED'
            control['docked'] = 1
            control['dock_status_known'] = 1
            if was_error:
                control['task_state'] = 'DOCKED'
                control['error_dock_recovered'] = 1
        elif msg.goal_type == 'UNDOCK':
            control['docked'] = 0
            control['dock_status_known'] = 1
            control['error_dock_recovered'] = 0

    def latest_task_state(self, robot_id: str):
        """재시작 뒤에도 제한을 유지하도록 DB의 가장 최근 작업 상태/결과를 조회한다."""
        if not os.path.exists(self.db_path):
            return None, None
        try:
            with sqlite3.connect(self.db_path) as conn:
                row = conn.execute(
                    """
                    SELECT state, result FROM tasks
                    WHERE assigned_robot_id = ?
                    ORDER BY created_at DESC LIMIT 1
                    """,
                    (robot_id,),
                ).fetchone()
            return (str(row[0]).upper(), str(row[1]).upper() if row[1] else None) if row else (None, None)
        except sqlite3.Error as error:
            self.get_logger().error(f'수동 제어 상태 DB 조회 실패: {error}')
            return None, None

    def manual_control_allowed(self, robot_id: str) -> bool:
        """ROS 실시간 상태와 DB 상태 중 하나가 CANCELED/ERROR일 때만 수동 제어를 허용한다."""
        robot_id = str(robot_id)
        control = self.control_states.get(robot_id)
        if control is None:
            return False
        if control['task_state'] in self.MANUAL_CONTROL_STATES or control['canceled']:
            return True
        state, result = self.latest_task_state(robot_id)
        return state in self.MANUAL_CONTROL_STATES or result in self.MANUAL_CONTROL_STATES

    def publish_zero_velocity(self, robot_id: str) -> None:
        """텔레옵 해제·도킹 전 로봇이 즉시 멈추도록 0 속도를 발행한다."""
        publisher = self.teleop_publishers.get(robot_id)
        if publisher is not None:
            publisher.publish(Twist())

    def fetch_latest_robot_status(self):
        """DB에서 각 로봇별 가장 최근 상태 1개씩 가져와 robot_id를 Key로 하는 딕셔너리로 반환"""
        if not os.path.exists(self.db_path):
            return {}

        try:        # 데이터베이스와 연결
            conn = sqlite3.connect(self.db_path)
            conn.row_factory = sqlite3.Row      # 컬럼명으로 접근(편의성)
            cursor = conn.cursor()

            # 각 robot_id별 가장 마지막(MAX) status_id 행만 추출(가장 최신 데이터)
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
        return {
            'robot_id': str(data['robot_id']),
            'mode': data.get('state') or 'IDLE',
            'battery': float(data.get('battery') or 0),
            'pose_x': float(data.get('position_x') or 0),
            'pose_y': float(data.get('position_y') or 0),
        }

    def fetch_hmi_robots(self): # HMI 대시보드용 로봇 상태 목록을 반환
        latest = self.fetch_latest_robot_status()   # 로봇의 현재 상태를 DB애서 가져옴
        robots = []

        with sqlite3.connect(self.db_path) as conn:
            conn.row_factory = sqlite3.Row
            for _, row in sorted(latest.items(), key=lambda item: str(item[0])):
                robot = self.to_hmi_robot(row)
                task = conn.execute(    # 가장 최근에 수행한 task 정보를 가져옴
                    """
                    SELECT * FROM tasks
                    WHERE assigned_robot_id = ?
                    ORDER BY created_at DESC LIMIT 1
                    """,
                    (robot['robot_id'],)
                ).fetchone()
                robot['current_task'] = dict(task) if task else None

                if task:
                    robot['mode'] = task['state']

                control = self.control_states.get(
                    robot['robot_id'],
                    {'estopped': 0, 'docked': 0, 'dock_status_known': 0},
                )
                dock_status_known = bool(control.get('dock_status_known'))
                if robot['mode'] == 'DOCKED' and dock_status_known and not control['docked']:
                    robot['mode'] = 'IDLE'
                # ASSIGNED 등 정상 작업 상태는 물리 dock 값으로 덮지 않고,
                # ERROR에서 실제 Dock 성공을 확인한 경우에만 운용 상태를 복구한다.
                if (dock_status_known and control['docked']
                        and control.get('error_dock_recovered')):
                    robot['mode'] = 'DOCKED'
                robot['estopped'] = int(bool(control['estopped']) or robot['mode'] == 'PAUSED') # 정지 여부 확인
                robot['dock_status_known'] = int(dock_status_known)
                robot['docked'] = int(bool(control['docked'])) if dock_status_known else 0
                robots.append(robot)
        return robots

    def set_emergency_stop(self, robot_id: str, active: bool):  # AMR별 소프트웨어 일시정지/해제 요청 토픽 발행
        robot_id = str(robot_id)
        if robot_id not in self.control_states:     # 대상 로봇 검증
            return False
        
        command = TaskCommand() 
        command.stamp = self.get_clock().now().to_msg()
        command.robot_id, command.command = robot_id, 'PAUSE' if active else 'RESUME'
        self.task_command_publisher.publish(command)    # 일시정지, 재개 명령을 퍼블리시

        self.control_states[robot_id]['estopped'] = int(active)
        self.get_logger().warn(f'AMR {robot_id} 일시정지 요청: {"ON" if active else "OFF"}')
        return True

    def set_dock(self, robot_id: str, dock: bool):
        robot_id = str(robot_id)
        if robot_id not in self.control_states: # 대상 로봇 검증
            return False
        if not self.manual_control_allowed(robot_id):
            return False
        control = self.control_states[robot_id]
        # 실제 센서값을 받기 전이거나 이미 목표 상태라면 잘못된 수동 액션을 보내지 않는다.
        if not control['dock_status_known'] or bool(control['docked']) == bool(dock):
            return False

        # 도킹 동작과 cmd_vel이 동시에 실행되지 않도록 텔레옵을 먼저 끈다.
        self.control_states[robot_id]['teleop_enabled'] = 0
        self.publish_zero_velocity(robot_id)
        
        command = TaskCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.robot_id, command.command = robot_id, 'DOCK' if dock else 'UNDOCK'
        self.task_command_publisher.publish(command)    # 도킹, 언도킹 명령을 퍼블리시

        # 요청 시점에는 상태를 바꾸지 않고 브릿지의 실제 액션 성공 결과를 기다린다.
        self.get_logger().info(f'AMR {robot_id} 도킹 요청: {"DOCK" if dock else "UNDOCK"}')
        return True

    def cancel_task(self, robot_id: str):
        if robot_id not in self.control_states:
            return False
        command = TaskCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.robot_id, command.command = robot_id, 'CANCEL'
        self.task_command_publisher.publish(command)
        self.control_states[robot_id]['teleop_enabled'] = 0
        return True

    def publish_teleop(self, robot_id: str, linear: float, angular: float):
        publisher = self.teleop_publishers.get(robot_id)
        control = self.control_states.get(robot_id)
        if publisher is None or control is None:
            return False
        # API를 직접 호출하더라도 텔레옵 토글과 작업 상태 조건을 모두 만족해야 한다.
        if not control['teleop_enabled'] or not self.manual_control_allowed(robot_id):
            return False
        linear, angular = float(linear), float(angular)
        # 비정상 숫자와 HMI에서 사용하지 않는 과도한 속도 명령을 차단한다.
        if (not math.isfinite(linear) or not math.isfinite(angular)
                or abs(linear) > self.MAX_TELEOP_LINEAR
                or abs(angular) > self.MAX_TELEOP_ANGULAR):
            return False
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        publisher.publish(command)
        return True

    def set_teleop_mode(self, robot_id: str, active: bool):
        robot_id = str(robot_id)
        control = self.control_states.get(robot_id)
        if control is None:
            return False
        if active and not self.manual_control_allowed(robot_id):
            return False

        # Task Manager가 남아 있는 자동 Nav2 goal을 취소한 뒤 키보드 cmd_vel을 받게 한다.
        if active:
            command = TaskCommand()
            command.stamp = self.get_clock().now().to_msg()
            command.robot_id, command.command = robot_id, 'TELEOP_ENABLE'
            self.task_command_publisher.publish(command)
        else:
            self.publish_zero_velocity(robot_id)
        control['teleop_enabled'] = int(active)
        return True

    @staticmethod
    def _read_pgm(path: Path):  # 지도 데이터를 HML이 사용하는 형식으로 변환
        data = path.read_bytes()    # 지도 데이터
        offset = 0

        def token():
            nonlocal offset     # 변수를 공유

            while offset < len(data):   # 바이너리 데이터를 읽으면서 공백, 주석(#) 등을 건너뛰고 토큰을 추출
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

        if token() != 'P5':     # 바이너리 흑백 이미지만 지원   
            raise ValueError('P5 형식의 PGM 지도만 지원합니다.')
        
        width, height, maximum = int(token()), int(token()), int(token())   # 지도의 크기, 명암 데이터 

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

    def fetch_map(self):    # 지도의 정보를 제공
        if not self.map_yaml_path.exists():     # 지도 yaml 파일 확인
            raise FileNotFoundError(str(self.map_yaml_path))
        
        meta = {}
        for raw_line in self.map_yaml_path.read_text(encoding='utf-8').splitlines():    # yaml 파일을 읽어 key: value 형식으로 파싱
            line = raw_line.split('#', 1)[0].strip()
            if not line or ':' not in line:
                continue
            key, value = line.split(':', 1)
            meta[key.strip()] = value.strip()

        image_path = (self.map_yaml_path.parent / meta['image']).resolve()      # 지도 pgm 파일 경로
        origin = [float(value.strip()) for value in meta['origin'].strip('[]').split(',')]
        width, height, cells = self._read_pgm(image_path)

        return {        # 지도의 정보 
            'width': width,
            'height': height,
            'resolution': float(meta['resolution']),
            'origin': origin,
            'cells': cells,
        }

# =========================================================
# 3. FastAPI REST 서버 구성
# =========================================================
app = FastAPI(title="AMR HMI Backend Server API")   # FastAPI 인스턴스 생성

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

HMI_USERNAME = os.environ.get('HMI_USERNAME', 'rokey')  # 사용자 인증 아이디
HMI_PASSWORD = os.environ.get('HMI_PASSWORD', '1234')   # 사용자 인증 비밀번호

HMI_DB_TABLES = {       # 등록된 데이터베이스 테이블만 조회할 수 있도록 통제
    'destinations': 'destination_id',
    'tasks': 'created_at',
    'error_logs': 'error_id',
    'robot_status_logs': 'status_id',
}

@app.middleware("http")     # Bearer 인증 토큰을 요구하는 미들웨어 -> 로그인을 해야 다른 페이지를 볼 수 있도록 함
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


@app.post('/api/auth/login')    # 로그인 생성시 Bearer 인증 토큰을 부여
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
    if not ros_node:    # 노드 생성 확인
        raise HTTPException(status_code=503, detail='ROS 노드가 준비되지 않았습니다.')
    result = []

    with sqlite3.connect(ros_node.db_path) as conn: # 데이터베이스의 테이블 목록을 가져옴
        existing = {
            row[0] for row in conn.execute(
                "SELECT name FROM sqlite_master WHERE type='table'"
            ).fetchall()
        }
        for table in HMI_DB_TABLES: # 허용 테이블에 대해서만 데이터를 수를 셈
            if table in existing:
                count = conn.execute(f'SELECT COUNT(*) FROM {table}').fetchone()[0]
                result.append({'name': table, 'count': count})
    return result


@app.get('/api/database/table/{table_name}')
def get_database_table(table_name: str, limit: int = 100):
    """화이트리스트에 등록된 테이블의 최근 데이터 조회."""
    if not ros_node:    # 노드 생성 확인
        raise HTTPException(status_code=503, detail='ROS 노드가 준비되지 않았습니다.')
    
    order_column = HMI_DB_TABLES.get(table_name)    # 허용 테이블에 대해서만 조회 가능
    if order_column is None:
        raise HTTPException(status_code=404, detail='조회가 허용되지 않은 테이블입니다.')
    
    safe_limit = max(1, min(limit, 500))    # 데이터 수 500개 제한

    with sqlite3.connect(ros_node.db_path) as conn:
        conn.row_factory = sqlite3.Row
        cursor = conn.execute(
            f'SELECT * FROM {table_name} ORDER BY {order_column} DESC LIMIT ?',
            (safe_limit,)
        )   # 테이블의 데이터를 가져옴
        rows = cursor.fetchall()
        columns = [description[0] for description in cursor.description]
    return {'table': table_name, 'columns': columns, 'rows': [dict(row) for row in rows]}


@app.get("/api/robots")
def get_hmi_robots():
    """HMI 대시보드용 로봇 상태 목록."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    return ros_node.fetch_hmi_robots()


@app.get("/api/destinations")
def get_hmi_destinations():
    """HMI 지도에 표시할 DB 목적지 ID·이름·map 좌표 목록."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    with sqlite3.connect(ros_node.db_path) as conn:
        conn.row_factory = sqlite3.Row
        rows = conn.execute(
            """
            SELECT destination_id, destination_name, position_x, position_y
            FROM destinations ORDER BY destination_id
            """
        ).fetchall()
    return [dict(row) for row in rows]


@app.get("/api/map")
def get_map():
    """Nav2 YAML/PGM 지도를 HMI 렌더러 형식으로 반환."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    try:
        return ros_node.fetch_map()
    except (OSError, KeyError, ValueError) as error:
        raise HTTPException(status_code=500, detail=f"지도 로드 실패: {error}") from error


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
    if robot_id not in ros_node.control_states:
        raise HTTPException(status_code=404, detail=f"지원하지 않는 AMR ID: {robot_id}")
    # 실제 도킹 상태를 아직 못 받았거나 CANCELED/ERROR가 아니면 충돌 응답으로 거부한다.
    if not ros_node.set_dock(robot_id, data.dock):
        raise HTTPException(
            status_code=409,
            detail="실제 도킹 상태 수신 및 CANCELED/ERROR 상태를 확인해 주세요.",
        )
    return {"accepted": True, "robot_id": robot_id, "docked": data.dock}


@app.post("/api/robot/{robot_id}/cancel")
def cancel_robot_task(robot_id: str):
    if not ros_node or not ros_node.cancel_task(robot_id):
        raise HTTPException(status_code=404, detail=f"지원하지 않는 AMR ID: {robot_id}")
    return {"accepted": True, "robot_id": robot_id}


@app.post("/api/robot/{robot_id}/teleop")
def teleop_robot(robot_id: str, data: TeleopRequest):
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    if robot_id not in ros_node.control_states:
        raise HTTPException(status_code=404, detail=f"지원하지 않는 AMR ID: {robot_id}")
    # 텔레옵 미활성화 또는 허용 상태가 아닌 요청은 cmd_vel로 전달하지 않는다.
    if not ros_node.publish_teleop(robot_id, data.linear, data.angular):
        raise HTTPException(status_code=409, detail="텔레옵 활성화와 CANCELED/ERROR 상태를 확인해 주세요.")
    return {"accepted": True, "robot_id": robot_id}


@app.post("/api/robot/{robot_id}/teleop/mode")
def set_robot_teleop_mode(robot_id: str, data: TeleopModeRequest):
    """키보드 텔레옵 활성화 토글. 비활성화할 때는 즉시 정지 속도를 발행한다."""
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    if robot_id not in ros_node.control_states:
        raise HTTPException(status_code=404, detail=f"지원하지 않는 AMR ID: {robot_id}")
    if not ros_node.set_teleop_mode(robot_id, data.active):
        raise HTTPException(status_code=409, detail="텔레옵은 CANCELED 또는 ERROR 상태에서만 활성화할 수 있습니다.")
    return {"accepted": True, "robot_id": robot_id, "active": data.active}


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
        # spin 중인 노드를 먼저 destroy하면 종료 중 executor 예외가 날 수 있다.
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        ros_node.destroy_node()


if __name__ == '__main__':
    main()
