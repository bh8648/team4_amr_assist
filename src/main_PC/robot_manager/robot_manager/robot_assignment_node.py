#!/usr/bin/env python3

import copy
import math
import os
from collections import deque
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from robot_status.msg import (
    RobotAssignment,
    RobotStatus,
    RobotError,
    TaskState,
)

class RobotAssignmentNode(Node):

    VALID_ROBOTS = ('robot5', 'robot11')

    def __init__(self):
        super().__init__('robot_assignment_node')

        self.declare_parameter('status_timeout_sec', 5.0)   # 로봇 상태의 유효 시간 (초)
        self.declare_parameter('minimum_battery', 20.0)     # 로봇에게 작업을 배정하기 위한 최소 배터리량
        self.declare_parameter('battery_penalty_weight', 0.05)      # 배터리 소모 패널티
        self.declare_parameter('heading_penalty_weight', 0.5)       # 각도 오차 패널티
        self.declare_parameter('assignment_timeout_sec', 30.0)      # 배정을 대기할 시간
        self.declare_parameter('retry_interval_sec', 5.0)           # 배정 실패시 5초 대기 후 재배정 시도
        self.declare_parameter('person_stop_distance', 0.6)         # 사람 좌표 앞에서 멈출 거리(m)
        self.declare_parameter('duplicate_call_window_sec', 10.0)   # 중복 호출 판정 시간(초)
        self.declare_parameter('duplicate_call_distance', 0.5)      # 중복 호출 판정 거리(m)
        project_root = Path(get_package_prefix('robot_manager')).resolve().parents[1]
        default_map_path = project_root / 'maps/map2.yaml'
        self.declare_parameter('map_yaml_path', os.environ.get('AMR_MAP_YAML', str(default_map_path)))

        self.status_timeout_sec = float(
            self.get_parameter('status_timeout_sec').value
        )

        self.minimum_battery = float(
            self.get_parameter('minimum_battery').value
        )

        self.battery_penalty_weight = float(
            self.get_parameter('battery_penalty_weight').value
        )

        self.heading_penalty_weight = float(
            self.get_parameter('heading_penalty_weight').value
        )

        self.retry_interval_sec = float(
            self.get_parameter('retry_interval_sec').value
        )

        self.assignment_timeout_sec = float(
            self.get_parameter('assignment_timeout_sec').value
        )
        self.person_stop_distance = float(self.get_parameter('person_stop_distance').value)
        self.duplicate_call_window_sec = float(
            self.get_parameter('duplicate_call_window_sec').value)
        self.duplicate_call_distance = float(
            self.get_parameter('duplicate_call_distance').value)
        if (self.person_stop_distance < 0.0 or self.duplicate_call_window_sec < 0.0
                or self.duplicate_call_distance < 0.0):
            raise ValueError('거리와 시간 파라미터는 0 이상이어야 합니다.')

        configured_map_path = Path(os.path.expanduser(self.get_parameter('map_yaml_path').value))
        self.map_yaml_path = (configured_map_path if configured_map_path.is_absolute() else project_root / configured_map_path).resolve()
        if not self.map_yaml_path.is_file():
            self.map_yaml_path = default_map_path.resolve()
        # 지도 크기뿐 아니라 검은 벽으로 둘러싸인 내부 자유 셀을 함께 로드한다.
        self.load_map_geometry()
        self.safe_approach_failed = False

        # robot_id -> (RobotStatus, 마지막 수신 시각)
        self.robot_statuses: Dict[str, Tuple[RobotStatus, Time]] = {}
        self.managed_states: Dict[str, str] = {}

        # 배정을 기다리는 Goal
        self.pending_goal: Optional[PointStamped] = None

        # pending Goal이 생성된 시각
        self.pending_since: Optional[Time] = None

        # 짧은 시간 안에 같은 사람이 반복 호출하는 것을 막기 위한 마지막 유효 호출 정보
        self.last_call_position: Optional[Tuple[float, float]] = None
        self.last_call_received_at: Optional[Time] = None

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        call_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        self.status_subscription = self.create_subscription(
            RobotStatus, '/robot_status',
            lambda msg: self.status_callback(str(msg.robot_id).strip(), msg), qos)
        # 로봇1,2 상태정보 수신
        self.goal_subscription = self.create_subscription(              # goal 목표 수신용 Subscriber
            PointStamped, '/person/call_position', self.goal_callback, call_qos
        )
        self.task_state_subscription = self.create_subscription(TaskState, '/task/state', self.task_state_callback, 10)

        self.assignment_publisher = self.create_publisher(              # 로봇 배정 결과 발행용 Publisher
            RobotAssignment, '/robot_assignment', 10
        )

        self.error_publisher = self.create_publisher(              # 로봇 오류 정보 발행용 Publisher
                    RobotError, '/robot_error', 10
                )

        self.retry_timer = self.create_timer(        # 재배정 타이머
            self.retry_interval_sec,
            self.retry_pending_assignment,
        )

        self.get_logger().info('AMR 배정 노드 시작\n')

    @staticmethod
    def read_pgm(path: Path) -> Tuple[int, int, bytes]:
        """P5 PGM의 크기와 픽셀을 읽어 지도 셀 판정에 사용한다."""
        data, offset, tokens = path.read_bytes(), 0, []
        while len(tokens) < 4:
            while offset < len(data) and data[offset] <= 32:
                offset += 1
            if offset < len(data) and data[offset] == 35:
                while offset < len(data) and data[offset] != 10:
                    offset += 1
                continue
            start = offset
            while offset < len(data) and data[offset] > 32 and data[offset] != 35:
                offset += 1
            tokens.append(data[start:offset].decode('ascii'))
        if tokens[0] != 'P5' or int(tokens[3]) != 255:
            raise ValueError('255 단계의 P5 PGM 지도만 지원합니다.')
        # maxval 뒤의 구분자만 제거한다. 바이너리 픽셀 값 자체가 32 이하일 수 있어 반복 제거하면 안 된다.
        if data[offset:offset + 2] == b'\r\n':
            offset += 2
        elif offset < len(data) and data[offset] <= 32:
            offset += 1
        width, height = int(tokens[1]), int(tokens[2])
        pixels = data[offset:offset + width * height]
        if len(pixels) != width * height:
            raise ValueError('PGM 지도 픽셀 수가 올바르지 않습니다.')
        return width, height, pixels

    def load_map_geometry(self) -> None:
        """Nav2 지도에서 검은 벽으로 차단된 내부의 흰색 자유 셀만 추출한다."""
        if not self.map_yaml_path.is_file():
            raise FileNotFoundError(f'지도 YAML을 찾을 수 없습니다: {self.map_yaml_path}')
        meta = {}
        for raw_line in self.map_yaml_path.read_text(encoding='utf-8').splitlines():
            line = raw_line.split('#', 1)[0].strip()
            if line and ':' in line:
                key, value = line.split(':', 1)
                meta[key.strip()] = value.strip()
        self.map_resolution = float(meta['resolution'])
        origin = [float(value.strip()) for value in meta['origin'].strip('[]').split(',')]
        self.map_origin_x, self.map_origin_y = origin[0], origin[1]
        image_path = (self.map_yaml_path.parent / meta['image']).resolve()
        self.map_width, self.map_height, pixels = self.read_pgm(image_path)
        self.map_min_x, self.map_min_y = self.map_origin_x, self.map_origin_y
        self.map_max_x = self.map_origin_x + self.map_width * self.map_resolution
        self.map_max_y = self.map_origin_y + self.map_height * self.map_resolution

        # negate=0 지도에서 검은색(점유 확률 0.65 초과)은 벽으로 취급한다.
        occupied_pixel_max = math.floor(255 * (1.0 - float(meta.get('occupied_thresh', 0.65))))
        free_pixel_min = math.floor(255 * (1.0 - float(meta.get('free_thresh', 0.25)))) + 1
        exterior = set()
        queue = deque()
        for x in range(self.map_width):
            queue.extend(((x, 0), (x, self.map_height - 1)))
        for y in range(self.map_height):
            queue.extend(((0, y), (self.map_width - 1, y)))
        # 지도 가장자리에서 검은 벽을 통과하지 않고 닿는 셀은 벽 바깥으로 분류한다.
        while queue:
            x, y = queue.popleft()
            if (x, y) in exterior or pixels[y * self.map_width + x] <= occupied_pixel_max:
                continue
            exterior.add((x, y))
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                    queue.append((nx, ny))
        # 내부이면서 흰색 자유 공간인 셀만 사람 위치와 AMR 목표로 허용한다.
        self.interior_free_cells = {
            (x, self.map_height - 1 - row)
            for row in range(self.map_height)
            for x in range(self.map_width)
            if pixels[row * self.map_width + x] >= free_pixel_min and (x, row) not in exterior
        }
        self.get_logger().info(
            f'검은 벽 내부 자유 셀 로드 완료: {len(self.interior_free_cells)}개, '
            f'사람 정지 거리={self.person_stop_distance:.2f}m')

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """map 좌표를 PGM의 아래쪽 원점 기준 grid 좌표로 변환한다."""
        return (math.floor((x - self.map_origin_x) / self.map_resolution),
                math.floor((y - self.map_origin_y) / self.map_resolution))

    def is_inside_walls(self, x: float, y: float) -> bool:
        """좌표가 검은 벽으로 둘러싸인 내부 흰색 자유 셀인지 확인한다."""
        return self.world_to_grid(x, y) in self.interior_free_cells

    def validate_target(self, target_x: float, target_y: float) -> str:
        """목표 좌표가 정상적인 숫자이고 지도 범위 안인지 검사한다."""
        if not math.isfinite(target_x) or not math.isfinite(target_y):
            return 'INVALID_COORDINATE'
        if not (self.map_min_x <= target_x < self.map_max_x and self.map_min_y <= target_y < self.map_max_y):
            return 'OUT_OF_MAP'
        if not self.is_inside_walls(target_x, target_y):
            return 'OUTSIDE_BLACK_WALLS'
        return ''

    def calculate_approach_point(self, status: RobotStatus, person_x: float, person_y: float) -> Tuple[float, float]:
        """로봇에서 사람으로 향하는 선상에서 사람보다 0.6m 앞의 정지점을 계산한다."""
        dx, dy = person_x - float(status.x), person_y - float(status.y)
        distance = math.hypot(dx, dy)
        if distance > 1e-6:
            return (person_x - dx / distance * self.person_stop_distance,
                    person_y - dy / distance * self.person_stop_distance)
        # 로봇과 사람 좌표가 같으면 현재 로봇 뒤쪽 방향에 정지점을 만든다.
        return (person_x - math.cos(float(status.yaw)) * self.person_stop_distance,
                person_y - math.sin(float(status.yaw)) * self.person_stop_distance)

    def status_callback(self, expected_robot_id: str, msg: RobotStatus) -> None:    # 로봇 상태와 실제 수신 시각을 저장
        robot_id = str(msg.robot_id)
        if robot_id not in self.VALID_ROBOTS:
            self.get_logger().warning(f'Unknown robot_id on /robot_status: {robot_id or "EMPTY"}')
            return
        if robot_id != expected_robot_id:
            self.get_logger().warning(f'토픽과 robot_id 불일치: expected={expected_robot_id}, received={robot_id}')
            return
        self.robot_statuses[robot_id] = (msg, self.get_clock().now())

    def task_state_callback(self, msg: TaskState) -> None:
        self.managed_states[str(msg.robot_id)] = str(msg.state).upper()

    @staticmethod       # 인스턴스 참조 없이 메모리를 효율적으로 사용하도록 정적 메서드로 지정
    def angle_difference(a: float, b: float) -> float:      # 두 각도의 최소 차이를 0~π 범위로 계산한다.
        return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)

    def suitability_score(self, status: RobotStatus, target_x: float, target_y: float) -> float:    # AMR 배정 적합 점수 계산

        robot_x = float(status.x)
        robot_y = float(status.y)
        robot_yaw = float(status.yaw)
        battery = float(status.battery)

        dx = target_x - robot_x
        dy = target_y - robot_y

        distance = math.hypot(dx, dy)   # 목표지점과의 직선 거리

        if distance > 0.0:
            target_yaw = math.atan2(dy, dx)     # 목표지점과의 방향 계산
        else:
            target_yaw = robot_yaw

        heading_error = self.angle_difference(target_yaw, robot_yaw,)    # 목표지점과의 방향 오차 계산
        battery_penalty = (100.0 - battery) * self.battery_penalty_weight   # 배터리 소모량에 따른 패널티 계산
        heading_penalty = (heading_error * self.heading_penalty_weight)     # 방향 오차에 대한 패널티 계산

        return (distance + battery_penalty + heading_penalty)   # 총 적합 점수 계산 (낮을수록 적합)

    def is_status_fresh(self, received_at: Time, now: Time) -> bool:    # 로봇 상태 메시지의 마지막 수신 시각이 유효 시간 내인지 검사
        age_sec = (now - received_at).nanoseconds / 1e9     # 현재 시각과 마지막 수신 시각의 차이를 초 단위로 계산

        return age_sec <= self.status_timeout_sec

    def is_managed_robot_busy(self, robot_id: str) -> bool:
        # 최초 상태를 포함해 Task Manager 상태가 정확히 DOCKED인 로봇만 배정할 수 있다.
        return self.managed_states.get(robot_id, 'DOCKED') != 'DOCKED'

    def select_robot(self, target_x: float, target_y: float):   # AMR 배정 후보 로봇을 선택하고 적합 점수 계산

        now = self.get_clock().now()
        candidates = []
        self.safe_approach_failed = False

        for robot_id, (status, received_at) in self.robot_statuses.items():

            # 1. 상태 메시지 유효 시간 검사
            if not self.is_status_fresh(received_at, now):
                continue

            # 2. 현재 작업 여부 검사: DOCKED 외 모든 상태(ERROR 포함)는 배정에서 제외한다.
            if self.is_managed_robot_busy(robot_id):
                continue

            # 3. 최소 배터리 검사
            battery = float(status.battery)
            if battery < self.minimum_battery:
                continue

            # 4. 사람 좌표 0.6m 앞의 정지점도 검은 벽 내부 자유 공간이어야 한다.
            approach_x, approach_y = self.calculate_approach_point(status, target_x, target_y)
            if not self.is_inside_walls(approach_x, approach_y):
                self.safe_approach_failed = True
                continue

            # 5. 실제 정지점까지의 거리·방향을 기준으로 적합 점수를 계산한다.
            score = self.suitability_score(status, approach_x, approach_y)

            candidates.append((score, -battery, robot_id, status, approach_x, approach_y))

        if not candidates:  # 후보가 없는 경우
            return None

        # 정렬 우선순위
        # 1. 적합 점수가 낮은 로봇
        # 2. 배터리가 높은 로봇
        # 3. robot_id가 작은 로봇
        return min(candidates, key=lambda item: item[:3])

    def get_failure_reason(self) -> str:
        """현재 로봇 상태를 바탕으로 배정 실패 원인을 반환한다."""

        if not self.robot_statuses:     # 로봇 상태 메시지를 수신한 적이 없는 경우
            return 'NO_ROBOT_STATUS'

        now = self.get_clock().now()

        fresh_statuses = [
            status
            for status, received_at in self.robot_statuses.values()     # (로봇 상태, 마지막 수신 시간)
            if self.is_status_fresh(received_at, now)       # 유효한 시간 범위내에 통신이 들어온 것만
        ]

        if not fresh_statuses:      # 유효한 시간 범위 내에 통신이 들어온 로봇 상태가 없는 경우
            return 'AMR_OFFLINE'

        idle_statuses = [status for robot_id, (status, received_at) in self.robot_statuses.items() if self.is_status_fresh(received_at, now) and not self.is_managed_robot_busy(robot_id)]

        if not idle_statuses:       # 로봇이 모두 작업 중인 경우
            return 'ALL_AMR_BUSY'

        battery_available_statuses = [
            status
            for status in idle_statuses
            if float(status.battery) >= self.minimum_battery
        ]

        if not battery_available_statuses:  # 로봇이 모두 배터리 부족인 경우
            return 'LOW_BATTERY'

        if self.safe_approach_failed:
            return 'NO_SAFE_APPROACH_POINT'

        return 'NO_AVAILABLE_AMR'

    def publish_assignment_result(self, goal: PointStamped, assigned: bool, robot_id: str = '', reason: str = '') -> None:
        """배정 결과 메시지를 생성하고 발행한다."""

        result = RobotAssignment()
        result.assigned_at = self.get_clock().now().to_msg()
        result.target_x = float(goal.point.x)
        result.target_y = float(goal.point.y)
        result.assigned = bool(assigned)
        result.robot_id = str(robot_id)

        if not result.assigned:
            error_msg = RobotError()
            error_msg.robot_id = ""
            error_msg.task_id = str(getattr(goal, 'task_id', ''))
            error_msg.error_code = reason
            self.error_publisher.publish(error_msg)

        self.assignment_publisher.publish(result)   # AMR 배정 결과 메시지 발행

    def attempt_assignment(self, goal: PointStamped, publish_failure: bool) -> bool:
        """Goal에 대해 AMR 배정을 시도하고 성공 여부를 반환한다."""

        # 수신한 작업자 위치를 실수형 목표 좌표로 변환한 뒤 가장 적합한 AMR을 탐색한다.
        target_x = float(goal.point.x)
        target_y = float(goal.point.y)
        selected = self.select_robot(target_x, target_y)

        # 가용 AMR이 없으면 실패 원인을 확인하고, 최초 시도일 때만 실패 메시지를 발행한다.
        if selected is None:
            reason = self.get_failure_reason()

            if publish_failure:
                self.publish_assignment_result(goal, False, '', reason)

            self.get_logger().warn(
                f'AMR 배정 실패: 목표=({target_x:.2f}, {target_y:.2f}), 원인={reason}'
            )
            return False

        # 선택 결과에서 점수와 로봇 정보를 꺼내 배정 성공 결과를 발행한다.
        score, _, robot_id, status, approach_x, approach_y = selected
        self.managed_states[robot_id] = 'ASSIGNED'
        # 원본 사람 좌표 대신 사람으로부터 0.6m 떨어진 안전 정지점을 Task Manager에 전달한다.
        approach_goal = copy.deepcopy(goal)
        approach_goal.point.x, approach_goal.point.y = approach_x, approach_y
        self.publish_assignment_result(approach_goal, True, robot_id, 'SUCCESS')

        self.get_logger().info(
            f'AMR {robot_id} 배정 성공: 사람=({target_x:.2f}, {target_y:.2f}), '
            f'정지점=({approach_x:.2f}, {approach_y:.2f}), '
            f'배터리={float(status.battery):.1f}%, 점수={score:.2f}'
        )
        return True


    # ================================================================
    # Goal 수신
    # ================================================================

    def is_duplicate_call(self, target_x: float, target_y: float, received_at: Time) -> bool:
        """마지막 유효 호출과 시간·거리가 모두 가까우면 중복으로 판단한다."""
        if self.last_call_position is None or self.last_call_received_at is None:
            return False
        elapsed_sec = (received_at - self.last_call_received_at).nanoseconds / 1e9
        distance = math.hypot(
            target_x - self.last_call_position[0],
            target_y - self.last_call_position[1],
        )
        return (0.0 <= elapsed_sec <= self.duplicate_call_window_sec
                and distance <= self.duplicate_call_distance)

    def goal_callback(self, goal: PointStamped) -> None:
        """새 작업자 호출 위치를 받아 AMR 배정을 시도한다."""
        
        # 새로 수신한 Goal의 목표 좌표를 비교와 로그 출력에 사용할 수 있도록 변환한다.
        target_x = float(goal.point.x)
        target_y = float(goal.point.y)

        if goal.header.frame_id != 'map':
            self.publish_assignment_result(goal, False, '', 'INVALID_FRAME_ID')
            self.get_logger().warn(f'작업자 호출 좌표 프레임 오류: {goal.header.frame_id or "EMPTY"}')
            return

        # 잘못된 숫자 또는 지도 범위 밖의 Goal은 로봇 선택과 재시도를 수행하지 않고 즉시 거부한다.
        invalid_reason = self.validate_target(target_x, target_y)
        if invalid_reason:
            self.publish_assignment_result(goal, False, '', invalid_reason)
            self.get_logger().warn(f'유효하지 않은 목표 좌표: ({target_x}, {target_y}), 원인={invalid_reason}')
            return

        received_at = self.get_clock().now()
        if self.is_duplicate_call(target_x, target_y, received_at):
            self.get_logger().info(
                f'중복 작업자 호출 무시: ({target_x:.2f}, {target_y:.2f}), '
                f'{self.duplicate_call_window_sec:.1f}초/{self.duplicate_call_distance:.2f}m 이내'
            )
            return

        # 이미 재배정을 기다리는 Goal이 있으면 기존 요청을 유지하고 새 요청을 거부한다.
        if self.pending_goal is not None:
            self.publish_assignment_result(goal, False, '', 'PENDING_ASSIGNMENT_EXISTS')
            self.get_logger().warn(
                f'이미 배정을 기다리는 작업이 있어 새 요청을 거부합니다: '
                f'({target_x:.2f}, {target_y:.2f})'
            )
            return

        self.last_call_position = (target_x, target_y)
        self.last_call_received_at = received_at

        # 대기 중인 작업이 없으면 즉시 AMR 배정을 시도한다.
        success = self.attempt_assignment(goal, publish_failure=True)

        if success:
            return

        # 즉시 배정하지 못한 Goal은 복사하여 저장하고 재배정 대기 시간을 시작한다.
        self.pending_goal = copy.deepcopy(goal)
        self.pending_since = received_at

        self.get_logger().info(f'배정 요청을 WAITING 상태로 저장: 목표=({target_x:.2f}, {target_y:.2f})')


    # ================================================================
    # 재배정 처리
    # ================================================================

    def retry_pending_assignment(self) -> None:
        """대기 중인 Goal에 대해 주기적으로 재배정을 시도한다."""

        # 재배정을 기다리는 Goal이나 시작 시각이 없으면 처리하지 않는다.
        if self.pending_goal is None or self.pending_since is None:
            return

        # 최초 배정 실패 이후 현재까지의 대기 시간을 초 단위로 계산한다.
        now = self.get_clock().now()
        elapsed_sec = (now - self.pending_since).nanoseconds / 1e9

        # 제한 시간을 넘기면 최종 실패 결과를 발행하고 대기 정보를 제거한다.
        if elapsed_sec >= self.assignment_timeout_sec:
            self.publish_assignment_result(
                self.pending_goal,
                False,
                '',
                'ASSIGNMENT_TIMEOUT'
            )

            self.get_logger().error(
                f'AMR 배정 최종 실패: {elapsed_sec:.1f}초 동안 가용 AMR 없음, '
                f'목표=({float(self.pending_goal.point.x):.2f}, '
                f'{float(self.pending_goal.point.y):.2f})'
            )

            self.clear_pending_goal()
            return

        # 제한 시간 이내라면 실패 메시지를 반복 발행하지 않고 배정만 다시 시도한다.
        success = self.attempt_assignment(self.pending_goal, publish_failure=False)

        # 재배정에 성공하면 대기 정보를 제거하여 타이머 재실행을 막는다.
        if success:
            self.get_logger().info(
                f'대기 중이던 작업 재배정 성공: 대기 시간={elapsed_sec:.1f}초'
            )
            self.clear_pending_goal()
            return

        self.get_logger().info(f'AMR 재배정 대기 중: 경과={elapsed_sec:.1f}초 / {self.assignment_timeout_sec:.1f}초')


    def clear_pending_goal(self) -> None:
        """대기 중인 Goal 정보를 초기화한다."""

        # 보관 중인 Goal과 대기 시작 시각을 함께 초기화한다.
        self.pending_goal = None
        self.pending_since = None


def main(args=None):
    # ROS 2 통신을 초기화하고 AMR 배정 노드를 생성한다.
    rclpy.init(args=args)
    node = RobotAssignmentNode()

    # 종료 요청이 들어올 때까지 노드의 콜백과 타이머를 실행한다.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 요청으로 AMR 배정 노드를 종료합니다.')
    finally:
        # 노드 자원을 해제하고 ROS 2가 실행 중이면 안전하게 종료한다.
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
