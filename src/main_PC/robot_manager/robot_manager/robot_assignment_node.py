#!/usr/bin/env python3

import copy
import math
import os
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
        super().__init__('amr_allocation_node')

        self.declare_parameter('status_timeout_sec', 5.0)   # 로봇 상태의 유효 시간 (초)
        self.declare_parameter('minimum_battery', 20.0)     # 로봇에게 작업을 배정하기 위한 최소 배터리량
        self.declare_parameter('battery_penalty_weight', 0.05)      # 배터리 소모 패널티
        self.declare_parameter('heading_penalty_weight', 0.5)       # 각도 오차 패널티
        self.declare_parameter('assignment_timeout_sec', 60.0)      # 배정을 대기할 시간
        self.declare_parameter('retry_interval_sec', 5.0)           # 배정 실패시 5초 대기 후 재배정 시도
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

        configured_map_path = Path(os.path.expanduser(self.get_parameter('map_yaml_path').value))
        self.map_yaml_path = (configured_map_path if configured_map_path.is_absolute() else project_root / configured_map_path).resolve()
        if not self.map_yaml_path.is_file():
            self.map_yaml_path = default_map_path.resolve()
        self.map_min_x, self.map_max_x, self.map_min_y, self.map_max_y = self.load_map_bounds()

        # robot_id -> (RobotStatus, 마지막 수신 시각)
        self.robot_statuses: Dict[str, Tuple[RobotStatus, Time]] = {}
        self.managed_states: Dict[str, str] = {}

        # 배정을 기다리는 Goal
        self.pending_goal: Optional[PointStamped] = None

        # pending Goal이 생성된 시각
        self.pending_since: Optional[Time] = None

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
    def read_pgm_size(path: Path) -> Tuple[int, int]:
        """PGM 헤더에서 지도의 가로·세로 셀 수를 읽는다."""
        data, offset, tokens = path.read_bytes(), 0, []
        while len(tokens) < 3:
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
        if tokens[0] not in ('P2', 'P5'):
            raise ValueError('지원하지 않는 PGM 형식입니다.')
        return int(tokens[1]), int(tokens[2])

    def load_map_bounds(self) -> Tuple[float, float, float, float]:
        """Nav2 YAML과 PGM 크기로 유효한 map 좌표 범위를 계산한다."""
        if not self.map_yaml_path.is_file():
            raise FileNotFoundError(f'지도 YAML을 찾을 수 없습니다: {self.map_yaml_path}')
        meta = {}
        for raw_line in self.map_yaml_path.read_text(encoding='utf-8').splitlines():
            line = raw_line.split('#', 1)[0].strip()
            if line and ':' in line:
                key, value = line.split(':', 1)
                meta[key.strip()] = value.strip()
        resolution = float(meta['resolution'])
        origin = [float(value.strip()) for value in meta['origin'].strip('[]').split(',')]
        image_path = (self.map_yaml_path.parent / meta['image']).resolve()
        width, height = self.read_pgm_size(image_path)
        bounds = origin[0], origin[0] + width * resolution, origin[1], origin[1] + height * resolution
        self.get_logger().info(f'지도 좌표 범위: {bounds[0]:.2f} ≤ x < {bounds[1]:.2f}, {bounds[2]:.2f} ≤ y < {bounds[3]:.2f}')
        return bounds

    def validate_target(self, target_x: float, target_y: float) -> str:
        """목표 좌표가 정상적인 숫자이고 지도 범위 안인지 검사한다."""
        if not math.isfinite(target_x) or not math.isfinite(target_y):
            return 'INVALID_COORDINATE'
        if not (self.map_min_x <= target_x < self.map_max_x and self.map_min_y <= target_y < self.map_max_y):
            return 'OUT_OF_MAP'
        return ''

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
        return self.managed_states.get(robot_id, 'DOCKED') != 'DOCKED'

    @staticmethod
    def has_robot_error(status: RobotStatus) -> bool:   # 로봇에 오류가 있는지 검사

        return bool(getattr(status, 'has_error', False))

    def select_robot(self, target_x: float, target_y: float):   # AMR 배정 후보 로봇을 선택하고 적합 점수 계산

        now = self.get_clock().now()
        candidates = []

        for robot_id, (status, received_at) in self.robot_statuses.items():

            # 1. 상태 메시지 유효 시간 검사
            if not self.is_status_fresh(received_at, now):
                continue

            # 2. 로봇 오류 검사
            if self.has_robot_error(status):
                continue

            # 3. 현재 작업 여부 검사
            if self.is_managed_robot_busy(robot_id):
                continue

            # 4. 최소 배터리 검사
            battery = float(status.battery)
            if battery < self.minimum_battery:
                continue

            # 5. 적합 점수 계산
            score = self.suitability_score(status, target_x, target_y)

            candidates.append((score, -battery, robot_id, status)) # 후보 리스트에 저장

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
        score, _, robot_id, status = selected
        self.managed_states[robot_id] = 'ASSIGNED'
        self.publish_assignment_result(goal, True, robot_id, 'SUCCESS')

        self.get_logger().info(
            f'AMR {robot_id} 배정 성공: 목표=({target_x:.2f}, {target_y:.2f}), '
            f'배터리={float(status.battery):.1f}%, 점수={score:.2f}'
        )
        return True


    # ================================================================
    # Goal 수신
    # ================================================================

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

        # 이미 재배정을 기다리는 Goal이 있으면 기존 요청을 유지하고 새 요청을 거부한다.
        if self.pending_goal is not None:
            self.publish_assignment_result(goal, False, '', 'PENDING_ASSIGNMENT_EXISTS')
            self.get_logger().warn(
                f'이미 배정을 기다리는 작업이 있어 새 요청을 거부합니다: '
                f'({target_x:.2f}, {target_y:.2f})'
            )
            return

        # 대기 중인 작업이 없으면 즉시 AMR 배정을 시도한다.
        success = self.attempt_assignment(goal, publish_failure=True)

        if success:
            return

        # 즉시 배정하지 못한 Goal은 복사하여 저장하고 재배정 대기 시간을 시작한다.
        self.pending_goal = copy.deepcopy(goal)
        self.pending_since = self.get_clock().now()

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
