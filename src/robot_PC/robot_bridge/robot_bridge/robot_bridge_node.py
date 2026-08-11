#!/usr/bin/env python3
from copy import deepcopy
import math
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Dock, Undock
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from robot_status.msg import NavigationResult, RobotError, RobotStatus, TaskCommand, TaskState
from robot_bridge.pose_utils import build_robot_status, is_followable_pose, quaternion_to_yaw


class RobotBridgeNode(Node):
    """설정된 로봇 한 대의 AMCL/Nav2/Create3를 중앙 시스템에 연결한다."""

    def __init__(self, robot_id: str = ''):
        super().__init__('robot_bridge_node')
        self.declare_parameter('robot_id', robot_id)
        self.robot_id = str(self.get_parameter('robot_id').value).strip()
        if self.robot_id not in ('robot5', 'robot11'):
            raise ValueError('robot_id는 robot5 또는 robot11이어야 합니다.')
        topic_prefix = f'/{self.robot_id}'
        # 추종 인식 좌표가 고주파로 들어와도 Nav2 goal은 기본 1Hz 이하로 제한한다.
        self.declare_parameter('follow_goal_update_hz', 1.0)
        self.declare_parameter('follow_goal_min_distance', 0.2)
        # 목표 위치가 같더라도 사람 방향이 이 각도(rad) 이상 바뀌면 회전 goal을 갱신한다.
        self.declare_parameter('follow_goal_min_yaw', 0.15)
        # 사람의 실제 위치를 goal로 쓰면 충돌할 수 있으므로 사람 앞에서 유지할 거리이다.
        self.declare_parameter('follow_distance', 0.9)
        # 카메라 프레임 몇 장이 잠깐 누락된 경우에는 바로 회전하지 않고 기다린다.
        self.declare_parameter('person_lost_grace_sec', 2.0)
        # 이 시간 동안 재검출하지 못하면 중앙 시스템에 WORKER_LOST를 보고한다.
        self.declare_parameter('person_search_timeout_sec', 60.0)
        # 한 번의 Nav2 Spin 요청으로 회전할 각도이다. 기본값은 한 바퀴이다.
        self.declare_parameter('search_spin_yaw', 2.0 * math.pi)
        follow_goal_update_hz = float(self.get_parameter('follow_goal_update_hz').value)
        self.follow_goal_min_distance = float(self.get_parameter('follow_goal_min_distance').value)
        self.follow_goal_min_yaw = float(self.get_parameter('follow_goal_min_yaw').value)
        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.person_lost_grace_sec = float(self.get_parameter('person_lost_grace_sec').value)
        self.person_search_timeout_sec = float(self.get_parameter('person_search_timeout_sec').value)
        self.search_spin_yaw = abs(float(self.get_parameter('search_spin_yaw').value))
        if (follow_goal_update_hz <= 0.0 or self.follow_goal_min_distance < 0.0
                or self.follow_goal_min_yaw < 0.0
                or self.follow_distance <= 0.0 or self.person_lost_grace_sec <= 0.0
                or self.person_search_timeout_sec <= self.person_lost_grace_sec
                or self.search_spin_yaw <= 0.0):
            raise ValueError('추종 거리·주기와 재탐색 시간/각도 파라미터가 올바르지 않습니다.')
        self.follow_goal_min_interval_ns = int(1e9 / follow_goal_update_hz)
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.latest_x: Optional[float] = None
        self.latest_y: Optional[float] = None
        self.latest_yaw: Optional[float] = None
        self.latest_battery_percent: Optional[float] = None
        self.current_task_state = ''
        self.current_task_id = ''
        self.worker_tracking_enabled = False
        self.worker_detected_task_id = ''
        self.worker_lost_task_id = ''
        # 추종 노드는 worker_detected Bool 대신 대상 확정 후 target_person_pose를 발행한다.
        # ASSIGNED에서 처음 받은 유효 pose를 FOLLOWING 전환 직후 사용할 수 있게 보관한다.
        self.pending_person_pose: Optional[PoseStamped] = None
        self.nav_goal_handle = None
        self.nav_goal_pending = False
        self.nav_generation = 0
        # 교체 중인 과거 goal까지 포함해 모든 FOLLOWING NavigateToPose 종료를 추적한다.
        # TRANSPORTING handoff는 이 두 컬렉션이 모두 비워진 뒤에만 완료로 보고한다.
        self.follow_goal_handles = {}
        self.follow_pending_generations = set()
        self.follow_stop_task_id = ''
        self.last_follow_goal_time_ns: Optional[int] = None
        self.last_follow_goal: Optional[tuple] = None
        # 마지막으로 정상 사람 좌표를 받은 시각이다. FOLLOWING 진입 시부터 유실 시간을 잰다.
        self.last_person_pose_time_ns: Optional[int] = None
        self.search_started_time_ns: Optional[int] = None
        # 마지막 사람 방향을 기준으로 왼쪽(+1) 또는 오른쪽(-1) 탐색 방향을 기억한다.
        self.last_person_turn_direction = 1.0
        self.spin_goal_handle = None
        self.spin_goal_pending = False
        self.spin_generation = 0
        self.spin_goal_handles = {}
        self.spin_pending_generations = set()

        # 로봇별 namespace의 실제 위치·배터리·제어 요청만 구독한다.
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, f'{topic_prefix}/amcl_pose', self.amcl_pose_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, f'{topic_prefix}/battery_state', self.battery_callback, qos_profile_sensor_data)
        self.task_state_sub = self.create_subscription(
            TaskState, '/task/state', self.task_state_callback, 10)
        self.pause_sub = self.create_subscription(
            Bool, f'{topic_prefix}/pause/request', self.pause_callback, 10)
        self.dock_sub = self.create_subscription(
            Bool, f'{topic_prefix}/dock/request', self.dock_callback, 10)
        self.target_person_pose_sub = self.create_subscription(
            PoseStamped, f'{topic_prefix}/target_person_pose', self.target_person_pose_callback, 10)
        # 로봇의 별도 인식 노드는 작업자를 찾으면 이 로컬 토픽에 True를 발행한다.
        self.worker_detected_sub = self.create_subscription(
            Bool, f'{topic_prefix}/worker_detected', self.worker_detected_callback, 10)
        # 외부 추종 노드가 유실을 직접 판정하는 기존 구성도 계속 호환한다.
        self.worker_lost_sub = self.create_subscription(
            Bool, f'{topic_prefix}/worker_tracking/lost', self.worker_lost_callback, 10)

        # 두 브릿지는 공통 /robot_status에 발행하고 메시지 robot_id로 구분한다.
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', status_qos)
        # 실제 액션 결과와 실패 오류를 중앙 Task Manager/DB에 전달한다.
        self.navigation_result_pub = self.create_publisher(NavigationResult, '/navigation/result', 10)
        self.error_pub = self.create_publisher(RobotError, '/robot_error', 10)
        self.task_command_pub = self.create_publisher(TaskCommand, '/task/command', 10)
        self.status_timer = self.create_timer(1.0, self.publish_robot_status)
        # 좌표 토픽이 완전히 끊겨도 유실을 감지할 수 있도록 별도 타이머로 감시한다.
        self.tracking_watchdog_timer = self.create_timer(0.2, self.tracking_watchdog_callback)

        # 이 PC에서 설정된 로봇의 Nav2/Create3 액션 서버에만 연결한다.
        self.nav_client = ActionClient(self, NavigateToPose, f'{topic_prefix}/navigate_to_pose')
        # 직접 cmd_vel을 발행하지 않고 Nav2 Behavior Server의 충돌 검사 가능한 Spin을 사용한다.
        self.spin_client = ActionClient(self, Spin, f'{topic_prefix}/spin')
        self.dock_client = ActionClient(self, Dock, f'{topic_prefix}/dock')
        self.undock_client = ActionClient(self, Undock, f'{topic_prefix}/undock')
        self.get_logger().info(f'{self.robot_id} 브릿지 노드 시작')

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """AMCL pose에서 map 기준 위치와 yaw를 저장한다."""
        pose = msg.pose.pose
        self.latest_x, self.latest_y = pose.position.x, pose.position.y
        q = pose.orientation
        # 공통 변환 유틸을 사용해 로봇별 계산 차이를 방지한다.
        self.latest_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def battery_callback(self, msg: BatteryState) -> None:
        """BatteryState의 0~1 비율을 DB 규격인 퍼센트로 변환한다."""
        self.latest_battery_percent = msg.percentage * 100.0

    def build_status_message(self) -> Optional[RobotStatus]:
        """필수 센서값이 준비된 경우에만 이 로봇의 상태 메시지를 만든다."""
        if self.latest_x is None or self.latest_battery_percent is None:
            return None
        # 상태 메시지 구성 역시 별도 유틸로 통일한다.
        return build_robot_status(
            self.robot_id, self.latest_battery_percent,
            self.latest_x, self.latest_y, self.latest_yaw)

    def publish_robot_status(self) -> None:
        """1초마다 robot_id가 포함된 상태를 공통 토픽으로 발행한다."""
        msg = self.build_status_message()
        if msg is not None:
            self.status_pub.publish(msg)

    def pause_callback(self, msg: Bool) -> None:
        """정지 요청 시 진행 중이거나 수락 대기 중인 추종 goal을 무효화한다."""
        if not msg.data:
            return
        self.cancel_follow_goal()
        self.cancel_search_spin()

    def dock_callback(self, msg: Bool) -> None:
        """True는 Dock, False는 Undock 액션으로 변환한다."""
        client, goal_type = ((self.dock_client, Dock.Goal) if msg.data
                             else (self.undock_client, Undock.Goal))
        if not client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn(f'{self.robot_id} 도킹 액션 서버 대기 중')
            return
        future = client.send_goal_async(goal_type())
        future.add_done_callback(
            lambda result: self.dock_response_callback(result, bool(msg.data)))

    def dock_response_callback(self, future, expected_docked: bool) -> None:
        """도킹 액션의 수락 여부와 최종 결과를 로그로 확인한다."""
        goal_handle = future.result()
        goal_type = 'DOCK' if expected_docked else 'UNDOCK'
        if not goal_handle.accepted:
            self.get_logger().warn(f'{self.robot_id} 도킹/언도킹 goal 거부됨')
            self.publish_action_result(goal_type, False, f'{goal_type}_GOAL_REJECTED')
            return
        goal_handle.get_result_async().add_done_callback(
            lambda result: self.dock_result_callback(result, expected_docked))

    def dock_result_callback(self, future, expected_docked: bool) -> None:
        """Create3의 실제 is_docked 값과 액션 상태가 모두 맞을 때만 성공으로 보고한다."""
        response = future.result()
        goal_type = 'DOCK' if expected_docked else 'UNDOCK'
        success = (response.status == GoalStatus.STATUS_SUCCEEDED
                   and bool(response.result.is_docked) == expected_docked)
        error_code = '' if success else f'{goal_type}_FAILED_STATUS_{response.status}'
        self.publish_action_result(goal_type, success, error_code)

    def task_state_callback(self, msg: TaskState) -> None:
        """공통 TaskState 중 설정된 robot_id의 상태만 저장한다."""
        if msg.robot_id == self.robot_id:
            previous_task_id = self.current_task_id
            previous_state = self.current_task_state
            self.current_task_state = msg.state
            self.current_task_id = msg.task_id
            # 새 Task의 첫 추종 goal은 이전 작업의 시간·거리 제한과 무관하게 즉시 허용한다.
            if msg.task_id != previous_task_id:
                self.last_follow_goal_time_ns = None
                self.last_follow_goal = None
                self.pending_person_pose = None
            # 최초 작업자 위치에 도착한 뒤부터 FOLLOWING 종료 전까지 로컬 인식/추적을 켠다.
            should_enable = (
                (msg.state == 'ASSIGNED' and msg.goal_type == 'TO_WORKER' and msg.goal_completed)
                or msg.state == 'FOLLOWING')
            self.set_worker_tracking_enabled(should_enable)
            if msg.state == 'FOLLOWING' and previous_state != 'FOLLOWING':
                # FOLLOWING 진입 직후 아직 첫 좌표가 없어도 grace 시간 이후 재탐색을 시작한다.
                self.last_person_pose_time_ns = self.get_clock().now().nanoseconds
                self.search_started_time_ns = None
                # 추종 노드의 첫 pose로 WORKER_DETECTED를 보낸 뒤 중앙 상태 응답을 기다렸으므로,
                # 그 사이 사람이 정지해 새 pose가 늦게 와도 저장한 좌표로 즉시 추종을 시작한다.
                pending_pose, self.pending_person_pose = self.pending_person_pose, None
                if pending_pose is not None:
                    self.process_following_person_pose(pending_pose)
            elif previous_state == 'FOLLOWING' and msg.state != 'FOLLOWING':
                # 배송 모드 등으로 넘어갈 때 추종 Navigate/Spin이 남아 제어권을 잡지 않게 정리한다.
                if msg.state == 'TRANSPORTING':
                    self.cancel_search_spin()
                    self.begin_follow_stop_handoff(msg.task_id)
                else:
                    self.cancel_follow_goal()
                    self.cancel_search_spin()
                self.last_person_pose_time_ns = None
                self.search_started_time_ns = None
            if msg.task_id != self.worker_detected_task_id:
                self.worker_detected_task_id = ''
            if msg.task_id != self.worker_lost_task_id:
                self.worker_lost_task_id = ''

    def set_worker_tracking_enabled(self, enabled: bool) -> None:
        """현재 작업 단계에서 로컬 추종 좌표를 받아들일지 기록한다."""
        if enabled == self.worker_tracking_enabled:
            return
        self.worker_tracking_enabled = enabled

    def worker_detected_callback(self, msg: Bool) -> None:
        """도착 대기 중 유효한 작업자 감지를 중앙 상태 전환 명령으로 한 번만 전달한다."""
        if (not msg.data or self.current_task_state != 'ASSIGNED'
                or not self.worker_tracking_enabled or not self.current_task_id
                or self.worker_detected_task_id == self.current_task_id):
            return
        self.publish_worker_detected_command('로컬 작업자 인식 성공')

    def publish_worker_detected_command(self, detail: str) -> None:
        """현재 Task의 작업자 인식 성공을 중앙에 정확히 한 번 전달한다."""
        if (not self.current_task_id
                or self.worker_detected_task_id == self.current_task_id):
            return
        command = TaskCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.command, command.robot_id = 'WORKER_DETECTED', self.robot_id
        command.task_id = self.current_task_id
        command.detail = f'{self.robot_id} {detail}'
        self.task_command_pub.publish(command)
        self.worker_detected_task_id = self.current_task_id

    def worker_lost_callback(self, msg: Bool) -> None:
        """외부 유실 신호도 현재 FOLLOWING Task의 WORKER_LOST 오류로 한 번만 전달한다."""
        if (not msg.data or self.current_task_state != 'FOLLOWING'
                or not self.current_task_id
                or self.worker_lost_task_id == self.current_task_id):
            return
        # 브릿지 watchdog과 외부 인식 노드가 동시에 판단해도 오류는 한 번만 발행한다.
        self.publish_worker_lost_once()

    def target_person_pose_callback(self, msg: PoseStamped) -> None:
        """추종 노드의 대상 pose를 작업자 감지 신호 또는 Nav2 추종 goal로 변환한다.

        병합한 ``reid_tracking_node``는 호출 좌표 주변의 대상 track을 확정하면
        ``/robotX/target_person_pose``를 발행하지만 별도의 ``worker_detected`` Bool은
        발행하지 않는다. 브릿지가 ASSIGNED 상태에서 첫 유효 pose를 감지 성공으로 변환해
        기존 중앙 Task Manager 인터페이스를 유지한다. FOLLOWING 이후에는 같은 pose를
        안전거리 Nav2 goal로 사용한다.
        """
        if self.current_task_state not in ('ASSIGNED', 'FOLLOWING') or msg.header.frame_id != 'map':
            return
        p, q = msg.pose.position, msg.pose.orientation
        # 원점 이동 사고를 막는 공통 pose 검증 유틸을 사용한다.
        if not is_followable_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w):
            self.get_logger().warn(f'{self.robot_id} 무효한 target_person_pose 무시')
            return

        if self.current_task_state == 'ASSIGNED':
            # 최초 접근이 실제 완료돼 브릿지가 추종 인식을 활성화한 뒤에만 감지 성공으로 본다.
            # 이렇게 해야 로봇이 이동 중 미리 보인 사람 때문에 FOLLOWING으로 조기 전환되지 않는다.
            if not self.worker_tracking_enabled or not self.current_task_id:
                return
            self.pending_person_pose = deepcopy(msg)
            self.publish_worker_detected_command('추종 대상 pose 확정')
            return

        self.process_following_person_pose(msg)

    def process_following_person_pose(self, msg: PoseStamped) -> None:
        """검증된 FOLLOWING pose의 유실 타이머와 Nav2 안전거리 goal을 갱신한다."""

        # 정상 좌표를 받는 순간 유실 타이머를 갱신하고 진행 중인 회전 탐색을 중단한다.
        self.last_person_pose_time_ns = self.get_clock().now().nanoseconds
        self.search_started_time_ns = None
        self.cancel_search_spin()

        goal = self.build_safe_follow_goal(msg)
        if goal is not None and self.should_send_follow_goal(goal):
            self.send_follow_goal(goal)

    def build_safe_follow_goal(self, person_pose: PoseStamped) -> Optional[PoseStamped]:
        """사람과 ``follow_distance``를 유지하면서 사람을 바라보는 map goal을 만든다.

        사람 위치 자체를 Nav2 goal로 전달하면 로봇 footprint가 사람과 겹칠 수 있다.
        현재 AMCL 위치에서 사람으로 향하는 단위 벡터를 구한 뒤, 사람 반대쪽으로
        설정 거리만큼 떨어진 점을 목표로 사용한다. 사람이 이미 안전거리 안에 있으면
        위치는 유지하고 방향만 사람 쪽으로 맞춰 불필요한 접근을 막는다.
        """
        if self.latest_x is None or self.latest_y is None:
            self.get_logger().warn(f'{self.robot_id} AMCL 위치가 없어 추종 goal을 만들 수 없습니다.')
            return None

        person_x = float(person_pose.pose.position.x)
        person_y = float(person_pose.pose.position.y)
        dx, dy = person_x - self.latest_x, person_y - self.latest_y
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return None

        # 마지막 관측 방향은 사람이 사라졌을 때 Spin 방향을 선택하는 데 사용한다.
        person_yaw = math.atan2(dy, dx)
        if self.latest_yaw is not None:
            yaw_error = math.atan2(
                math.sin(person_yaw - self.latest_yaw),
                math.cos(person_yaw - self.latest_yaw))
            if abs(yaw_error) > 1e-3:
                self.last_person_turn_direction = 1.0 if yaw_error > 0.0 else -1.0

        goal = PoseStamped()
        goal.header = deepcopy(person_pose.header)
        if distance <= self.follow_distance:
            goal.pose.position.x, goal.pose.position.y = self.latest_x, self.latest_y
        else:
            unit_x, unit_y = dx / distance, dy / distance
            goal.pose.position.x = person_x - self.follow_distance * unit_x
            goal.pose.position.y = person_y - self.follow_distance * unit_y
        goal.pose.orientation.z = math.sin(person_yaw / 2.0)
        goal.pose.orientation.w = math.cos(person_yaw / 2.0)
        return goal

    def should_send_follow_goal(self, pose: PoseStamped) -> bool:
        """최대 1Hz이며 이전 전송 위치에서 0.2m 이상 변한 추종 목표만 허용한다."""
        if self.nav_goal_pending:
            return False
        now_ns = self.get_clock().now().nanoseconds
        if (self.last_follow_goal_time_ns is not None
                and now_ns - self.last_follow_goal_time_ns < self.follow_goal_min_interval_ns):
            return False
        p = pose.pose.position
        if self.last_follow_goal is not None:
            last_x, last_y, last_yaw = self.last_follow_goal
            distance = math.hypot(p.x - last_x, p.y - last_y)
            q = pose.pose.orientation
            goal_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            yaw_change = abs(math.atan2(
                math.sin(goal_yaw - last_yaw), math.cos(goal_yaw - last_yaw)))
            # 사람과 이미 안전거리 안에 있으면 위치 변화가 거의 없으므로,
            # 방향 변화까지 함께 보아 제자리 회전 goal이 누락되지 않게 한다.
            if (distance < self.follow_goal_min_distance
                    and yaw_change < self.follow_goal_min_yaw):
                return False
        return True

    def send_follow_goal(self, pose: PoseStamped) -> None:
        """새 추종 좌표가 오면 이전 추종 goal을 취소하고 최신 goal로 교체한다."""
        if not self.nav_client.server_is_ready():
            self.get_logger().warn(f'{self.robot_id} navigate_to_pose 액션 서버 대기 중')
            return
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
        goal = NavigateToPose.Goal()
        goal.pose = deepcopy(pose)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.nav_generation += 1
        generation = self.nav_generation
        # goal 수락 응답 전 추가 요청이 쌓이지 않도록 pending 상태를 먼저 기록한다.
        self.nav_goal_pending = True
        self.follow_pending_generations.add(generation)
        future = self.nav_client.send_goal_async(goal)
        self.last_follow_goal_time_ns = self.get_clock().now().nanoseconds
        q = goal.pose.pose.orientation
        self.last_follow_goal = (
            goal.pose.pose.position.x, goal.pose.pose.position.y,
            quaternion_to_yaw(q.x, q.y, q.z, q.w))
        self.get_logger().info(
            f'{self.robot_id}: FOLLOWING Nav2 goal 전송 '
            f'x={goal.pose.pose.position.x:.3f}, y={goal.pose.pose.position.y:.3f}, '
            f'generation={generation}')
        future.add_done_callback(lambda result: self.follow_goal_response_callback(result, generation))

    def cancel_follow_goal(self) -> None:
        """현재 추종 NavigateToPose와 아직 도착하지 않은 응답을 모두 무효화한다."""
        self.nav_generation += 1
        self.nav_goal_pending = False
        for goal_handle in tuple(self.follow_goal_handles.values()):
            goal_handle.cancel_goal_async()
        self.nav_goal_handle = None

    def begin_follow_stop_handoff(self, task_id: str) -> None:
        """배송 goal과 겹치지 않도록 모든 추종 goal의 최종 종료를 기다린다."""
        self.follow_stop_task_id = task_id
        self.nav_generation += 1
        self.nav_goal_pending = False
        for goal_handle in tuple(self.follow_goal_handles.values()):
            goal_handle.cancel_goal_async()
        self.nav_goal_handle = None
        self.publish_follow_stopped_if_ready()

    def publish_follow_stopped_if_ready(self) -> None:
        """수락 대기·실행 중인 추종 goal이 하나도 없을 때 중앙에 handoff를 확인한다."""
        if (not self.follow_stop_task_id or self.follow_pending_generations
                or self.follow_goal_handles or self.spin_pending_generations
                or self.spin_goal_handles):
            return
        task_id, self.follow_stop_task_id = self.follow_stop_task_id, ''
        result = NavigationResult()
        result.stamp = self.get_clock().now().to_msg()
        result.task_id, result.robot_id = task_id, self.robot_id
        result.goal_type, result.success = 'FOLLOWING_STOPPED', True
        self.navigation_result_pub.publish(result)
        self.get_logger().info(
            f'{self.robot_id}: FOLLOWING Nav2 goal 종료 확인, task_id={task_id}')

    def tracking_watchdog_callback(self) -> None:
        """사람 좌표 유실 시 Nav2 Spin 탐색을 시작하고 제한 시간 후 오류를 보고한다."""
        if self.current_task_state != 'FOLLOWING' or self.last_person_pose_time_ns is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        missing_sec = (now_ns - self.last_person_pose_time_ns) / 1e9
        if missing_sec < self.person_lost_grace_sec:
            return

        # 유실 직후 마지막 사람 위치로 계속 주행하지 않도록 추종 goal부터 취소한다.
        if self.search_started_time_ns is None:
            self.search_started_time_ns = now_ns
            self.cancel_follow_goal()
            self.get_logger().warn(f'{self.robot_id} 작업자 유실: Nav2 회전 재탐색 시작')

        search_sec = (now_ns - self.search_started_time_ns) / 1e9
        if search_sec >= self.person_search_timeout_sec:
            self.cancel_search_spin()
            self.publish_worker_lost_once()
            return
        if self.spin_goal_handle is None and not self.spin_goal_pending:
            self.send_search_spin_goal()

    def send_search_spin_goal(self) -> None:
        """마지막 관측 방향으로 Nav2 Spin goal을 보내 장애물 충돌 검사를 유지한다."""
        if not self.spin_client.server_is_ready():
            self.get_logger().warn(f'{self.robot_id} spin 액션 서버 대기 중')
            return
        goal = Spin.Goal()
        goal.target_yaw = self.last_person_turn_direction * self.search_spin_yaw
        # 전체 탐색 제한은 watchdog이 담당하므로 개별 Spin에는 남은 시간을 허용한다.
        remaining = max(1.0, self.person_search_timeout_sec)
        goal.time_allowance.sec = int(remaining)
        goal.time_allowance.nanosec = int((remaining - int(remaining)) * 1e9)
        self.spin_generation += 1
        generation = self.spin_generation
        self.spin_goal_pending = True
        self.spin_pending_generations.add(generation)
        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(lambda result: self.spin_goal_response_callback(result, generation))

    def spin_goal_response_callback(self, future, generation: int) -> None:
        """오래된 Spin 응답은 즉시 취소하고 현재 탐색 응답만 보관한다."""
        goal_handle = future.result()
        self.spin_pending_generations.discard(generation)
        if generation != self.spin_generation or self.current_task_state != 'FOLLOWING':
            if goal_handle.accepted:
                self.spin_goal_handles[generation] = goal_handle
                goal_handle.cancel_goal_async()
                goal_handle.get_result_async().add_done_callback(
                    lambda result: self.spin_result_callback(
                        result, goal_handle, generation))
            self.publish_follow_stopped_if_ready()
            return
        self.spin_goal_pending = False
        if not goal_handle.accepted:
            self.get_logger().warn(f'{self.robot_id} spin goal이 거부되었습니다.')
            self.publish_follow_stopped_if_ready()
            return
        self.spin_goal_handle = goal_handle
        self.spin_goal_handles[generation] = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda result: self.spin_result_callback(result, goal_handle, generation))

    def spin_result_callback(self, _future, goal_handle, generation: int) -> None:
        """한 바퀴 탐색이 끝나면 handle을 비워 watchdog이 필요 시 다음 탐색을 보낸다."""
        self.spin_goal_handles.pop(generation, None)
        if generation == self.spin_generation and self.spin_goal_handle is goal_handle:
            self.spin_goal_handle = None
        self.publish_follow_stopped_if_ready()

    def cancel_search_spin(self) -> None:
        """재검출·상태 전환·정지 시 진행 중인 Spin과 지연 응답을 무효화한다."""
        self.spin_generation += 1
        self.spin_goal_pending = False
        for goal_handle in tuple(self.spin_goal_handles.values()):
            goal_handle.cancel_goal_async()
        self.spin_goal_handle = None
        self.publish_follow_stopped_if_ready()

    def publish_worker_lost_once(self) -> None:
        """같은 Task에 WORKER_LOST 오류가 중복 발행되지 않도록 한 번만 보고한다."""
        if not self.current_task_id or self.worker_lost_task_id == self.current_task_id:
            return
        error = RobotError()
        error.robot_id, error.task_id = self.robot_id, self.current_task_id
        error.error_code = 'WORKER_LOST'
        self.error_pub.publish(error)
        self.worker_lost_task_id = self.current_task_id

    def follow_goal_response_callback(self, future, generation: int) -> None:
        """정지 또는 최신 goal보다 오래된 응답은 즉시 취소한다."""
        goal_handle = future.result()
        self.follow_pending_generations.discard(generation)
        if generation != self.nav_generation:
            if goal_handle.accepted:
                self.follow_goal_handles[generation] = goal_handle
                goal_handle.cancel_goal_async()
                goal_handle.get_result_async().add_done_callback(
                    lambda result: self.follow_result_callback(
                        result, goal_handle, generation))
            self.publish_follow_stopped_if_ready()
            return
        self.nav_goal_pending = False
        if not goal_handle.accepted:
            self.get_logger().warn(f'{self.robot_id} 추종 goal 거부됨')
            self.publish_action_result('FOLLOWING', False, 'FOLLOW_GOAL_REJECTED')
            self.publish_follow_stopped_if_ready()
            return
        self.nav_goal_handle = goal_handle
        self.follow_goal_handles[generation] = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda result: self.follow_result_callback(result, goal_handle, generation))

    def follow_result_callback(self, future, goal_handle, generation: int) -> None:
        """최신 추종 goal의 실제 실패만 오류로 보고하고 교체·정지 취소는 무시한다."""
        self.follow_goal_handles.pop(generation, None)
        if self.nav_goal_handle is goal_handle:
            self.nav_goal_handle = None
        if generation != self.nav_generation:
            self.publish_follow_stopped_if_ready()
            return
        response = future.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self.publish_action_result(
                'FOLLOWING', False, f'FOLLOW_FAILED_STATUS_{response.status}')
        self.publish_follow_stopped_if_ready()

    def publish_action_result(self, goal_type: str, success: bool, error_code: str) -> None:
        """액션 결과를 Task Manager로 보내고 실패 오류는 DB 기록 토픽에도 발행한다."""
        result = NavigationResult()
        result.stamp = self.get_clock().now().to_msg()
        result.task_id, result.robot_id = self.current_task_id, self.robot_id
        result.goal_type, result.success, result.error_code = goal_type, success, error_code
        self.navigation_result_pub.publish(result)
        if not success:
            error = RobotError()
            error.robot_id, error.task_id, error.error_code = self.robot_id, self.current_task_id, error_code
            self.error_pub.publish(error)


def main(args=None):
    """robot_id 파라미터로 선택된 로봇 브릿지 실행 진입점."""
    rclpy.init(args=args)
    node = RobotBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
