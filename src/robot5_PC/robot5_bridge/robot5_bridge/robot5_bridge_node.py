#!/usr/bin/env python3
from copy import deepcopy
import math
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Dock, Undock
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from robot_status.msg import NavigationResult, RobotError, RobotStatus, TaskCommand, TaskState
from robot5_bridge.pose_utils import build_robot_status, is_followable_pose, quaternion_to_yaw

# 이 패키지는 robot5 PC에만 배포하므로 로봇 ID를 robot5로 고정한다.
ROBOT_ID = 'robot5'


class Robot5BridgeNode(Node):
    """robot5의 AMCL/Nav2/Create3 인터페이스를 중앙 시스템 토픽으로 연결한다."""

    def __init__(self):
        super().__init__('robot5_bridge_node')
        # 추종 인식 좌표가 고주파로 들어와도 Nav2 goal은 기본 1Hz 이하로 제한한다.
        self.declare_parameter('follow_goal_update_hz', 1.0)
        self.declare_parameter('follow_goal_min_distance', 0.2)
        follow_goal_update_hz = float(self.get_parameter('follow_goal_update_hz').value)
        self.follow_goal_min_distance = float(self.get_parameter('follow_goal_min_distance').value)
        if follow_goal_update_hz <= 0.0 or self.follow_goal_min_distance < 0.0:
            raise ValueError('추종 goal 주기는 0보다 크고 최소 이동 거리는 0 이상이어야 합니다.')
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
        self.nav_goal_handle = None
        self.nav_goal_pending = False
        self.nav_generation = 0
        self.last_follow_goal_time_ns: Optional[int] = None
        self.last_follow_goal: Optional[tuple] = None

        # robot5 namespace의 실제 위치·배터리·제어 요청만 구독한다.
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/robot5/amcl_pose', self.amcl_pose_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, '/robot5/battery_state', self.battery_callback, qos_profile_sensor_data)
        self.task_state_sub = self.create_subscription(
            TaskState, '/task/state', self.task_state_callback, 10)
        self.pause_sub = self.create_subscription(
            Bool, '/robot5/pause/request', self.pause_callback, 10)
        self.dock_sub = self.create_subscription(
            Bool, '/robot5/dock/request', self.dock_callback, 10)
        self.target_person_pose_sub = self.create_subscription(
            PoseStamped, '/robot5/target_person_pose', self.target_person_pose_callback, 10)
        # 로봇의 별도 인식 노드는 작업자를 찾으면 이 로컬 토픽에 True를 발행한다.
        self.worker_detected_sub = self.create_subscription(
            Bool, '/robot5/worker_detected', self.worker_detected_callback, 10)
        # 추종 인식 노드가 60초 재탐색 후에도 사람을 못 찾았을 때만 True를 보낸다.
        self.worker_lost_sub = self.create_subscription(
            Bool, '/robot5/worker_tracking/lost', self.worker_lost_callback, 10)

        # 두 브릿지는 공통 /robot_status에 발행하고 메시지 robot_id로 구분한다.
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', status_qos)
        # 실제 액션 결과와 실패 오류를 중앙 Task Manager/DB에 전달한다.
        self.navigation_result_pub = self.create_publisher(NavigationResult, '/navigation/result', 10)
        self.error_pub = self.create_publisher(RobotError, '/robot_error', 10)
        self.task_command_pub = self.create_publisher(TaskCommand, '/task/command', 10)
        # 늦게 시작한 인식 노드도 현재 활성화 상태를 받도록 enable 신호를 latched QoS로 유지한다.
        tracking_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.worker_tracking_enable_pub = self.create_publisher(
            Bool, '/robot5/worker_tracking/enable', tracking_qos)
        self.status_timer = self.create_timer(1.0, self.publish_robot_status)

        # robot5에서 실행 중인 Nav2/Create3 액션 서버에만 연결한다.
        self.nav_client = ActionClient(self, NavigateToPose, '/robot5/navigate_to_pose')
        self.dock_client = ActionClient(self, Dock, '/robot5/dock')
        self.undock_client = ActionClient(self, Undock, '/robot5/undock')
        self.get_logger().info('robot5 브릿지 노드 시작')

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """AMCL pose에서 map 기준 위치와 yaw를 저장한다."""
        pose = msg.pose.pose
        self.latest_x, self.latest_y = pose.position.x, pose.position.y
        q = pose.orientation
        # robot11 브릿지와 동일한 변환 유틸을 사용해 계산 차이를 방지한다.
        self.latest_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def battery_callback(self, msg: BatteryState) -> None:
        """BatteryState의 0~1 비율을 DB 규격인 퍼센트로 변환한다."""
        self.latest_battery_percent = msg.percentage * 100.0

    def build_status_message(self) -> Optional[RobotStatus]:
        """필수 센서값이 준비된 경우에만 robot5 상태 메시지를 만든다."""
        if self.latest_x is None or self.latest_battery_percent is None:
            return None
        # 상태 메시지 구성 역시 별도 유틸로 통일한다.
        return build_robot_status(
            ROBOT_ID, self.latest_battery_percent,
            self.latest_x, self.latest_y, self.latest_yaw)

    def publish_robot_status(self) -> None:
        """1초마다 robot5 ID가 포함된 상태를 공통 토픽으로 발행한다."""
        msg = self.build_status_message()
        if msg is not None:
            self.status_pub.publish(msg)

    def pause_callback(self, msg: Bool) -> None:
        """정지 요청 시 진행 중이거나 수락 대기 중인 추종 goal을 무효화한다."""
        if not msg.data:
            return
        self.nav_generation += 1
        # 수락 응답 대기 중인 goal도 generation으로 무효화하고 추가 전송을 허용한다.
        self.nav_goal_pending = False
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None

    def dock_callback(self, msg: Bool) -> None:
        """True는 Dock, False는 Undock 액션으로 변환한다."""
        client, goal_type = ((self.dock_client, Dock.Goal) if msg.data
                             else (self.undock_client, Undock.Goal))
        if not client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('robot5 도킹 액션 서버 대기 중')
            return
        future = client.send_goal_async(goal_type())
        future.add_done_callback(
            lambda result: self.dock_response_callback(result, bool(msg.data)))

    def dock_response_callback(self, future, expected_docked: bool) -> None:
        """도킹 액션의 수락 여부와 최종 결과를 로그로 확인한다."""
        goal_handle = future.result()
        goal_type = 'DOCK' if expected_docked else 'UNDOCK'
        if not goal_handle.accepted:
            self.get_logger().warn('robot5 도킹/언도킹 goal 거부됨')
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
        """공통 TaskState 중 robot5 상태만 저장한다."""
        if msg.robot_id == ROBOT_ID:
            previous_task_id = self.current_task_id
            self.current_task_state = msg.state
            self.current_task_id = msg.task_id
            # 새 Task의 첫 추종 goal은 이전 작업의 시간·거리 제한과 무관하게 즉시 허용한다.
            if msg.task_id != previous_task_id:
                self.last_follow_goal_time_ns = None
                self.last_follow_goal = None
            # 최초 작업자 위치에 도착한 뒤부터 FOLLOWING 종료 전까지 로컬 인식/추적을 켠다.
            should_enable = (
                (msg.state == 'ASSIGNED' and msg.goal_type == 'TO_WORKER' and msg.goal_completed)
                or msg.state == 'FOLLOWING')
            self.set_worker_tracking_enabled(should_enable)
            if msg.task_id != self.worker_detected_task_id:
                self.worker_detected_task_id = ''
            if msg.task_id != self.worker_lost_task_id:
                self.worker_lost_task_id = ''

    def set_worker_tracking_enabled(self, enabled: bool) -> None:
        """상태가 실제로 바뀔 때만 로컬 인식/추적 노드에 활성화 신호를 보낸다."""
        if enabled == self.worker_tracking_enabled:
            return
        self.worker_tracking_enabled = enabled
        self.worker_tracking_enable_pub.publish(Bool(data=enabled))

    def worker_detected_callback(self, msg: Bool) -> None:
        """도착 대기 중 유효한 작업자 감지를 중앙 상태 전환 명령으로 한 번만 전달한다."""
        if (not msg.data or self.current_task_state != 'ASSIGNED'
                or not self.worker_tracking_enabled or not self.current_task_id
                or self.worker_detected_task_id == self.current_task_id):
            return
        command = TaskCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.command, command.robot_id = 'WORKER_DETECTED', ROBOT_ID
        command.task_id = self.current_task_id
        command.detail = 'robot5 로컬 작업자 인식 성공'
        self.task_command_pub.publish(command)
        self.worker_detected_task_id = self.current_task_id

    def worker_lost_callback(self, msg: Bool) -> None:
        """60초 재탐색 최종 실패를 현재 FOLLOWING Task의 WORKER_LOST 오류로 한 번만 전달한다."""
        if (not msg.data or self.current_task_state != 'FOLLOWING'
                or not self.current_task_id
                or self.worker_lost_task_id == self.current_task_id):
            return
        # 유실 직후에는 마지막 goal을 유지하고, 인식 노드가 최종 실패를 보낸 이 시점에만 오류 처리한다.
        error = RobotError()
        error.robot_id, error.task_id = ROBOT_ID, self.current_task_id
        error.error_code = 'WORKER_LOST'
        self.error_pub.publish(error)
        self.worker_lost_task_id = self.current_task_id

    def target_person_pose_callback(self, msg: PoseStamped) -> None:
        """FOLLOWING 상태의 정상적인 map pose만 Nav2 추종 goal로 보낸다."""
        if self.current_task_state != 'FOLLOWING' or msg.header.frame_id != 'map':
            return
        p, q = msg.pose.position, msg.pose.orientation
        # 원점 이동 사고를 막는 공통 pose 검증 유틸을 사용한다.
        if not is_followable_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w):
            self.get_logger().warn('robot5 무효한 target_person_pose 무시')
            return
        if self.should_send_follow_goal(msg):
            self.send_follow_goal(msg)

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
            last_x, last_y = self.last_follow_goal
            distance = math.hypot(p.x - last_x, p.y - last_y)
            if distance < self.follow_goal_min_distance:
                return False
        return True

    def send_follow_goal(self, pose: PoseStamped) -> None:
        """새 추종 좌표가 오면 이전 추종 goal을 취소하고 최신 goal로 교체한다."""
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('robot5 navigate_to_pose 액션 서버 대기 중')
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
        future = self.nav_client.send_goal_async(goal)
        self.last_follow_goal_time_ns = self.get_clock().now().nanoseconds
        self.last_follow_goal = (goal.pose.pose.position.x, goal.pose.pose.position.y)
        future.add_done_callback(lambda result: self.follow_goal_response_callback(result, generation))

    def follow_goal_response_callback(self, future, generation: int) -> None:
        """정지 또는 최신 goal보다 오래된 응답은 즉시 취소한다."""
        goal_handle = future.result()
        if generation != self.nav_generation:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        self.nav_goal_pending = False
        if not goal_handle.accepted:
            self.get_logger().warn('robot5 추종 goal 거부됨')
            self.publish_action_result('FOLLOWING', False, 'FOLLOW_GOAL_REJECTED')
            return
        self.nav_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda result: self.follow_result_callback(result, goal_handle, generation))

    def follow_result_callback(self, future, goal_handle, generation: int) -> None:
        """최신 추종 goal의 실제 실패만 오류로 보고하고 교체·정지 취소는 무시한다."""
        if generation != self.nav_generation:
            return
        if self.nav_goal_handle is goal_handle:
            self.nav_goal_handle = None
        response = future.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self.publish_action_result(
                'FOLLOWING', False, f'FOLLOW_FAILED_STATUS_{response.status}')

    def publish_action_result(self, goal_type: str, success: bool, error_code: str) -> None:
        """액션 결과를 Task Manager로 보내고 실패 오류는 DB 기록 토픽에도 발행한다."""
        result = NavigationResult()
        result.stamp = self.get_clock().now().to_msg()
        result.task_id, result.robot_id = self.current_task_id, ROBOT_ID
        result.goal_type, result.success, result.error_code = goal_type, success, error_code
        self.navigation_result_pub.publish(result)
        if not success:
            error = RobotError()
            error.robot_id, error.task_id, error.error_code = ROBOT_ID, self.current_task_id, error_code
            self.error_pub.publish(error)


def main(args=None):
    """robot5 PC 전용 브릿지 실행 진입점."""
    rclpy.init(args=args)
    node = Robot5BridgeNode()
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
