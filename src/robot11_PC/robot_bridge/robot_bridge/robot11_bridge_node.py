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

from robot_bridge.pose_utils import build_robot_status, is_followable_pose, quaternion_to_yaw

ROBOT_ID = 'robot11'


class Robot11BridgeNode(Node):
    def __init__(self):
        super().__init__('robot11_bridge_node')

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
        self.nav_goal_handle = None
        self.nav_goal_pending = False

        self.current_task_state: str = ''
        self.current_task_id: str = ''
        self.worker_tracking_enabled = False
        self.worker_detected_task_id = ''
        self.worker_lost_task_id = ''
        self.nav_generation = 0
        self.last_follow_goal_time_ns: Optional[int] = None
        self.last_follow_goal: Optional[tuple] = None

        # --------------------------------subscription---------------------------------------------------------------
        self.amcl_sub = self.create_subscription(       # 로봇 위치 수신
            PoseWithCovarianceStamped, f'/{ROBOT_ID}/amcl_pose', self.amcl_pose_callback, 10)
        # Create3/TurtleBot4 센서 토픽은 BEST_EFFORT로 발행되는 경우가 많다.
        # BEST_EFFORT 구독자는 BEST_EFFORT/RELIABLE 발행자 모두와 호환된다.
        self.battery_sub = self.create_subscription(        # 로봇 배터리 상태 수신
            BatteryState, f'/{ROBOT_ID}/battery_state', self.battery_callback,
            qos_profile_sensor_data)

        self.task_state_sub = self.create_subscription(     # 로봇의 작업상태 수신
                    TaskState, '/task/state', self.task_state_callback, 10)

        self.pause_sub = self.create_subscription(      # 일시정지 요청 수신
                    Bool, f'/{ROBOT_ID}/pause/request', self.pause_callback, 10)
        
        self.dock_sub = self.create_subscription(       # 도킹 요청 수신
            Bool, f'/{ROBOT_ID}/dock/request', self.dock_callback, 10)

        # --------------------------------publisher---------------------------------------------------------------
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', status_qos)   # 로봇 상태 퍼블리시
        # 실제 액션 결과와 실패 오류를 중앙 Task Manager/DB에 전달한다.
        self.navigation_result_pub = self.create_publisher(NavigationResult, '/navigation/result', 10)
        self.error_pub = self.create_publisher(RobotError, '/robot_error', 10)
        self.status_timer = self.create_timer(1.0, self.publish_robot_status)   # 1초마다 로봇 상태 퍼블리시

        # --------------------------------Action_client---------------------------------------------------------------
        self.nav_client = ActionClient(self, NavigateToPose, f'/{ROBOT_ID}/navigate_to_pose')       # Goal 요청 액션 클라이언트
        self.dock_client = ActionClient(self, Dock, f'/{ROBOT_ID}/dock')        # 도킹 요청 액션 클라이언트
        self.undock_client = ActionClient(self, Undock, f'/{ROBOT_ID}/undock')  # 언도킹 요청 액션 클라이언트

        self.target_person_pose_sub = self.create_subscription(
            PoseStamped, f'/{ROBOT_ID}/target_person_pose', self.target_person_pose_callback, 10)
        # 로봇의 별도 인식 노드는 작업자를 찾으면 이 로컬 토픽에 True를 발행한다.
        self.worker_detected_sub = self.create_subscription(
            Bool, f'/{ROBOT_ID}/worker_detected', self.worker_detected_callback, 10)
        # 추종 인식 노드가 60초 재탐색 후에도 사람을 못 찾았을 때만 True를 보낸다.
        self.worker_lost_sub = self.create_subscription(
            Bool, f'/{ROBOT_ID}/worker_tracking/lost', self.worker_lost_callback, 10)
        self.task_command_pub = self.create_publisher(TaskCommand, '/task/command', 10)
        # 늦게 시작한 인식 노드도 현재 활성화 상태를 받도록 enable 신호를 latched QoS로 유지한다.
        tracking_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.worker_tracking_enable_pub = self.create_publisher(
            Bool, f'/{ROBOT_ID}/worker_tracking/enable', tracking_qos)
        
        self.get_logger().info(f'{ROBOT_ID} 브릿지 노드 시작')

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:   # 로봇의 현재 위치와 각도(라디안)를 받아옴
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.latest_x, self.latest_y = position.x, position.y
        self.latest_yaw = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)

    def battery_callback(self, msg: BatteryState) -> None:      # 배터리 상태 받아옴(% 단위)
        self.latest_battery_percent = msg.percentage * 100.0

    def build_status_message(self) -> Optional[RobotStatus]:    # 로봇의 상태를 메시지 형태로 정리
        if self.latest_x is None or self.latest_battery_percent is None:
            return None
        return build_robot_status(
            ROBOT_ID, self.latest_battery_percent, self.latest_x, self.latest_y, self.latest_yaw)

    def publish_robot_status(self) -> None:     # 로봇 상태 퍼블리시(아이디, 배터리, x, y, yaw)
        msg = self.build_status_message()
        if msg is not None:
            self.status_pub.publish(msg)

    def pause_callback(self, msg: Bool) -> None:    # 진행 중인 Action goal 중단 요청
        if not msg.data:
            return
        # 아직 응답이 오지 않은 in-flight goal도 무효화한다.
        # (generation을 올려두면 뒤늦게 수락된 goal이 응답 콜백에서 취소된다)
        self.nav_generation += 1
        # 수락 응답 대기 중인 goal도 generation으로 무효화하고 추가 전송을 허용한다.
        self.nav_goal_pending = False
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None

    def dock_callback(self, msg: Bool) -> None:     # 도킹/언도킹 수행
        if msg.data:
            self._send_dock_goal()
        else:
            self._send_undock_goal()

    def _send_dock_goal(self) -> None:      # 도킹 액션 보내기
        if not self.dock_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('dock 액션 서버 대기 중')
            return
        future = self.dock_client.send_goal_async(Dock.Goal())
        future.add_done_callback(self._dock_response_callback)

    def _send_undock_goal(self) -> None:        # 언도킹 액션 보내기
        if not self.undock_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('undock 액션 서버 대기 중')
            return
        future = self.undock_client.send_goal_async(Undock.Goal())
        future.add_done_callback(self._undock_response_callback)

    def _dock_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('dock goal 거부됨')
            self.publish_action_result('DOCK', False, 'DOCK_GOAL_REJECTED')
            return
        goal_handle.get_result_async().add_done_callback(
            lambda result: self._dock_result_callback(result, True))

    def _undock_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('undock goal 거부됨')
            self.publish_action_result('UNDOCK', False, 'UNDOCK_GOAL_REJECTED')
            return
        goal_handle.get_result_async().add_done_callback(
            lambda result: self._dock_result_callback(result, False))

    def _dock_result_callback(self, future, expected_docked: bool) -> None:
        """Create3의 실제 결과와 액션 상태를 함께 검사해 성공 여부를 중앙에 보고한다."""
        response = future.result()
        goal_type = 'DOCK' if expected_docked else 'UNDOCK'
        success = (response.status == GoalStatus.STATUS_SUCCEEDED
                   and bool(response.result.is_docked) == expected_docked)
        error_code = '' if success else f'{goal_type}_FAILED_STATUS_{response.status}'
        self.publish_action_result(goal_type, success, error_code)

    def task_state_callback(self, msg: TaskState) -> None:      # 현재 로봇의 작업상태 저장
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
        command.detail = 'robot11 로컬 작업자 인식 성공'
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

    def target_person_pose_callback(self, msg: PoseStamped) -> None:     # 추종 상태일 때만 goal 보내기
        if self.current_task_state != 'FOLLOWING':
            return
        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f"target_person_pose의 frame_id가 'map'이 아님 ({msg.header.frame_id}) — 무시")
            return
        position = msg.pose.position
        orientation = msg.pose.orientation
        if not is_followable_pose(position.x, position.y, position.z,
                                  orientation.x, orientation.y, orientation.z, orientation.w):
            self.get_logger().warn('무효한 target_person_pose 무시 (추적 대상 없음)')
            return
        if self.should_send_follow_goal(msg):
            self._send_follow_goal(msg)

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

    def _send_follow_goal(self, pose: PoseStamped) -> None:
        # 이 콜백은 카메라 프레임레이트(10~30Hz)로 불리므로 블로킹 대기를 쓰면 안 된다.
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('navigate_to_pose 액션 서버 대기 중')
            return
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
        goal = NavigateToPose.Goal()
        goal.pose = deepcopy(pose)  # 수신 메시지를 직접 변형하지 않는다
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.nav_generation += 1
        generation = self.nav_generation
        # goal 수락 응답 전 추가 요청이 쌓이지 않도록 pending 상태를 먼저 기록한다.
        self.nav_goal_pending = True
        future = self.nav_client.send_goal_async(goal)
        self.last_follow_goal_time_ns = self.get_clock().now().nanoseconds
        self.last_follow_goal = (goal.pose.pose.position.x, goal.pose.pose.position.y)
        future.add_done_callback(lambda result: self._follow_goal_response_callback(result, generation))

    def _follow_goal_response_callback(self, future, generation: int) -> None:
        goal_handle = future.result()
        if generation != self.nav_generation:
            # pause 또는 더 새로운 goal이 이 goal을 무효화했다. 수락된 상태로 두면
            # 로봇이 계속 주행하므로 반드시 취소한다.
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        self.nav_goal_pending = False
        if not goal_handle.accepted:
            self.get_logger().warn('follow goal 거부됨')
            self.publish_action_result('FOLLOWING', False, 'FOLLOW_GOAL_REJECTED')
            return
        self.nav_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda result: self._follow_result_callback(result, goal_handle, generation))

    def _follow_result_callback(self, future, goal_handle, generation: int) -> None:
        """최신 추종 goal의 실제 실패만 오류로 보고하고 교체·정지로 취소된 goal은 무시한다."""
        if generation != self.nav_generation:
            return
        if self.nav_goal_handle is goal_handle:
            self.nav_goal_handle = None
        response = future.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self.publish_action_result(
                'FOLLOWING', False, f'FOLLOW_FAILED_STATUS_{response.status}')

    def publish_action_result(self, goal_type: str, success: bool, error_code: str) -> None:
        """액션 결과를 Task Manager에 보내고 실패이면 동일 오류를 DB 기록 토픽에도 발행한다."""
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
    rclpy.init(args=args)
    node = Robot11BridgeNode()
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
