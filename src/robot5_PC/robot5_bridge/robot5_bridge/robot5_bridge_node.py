#!/usr/bin/env python3
from copy import deepcopy
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Dock, Undock
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from robot_status.msg import RobotStatus, TaskState
from robot5_bridge.pose_utils import build_robot_status, is_followable_pose, quaternion_to_yaw

# 이 패키지는 robot5 PC에만 배포하므로 로봇 ID를 robot5로 고정한다.
ROBOT_ID = 'robot5'


class Robot5BridgeNode(Node):
    """robot5의 AMCL/Nav2/Create3 인터페이스를 중앙 시스템 토픽으로 연결한다."""

    def __init__(self):
        super().__init__('robot5_bridge_node')
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.latest_x: Optional[float] = None
        self.latest_y: Optional[float] = None
        self.latest_yaw: Optional[float] = None
        self.latest_battery_percent: Optional[float] = None
        self.current_task_state = ''
        self.nav_goal_handle = None
        self.nav_generation = 0

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

        # 두 브릿지는 공통 /robot_status에 발행하고 메시지 robot_id로 구분한다.
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', status_qos)
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
        future.add_done_callback(self.dock_response_callback)

    def dock_response_callback(self, future) -> None:
        """도킹 액션의 수락 여부와 최종 결과를 로그로 확인한다."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('robot5 도킹/언도킹 goal 거부됨')
            return
        goal_handle.get_result_async().add_done_callback(
            lambda result: self.get_logger().info(f'robot5 도킹 액션 결과: {result.result().result}'))

    def task_state_callback(self, msg: TaskState) -> None:
        """공통 TaskState 중 robot5 상태만 저장한다."""
        if msg.robot_id == ROBOT_ID:
            self.current_task_state = msg.state

    def target_person_pose_callback(self, msg: PoseStamped) -> None:
        """FOLLOWING 상태의 정상적인 map pose만 Nav2 추종 goal로 보낸다."""
        if self.current_task_state != 'FOLLOWING' or msg.header.frame_id != 'map':
            return
        p, q = msg.pose.position, msg.pose.orientation
        # 원점 이동 사고를 막는 공통 pose 검증 유틸을 사용한다.
        if not is_followable_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w):
            self.get_logger().warn('robot5 무효한 target_person_pose 무시')
            return
        self.send_follow_goal(msg)

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
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda result: self.follow_goal_response_callback(result, generation))

    def follow_goal_response_callback(self, future, generation: int) -> None:
        """정지 또는 최신 goal보다 오래된 응답은 즉시 취소한다."""
        goal_handle = future.result()
        if generation != self.nav_generation:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().warn('robot5 추종 goal 거부됨')
            return
        self.nav_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda _result: self.clear_follow_goal(goal_handle))

    def clear_follow_goal(self, goal_handle) -> None:
        """완료된 goal handle이 다음 추종 명령을 방해하지 않도록 제거한다."""
        if self.nav_goal_handle is goal_handle:
            self.nav_goal_handle = None


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
