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

from robot_bridge.pose_utils import build_robot_status, is_followable_pose, quaternion_to_yaw

ROBOT_ID = 'robot5'


class Robot5BridgeNode(Node):
    def __init__(self):
        super().__init__('robot5_bridge_node')

        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.latest_x: Optional[float] = None
        self.latest_y: Optional[float] = None
        self.latest_yaw: Optional[float] = None
        self.latest_battery_percent: Optional[float] = None

        self.current_task_state: str = ''
        self.nav_generation = 0

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, f'/{ROBOT_ID}/amcl_pose', self.amcl_pose_callback, 10)
        # Create3/TurtleBot4 센서 토픽은 BEST_EFFORT로 발행되는 경우가 많다.
        # BEST_EFFORT 구독자는 BEST_EFFORT/RELIABLE 발행자 모두와 호환된다.
        self.battery_sub = self.create_subscription(
            BatteryState, f'/{ROBOT_ID}/battery_state', self.battery_callback,
            qos_profile_sensor_data)

        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', status_qos)
        self.status_timer = self.create_timer(1.0, self.publish_robot_status)

        self.nav_goal_handle = None

        self.pause_sub = self.create_subscription(
            Bool, f'/{ROBOT_ID}/pause/request', self.pause_callback, 10)

        self.nav_client = ActionClient(self, NavigateToPose, f'/{ROBOT_ID}/navigate_to_pose')

        self.dock_sub = self.create_subscription(
            Bool, f'/{ROBOT_ID}/dock/request', self.dock_callback, 10)

        self.dock_client = ActionClient(self, Dock, f'/{ROBOT_ID}/dock')
        self.undock_client = ActionClient(self, Undock, f'/{ROBOT_ID}/undock')

        self.target_person_pose_sub = self.create_subscription(
            PoseStamped, f'/{ROBOT_ID}/target_person_pose', self.target_person_pose_callback, 10)
        self.task_state_sub = self.create_subscription(
            TaskState, '/task/state', self.task_state_callback, 10)

        self.get_logger().info(f'{ROBOT_ID} 브릿지 노드 시작')

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.latest_x, self.latest_y = position.x, position.y
        self.latest_yaw = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)

    def battery_callback(self, msg: BatteryState) -> None:
        self.latest_battery_percent = msg.percentage * 100.0

    def build_status_message(self) -> Optional[RobotStatus]:
        if self.latest_x is None or self.latest_battery_percent is None:
            return None
        return build_robot_status(
            ROBOT_ID, self.latest_battery_percent, self.latest_x, self.latest_y, self.latest_yaw)

    def publish_robot_status(self) -> None:
        msg = self.build_status_message()
        if msg is not None:
            self.status_pub.publish(msg)

    def pause_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        # 아직 응답이 오지 않은 in-flight goal도 무효화한다.
        # (generation을 올려두면 뒤늦게 수락된 goal이 응답 콜백에서 취소된다)
        self.nav_generation += 1
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None

    def dock_callback(self, msg: Bool) -> None:
        if msg.data:
            self._send_dock_goal()
        else:
            self._send_undock_goal()

    def _send_dock_goal(self) -> None:
        if not self.dock_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('dock 액션 서버 대기 중')
            return
        future = self.dock_client.send_goal_async(Dock.Goal())
        future.add_done_callback(self._dock_response_callback)

    def _send_undock_goal(self) -> None:
        if not self.undock_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('undock 액션 서버 대기 중')
            return
        future = self.undock_client.send_goal_async(Undock.Goal())
        future.add_done_callback(self._undock_response_callback)

    def _dock_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('dock goal 거부됨')
            return
        goal_handle.get_result_async().add_done_callback(
            lambda result: self.get_logger().info(f'dock 결과: is_docked={result.result().result.is_docked}'))

    def _undock_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('undock goal 거부됨')
            return
        goal_handle.get_result_async().add_done_callback(
            lambda result: self.get_logger().info(f'undock 결과: is_docked={result.result().result.is_docked}'))

    def task_state_callback(self, msg: TaskState) -> None:
        if msg.robot_id == ROBOT_ID:
            self.current_task_state = msg.state

    def target_person_pose_callback(self, msg: PoseStamped) -> None:
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
        self._send_follow_goal(msg)

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
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda result: self._follow_goal_response_callback(result, generation))

    def _follow_goal_response_callback(self, future, generation: int) -> None:
        goal_handle = future.result()
        if generation != self.nav_generation:
            # pause 또는 더 새로운 goal이 이 goal을 무효화했다. 수락된 상태로 두면
            # 로봇이 계속 주행하므로 반드시 취소한다.
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().warn('follow goal 거부됨')
            return
        self.nav_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda _result: self._follow_result_callback(goal_handle))

    def _follow_result_callback(self, goal_handle) -> None:
        # goal이 정상 종료되면 stale handle이 남지 않도록 비운다.
        if self.nav_goal_handle is goal_handle:
            self.nav_goal_handle = None


def main(args=None):
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
