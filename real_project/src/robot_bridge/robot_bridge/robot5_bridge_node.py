#!/usr/bin/env python3
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Dock, Undock
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from robot_status.msg import RobotStatus, TaskState

from robot_bridge.pose_utils import build_robot_status, is_valid_quaternion, quaternion_to_yaw

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
        self.battery_sub = self.create_subscription(
            BatteryState, f'/{ROBOT_ID}/battery_state', self.battery_callback, 10)

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
        if msg.data and self.nav_goal_handle is not None:
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
        orientation = msg.pose.orientation
        if not is_valid_quaternion(orientation.x, orientation.y, orientation.z, orientation.w):
            self.get_logger().warn('무효한 target_person_pose 무시 (추적 대상 없음)')
            return
        self._send_follow_goal(msg)

    def _send_follow_goal(self, pose: PoseStamped) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('navigate_to_pose 액션 서버 대기 중')
            return
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.nav_generation += 1
        generation = self.nav_generation
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda result: self._follow_goal_response_callback(result, generation))

    def _follow_goal_response_callback(self, future, generation: int) -> None:
        if generation != self.nav_generation:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('follow goal 거부됨')
            return
        self.nav_goal_handle = goal_handle


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
