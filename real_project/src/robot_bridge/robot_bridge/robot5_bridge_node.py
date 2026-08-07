#!/usr/bin/env python3
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from robot_status.msg import RobotStatus

from robot_bridge.pose_utils import build_robot_status, quaternion_to_yaw

ROBOT_ID = 'robot5'


class Robot5BridgeNode(Node):
    def __init__(self):
        super().__init__('robot5_bridge_node')

        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.latest_x: Optional[float] = None
        self.latest_y: Optional[float] = None
        self.latest_yaw: Optional[float] = None
        self.latest_battery_percent: Optional[float] = None

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
