#!/usr/bin/env python3

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


class Pc3RobotStatusNode(Node):
    def __init__(self) -> None:
        super().__init__('pc3_robot_status_node')

        # ------------------------------------------------------------------
        # PC3 상태 발행 노드:
        # 1) 배터리와 pose를 읽는다.
        # 2) 중앙/DB에서 바로 쓰기 쉬운 /robot_status JSON을 만든다.
        # 3) 지금은 단순히 robot_id, battery, x, y, yaw만 올린다.
        #
        # 추후 구체화되면:
        # - dock 상태
        # - 고장 상태
        # - 현재 mode
        # - ETA
        # 같은 필드를 확장할 수 있다.
        # ------------------------------------------------------------------
        self.declare_parameter('robot_id', 11)
        self.declare_parameter('robot_status_topic', '/robot_status')
        self.declare_parameter('battery_topic', '/robot11/battery_state')
        self.declare_parameter('amcl_pose_topic', '')
        self.declare_parameter('odom_topic', '/robot11/odom')
        self.declare_parameter('publish_hz', 2.0)
        self.declare_parameter('battery_scale', 100.0)
        self.declare_parameter('allow_poseless_publish', True)
        self.declare_parameter('default_x', 0.0)
        self.declare_parameter('default_y', 0.0)
        self.declare_parameter('default_yaw', 0.0)

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.robot_status_topic = self.get_parameter('robot_status_topic').value
        self.battery_topic = self.get_parameter('battery_topic').value
        self.amcl_pose_topic = self.get_parameter('amcl_pose_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.battery_scale = float(self.get_parameter('battery_scale').value)
        self.allow_poseless_publish = bool(self.get_parameter('allow_poseless_publish').value)
        self.default_x = float(self.get_parameter('default_x').value)
        self.default_y = float(self.get_parameter('default_y').value)
        self.default_yaw = float(self.get_parameter('default_yaw').value)

        self.status_pub = self.create_publisher(String, self.robot_status_topic, 10)
        self.create_subscription(BatteryState, self.battery_topic, self.battery_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        if self.amcl_pose_topic:
            self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose_topic, self.amcl_pose_callback, 10)

        self.latest_pose: PoseStamped | None = None
        self.latest_battery = 0.0
        self.create_timer(1.0 / max(self.publish_hz, 0.5), self.publish_robot_status)

    def battery_callback(self, msg: BatteryState) -> None:
        # percentage가 0~1 스케일인지 0~100 스케일인지는 장비 설정마다 다를 수 있다.
        # 현재는 battery_scale 파라미터로 맞춘다.
        # 추후 실제 로봇 값 확인 후 scale과 최소 임계값 정책을 확정해야 한다.
        self.latest_battery = float(msg.percentage) * self.battery_scale

    def odom_callback(self, msg: Odometry) -> None:
        # 최소 동작에서는 odom만 있어도 robot_status를 만들 수 있게 한다.
        # 나중에 localization 신뢰도가 중요해지면 amcl_pose를 우선 사용하도록
        # 정책을 더 명확히 분리할 수 있다.
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.latest_pose = pose

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        # amcl_pose가 들어오면 map 기준 pose로 상태를 갱신한다.
        # 현재는 단순 overwrite 방식인데, 추후 stale 비교나 covariance 검사도 가능하다.
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.latest_pose = pose

    def compose_robot_status(self) -> dict | None:
        # 중앙/DB/HMI가 보기 쉬운 공통 JSON 포맷을 만드는 함수다.
        # 현재는 핵심 5값만 넣지만, 이후 다음 필드를 자연스럽게 확장 가능하다.
        # - docked
        # - failure_state
        # - current_mode
        # - eta
        # - task_id
        if self.latest_pose is None and not self.allow_poseless_publish:
            return None

        x = self.default_x
        y = self.default_y
        yaw = self.default_yaw
        has_pose = False

        if self.latest_pose is not None:
            x = float(self.latest_pose.pose.position.x)
            y = float(self.latest_pose.pose.position.y)
            yaw = float(self.quaternion_to_yaw(self.latest_pose.pose.orientation))
            has_pose = True

        return {
            'robot_id': self.robot_id,
            'battery': float(self.latest_battery),
            'x': x,
            'y': y,
            'yaw': yaw,
            'has_pose': has_pose,
        }

    def publish_robot_status(self) -> None:
        # 현재는 publish_hz 주기로 꾸준히 상태를 올린다.
        # 추후 변화가 있을 때만 publish하는 방식으로 바꿀 수도 있다.
        payload = self.compose_robot_status()
        if payload is not None:
            self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))

    def quaternion_to_yaw(self, quat: Quaternion) -> float:
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Pc3RobotStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
