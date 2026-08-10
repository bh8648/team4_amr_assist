#!/usr/bin/env python3
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class WorkerTrackingBridgeNode(Node):
    """robot_bridge_node가 기대하는 worker_tracking/enable-detected-lost 계약을 채운다."""

    def __init__(self, robot_id: str = ''):
        super().__init__('worker_tracking_bridge_node')
        self.declare_parameter('robot_id', robot_id)
        self.robot_id = str(self.get_parameter('robot_id').value).strip()
        if self.robot_id not in ('robot5', 'robot11'):
            raise ValueError('robot_id는 robot5 또는 robot11이어야 합니다.')
        self.declare_parameter('worker_lost_timeout', 60.0)
        self.worker_lost_timeout = float(self.get_parameter('worker_lost_timeout').value)
        if self.worker_lost_timeout <= 0.0:
            raise ValueError('worker_lost_timeout은 0보다 커야 합니다.')
        topic_prefix = f'/{self.robot_id}'

        self.enabled = False
        self.last_pose_time_ns: Optional[int] = None
        self.detected_sent = False
        self.lost_sent = False

        # robot_bridge_node가 늦게 뜬 이 노드에도 현재 enable 상태를 즉시 전달하도록
        # 발행 측과 동일한 latched QoS로 구독한다.
        enable_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable_sub = self.create_subscription(
            Bool, f'{topic_prefix}/worker_tracking/enable', self.enable_callback, enable_qos)
        self.raw_pose_sub = self.create_subscription(
            PoseStamped, f'{topic_prefix}/target_person_pose_raw', self.raw_pose_callback, 10)

        self.target_pose_pub = self.create_publisher(PoseStamped, f'{topic_prefix}/target_person_pose', 10)
        self.worker_detected_pub = self.create_publisher(Bool, f'{topic_prefix}/worker_detected', 10)
        self.worker_lost_pub = self.create_publisher(Bool, f'{topic_prefix}/worker_tracking/lost', 10)

        self.lost_check_timer = self.create_timer(1.0, self.check_worker_lost)
        self.get_logger().info(f'{self.robot_id} 작업자 추적 브릿지 노드 시작')

    def enable_callback(self, msg: Bool) -> None:
        """enable이 새로 켜질 때마다 이번 추종 구간의 감지/유실 상태를 초기화한다."""
        if msg.data and not self.enabled:
            self.last_pose_time_ns = None
            self.detected_sent = False
            self.lost_sent = False
        self.enabled = msg.data

    def raw_pose_callback(self, msg: PoseStamped) -> None:
        """활성화 상태일 때만 추종 좌표를 중계하고 최초 수신을 감지 신호로 알린다."""
        if not self.enabled:
            return
        self.target_pose_pub.publish(msg)
        self.last_pose_time_ns = self.get_clock().now().nanoseconds
        self.lost_sent = False
        if not self.detected_sent:
            self.worker_detected_pub.publish(Bool(data=True))
            self.detected_sent = True

    def check_worker_lost(self) -> None:
        """활성화 상태에서 유실 제한시간을 넘기면 한 번만 유실 신호를 보낸다."""
        if not self.enabled or self.last_pose_time_ns is None or self.lost_sent:
            return
        elapsed_s = (self.get_clock().now().nanoseconds - self.last_pose_time_ns) / 1e9
        if elapsed_s >= self.worker_lost_timeout:
            self.worker_lost_pub.publish(Bool(data=True))
            self.lost_sent = True


def main(args=None):
    """robot_id 파라미터로 선택된 작업자 추적 브릿지 실행 진입점."""
    rclpy.init(args=args)
    node = WorkerTrackingBridgeNode()
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
