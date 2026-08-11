#!/usr/bin/env python3
import os
import sqlite3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from robot_status.msg import Destination, DestinationList


class DestinationManagerNode(Node):
    """중앙 DB의 목적지 설정을 모든 로봇 HMI와 Task Manager에 배포한다."""

    def __init__(self):
        super().__init__('destination_manager_node')
        self.declare_parameter('db_path', os.environ.get('AMR_DB_PATH', os.path.abspath('amr.db')))
        self.db_path = os.path.abspath(os.path.expanduser(self.get_parameter('db_path').value))
        # 설정 데이터는 변경 빈도가 낮으므로 늦게 접속한 PC도 마지막 목록을 받는 QoS를 사용한다.
        destination_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.destination_pub = self.create_publisher(
            DestinationList, '/destinations', destination_qos)
        self.publish_destinations()
        # DB 관리 화면에서 목적지가 바뀌어도 별도 재시작 없이 최대 5초 안에 반영한다.
        self.reload_timer = self.create_timer(5.0, self.publish_destinations)
        self.get_logger().info(f'목적지 배포 노드 시작: {self.db_path}')

    def read_destinations(self):
        """사용자 스키마의 destinations 테이블을 ID 순서로 조회한다."""
        if not os.path.exists(self.db_path):
            raise FileNotFoundError(self.db_path)
        with sqlite3.connect(self.db_path) as conn:
            return conn.execute(
                """
                SELECT destination_id, destination_name, position_x, position_y
                FROM destinations ORDER BY destination_id
                """
            ).fetchall()

    def publish_destinations(self) -> None:
        """DB 조회 성공 시에만 새 목적지 목록을 발행해 마지막 정상 설정을 보존한다."""
        try:
            rows = self.read_destinations()
        except (OSError, sqlite3.Error) as error:
            self.get_logger().error(f'목적지 DB 조회 실패: {error}')
            return
        result = DestinationList()
        result.stamp = self.get_clock().now().to_msg()
        for destination_id, name, x, y in rows:
            item = Destination()
            item.destination_id, item.destination_name = str(destination_id), str(name)
            item.position_x, item.position_y = float(x), float(y)
            result.destinations.append(item)
        self.destination_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = DestinationManagerNode()
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
