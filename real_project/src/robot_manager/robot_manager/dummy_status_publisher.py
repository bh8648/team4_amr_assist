    #!/usr/bin/env python3
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from robot_status.msg import RobotStatus


class DummyStatusPublisher(Node):
    def __init__(self):
        super().__init__('dummy_status_publisher')

        # Best Effort QoS 설정 (db_manager_node와 일치)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 퍼블리셔 생성
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', qos_profile)
        # self.error_pub = self.create_publisher(RobotError, '/robot_error', 10)

        # 상태 발행 타이머 (1초마다 발행 -> 1Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # turtle5_map.yaml 기준 지도 경계. margin만큼 안쪽에서만 움직인다.
        self.declare_parameter('map_min_x', -5.31)
        self.declare_parameter('map_max_x', 0.59)
        self.declare_parameter('map_min_y', -5.673)
        self.declare_parameter('map_max_y', 1.377)
        self.declare_parameter('map_margin', 0.2)
        self.map_min_x = float(self.get_parameter('map_min_x').value)
        self.map_max_x = float(self.get_parameter('map_max_x').value)
        self.map_min_y = float(self.get_parameter('map_min_y').value)
        self.map_max_y = float(self.get_parameter('map_max_y').value)
        self.map_margin = float(self.get_parameter('map_margin').value)

        # 랜덤 에러 발생 타이머 (15초마다 일정 확률로 에러 발생)
        # self.error_timer = self.create_timer(15.0, self.publish_error_sample)

        # 테스트용 가상 데이터 초기화
        self.tick = 0
        self.robots = {
            5: {'battery': 95.0, 'x': -3.6, 'y': -3.5, 'yaw': 0.0, 'radius': 1.0},
            11: {'battery': 80.0, 'x': -1.5, 'y': -3.0, 'yaw': 3.14, 'radius': 0.8}
        }

        self.get_logger().info('Dummy Status Publisher가 시작되었습니다. (로봇 ID: 5, 11)')

    def publish_status(self):
        """로봇 5와 11의 가상 위치 및 배터리 정보 퍼블리시"""
        self.tick += 1

        for robot_id, data in self.robots.items():
            msg = RobotStatus()
            msg.robot_id = robot_id
            msg.current_task_id = ''

            # 원형으로 움직이는 가상 궤적 및 Yaw 변화 계산
            radius = data['radius']
            angle = (self.tick * 0.05) % (2 * math.pi)

            raw_x = data['x'] + radius * math.cos(angle)
            raw_y = data['y'] + radius * math.sin(angle)
            msg.x = round(max(self.map_min_x + self.map_margin,
                              min(self.map_max_x - self.map_margin, raw_x)), 2)
            msg.y = round(max(self.map_min_y + self.map_margin,
                              min(self.map_max_y - self.map_margin, raw_y)), 2)
            msg.yaw = round(angle, 2)

            # 배터리 소모 모사 (0.05%씩 감소)
            data['battery'] = max(10.0, data['battery'] - 0.05)
            msg.battery = round(data['battery'], 1)

            self.status_pub.publish(msg)
            self.get_logger().info(
                f'[ID: {msg.robot_id}] Status Pub -> Bat: {msg.battery}%, X: {msg.x}, Y: {msg.y}, Yaw: {msg.yaw}'
            )

    # def publish_error_sample(self):
    #     """테스트용 가상 에러 로그 발행 (50% 확률)"""
    #     if random.random() < 0.5:
    #         robot_id = random.choice([5, 11])
    #         error_codes = ['E-101 (Battery Low)', 'E-202 (Motor Overheat)', 'E-301 (Obstacle Blocked)']
            
    #         error_msg = RobotError()
    #         error_msg.robot_id = robot_id
    #         error_msg.task_id = f'TASK_{random.randint(100, 999)}'
    #         error_msg.error_code = random.choice(error_codes)

    #         self.error_pub.publish(error_msg)
    #         self.get_logger().warn(
    #             f'[ID: {error_msg.robot_id}] Error Pub -> Code: {error_msg.error_code}, Task: {error_msg.task_id}'
    #         )


def main(args=None):
    rclpy.init(args=args)
    node = DummyStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
