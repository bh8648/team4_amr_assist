#!/usr/bin/env python3
import math
import os
import random
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from robot_status.msg import NavigationResult, RobotError, RobotStatus, TaskState


class DummyStatusPublisher(Node):
    """HMI/Task Manager 명령에 반응하는 2대의 간단한 AMR 시뮬레이터."""

    def __init__(self):
        super().__init__('dummy_status_publisher')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.status_publisher = self.create_publisher(RobotStatus, '/robot_status', qos)
        self.error_pub = self.create_publisher(RobotError, '/robot_error', 10)
        self.navigation_result_pub = self.create_publisher(NavigationResult, '/navigation/result', 10)
        self.task_state_sub = self.create_subscription(TaskState, '/task/state', self.task_state_callback, 10)

        self.declare_parameter('linear_speed', 0.22)
        self.declare_parameter('angular_speed', 1.2)
        self.declare_parameter('arrival_tolerance', 0.08)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.arrival_tolerance = float(self.get_parameter('arrival_tolerance').value)

        project_root = Path(get_package_prefix('robot_manager')).resolve().parents[1]
        default_map_path = project_root / 'real_project/amr_delivery_ui/frontend/maps/map2.yaml'
        self.declare_parameter('map_yaml_path', os.environ.get('AMR_MAP_YAML', str(default_map_path)))
        configured_map_path = Path(os.path.expanduser(self.get_parameter('map_yaml_path').value))
        map_yaml_path = (configured_map_path if configured_map_path.is_absolute() else project_root / configured_map_path).resolve()
        if not map_yaml_path.is_file():
            map_yaml_path = default_map_path.resolve()
        self.bounds = self.load_map_bounds(map_yaml_path)

        self.robots = {
            'robot5': self.make_robot(0.0, 0.0, 0.0, 95.0),
            'robot11': self.make_robot(-2.235, -5.022, -1.528, 80.0),
        }
        for robot_id in self.robots:
            self.create_subscription(Bool, f'/{robot_id}/pause/request', lambda msg, rid=robot_id: self.pause_callback(rid, msg), 10)
            self.create_subscription(Bool, f'/{robot_id}/dock/request', lambda msg, rid=robot_id: self.dock_callback(rid, msg), 10)
            self.create_subscription(Twist, f'/{robot_id}/cmd_vel', lambda msg, rid=robot_id: self.cmd_vel_callback(rid, msg), 10)

        self.last_update = self.get_clock().now()
        self.create_timer(0.1, self.update_simulation)
        self.create_timer(1.0, self.publish_status)
        self.create_timer(15.0, self.publish_error_sample)
        self.get_logger().info('Dummy AMR Simulator 시작: robot5, robot11')

    @staticmethod
    def make_robot(x, y, yaw, battery):
        return {'x': x, 'y': y, 'yaw': yaw, 'battery': battery, 'dock': (x, y, yaw), 'docked': True, 'paused': False, 'state': 'DOCKED', 'task_id': '', 'goal_type': '', 'target': None, 'completed_goal': None, 'result_sent': False, 'cmd_linear': 0.0, 'cmd_angular': 0.0, 'cmd_until': 0.0}

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def read_pgm_size(path: Path):
        """PGM 헤더에서 지도의 가로·세로 셀 수를 읽는다 (robot_assignment_node와 동일 로직)."""
        data, offset, tokens = path.read_bytes(), 0, []
        while len(tokens) < 3:
            while offset < len(data) and data[offset] <= 32:
                offset += 1
            if offset < len(data) and data[offset] == 35:
                while offset < len(data) and data[offset] != 10:
                    offset += 1
                continue
            start = offset
            while offset < len(data) and data[offset] > 32 and data[offset] != 35:
                offset += 1
            tokens.append(data[start:offset].decode('ascii'))
        if tokens[0] not in ('P2', 'P5'):
            raise ValueError('지원하지 않는 PGM 형식입니다.')
        return int(tokens[1]), int(tokens[2])

    def load_map_bounds(self, map_yaml_path: Path):
        """Nav2 YAML과 PGM 크기로 (min_x, max_x, min_y, max_y) 범위를 계산한다."""
        if not map_yaml_path.is_file():
            raise FileNotFoundError(f'지도 YAML을 찾을 수 없습니다: {map_yaml_path}')
        meta = {}
        for raw_line in map_yaml_path.read_text(encoding='utf-8').splitlines():
            line = raw_line.split('#', 1)[0].strip()
            if line and ':' in line:
                key, value = line.split(':', 1)
                meta[key.strip()] = value.strip()
        resolution = float(meta['resolution'])
        origin = [float(value.strip()) for value in meta['origin'].strip('[]').split(',')]
        image_path = (map_yaml_path.parent / meta['image']).resolve()
        width, height = self.read_pgm_size(image_path)
        bounds = origin[0], origin[0] + width * resolution, origin[1], origin[1] + height * resolution
        self.get_logger().info(f'지도 좌표 범위: {bounds[0]:.2f} ≤ x < {bounds[1]:.2f}, {bounds[2]:.2f} ≤ y < {bounds[3]:.2f}')
        return bounds

    def pause_callback(self, robot_id, msg):
        robot = self.robots[robot_id]
        robot['paused'] = bool(msg.data)
        if robot['paused']:
            robot['cmd_linear'] = robot['cmd_angular'] = 0.0

    def dock_callback(self, robot_id, msg):
        robot = self.robots[robot_id]
        if msg.data:
            robot['docked'] = False
            robot['target'] = robot['dock']
            robot['goal_type'] = robot['goal_type'] or 'MANUAL_DOCK'
            robot['result_sent'] = False
        else:
            robot['docked'] = False
            if robot['goal_type'] == 'MANUAL_DOCK':
                robot['target'] = None
                robot['goal_type'] = ''

    def cmd_vel_callback(self, robot_id, msg):
        robot = self.robots[robot_id]
        if robot['paused'] or robot['docked']:
            return
        robot['cmd_linear'] = float(msg.linear.x)
        robot['cmd_angular'] = float(msg.angular.z)
        robot['cmd_until'] = self.get_clock().now().nanoseconds / 1e9 + 0.35

    def task_state_callback(self, msg):
        robot = self.robots.get(str(msg.robot_id))
        if robot is None:
            return
        goal_key = (msg.task_id, msg.goal_type)
        robot['state'] = msg.state
        robot['task_id'] = msg.task_id
        robot['paused'] = msg.state == 'PAUSED'
        if msg.state in ('ASSIGNED', 'TRANSPORTING', 'RETURNING') and msg.goal_type and robot['completed_goal'] != goal_key:
            robot['target'] = (float(msg.target_x), float(msg.target_y), float(msg.target_yaw))
            robot['goal_type'] = msg.goal_type
            robot['docked'] = False
            robot['result_sent'] = False
        elif msg.state == 'DOCKED':
            robot['target'] = None
            robot['docked'] = True

    def update_simulation(self):
        now = self.get_clock().now()
        dt = min(0.2, max(0.0, (now - self.last_update).nanoseconds / 1e9))
        self.last_update = now
        now_sec = now.nanoseconds / 1e9
        for robot_id, robot in self.robots.items():
            if robot['paused']:
                continue
            if now_sec <= robot['cmd_until']:
                robot['yaw'] = self.normalize_angle(robot['yaw'] + robot['cmd_angular'] * dt)
                self.move_linear(robot, robot['cmd_linear'] * dt)
                self.consume_battery(robot, dt, abs(robot['cmd_linear']) > 0.001 or abs(robot['cmd_angular']) > 0.001)
            elif robot['target'] is not None:
                self.move_to_target(robot_id, robot, dt)

    def move_linear(self, robot, distance):
        min_x, max_x, min_y, max_y = self.bounds
        robot['x'] = max(min_x, min(max_x, robot['x'] + math.cos(robot['yaw']) * distance))
        robot['y'] = max(min_y, min(max_y, robot['y'] + math.sin(robot['yaw']) * distance))

    def move_to_target(self, robot_id, robot, dt):
        target_x, target_y, target_yaw = robot['target']
        dx, dy = target_x - robot['x'], target_y - robot['y']
        distance = math.hypot(dx, dy)
        if distance <= self.arrival_tolerance:
            robot['x'], robot['y'], robot['yaw'] = target_x, target_y, self.normalize_angle(target_yaw)
            robot['target'] = None
            robot['docked'] = robot['goal_type'] in ('TO_DOCK', 'MANUAL_DOCK')
            if robot['goal_type'] in ('TO_WORKER', 'TO_DESTINATION', 'TO_DOCK') and not robot['result_sent']:
                self.publish_navigation_result(robot_id, robot)
                robot['completed_goal'] = (robot['task_id'], robot['goal_type'])
            robot['result_sent'] = True
            return
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - robot['yaw'])
        robot['yaw'] = self.normalize_angle(robot['yaw'] + max(-self.angular_speed * dt, min(self.angular_speed * dt, yaw_error)))
        if abs(yaw_error) < 0.45:
            self.move_linear(robot, min(self.linear_speed * dt, distance))
            self.consume_battery(robot, dt, True)

    @staticmethod
    def consume_battery(robot, dt, moving):
        if moving:
            robot['battery'] = max(5.0, robot['battery'] - 0.01 * dt)
        elif robot['docked']:
            robot['battery'] = min(100.0, robot['battery'] + 0.05 * dt)

    def publish_navigation_result(self, robot_id, robot):
        msg = NavigationResult()
        msg.stamp = self.get_clock().now().to_msg()
        msg.task_id = robot['task_id']
        msg.robot_id = robot_id
        msg.goal_type = robot['goal_type']
        msg.success = True
        msg.error_code = ''
        self.navigation_result_pub.publish(msg)

    def publish_status(self):
        for robot_id, robot in self.robots.items():
            if robot['docked'] and not robot['paused']:
                self.consume_battery(robot, 1.0, False)
            msg = RobotStatus()
            msg.robot_id = robot_id
            msg.battery = round(robot['battery'], 1)
            msg.x, msg.y, msg.yaw = round(robot['x'], 3), round(robot['y'], 3), round(robot['yaw'], 3)
            self.status_publisher.publish(msg)

    def publish_error_sample(self):
        if random.random() >= 0.3:
            return
        msg = RobotError()
        msg.robot_id = random.choice(list(self.robots))
        msg.task_id = self.robots[msg.robot_id]['task_id']
        msg.error_code = random.choice(['E-101_BATTERY_LOW', 'E-202_MOTOR_OVERHEAT', 'E-301_OBSTACLE_BLOCKED'])
        self.error_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyStatusPublisher()
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
