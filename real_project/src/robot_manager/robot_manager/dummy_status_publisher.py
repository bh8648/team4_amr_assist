#!/usr/bin/env python3
import math
import random

import rclpy
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
        self.status_publishers = {robot_id: self.create_publisher(RobotStatus, f'/{robot_id}/robot_status', qos) for robot_id in ('robot5', 'robot11')}
        self.error_pub = self.create_publisher(RobotError, '/robot_error', 10)
        self.navigation_result_pub = self.create_publisher(NavigationResult, '/navigation/result', 10)
        self.task_state_sub = self.create_subscription(TaskState, '/task/state', self.task_state_callback, 10)

        self.declare_parameter('linear_speed', 0.22)
        self.declare_parameter('angular_speed', 1.2)
        self.declare_parameter('arrival_tolerance', 0.08)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.arrival_tolerance = float(self.get_parameter('arrival_tolerance').value)

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
            self.status_publishers[robot_id].publish(msg)

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
