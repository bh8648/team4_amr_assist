#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from robot_status.msg import AssignmentGoal, RobotAssignment, RobotStatus


class RobotAssignmentNode(Node):
    """가용 AMR 중 목표 지점 이동에 가장 적합한 로봇을 선택한다."""

    def __init__(self):
        super().__init__('robot_assignment_node')

        self.declare_parameter('status_timeout_sec', 5.0)
        self.declare_parameter('minimum_battery', 20.0)
        self.declare_parameter('battery_penalty_weight', 0.05)
        self.declare_parameter('heading_penalty_weight', 0.5)

        self.status_timeout_sec = float(
            self.get_parameter('status_timeout_sec').value
        )
        self.minimum_battery = float(
            self.get_parameter('minimum_battery').value
        )
        self.battery_penalty_weight = float(
            self.get_parameter('battery_penalty_weight').value
        )
        self.heading_penalty_weight = float(
            self.get_parameter('heading_penalty_weight').value
        )

        # robot_id -> (RobotStatus, last_received_time)
        self.robot_statuses = {}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.status_subscription = self.create_subscription(
            RobotStatus, '/robot_status', self.status_callback, qos
        )
        self.goal_subscription = self.create_subscription(
            AssignmentGoal, '/assignment_goal', self.goal_callback, 10
        )
        self.assignment_publisher = self.create_publisher(
            RobotAssignment, '/robot_assignment', 10
        )

        self.get_logger().info(
            'AMR 배정 노드 시작: /assignment_goal -> /robot_assignment'
        )

    def status_callback(self, msg):
        self.robot_statuses[msg.robot_id] = (msg, self.get_clock().now())

    @staticmethod
    def angle_difference(a, b):
        return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)

    def suitability_score(self, status, target_x, target_y):
        """낮을수록 적합: 이동 거리 + 저전력 패널티 + 방향 전환 패널티."""
        dx = target_x - float(status.x)
        dy = target_y - float(status.y)
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx) if distance > 0.0 else status.yaw
        heading_error = self.angle_difference(target_yaw, float(status.yaw))
        battery_penalty = (100.0 - float(status.battery)) * self.battery_penalty_weight
        return distance + battery_penalty + heading_error * self.heading_penalty_weight

    def select_robot(self, target_x, target_y):
        now = self.get_clock().now()
        candidates = []

        for robot_id, (status, received_at) in self.robot_statuses.items():
            age = (now - received_at).nanoseconds / 1e9
            if age > self.status_timeout_sec:
                continue
            if status.current_task_id.strip():
                continue
            if float(status.battery) < self.minimum_battery:
                continue

            score = self.suitability_score(status, target_x, target_y)
            candidates.append((score, -float(status.battery), robot_id, status))

        if not candidates:
            return None
        return min(candidates, key=lambda item: item[:3])

    def goal_callback(self, goal):
        result = RobotAssignment()
        result.assigned_at = self.get_clock().now().to_msg()
        result.target_x = goal.x
        result.target_y = goal.y

        selected = self.select_robot(float(goal.x), float(goal.y))
        if selected is None:
            result.assigned = False
            result.robot_id = -1
            self.get_logger().warn(
                f'배정 실패: 목표 ({goal.x:.2f}, {goal.y:.2f}), 가용 AMR 없음'
            )
        else:
            score, _, robot_id, status = selected
            result.assigned = True
            result.robot_id = robot_id
            self.get_logger().info(
                f'AMR {robot_id} 배정: 목표 ({goal.x:.2f}, {goal.y:.2f}), '
                f'배터리 {status.battery:.1f}%, 점수 {score:.2f}'
            )

        self.assignment_publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = RobotAssignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
