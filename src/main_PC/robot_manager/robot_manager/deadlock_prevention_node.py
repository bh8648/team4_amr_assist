#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from robot_status.msg import DeadlockPermission, RobotStatus, TaskState


@dataclass
class RobotSnapshot:
    x: float
    y: float
    received_at: Time
    previous_position: Optional[Tuple[float, float]] = None


class DeadlockPreventionNode(Node):
    """두 AMR의 거리와 작업 상태를 이용해 근접 교착을 방지한다."""

    ROBOT5 = 'robot5'
    ROBOT11 = 'robot11'
    MOVING_STATES = {'ASSIGNED', 'FOLLOWING', 'TRANSPORTING', 'RETURNING'}
    EXCLUDED_STATES = {'DOCKED', 'PAUSED', 'ERROR'}

    def __init__(self):
        super().__init__('deadlock_prevention_node')
        self.declare_parameter('conflict_distance_m', 0.7)
        self.declare_parameter('release_distance_m', 0.9)
        self.declare_parameter('status_timeout_sec', 2.0)
        self.declare_parameter('movement_epsilon_m', 0.03)
        self.declare_parameter('check_interval_sec', 0.2)
        self.conflict_distance = float(self.get_parameter('conflict_distance_m').value)
        self.release_distance = float(self.get_parameter('release_distance_m').value)
        self.status_timeout = float(self.get_parameter('status_timeout_sec').value)
        self.movement_epsilon = float(self.get_parameter('movement_epsilon_m').value)
        if self.release_distance <= self.conflict_distance:
            raise ValueError('release_distance_m은 conflict_distance_m보다 커야 합니다.')
        self.snapshots: Dict[str, RobotSnapshot] = {}
        self.task_states: Dict[str, Tuple[str, str]] = {}
        self.active_conflict: Optional[Tuple[str, str]] = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.status_subscriptions = [self.create_subscription(RobotStatus, f'/{robot_id}/robot_status', lambda msg, expected=robot_id: self.status_callback(expected, msg), qos) for robot_id in (self.ROBOT5, self.ROBOT11)]
        self.task_state_sub = self.create_subscription(TaskState, '/task/state', self.task_state_callback, 10)
        self.permission_pub = self.create_publisher(DeadlockPermission, '/deadlock/permission', 10)
        self.timer = self.create_timer(float(self.get_parameter('check_interval_sec').value), self.evaluate)
        self.get_logger().info(f'근접 교착 방지 시작: 진입 {self.conflict_distance:.2f}m, 해제 {self.release_distance:.2f}m, robot5 우선')

    @staticmethod
    def normalize_robot_id(robot_id: str) -> str:
        value = str(robot_id).strip()
        return value if value.startswith('robot') else f'robot{value}'

    def status_callback(self, expected_robot_id: str, msg: RobotStatus) -> None:
        robot_id = self.normalize_robot_id(msg.robot_id)
        if robot_id != expected_robot_id:
            self.get_logger().warning(f'토픽과 robot_id 불일치: expected={expected_robot_id}, received={robot_id}')
            return
        previous = self.snapshots.get(robot_id)
        previous_position = (previous.x, previous.y) if previous else None
        self.snapshots[robot_id] = RobotSnapshot(float(msg.x), float(msg.y), self.get_clock().now(), previous_position)

    def task_state_callback(self, msg: TaskState) -> None:
        robot_id = self.normalize_robot_id(msg.robot_id)
        if robot_id in (self.ROBOT5, self.ROBOT11):
            self.task_states[robot_id] = (msg.state.strip().upper(), msg.task_id)

    def status_is_fresh(self, snapshot: RobotSnapshot, now: Time) -> bool:
        return (now - snapshot.received_at).nanoseconds / 1e9 <= self.status_timeout

    def robot_is_moving_candidate(self, robot_id: str) -> bool:
        state = self.task_states.get(robot_id, ('UNKNOWN', ''))[0]
        if state in self.EXCLUDED_STATES:
            return False
        if state in self.MOVING_STATES:
            return True
        snapshot = self.snapshots[robot_id]
        if snapshot.previous_position is None:
            return False
        return math.hypot(snapshot.x - snapshot.previous_position[0], snapshot.y - snapshot.previous_position[1]) >= self.movement_epsilon

    def evaluate(self) -> None:
        if self.active_conflict:
            robot5_state = self.task_states.get(self.ROBOT5, ('UNKNOWN', ''))[0]
            robot11_state = self.task_states.get(self.ROBOT11, ('UNKNOWN', ''))[0]
            if robot5_state in self.EXCLUDED_STATES:
                self.clear_conflict('PRIORITY_ROBOT_NOT_MOVING')
                return
            if robot11_state in ('DOCKED', 'ERROR'):
                self.clear_conflict('YIELDING_ROBOT_INACTIVE')
                return
        if self.ROBOT5 not in self.snapshots or self.ROBOT11 not in self.snapshots:
            return
        now = self.get_clock().now()
        robot5, robot11 = self.snapshots[self.ROBOT5], self.snapshots[self.ROBOT11]
        if not self.status_is_fresh(robot5, now) or not self.status_is_fresh(robot11, now):
            return
        distance = math.hypot(robot5.x - robot11.x, robot5.y - robot11.y)
        if self.active_conflict:
            if distance >= self.release_distance:
                self.clear_conflict('SAFE_DISTANCE_RECOVERED')
            return
        robot5_active = self.robot_is_moving_candidate(self.ROBOT5)
        robot11_active = self.robot_is_moving_candidate(self.ROBOT11)
        if not robot5_active or not robot11_active:
            return
        if distance < self.conflict_distance:
            self.active_conflict = (self.ROBOT5, self.ROBOT11)
            self.publish_permission(self.ROBOT5, True, 'ROBOT5_PRIORITY')
            self.publish_permission(self.ROBOT11, False, f'WAIT_FOR_ROBOT5_DISTANCE_{distance:.2f}M')
            self.get_logger().warn(f'근접 교착 제어 시작: 거리={distance:.2f}m, robot5 진행, robot11 정지')

    def clear_conflict(self, reason: str) -> None:
        if not self.active_conflict:
            return
        _, yielding_robot = self.active_conflict
        self.active_conflict = None
        self.publish_permission(yielding_robot, True, reason)
        self.get_logger().info(f'근접 교착 제어 해제: {yielding_robot} 재개 ({reason})')

    def publish_permission(self, robot_id: str, granted: bool, reason: str) -> None:
        state, task_id = self.task_states.get(robot_id, ('UNKNOWN', ''))
        msg = DeadlockPermission()
        msg.stamp = self.get_clock().now().to_msg()
        msg.robot_id, msg.task_id, msg.zone_id = robot_id, task_id, 'PROXIMITY'
        msg.granted, msg.reason = granted, reason
        self.permission_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DeadlockPreventionNode()
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
