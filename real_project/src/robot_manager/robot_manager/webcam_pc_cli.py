#!/usr/bin/env python3
import os
import sqlite3
from typing import Dict, List

import rclpy
from rclpy.node import Node

from robot_status.msg import AssignmentGoal, RobotAssignment, RobotError, TaskCommand, TaskState


class WebcamPcCliNode(Node):
    def __init__(self):
        super().__init__('webcam_pc_cli_node')
        default_db_path = os.path.abspath('amr.db')
        self.declare_parameter('db_path', os.environ.get('AMR_DB_PATH', default_db_path))
        self.db_path = os.path.abspath(os.path.expanduser(self.get_parameter('db_path').value))

        self.task_cache: Dict[str, TaskState] = {}

        self.assignment_goal_pub = self.create_publisher(AssignmentGoal, '/assignment_goal', 10)
        self.task_command_pub = self.create_publisher(TaskCommand, '/task/command', 10)

        self.assignment_sub = self.create_subscription(
            RobotAssignment, '/robot_assignment', self.assignment_callback, 10)
        self.task_state_sub = self.create_subscription(
            TaskState, '/task/state', self.task_state_callback, 10)
        self.error_sub = self.create_subscription(
            RobotError, '/robot_error', self.error_callback, 10)

        self.get_logger().info('webcam_pc_cli 노드 시작')

    def assignment_callback(self, msg: RobotAssignment) -> None:
        if msg.assigned:
            print(f'[배정 성공] robot_id={msg.robot_id}, 목표=({msg.target_x:.2f}, {msg.target_y:.2f})')
        else:
            print(f'[배정 실패] 목표=({msg.target_x:.2f}, {msg.target_y:.2f})')

    def task_state_callback(self, msg: TaskState) -> None:
        self.task_cache[msg.robot_id] = msg

    def error_callback(self, msg: RobotError) -> None:
        print(f'[오류] robot_id={msg.robot_id}, task_id={msg.task_id}, error_code={msg.error_code}')

    def fetch_destinations(self) -> List[dict]:
        with sqlite3.connect(self.db_path) as conn:
            conn.row_factory = sqlite3.Row
            rows = conn.execute('SELECT * FROM destinations').fetchall()
        return [dict(row) for row in rows]


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPcCliNode()
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
