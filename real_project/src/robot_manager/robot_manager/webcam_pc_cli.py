#!/usr/bin/env python3
import os
import sqlite3
import threading
from typing import Dict, List

import rclpy
from rclpy.node import Node

from robot_status.msg import AssignmentGoal, RobotAssignment, RobotError, TaskCommand, TaskState

from robot_manager.webcam_pc_cli_utils import (
    parse_call_args,
    parse_command,
    select_active_robot,
    select_destination,
)

QUIT = object()


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

    def cmd_call(self, args: List[str]) -> None:
        parsed, error = parse_call_args(args)
        if error:
            print(error)
            return
        x, y = parsed
        goal = AssignmentGoal()
        goal.x, goal.y = float(x), float(y)
        self.assignment_goal_pub.publish(goal)
        print(f'[호출] AssignmentGoal(x={x}, y={y}) 발행')

    def cmd_list_destinations(self, args: List[str]) -> None:
        destinations = self.fetch_destinations()
        if not destinations:
            print('등록된 목적지 없음')
            return
        for destination in destinations:
            print(f"{destination['destination_id']}: {destination['destination_name']} "
                  f"({destination['position_x']}, {destination['position_y']}, {destination['orientation_yaw']})")

    def _active_robot_task(self):
        task_states = {robot_id: msg.state for robot_id, msg in self.task_cache.items()}
        robot_id, error = select_active_robot(task_states)
        if error:
            print(error)
            return None, None
        return robot_id, self.task_cache[robot_id].task_id

    def _publish_task_command(self, command: str, robot_id: str, task_id: str,
                               target_x: float = 0.0, target_y: float = 0.0, target_yaw: float = 0.0) -> None:
        msg = TaskCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command, msg.robot_id, msg.task_id = command, robot_id, task_id
        msg.target_x, msg.target_y, msg.target_yaw = target_x, target_y, target_yaw
        self.task_command_pub.publish(msg)

    def cmd_worker_detected(self, args: List[str]) -> None:
        robot_id, task_id = self._active_robot_task()
        if robot_id is None:
            return
        self._publish_task_command('WORKER_DETECTED', robot_id, task_id)
        print(f'[작업자감지] robot_id={robot_id} 발행')

    def cmd_deliver(self, args: List[str]) -> None:
        robot_id, task_id = self._active_robot_task()
        if robot_id is None:
            return
        requested_id = args[0] if args else None
        destination, error = select_destination(self.fetch_destinations(), requested_id)
        if error:
            print(error)
            return
        self._publish_task_command(
            'START_TRANSPORT', robot_id, task_id,
            destination['position_x'], destination['position_y'], destination['orientation_yaw'])
        print(f"[배송모드] robot_id={robot_id}, 목적지={destination['destination_id']} 발행")

    def cmd_confirm(self, args: List[str]) -> None:
        robot_id, task_id = self._active_robot_task()
        if robot_id is None:
            return
        self._publish_task_command('DELIVERY_CONFIRMED', robot_id, task_id)
        print(f'[배송확인] robot_id={robot_id} 발행')

    def cmd_status(self, args: List[str]) -> None:
        if not self.task_cache:
            print('캐싱된 로봇 상태 없음')
            return
        for robot_id, msg in self.task_cache.items():
            print(f'{robot_id}: {msg.state} (task_id={msg.task_id})')

    def cmd_quit(self, args: List[str]):
        return QUIT

    def run_cli(self) -> None:
        handlers = {
            '호출': self.cmd_call,
            '목적지목록': self.cmd_list_destinations,
            '작업자감지': self.cmd_worker_detected,
            '배송모드': self.cmd_deliver,
            '배송확인': self.cmd_confirm,
            '상태': self.cmd_status,
            '종료': self.cmd_quit,
        }
        print('webcam_pc_cli 준비 완료. 명령: 호출/목적지목록/작업자감지/배송모드/배송확인/상태/종료')
        while rclpy.ok():
            try:
                line = input('> ')
            except EOFError:
                break
            command, args = parse_command(line)
            if not command:
                continue
            handler = handlers.get(command)
            if handler is None:
                print(f'알 수 없는 명령: {command}')
                continue
            if handler(args) is QUIT:
                break


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPcCliNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_cli()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
