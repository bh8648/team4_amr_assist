#!/usr/bin/env python3
import math
import os
import sqlite3
import threading
from typing import Dict, List

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from robot_status.msg import AssignmentGoal, RobotAssignment, RobotError, TaskCommand, TaskState

from robot_manager.webcam_pc_cli_utils import (
    FOLLOWING_MOCK_POSES,
    parse_call_args,
    parse_command,
    parse_interval,
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
        self.target_pose_pub = self.create_publisher(PoseStamped, '/robot11/target_person_pose', 10)

        self.following_timer = None
        self.following_index = 0

        self.assignment_sub = self.create_subscription(
            RobotAssignment, '/robot_assignment', self.assignment_callback, 10)
        self.task_state_sub = self.create_subscription(
            TaskState, '/task/state', self.task_state_callback, 10)
        self.error_sub = self.create_subscription(
            RobotError, '/robot_error', self.error_callback, 10)

        self.get_logger().info(f'webcam_pc_cli 노드 시작 (DB 경로: {self.db_path})')
        if not os.path.exists(self.db_path):
            self.get_logger().warn(f'DB 파일이 존재하지 않습니다: {self.db_path}')

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

    def _active_robot_task(self, requested_id=None):
        task_states = {robot_id: msg.state for robot_id, msg in dict(self.task_cache).items()}
        robot_id, error = select_active_robot(task_states, requested_id)
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
        requested_id = args[0] if args else None
        robot_id, task_id = self._active_robot_task(requested_id)
        if robot_id is None:
            return
        self._publish_task_command('WORKER_DETECTED', robot_id, task_id)
        print(f'[작업자감지] robot_id={robot_id} 발행')

    def cmd_deliver(self, args: List[str]) -> None:
        remaining = list(args)
        requested_robot_id = None
        if remaining and remaining[0].startswith('robot'):
            requested_robot_id = remaining.pop(0)
        robot_id, task_id = self._active_robot_task(requested_robot_id)
        if robot_id is None:
            return
        requested_destination_id = remaining[0] if remaining else None
        destination, error = select_destination(self.fetch_destinations(), requested_destination_id)
        if error:
            print(error)
            return
        self._publish_task_command(
            'START_TRANSPORT', robot_id, task_id,
            destination['position_x'], destination['position_y'], destination['orientation_yaw'])
        print(f"[배송모드] robot_id={robot_id}, 목적지={destination['destination_id']} 발행")

    def cmd_confirm(self, args: List[str]) -> None:
        requested_id = args[0] if args else None
        robot_id, task_id = self._active_robot_task(requested_id)
        if robot_id is None:
            return
        self._publish_task_command('DELIVERY_CONFIRMED', robot_id, task_id)
        print(f'[배송확인] robot_id={robot_id} 발행')

    def cmd_status(self, args: List[str]) -> None:
        cached = dict(self.task_cache)
        if not cached:
            print('캐싱된 로봇 상태 없음')
            return
        for robot_id, msg in cached.items():
            print(f'{robot_id}: {msg.state} (task_id={msg.task_id})')

    def _stop_following_timer(self) -> None:
        if self.following_timer is not None:
            self.destroy_timer(self.following_timer)
            self.following_timer = None

    def cmd_follow_start(self, args: List[str]) -> None:
        interval, error = parse_interval(args)
        if error:
            print(error)
            return
        if self.following_timer is not None:
            print('이미 진행 중입니다. 먼저 추종중지를 입력하세요')
            return
        robot_state = self.task_cache.get('robot11')
        if robot_state is None or robot_state.state != 'FOLLOWING':
            print('[경고] robot11이 FOLLOWING 상태가 아닙니다 — robot_bridge가 target_person_pose를 무시할 수 있습니다')
        self.following_index = 0
        self.following_timer = self.create_timer(interval, self._publish_next_following_pose)
        print(f'[추종시작] 간격초={interval}')

    def cmd_follow_stop(self, args: List[str]) -> None:
        if self.following_timer is None:
            return
        self._stop_following_timer()
        print('[추종중지]')

    def _publish_next_following_pose(self) -> None:
        if self.following_index >= len(FOLLOWING_MOCK_POSES):
            self._stop_following_timer()
            print('[추종완료] mock 좌표 10개 발행 종료')
            return
        x, y, yaw = FOLLOWING_MOCK_POSES[self.following_index]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self.target_pose_pub.publish(msg)
        print(f'[추종 {self.following_index + 1}/{len(FOLLOWING_MOCK_POSES)}] ({x}, {y}, {yaw:.3f})')
        self.following_index += 1

    def cmd_quit(self, args: List[str]):
        return QUIT

    def run_cli(self) -> None:
        handlers = {
            '호출': self.cmd_call,
            '목적지목록': self.cmd_list_destinations,
            '작업자감지': self.cmd_worker_detected,
            '추종시작': self.cmd_follow_start,
            '추종중지': self.cmd_follow_stop,
            '배송모드': self.cmd_deliver,
            '배송확인': self.cmd_confirm,
            '상태': self.cmd_status,
            '종료': self.cmd_quit,
        }
        print('webcam_pc_cli 준비 완료. 명령: 호출/목적지목록/작업자감지/추종시작/추종중지/배송모드/배송확인/상태/종료')
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
            try:
                if handler(args) is QUIT:
                    break
            except Exception as exc:
                print(f'[오류] 명령 처리 중 예외 발생: {exc}')


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
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
