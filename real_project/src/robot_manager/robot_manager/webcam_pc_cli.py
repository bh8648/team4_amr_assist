#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from robot_status.msg import RobotError, TaskState

from robot_manager.webcam_pc_cli_utils import (
    FOLLOWING_MOCK_POSES,
    parse_command,
    parse_interval,
)

QUIT = object()


class WebcamPcCliNode(Node):
    def __init__(self):
        super().__init__('webcam_pc_cli_node')

        self.task_cache = {}

        self.target_pose_pub = self.create_publisher(PoseStamped, '/robot11/target_person_pose', 10)

        self.following_timer = None
        self.following_index = 0

        self.task_state_sub = self.create_subscription(
            TaskState, '/task/state', self.task_state_callback, 10)
        self.error_sub = self.create_subscription(
            RobotError, '/robot_error', self.error_callback, 10)

        self.get_logger().info('webcam_pc_cli 노드 시작 (FOLLOWING mock 전용 — 호출/작업자감지/배송은 실물 웹캠 PC와 HMI가 담당)')

    def task_state_callback(self, msg: TaskState) -> None:
        self.task_cache[msg.robot_id] = msg

    def error_callback(self, msg: RobotError) -> None:
        print(f'[오류] robot_id={msg.robot_id}, task_id={msg.task_id}, error_code={msg.error_code}')

    def cmd_status(self, args) -> None:
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

    def cmd_follow_start(self, args) -> None:
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

    def cmd_follow_stop(self, args) -> None:
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

    def cmd_quit(self, args):
        return QUIT

    def run_cli(self) -> None:
        handlers = {
            '추종시작': self.cmd_follow_start,
            '추종중지': self.cmd_follow_stop,
            '상태': self.cmd_status,
            '종료': self.cmd_quit,
        }
        print('webcam_pc_cli 준비 완료 (FOLLOWING mock 전용). 명령: 추종시작/추종중지/상태/종료')
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
