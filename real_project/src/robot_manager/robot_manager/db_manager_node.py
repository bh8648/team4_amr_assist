#!/usr/bin/env python3
import os
import sqlite3
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# 커스텀 메시지 import (패키지명: robot_status 기준)
from robot_status.msg import RobotAssignment, RobotError, RobotStatus, TaskState


class DbManagerNode(Node):
    VALID_ROBOTS = ('robot5', 'robot11')

    def __init__(self):
        super().__init__('db_manager_node')
        
        # 1. SQLite DB 연결 (외래키 제약조건만 활성화)
        default_db_path = os.path.abspath('amr.db')
        self.declare_parameter('db_path', os.environ.get('AMR_DB_PATH', default_db_path))
        self.db_path = os.path.abspath(os.path.expanduser(self.get_parameter('db_path').value))
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.init_db_settings()

        # 2. 로봇별 마지막 수신 시간 및 최신 메시지 저장용 딕셔너리
        self.last_msg_time = {}   # { robot_id(str): timestamp_sec }
        self.latest_status = {}   # { robot_id(str): msg_data }
        self.task_states = {}

        # 3-1. Robot Status Subscriber 설정 (상태 수신)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.status_subscriptions = [
            self.create_subscription(
                RobotStatus,
                f'/{robot_id}/robot_status',
                lambda msg, expected=robot_id: self.status_callback(expected, msg),
                qos_profile
            )
            for robot_id in self.VALID_ROBOTS
        ]
        self.assignment_subscription = self.create_subscription(RobotAssignment, '/robot_assignment', self.assignment_callback, 10)
        self.task_state_subscription = self.create_subscription(TaskState, '/task/state', self.task_state_callback, 10)

        # 3-2. Robot Error Subscriber 설정 (오류 발생 시 즉시 DB 저장)
        self.error_subscription = self.create_subscription(RobotError, '/robot_error', self.error_callback, 10)

        # 4. DB 주기적 저장 타이머 (1초 주기 - 상태 로그용)
        self.db_timer = self.create_timer(1.0, self.save_status_to_db)
        
        self.get_logger().info('DB Manager Node가 성공적으로 시작되었습니다.')

    def init_db_settings(self):
        """SQLite 외래키(FK) 제약조건 활성화"""
        cursor = self.conn.cursor()
        cursor.execute("PRAGMA foreign_keys = ON;")
        self.conn.commit()

    def status_callback(self, expected_robot_id, msg):
        """로봇 상태 메시지 수신 시 타임스탬프와 최신 데이터 갱신"""

        current_time = self.get_clock().now().nanoseconds / 1e9  # 초 단위 변환

        # msg.robot_id는 string ('robot5' 또는 'robot11')
        robot_id = str(msg.robot_id)
        if robot_id != expected_robot_id:
            self.get_logger().warning(f'토픽과 robot_id 불일치: expected={expected_robot_id}, received={robot_id}')
            return
        self.last_msg_time[robot_id] = current_time
        self.latest_status[robot_id] = msg

    def error_callback(self, msg):
        """로봇 오류 메시지 수신 시 error_logs 테이블에 즉시 저장"""
        cursor = self.conn.cursor()

        # task_id가 없거나 비어있으면 None(NULL) 매핑
        task_id = msg.task_id if getattr(msg, 'task_id', None) else None

        query = """
            INSERT INTO error_logs (robot_id, task_id, error_code)
            VALUES (?, ?, ?)
        """
        data = (msg.robot_id, task_id, msg.error_code)

        try:
            cursor.execute(query, data)
            self.conn.commit()  # 에러 로그는 즉시 DB 반영
            self.get_logger().warn(f'[{msg.robot_id}] 오류 기록 완료: {msg.error_code}')
        except sqlite3.Error as e:
            self.get_logger().error(f'[{msg.robot_id}] 에러 로그 DB 저장 실패: {e}')

    def assignment_callback(self, msg):
        """배정 결과를 tasks 테이블에 기록한다."""
        if not msg.assigned:
            self.get_logger().warn('가용 AMR이 없어 작업이 생성되지 않았습니다.')
            return

        assigned_at = datetime.fromtimestamp(msg.assigned_at.sec + msg.assigned_at.nanosec / 1e9).strftime('%Y-%m-%d %H:%M:%S')
        task_id = f'TASK_{msg.assigned_at.sec}_{msg.assigned_at.nanosec}'
        cursor = self.conn.cursor()
        destination = cursor.execute(
            """
            SELECT destination_id FROM destinations
            WHERE ABS(position_x - ?) < 0.0001
              AND ABS(position_y - ?) < 0.0001
            LIMIT 1
            """,
            (float(msg.target_x), float(msg.target_y))
        ).fetchone()
        destination_id = destination[0] if destination else None

        try:
            cursor.execute(
                """
                INSERT INTO tasks (
                    task_id, assigned_robot_id, destination_id, state,
                    created_at, started_at
                ) VALUES (?, ?, ?, 'ASSIGNED', ?, ?)
                ON CONFLICT(task_id) DO UPDATE SET assigned_robot_id=excluded.assigned_robot_id,
                    destination_id=COALESCE(excluded.destination_id, tasks.destination_id)
                """,
                (task_id, str(msg.robot_id), destination_id, assigned_at, assigned_at)
            )
            self.conn.commit()
            self.get_logger().info(f'작업 {task_id} 저장 완료: AMR {msg.robot_id}, 목표 ({msg.target_x:.2f}, {msg.target_y:.2f})')
        except sqlite3.Error as e:
            self.get_logger().error(f'배정 작업 DB 저장 실패: {e}')

    def task_state_callback(self, msg):
        """Task Manager의 상태 전환을 tasks 테이블에 저장한다."""
        robot_id = str(msg.robot_id)
        self.task_states[robot_id] = msg
        completed = msg.state in ('DOCKED', 'ERROR')
        try:
            self.conn.execute(
                """
                INSERT INTO tasks (task_id, assigned_robot_id, state, result, created_at, started_at, completed_at)
                VALUES (?, ?, ?, ?, datetime('now', 'localtime'), datetime('now', 'localtime'), CASE WHEN ? THEN datetime('now', 'localtime') END)
                ON CONFLICT(task_id) DO UPDATE SET assigned_robot_id=excluded.assigned_robot_id, state=excluded.state,
                    result=excluded.result, completed_at=CASE WHEN ? THEN datetime('now', 'localtime') ELSE tasks.completed_at END
                """,
                (msg.task_id, robot_id, msg.state, msg.detail or None, completed, completed)
            )
            self.conn.commit()
            self.get_logger().info(f'Task 상태 저장: {msg.task_id} · {robot_id} · {msg.state}')
        except sqlite3.Error as e:
            self.get_logger().error(f'Task 상태 DB 저장 실패: {e}')

    def save_status_to_db(self):
        """1초마다 호출: 5초 수신 여부로 ONLINE/OFFLINE 판별 후 Insert"""
        if not self.latest_status:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        cursor = self.conn.cursor()

        for robot_id, msg in self.latest_status.items():
            last_time = self.last_msg_time.get(robot_id, 0)
            time_diff = current_time - last_time

            # 수신 간격이 5초 이상이면 OFFLINE, 미만이면 ONLINE
            online_state = "ONLINE" if time_diff < 5.0 else "OFFLINE"

            # 작업 상태는 현재 메시지에 없으므로 기본 상태를 사용한다.
            # 연결 여부는 별도의 online 컬럼에 저장한다.
            task_state = self.task_states.get(str(robot_id))
            robot_state = task_state.state if task_state else "DOCKED"
            task_id = task_state.task_id if task_state and task_state.state not in ('DOCKED', 'ERROR') else None if task_state else msg.current_task_id or None

            query = """
                INSERT INTO robot_status_logs (
                    robot_id, online, state, current_task_id, battery, position_x, position_y, orientation_yaw
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """
            
            data = (robot_id, online_state, robot_state, task_id, float(msg.battery), float(msg.x), float(msg.y), float(msg.yaw))

            try:
                cursor.execute(query, data)
            except sqlite3.Error as e:
                self.get_logger().error(f'[{robot_id}] DB 저장 실패: {e}')

        self.conn.commit()

    def destroy_node(self):
        """노드 종료 시 DB 연결 닫기"""
        self.conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DbManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
