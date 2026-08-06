#!/usr/bin/env python3

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import String


class Pc2GoalCoordinatorNode(Node):
    def __init__(self) -> None:
        super().__init__('pc2_goal_coordinator_node')

        # ------------------------------------------------------------------
        # PC2 역할:
        # 1) PC1이 만든 worker map pose를 받는다.
        # 2) 현재는 단순 규칙으로 AMR 하나를 선택한다.
        # 3) goal_mode=APPROACH 로 최종 goal command를 만든다.
        # 4) PC3 executor가 읽는 /amr_goal_command로 보낸다.
        #
        # 현재는 "작업자 위치 이동" 시나리오만 목표라서 단순화했다.
        # 즉 배정 알고리즘은 robot_id 고정 또는 최소 상태 확인 수준이다.
        #
        # 추후 채워야 할 부분:
        # - 여러 AMR 상태 비교 배정
        # - worker pose 유효성/벽 근접도 검사
        # - 배송/복귀 mode 전환
        # - 작업 요청/취소/재배정 상태머신
        # ------------------------------------------------------------------
        self.declare_parameter('worker_pose_topic', '/pc1/worker_pose')
        self.declare_parameter('robot_status_topic', '/robot_status')
        self.declare_parameter('amr_goal_command_topic', '/amr_goal_command')
        self.declare_parameter('nav_result_topic', '/amr_nav_result')
        self.declare_parameter('status_topic', '/pc2/status')
        self.declare_parameter('loop_hz', 2.0)
        self.declare_parameter('assigned_robot_id', 11)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('approach_offset_m', 0.0)
        self.declare_parameter('default_goal_yaw', 0.0)

        self.worker_pose_topic = self.get_parameter('worker_pose_topic').value
        self.robot_status_topic = self.get_parameter('robot_status_topic').value
        self.amr_goal_command_topic = self.get_parameter('amr_goal_command_topic').value
        self.nav_result_topic = self.get_parameter('nav_result_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.loop_hz = float(self.get_parameter('loop_hz').value)
        self.assigned_robot_id = int(self.get_parameter('assigned_robot_id').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.approach_offset_m = float(self.get_parameter('approach_offset_m').value)
        self.default_goal_yaw = float(self.get_parameter('default_goal_yaw').value)

        self.goal_pub = self.create_publisher(String, self.amr_goal_command_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(PoseStamped, self.worker_pose_topic, self.worker_pose_callback, 10)
        self.create_subscription(String, self.robot_status_topic, self.robot_status_callback, 10)
        self.create_subscription(String, self.nav_result_topic, self.nav_result_callback, 10)

        self.latest_worker_pose: PoseStamped | None = None
        self.latest_robot_status = None
        self.current_job_id = ''
        self.active_job = False
        self.last_goal_sent = None

        self.publish_status('READY', 'pc2 coordinator ready')
        timer_period = 1.0 / max(self.loop_hz, 0.5)
        self.create_timer(timer_period, self.main_loop)

    def worker_pose_callback(self, msg: PoseStamped) -> None:
        # PC1에서 계산한 최신 worker map pose를 저장한다.
        # 현재는 auto_start=true면 pose가 한 번 들어오는 즉시 job_id를 만들고,
        # main_loop가 다음 주기에 바로 APPROACH goal을 보내는 구조다.
        #
        # 추후에는 여기서:
        # - 작업 요청 승인 여부
        # - 같은 작업자 pose의 중복 처리
        # - 작업자 호출 유효 시간
        # 을 같이 판단할 수 있다.
        self.latest_worker_pose = msg
        if self.auto_start and not self.active_job:
            self.current_job_id = f'job-{self.get_clock().now().nanoseconds}'
            self.publish_status('WORKER_POSE_RECEIVED', 'worker pose received from pc1')

    def robot_status_callback(self, msg: String) -> None:
        # 지금은 상태를 깊게 활용하지 않고 "향후 배정 로직 자리"만 잡아둔다.
        # 나중에는 배터리, 위치, 고장 상태, busy 여부를 여기서 읽어
        # 어느 AMR에 보낼지 점수화하게 된다.
        try:
            self.latest_robot_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.publish_status('WARN', 'robot_status json decode failed')

    def nav_result_callback(self, msg: String) -> None:
        # executor가 올린 결과를 받아 현재 job 종료 여부를 판단한다.
        # 지금은 성공/실패만 보지만, 나중에는:
        # - blocked
        # - timeout
        # - cancelled
        # - dock_required
        # 같은 세부 코드로 분기하게 된다.
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.publish_status('WARN', 'nav_result json decode failed')
            return

        if int(payload.get('robot_id', -1)) != self.assigned_robot_id:
            return

        result = payload.get('result', '')
        if result == 'goal_reached':
            self.active_job = False
            self.publish_status('DONE', 'worker approach complete')
        elif result == 'failed':
            self.active_job = False
            self.publish_status('FAILED', f'worker approach failed: {payload.get("reason", "")}')

    def main_loop(self) -> None:
        # 중앙 PC2의 최소 상태머신 역할을 하는 루프다.
        # 현재 규칙은 매우 단순하다:
        # - active_job이면 아무것도 안 함
        # - worker pose가 있으면 APPROACH goal 생성
        #
        # 추후 이 루프에서 가장 많이 늘어날 부분:
        # - 다중 로봇 배정
        # - 배송/복귀 전이
        # - 작업 취소 / 재배정
        # - 안전 접근점 검증
        if self.active_job:
            return

        if self.latest_worker_pose is None:
            return

        goal_pose = self.compute_approach_goal(self.latest_worker_pose)
        self.publish_goal_command(goal_pose)
        self.active_job = True

    def compute_approach_goal(self, worker_pose: PoseStamped) -> PoseStamped:
        # 지금은 worker pose를 그대로 goal로 쓰거나, 단순 offset만 적용한다.
        # 추후에는 벽과의 거리, 점유맵, 접근각 정책이 여기 들어간다.
        yaw = self.default_goal_yaw
        if self.approach_offset_m <= 1e-6:
            goal = PoseStamped()
            goal.header = worker_pose.header
            goal.pose = worker_pose.pose
            if self.is_zero_orientation(goal):
                goal.pose.orientation = self.yaw_to_quaternion(yaw)
            return goal

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = worker_pose.pose.position.x - math.cos(yaw) * self.approach_offset_m
        goal.pose.position.y = worker_pose.pose.position.y - math.sin(yaw) * self.approach_offset_m
        goal.pose.position.z = 0.0
        goal.pose.orientation = self.yaw_to_quaternion(yaw)
        return goal

    def publish_goal_command(self, goal_pose: PoseStamped) -> None:
        # 현재 PC2 -> PC3 인터페이스는 JSON String이다.
        # 나중에 커스텀 msg로 바꿀 수는 있지만,
        # 지금은 빠르게 디버깅하려고 사람이 읽기 쉬운 JSON으로 유지한다.
        payload = {
            'robot_id': self.assigned_robot_id,
            'job_id': self.current_job_id or f'job-{self.get_clock().now().nanoseconds}',
            'command': 'start_goal',
            'goal_mode': 'APPROACH',
            'goal_pose': {
                'x': goal_pose.pose.position.x,
                'y': goal_pose.pose.position.y,
                'yaw': self.quaternion_to_yaw(goal_pose.pose.orientation),
            },
        }
        self.goal_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))
        self.last_goal_sent = payload
        self.publish_status('GOAL_SENT', f'goal sent to robot {self.assigned_robot_id}')

    def publish_status(self, state: str, detail: str) -> None:
        # 중앙 상태는 HMI/DB/디버깅에서 가장 먼저 보게 되는 토픽이라
        # 최소한 job_id와 assigned_robot_id는 항상 같이 넣어둔다.
        payload = {
            'state': state,
            'detail': detail,
            'assigned_robot_id': self.assigned_robot_id,
            'job_id': self.current_job_id,
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))

    def is_zero_orientation(self, pose: PoseStamped) -> bool:
        return (
            pose.pose.orientation.x == 0.0
            and pose.pose.orientation.y == 0.0
            and pose.pose.orientation.z == 0.0
            and pose.pose.orientation.w == 0.0
        )

    def quaternion_to_yaw(self, quat: Quaternion) -> float:
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Pc2GoalCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
