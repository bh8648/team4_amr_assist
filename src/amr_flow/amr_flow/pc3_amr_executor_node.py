#!/usr/bin/env python3

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String

try:
    from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
except Exception:  # pragma: no cover
    TurtleBot4Navigator = None


class Pc3AmrExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__('pc3_amr_executor_node')

        # ------------------------------------------------------------------
        # PC3 역할:
        # 1) PC2 중앙이 만든 최종 goal command를 받는다.
        # 2) robot_id가 자기 것인지 확인한다.
        # 3) 지금은 APPROACH 시나리오 위주로 Nav2 실행을 담당한다.
        # 4) 상태와 결과를 중앙으로 다시 올린다.
        #
        # 현재 구현은 AMR 실제 운용과 테스트 둘 다 고려했다.
        # - simulate_nav=true: 실제 로봇 없이 가상 완료
        # - simulate_nav=false: TurtleBot4Navigator로 실제 주행
        #
        # 추후 채워야 할 부분:
        # - FOLLOWING 모드 goal 갱신
        # - DELIVERY / RETURN / DOCK mode 분기
        # - timeout / blocked / recovery 감시
        # - dock / undock 하드웨어 액션 연동
        # ------------------------------------------------------------------
        self.declare_parameter('robot_id', 11)
        self.declare_parameter('goal_command_topic', '/amr_goal_command')
        self.declare_parameter('nav_status_topic', '/amr_nav_status')
        self.declare_parameter('nav_result_topic', '/amr_nav_result')
        self.declare_parameter('debug_goal_topic', '/amr_debug_goal')
        self.declare_parameter('amcl_pose_topic', '/robot11/amcl_pose')
        self.declare_parameter('odom_topic', '/robot11/odom')
        self.declare_parameter('loop_hz', 5.0)
        self.declare_parameter('simulate_nav', False)
        self.declare_parameter('simulate_goal_complete_sec', 3.0)
        self.declare_parameter('default_goal_yaw', 0.0)
        self.declare_parameter('use_initial_pose', False)
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.goal_command_topic = self.get_parameter('goal_command_topic').value
        self.nav_status_topic = self.get_parameter('nav_status_topic').value
        self.nav_result_topic = self.get_parameter('nav_result_topic').value
        self.debug_goal_topic = self.get_parameter('debug_goal_topic').value
        self.amcl_pose_topic = self.get_parameter('amcl_pose_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.loop_hz = float(self.get_parameter('loop_hz').value)
        self.simulate_nav = bool(self.get_parameter('simulate_nav').value)
        self.simulate_goal_complete_sec = float(self.get_parameter('simulate_goal_complete_sec').value)
        self.default_goal_yaw = float(self.get_parameter('default_goal_yaw').value)
        self.use_initial_pose = bool(self.get_parameter('use_initial_pose').value)
        self.initial_pose_x = float(self.get_parameter('initial_pose_x').value)
        self.initial_pose_y = float(self.get_parameter('initial_pose_y').value)
        self.initial_pose_yaw = float(self.get_parameter('initial_pose_yaw').value)

        self.status_pub = self.create_publisher(String, self.nav_status_topic, 10)
        self.result_pub = self.create_publisher(String, self.nav_result_topic, 10)
        self.debug_goal_pub = self.create_publisher(PoseStamped, self.debug_goal_topic, 10)

        self.create_subscription(String, self.goal_command_topic, self.goal_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose_topic, self.amcl_pose_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.navigator = None
        if not self.simulate_nav and TurtleBot4Navigator is not None:
            self.navigator = TurtleBot4Navigator()
            if self.use_initial_pose:
                self.navigator.setInitialPose(self.make_pose(self.initial_pose_x, self.initial_pose_y, self.initial_pose_yaw))
            self.navigator.waitUntilNav2Active()

        self.job_id = ''
        self.current_mode = 'IDLE'
        self.current_goal: PoseStamped | None = None
        self.goal_started_at = None
        self.active_goal = False
        self.robot_pose: PoseStamped | None = None
        self.odom_yaw = 0.0

        self.publish_nav_status('READY', 'pc3 executor ready')
        timer_period = 1.0 / max(self.loop_hz, 0.5)
        self.create_timer(timer_period, self.main_loop)

    def goal_callback(self, msg: String) -> None:
        # 현재 중앙-PC3 인터페이스는 JSON String이다.
        # 예:
        # {"robot_id":11,"job_id":"job-1","command":"start_goal","goal_mode":"APPROACH","goal_pose":{"x":1.0,"y":2.0,"yaw":0.0}}
        #
        # 이 함수는 "중앙이 무엇을 시켰는지"를 해석하는 입구다.
        # 추후 FOLLOWING/DELIVERY/RETURN이 붙어도 결국 이 함수에서
        # mode와 payload를 해석한 뒤 내부 실행기로 넘기게 된다.
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.publish_nav_result('failed', reason='invalid_goal_command_json')
            return

        if int(payload.get('robot_id', -1)) != self.robot_id:
            return

        self.job_id = payload.get('job_id', '')
        command = payload.get('command', 'start_goal')
        if command == 'cancel':
            self.cancel_current_goal()
            self.publish_nav_result('cancelled')
            return

        goal_mode = payload.get('goal_mode', 'APPROACH')
        goal_pose = self.pose_from_dict(payload.get('goal_pose', {}))
        if goal_pose is None:
            self.publish_nav_result('failed', reason='invalid_goal_pose')
            return

        self.current_mode = str(goal_mode).upper()
        self.current_goal = goal_pose
        self.start_goal_execution()

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        # map 기준 로봇 pose를 저장한다.
        # 현재 APPROACH 최소 시나리오에서는 직접 많이 안 쓰지만,
        # 추후에는 goal 도착 판단, safety check, following mode에서 중요해진다.
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.robot_pose = pose

    def odom_callback(self, msg: Odometry) -> None:
        # odom yaw는 빠르게 현재 heading을 참고하기 위한 최소 정보다.
        # 추후 blocked 판단이나 회전 보정이 들어갈 때 활용 가능하다.
        self.odom_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def start_goal_execution(self) -> None:
        # 이 함수는 "실제 이동 시작"의 시작점이다.
        # simulate_nav면 시간 기반 가상 완료로, 실기면 Nav2 액션으로 간다.
        # 지금은 mode별 실행 차이가 거의 없지만,
        # 나중에는 APPROACH/FOLLOWING/DELIVERY/RETURN 분기를 여기서 세분화할 수 있다.
        if self.current_goal is None:
            self.publish_nav_result('failed', reason='missing_goal')
            return

        if self.simulate_nav:
            self.goal_started_at = self.get_clock().now()
            self.active_goal = True
            self.debug_goal_pub.publish(self.current_goal)
            self.publish_nav_status('RUNNING', f'simulated {self.current_mode} started')
            self.publish_nav_result('goal_sent', goal_mode=self.current_mode)
            return

        self.send_nav_goal()

    def send_nav_goal(self) -> None:
        # 실제 TurtleBot4Navigator를 쓰는 경로다.
        # 지금은 startToPose 한 번으로 끝나지만,
        # following 모드가 붙으면 cancel + re-send 주기가 여기 근처에 추가된다.
        if self.navigator is None or self.current_goal is None:
            self.publish_nav_result('failed', reason='navigator_not_ready')
            return

        try:
            self.navigator.startToPose(self.current_goal)
            self.goal_started_at = self.get_clock().now()
            self.active_goal = True
            self.debug_goal_pub.publish(self.current_goal)
            self.publish_nav_status('RUNNING', f'{self.current_mode} goal sent to nav2')
            self.publish_nav_result('goal_sent', goal_mode=self.current_mode)
        except Exception:
            self.publish_nav_result('failed', reason='goal_send_failed')

    def cancel_current_goal(self) -> None:
        # 중앙 취소 명령이나 추후 안전정지 로직이 들어오면 이 함수가 공통 정지 경로가 된다.
        if self.navigator is not None and self.active_goal and hasattr(self.navigator, 'cancelTask'):
            self.navigator.cancelTask()
        self.active_goal = False
        self.goal_started_at = None
        self.current_goal = None
        self.publish_nav_status('CANCELLED', 'goal cancelled')

    def monitor_nav_result(self) -> None:
        # 현재 goal이 끝났는지 감시하는 루프성 함수다.
        # 지금은 simulated complete 또는 Nav2 task complete만 본다.
        # 추후 여기에:
        # - timeout
        # - stuck / blocked
        # - recovery 반복 횟수
        # - goal tolerance 재확인
        # 을 넣으면 된다.
        if not self.active_goal or self.goal_started_at is None:
            return

        if self.simulate_nav:
            elapsed = self.get_clock().now() - self.goal_started_at
            if elapsed >= Duration(seconds=self.simulate_goal_complete_sec):
                self.active_goal = False
                self.goal_started_at = None
                self.publish_nav_status('DONE', f'simulated {self.current_mode} complete')
                self.publish_nav_result('goal_reached', goal_mode=self.current_mode)
            return

        if self.navigator is not None and hasattr(self.navigator, 'isTaskComplete') and self.navigator.isTaskComplete():
            self.active_goal = False
            self.goal_started_at = None
            self.publish_nav_status('DONE', f'{self.current_mode} complete')
            self.publish_nav_result('goal_reached', goal_mode=self.current_mode)

    def main_loop(self) -> None:
        # executor 메인 루프는 일단 결과 감시에 집중한다.
        # mode가 복잡해지면 주기적 health check와 state publish가 여기에 늘어날 수 있다.
        self.monitor_nav_result()

    def publish_nav_status(self, state: str, detail: str) -> None:
        # 상태 토픽은 "현재 진행 중인 상태"를 계속 알리는 용도다.
        # 중앙은 result보다 status를 더 자주 참고해서 HMI를 갱신하게 된다.
        payload = {
            'robot_id': self.robot_id,
            'job_id': self.job_id,
            'state': state,
            'detail': detail,
            'goal_mode': self.current_mode,
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))

    def publish_nav_result(self, result: str, **extra) -> None:
        # result는 이벤트성 응답이다.
        # goal_sent / goal_reached / failed / cancelled 같은 확정 결과를 올린다.
        # 추후 error_code, blocked_reason, dock_state를 extra 필드로 실어 보낼 수 있다.
        payload = {
            'robot_id': self.robot_id,
            'job_id': self.job_id,
            'result': result,
            'goal_mode': self.current_mode,
        }
        payload.update(extra)
        self.result_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))

    def pose_from_dict(self, data: dict) -> PoseStamped | None:
        # 중앙이 JSON으로 준 goal_pose를 ROS PoseStamped로 바꾸는 어댑터다.
        # 인터페이스 형식을 바꾸면 이 함수만 수정하면 되도록 분리해둔다.
        try:
            x = float(data['x'])
            y = float(data['y'])
            yaw = float(data.get('yaw', self.default_goal_yaw))
        except (KeyError, TypeError, ValueError):
            return None
        return self.make_pose(x, y, yaw)

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.yaw_to_quaternion(yaw)
        return pose

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
    node = Pc3AmrExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
