# call_position_nav_test.py
#
# 손동작 호출 파이프라인 전체(제스처 -> call_trigger -> call_position)를
# 실제 AMR 주행까지 엮어서 한 번 확인해보기 위한 테스트 노드. "시험용" -
# 반복 호출/에러 복구 같은 건 안 다루고, person/call_position이 올 때마다
# 그 좌표로 Nav2 목표를 보내고 로봇이 그 근처(기본 0.7m 이내)까지 오면
# 멈추는 것까지만 확인함.
#
# 사전 조건 (AMR PC에서):
#   - amcl(localization)과 nav2 bringup이 이미 실행 중이어야 함
#     (`ros2 launch nav2_bringup ...` 등 - 이 패키지가 하는 일이 아님)
#   - pose_locator_node가 다른 머신에서 돌면서 person/call_position을
#     publish할 수 있어야 함 (hand_gesture_caller의 wrist_gesture_node까지
#     같이 떠 있어야 실제로 제스처로 트리거됨)
#
# 사용법:
#   ros2 run person_locator call_position_nav_test
#
# 로봇이 사람 좌표 정중앙까지 파고들면 안 되므로, 목표를 사람의 정확한
# (x, y)로 보내되 Nav2가 주는 실시간 feedback.distance_remaining이
# stop_distance_m 이하로 떨어지는 순간 이 노드가 직접 cancelTask()를
# 호출해서 그 자리에서 세움. Nav2쪽 goal_checker 허용 오차를 건드리지
# 않고 이 노드 하나로 테스트할 수 있게 하려고 일부러 이렇게 함.

import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class CallPositionNavTest(BasicNavigator):

    def __init__(self):
        super().__init__(node_name='call_position_nav_test')

        self.declare_parameter('call_position_topic', 'person/call_position')
        # "0.7m까지 오는" 요구사항 - 남은 거리가 이 값 이하가 되면 그 자리에서 정지
        self.declare_parameter('stop_distance_m', 0.7)

        call_position_topic = self.get_parameter('call_position_topic').value
        self.stop_distance_m = self.get_parameter('stop_distance_m').value

        # 구독 콜백은 그냥 최신 좌표만 받아두고, 실제 주행 로직(goToPose +
        # isTaskComplete 폴링)은 main()의 메인 루프에서만 실행함. isTaskComplete가
        # 내부적으로 이 노드를 다시 spin하기 때문에, 만약 콜백 안에서 바로
        # 주행을 시작해버리면 "spin이 콜백을 실행하는 중에 그 콜백이 다시
        # spin을 호출"하는 중첩 spin이 돼서 꼬일 수 있음 - 그래서 이렇게 분리함
        self.pending_point = None

        self.create_subscription(
            PointStamped, call_position_topic, self._on_call_position, 10
        )

        self.get_logger().info(
            f'call_position_nav_test: "{call_position_topic}" 대기 중 '
            f'(목표 {self.stop_distance_m}m 이내로 오면 정지)'
        )

    def _on_call_position(self, msg):
        self.pending_point = msg

    def navigate_to_point(self, point_msg):
        goal = PoseStamped()
        goal.header.frame_id = point_msg.header.frame_id or 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = point_msg.point.x
        goal.pose.position.y = point_msg.point.y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0  # 방향은 이 테스트에서 신경 안 씀

        self.get_logger().info(
            f'call_position 수신 ({goal.pose.position.x:.2f}, '
            f'{goal.pose.position.y:.2f}) -> Nav2 목표 전송'
        )
        self.goToPose(goal)

        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback is not None and feedback.distance_remaining <= self.stop_distance_m:
                self.get_logger().info(
                    f'남은 거리 {feedback.distance_remaining:.2f}m '
                    f'(<= {self.stop_distance_m}m) -> 여기서 정지'
                )
                self.cancelTask()
                break

        result = self.getResult()
        if result == TaskResult.CANCELED:
            self.get_logger().info('의도한 대로 목표 근처에서 정지함 (CANCELED)')
        elif result == TaskResult.SUCCEEDED:
            self.get_logger().info('목표 지점까지 도달함 (SUCCEEDED)')
        else:
            self.get_logger().warn(f'주행 실패/중단: {result}')


def main(args=None):
    rclpy.init(args=args)
    node = CallPositionNavTest()
    node.waitUntilNav2Active()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pending_point is not None:
                point_msg = node.pending_point
                node.pending_point = None
                node.navigate_to_point(point_msg)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
