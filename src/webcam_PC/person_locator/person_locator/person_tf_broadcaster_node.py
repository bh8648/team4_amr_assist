# person_tf_broadcaster_node.py
#
# 파이프라인의 마지막 단계: pose_locator_node가 publish한 사람의 map 프레임
# (x, y)를 실제 ROS2 TF로 바꿔줌. 이렇게 해두면 AMR/Nav2 쪽(다른 팀원 담당)이
# 이 패키지의 커스텀 토픽을 알 필요 없이, 평범한 tf2 lookup으로
# (예: tf_buffer.lookup_transform('map', 'person', ...)) "지금 사람이
# 어디 있는지"를 바로 알 수 있음.
#
# pose_locator_node에 합쳐 넣지 않고 일부러 이렇게 작은 별도 노드로
# 분리해둔 이유: 무거운 YOLO 추론과 다른 머신에서 돌 수 있게 하기 위함 -
# 예를 들어 Turtlebot4를 실제로 구동하는 PC에서 Nav2 바로 옆에 띄울 수 있음.
# 이 노드는 작은 PointStamped 메시지 하나만 받아 처리하면 되기 때문에 가능함.
#
# 패턴은 rtabmap_ros의 transform_to_tf.py("subscribe -> sendTransform" 구조를
# 다른 입력 메시지 타입에 대해 똑같이 하는 코드)를 참고해서 확인함.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class PersonTfBroadcasterNode(Node):

    def __init__(self):
        super().__init__('person_tf_broadcaster_node')

        # --- 파라미터 ---
        self.declare_parameter('input_topic', 'person/position')
        # 부모 프레임 - pose_locator_node가 PointStamped에 찍는 frame_id('map')와
        # 반드시 일치해야 함
        self.declare_parameter('parent_frame_id', 'map')
        # 이 노드가 만드는 TF 프레임 이름. AMR/Nav2 코드가 이 이름으로
        # lookup함, 예: `ros2 run tf2_ros tf2_echo map person`
        self.declare_parameter('child_frame_id', 'person')

        input_topic = self.get_parameter('input_topic').value
        self.parent_frame_id = self.get_parameter('parent_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        # TransformBroadcaster가 tf2가 기대하는 포맷으로 전역 /tf 토픽에
        # publish하는 번거로운 부분을 알아서 처리해줌
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            PointStamped,
            input_topic,
            self.point_callback,
            10,
        )

        self.get_logger().info(
            f'person_tf_broadcaster_node: "{input_topic}" -> TF '
            f'"{self.parent_frame_id}" -> "{self.child_frame_id}"'
        )

    def point_callback(self, point_msg):
        # TF 메시지를 구성함. TF는 6자유도 전체(이동 + 회전)를 표현하는 게
        # 원칙이지만, 우리가 가진 건 2D 지면 위치 하나뿐이므로 z = 0으로
        # 두고 회전은 identity quaternion으로 남겨둠 (점 하나만으로는
        # 사람이 "어느 방향을 보고 있는지" 알 수 없으므로)
        t = TransformStamped()

        # point_msg의 stamp를 그대로 쓰는 대신 "지금"으로 다시 찍음 - 이렇게
        # 하면 상위 메시지가 약간 순서가 뒤섞여 도착하더라도 tf2의
        # 보간/외삽 로직이 항상 최신의, 단조 증가하는 시간을 보게 됨
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.child_frame_id

        t.transform.translation.x = point_msg.point.x
        t.transform.translation.y = point_msg.point.y
        t.transform.translation.z = 0.0

        # identity quaternion (x=0, y=0, z=0, w=1) = "회전 없음"
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PersonTfBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
