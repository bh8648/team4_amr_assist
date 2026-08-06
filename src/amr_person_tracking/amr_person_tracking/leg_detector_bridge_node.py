#!/usr/bin/env python3
"""
leg_detector_bridge_node.py

[역할]
터틀봇4는 카메라 장착 높이가 낮아, 사람에게 가까이 접근하면(초근접 구간) OAK-D RGB/Depth
프레임에 사람의 다리만 잡히는 경우가 많다. 이 구간에서는 YOLO-pose 기반 발끝 검출보다 2D 라이다
다리(leg) 검출이 더 신뢰도가 높다 — RPLIDAR 스캔 평면 자체가 다리 높이와 맞아떨어지고, 라이다는
range+bearing(거리+각도)을 직접 측정하는 센서라 호모그래피나 depth 역투영 같은 변환이 필요 없기
때문이다.

이 노드는 라이다 다리검출 패키지(예: mowito/ros2_leg_detector — 원래 ROS2 Foxy 기준으로 검증된
패키지이며, 이 워크스페이스가 쓰는 ROS2 배포판에서 그대로 빌드되는지는 실제 설치 시 별도 확인이
필요하다)의 출력을 oakd_detector_node와 동일한 스키마(vision_msgs/Detection3DArray)로 변환해
재식별/트래킹 노드에 편입시키는 얇은 컨버터다. 다리쌍의 신원(누구인지)은 이 노드가 판단하지
않는다 — 위치/속도와 라이다 쪽 트랙 ID만 실어 보내고, 신원 매칭(웹캠이 추종 중이던 타겟과의
대조)은 reid_tracking_node가 담당한다.

[입력 토픽]
  - <namespace>/people_tracked (라이다 다리검출 패키지의 커스텀 메시지 - 정확한 메시지
    패키지/타입은 실제 설치 후 확인 필요. 원본 프로젝트 기준 다리쌍의 위치+속도+트랙 ID를 담은
    PersonArray류로 추정)

[출력 토픽]
  - <namespace>/vision/leg_detections_3d (vision_msgs/Detection3DArray, frame_id=map)

[참고]
  - 입력 메시지 타입이 아직 미확정이므로, 이 스켈레톤에서는 subscription 생성 자체를 To-do로
    남겨둔다 (존재 여부가 불확실한 메시지 패키지를 import하지 않기 위함). 실제 ros2_leg_detector를
    설치한 뒤 메시지 타입을 확인하고 import + subscription을 채워 넣으면 된다.
"""

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection3DArray

import tf2_ros


class LegDetectorBridgeNode(Node):

    def __init__(self):
        super().__init__('leg_detector_bridge_node')

        self.declare_parameter('people_tracked_topic', '/robot5/people_tracked')
        self.declare_parameter('leg_detections_topic', '/robot5/vision/leg_detections_3d')
        self.declare_parameter('map_frame', 'map')

        self.people_tracked_topic = self.get_parameter('people_tracked_topic').value
        leg_detections_topic = self.get_parameter('leg_detections_topic').value

        self.leg_detections_pub = self.create_publisher(Detection3DArray, leg_detections_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # To-do: 라이다 다리검출 패키지(예: ros2_leg_detector) 설치 후 실제 메시지 타입을 확인해
        #        아래와 같이 subscription을 생성한다.
        #        예) from <실제_메시지_패키지>.msg import PersonArray
        #            self.people_tracked_sub = self.create_subscription(
        #                PersonArray, self.people_tracked_topic, self.people_tracked_callback, 10)
        self.people_tracked_sub = None

        self.get_logger().info(
            f'leg_detector_bridge_node 시작 | {self.people_tracked_topic} -> {leg_detections_topic} '
            f'(입력 subscription은 메시지 타입 확인 후 연결 필요)'
        )

    def people_tracked_callback(self, msg):
        # To-do: msg에서 다리쌍 클러스터별 위치(x, y) + 속도(vx, vy) + 라이다 트랙 ID 파싱
        # To-do: tf_buffer.lookup_transform('map', <라이다 프레임>, ...)으로 map 프레임 변환
        #        (라이다는 range+bearing을 직접 측정하므로 depth 역투영/호모그래피 불필요,
        #         프레임 변환만 하면 됨)
        # To-do: 다리쌍마다 Detection3D를 만들어 id 필드에 라이다 트랙 ID를 담고
        #        Detection3DArray(frame_id=map)로 채워 발행
        self.publish_leg_detections([], msg.header)

    def publish_leg_detections(self, detections, header):
        out_msg = Detection3DArray()
        out_msg.header = header
        out_msg.header.frame_id = self.get_parameter('map_frame').value
        out_msg.detections = detections
        self.leg_detections_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LegDetectorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
