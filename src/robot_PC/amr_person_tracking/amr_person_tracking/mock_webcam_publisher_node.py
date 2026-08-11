#!/usr/bin/env python3
"""
mock_webcam_publisher_node.py

[역할]
reid_tracking_node의 end-to-end 증거영상 확보용 테스트 전용 노드. 이 워크스페이스에는 실제
고정캠(웹캠) 로컬라이제이션 스트림이 없어서(별도 저장소 담당, docs/vision_pipeline.md 참고),
reid_tracking_node의 "웹캠 -> OAK-D -> 라이다" 전체 융합 경로를 실제로 돌려볼 방법이 없었다.

이 노드는 실제 라이다 다리검출 출력(leg_detector_bridge_node의 /vision/leg_detections_3d,
곡률필터+칼만 배경필터를 이미 통과한 실측 데이터)을 구독해, 작은 무작위 오프셋을 더하고
지연을 줘서 "같은 실제 사람을 보고 있지만 독립적인 출처"처럼 보이는 /vision/webcam/
detections_3d를 재발행한다. 실제 호모그래피/YOLO 웹캠 파이프라인을 대체하는 게 아니라,
reid_tracking_node의 게이팅/블렌딩/스왑검증 로직이 실제 메시지 왕복(rclpy pub/sub, QoS)
위에서 정상 동작하는지 확인하기 위한 배선 검증용 더미다 - 위치의 "정확도"를 검증하는 게
아니라 "메커니즘"을 검증하는 목적임을 분명히 해둔다.

[입력 토픽]
  - <namespace>/vision/leg_detections_3d (vision_msgs/Detection3DArray, leg_detector_bridge_node)

[출력 토픽]
  - <webcam_topic> (vision_msgs/Detection3DArray, frame_id=map_frame 파라미터와 동일)
"""
import random
from collections import deque

import rclpy
from rclpy.node import Node

from vision_msgs.msg import (
    Detection3D, Detection3DArray, ObjectHypothesis, ObjectHypothesisWithPose)

from amr_person_tracking.tracking_utils import extract_person_position


class MockWebcamPublisherNode(Node):

    def __init__(self):
        super().__init__('mock_webcam_publisher_node')

        self.declare_parameter('leg_detections_topic', '/robot5/vision/leg_detections_3d')
        self.declare_parameter('webcam_detections_topic', '/vision/webcam/detections_3d')
        self.declare_parameter('map_frame', 'odom')
        # 실제 독립 출처처럼 보이게 하는 요소 - 위치 오프셋(m)과 지연(메시지 개수 단위)
        self.declare_parameter('position_noise_std', 0.1)
        self.declare_parameter('delay_messages', 3)

        leg_topic = self.get_parameter('leg_detections_topic').value
        webcam_topic = self.get_parameter('webcam_detections_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.noise_std = self.get_parameter('position_noise_std').value
        self.delay_messages = self.get_parameter('delay_messages').value

        self._buffer = deque(maxlen=max(self.delay_messages + 1, 1))

        self.sub = self.create_subscription(
            Detection3DArray, leg_topic, self.on_leg_detections, 10)
        self.pub = self.create_publisher(Detection3DArray, webcam_topic, 10)

        self.get_logger().info(
            f'mock_webcam_publisher_node 시작 (테스트 전용) | {leg_topic} -> {webcam_topic} '
            f'(noise_std={self.noise_std}m, delay={self.delay_messages}msg)'
        )

    def on_leg_detections(self, msg: Detection3DArray):
        # 실제 사람 위치 하나만 대표로 뽑는다 - 웹캠은 넓은 FOV에서 한 명을 이미 추종 중이라고
        # 가정하는 시나리오라, 라이다 쪽 노이즈 후보를 전부 따라할 필요는 없다.
        position = None
        for det in msg.detections:
            pos = extract_person_position(det)
            if pos is not None:
                position = pos
                break

        self._buffer.append((position, msg.header.stamp))
        if len(self._buffer) < self._buffer.maxlen:
            return  # 지연 버퍼가 아직 안 채워짐

        delayed_position, delayed_stamp = self._buffer[0]
        if delayed_position is None:
            return  # 그 시점엔 라이다도 아무도 못 봤음 - 웹캠도 조용히 있는다고 근사

        x, y, z = delayed_position
        x += random.gauss(0.0, self.noise_std)
        y += random.gauss(0.0, self.noise_std)

        out = Detection3DArray()
        out.header.stamp = delayed_stamp
        out.header.frame_id = self.map_frame

        det = Detection3D()
        det.header.stamp = delayed_stamp
        det.header.frame_id = self.map_frame
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis = ObjectHypothesis(class_id='person', score=0.9)
        hyp.pose.pose.position.x = x
        hyp.pose.pose.position.y = y
        hyp.pose.pose.position.z = z
        hyp.pose.pose.orientation.w = 1.0
        det.results.append(hyp)
        det.bbox.center.position.x = x
        det.bbox.center.position.y = y
        det.bbox.center.orientation.w = 1.0
        out.detections.append(det)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MockWebcamPublisherNode()
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
