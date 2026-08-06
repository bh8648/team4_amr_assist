#!/usr/bin/env python3
"""
leg_detector_bridge_node.py

[역할]
터틀봇4는 카메라 장착 높이가 낮아, 사람에게 가까이 접근하면(초근접 구간) OAK-D RGB/Depth
프레임에 사람의 다리만 잡히는 경우가 많다. 이 구간에서는 YOLO-pose 기반 발끝 검출보다 2D 라이다
다리(leg) 검출이 더 신뢰도가 높다 — RPLIDAR 스캔 평면 자체가 다리 높이와 맞아떨어지고, 라이다는
range+bearing(거리+각도)을 직접 측정하는 센서라 호모그래피나 depth 역투영 같은 변환이 필요 없기
때문이다.

이 노드는 라이다 다리검출 패키지(mowito/ros2_leg_detector, https://github.com/mowito/ros2_leg_detector)의
출력을 oakd_detector_node와 동일한 스키마(vision_msgs/Detection3DArray)로 변환해 재식별/트래킹
노드에 편입시키는 얇은 컨버터다. 다리쌍의 신원(누구인지)은 이 노드가 판단하지 않는다 — 위치와
라이다 쪽 트랙 ID만 실어 보내고, 신원 매칭(웹캠이 추종 중이던 타겟과의 대조)은 reid_tracking_node가
담당한다.

[메시지 타입 - GitHub 소스로 확인 완료, 실제 설치/빌드는 별도]
ros2_leg_detector는 rosdep/apt 배포가 없는 소스 전용 패키지라 이 노드를 실행하려면
`leg_detector_msgs`, `leg_detector` 두 패키지를 클론해 워크스페이스에서 직접 빌드해야 한다.
- 원본은 ROS2 Foxy 기준으로 작성됨 (이 워크스페이스는 Humble) - 빌드 호환 여부 미검증
- package.xml에 OpenCV 3.4.12를 명시 의존 - 이 워크스페이스의 opencv-python(ultralytics용)과
  버전이 다를 수 있어 실제 설치 시 충돌 확인 필요

`leg_detector_msgs/msg/PersonArray` (경로: src/leg_detector_msgs/msg/PersonArray.msg):
    std_msgs/Header header
    Person[] people
`leg_detector_msgs/msg/Person`:
    geometry_msgs/Pose pose
    uint32 id

주의 - Person에는 속도도, 신뢰도(confidence)도 없다:
  - 속도: 이 노드는 속도를 계산해 싣지 않는다. Detection3D에는 애초에 속도 필드가 없고,
    reid_tracking_node가 문서 7번의 락온 상태머신에서 라이다 트랙 id별 위치 이력을 어차피
    여러 프레임 버퍼링해야 하므로, 그 이력에서 직접 미분해 속도를 구하는 편이 이 노드가
    구한 속도를 다시 신뢰도 필드에 욱여넣어 넘기는 것보다 자연스럽다.
  - 신뢰도: leg_detector_msgs/Leg에는 confidence가 있지만 최종 산출물인 Person에는 없다
    (트래커 내부에서 이미 필터링됐다는 전제). 아래 leg_detection_score 파라미터로 고정값을 싣는다.

publish_people_frame(사실상 header.frame_id)이 라이다 원본 트래커의 fixed_frame 파라미터
기본값("laser")을 그대로 따르는 구성일 수도, 이미 map/odom으로 변환돼 나오는 구성일 수도 있어
프레임을 가정하지 않는다 - oakd_detector_node와 동일하게 매번 tf2로 map 변환한다.

[입력 토픽]
  - <namespace>/people_tracked (leg_detector_msgs/msg/PersonArray)

[출력 토픽]
  - <namespace>/vision/leg_detections_3d (vision_msgs/Detection3DArray, frame_id=map)
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesis, ObjectHypothesisWithPose

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (PointStamped tf2 변환 등록)


class LegDetectorBridgeNode(Node):

    def __init__(self):
        super().__init__('leg_detector_bridge_node')

        self.declare_parameter('people_tracked_topic', '/robot5/people_tracked')
        self.declare_parameter('leg_detections_topic', '/robot5/vision/leg_detections_3d')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tf_timeout', 0.2)
        # Person.msg에는 신뢰도가 없어 고정값을 싣는다 - 실제 검출 신뢰도가 아니라
        # "라이다 다리검출 출처"라는 표시에 가깝다.
        self.declare_parameter('leg_detection_score', 0.7)

        people_tracked_topic = self.get_parameter('people_tracked_topic').value
        leg_detections_topic = self.get_parameter('leg_detections_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout').value)
        self.leg_detection_score = self.get_parameter('leg_detection_score').value

        self.leg_detections_pub = self.create_publisher(Detection3DArray, leg_detections_topic, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # leg_detector_msgs는 rosdep/apt 배포가 없어 ros2_leg_detector를 워크스페이스에
        # 직접 빌드해 넣기 전까지는 이 import가 실패한다 - 이 노드는 그 패키지 없이는
        # 원천적으로 동작할 수 없으므로(선택 기능이 아님) 조용히 넘어가지 않고 그대로 터뜨린다.
        from leg_detector_msgs.msg import PersonArray

        self.people_tracked_sub = self.create_subscription(
            PersonArray, people_tracked_topic, self.people_tracked_callback, 10)

        self.get_logger().info(
            f'leg_detector_bridge_node 시작 | {people_tracked_topic} -> {leg_detections_topic}'
        )

    def people_tracked_callback(self, msg):
        detections = []
        for person in msg.people:
            pt = PointStamped()
            pt.header.frame_id = msg.header.frame_id
            pt.header.stamp = msg.header.stamp
            pt.point = person.pose.position

            try:
                pt_map = self.tf_buffer.transform(pt, self.map_frame, timeout=self.tf_timeout)
            except Exception as exc:
                self.get_logger().warn(
                    f'{self.map_frame} 변환 실패: {exc}', throttle_duration_sec=2.0)
                continue

            detections.append(self._make_detection3d(pt_map, person.id, msg.header))

        self.publish_leg_detections(detections, msg.header)

    def _make_detection3d(self, pt_map, track_id, header):
        det = Detection3D()
        det.header = header
        det.header.frame_id = self.map_frame

        primary = ObjectHypothesisWithPose()
        primary.hypothesis = ObjectHypothesis(class_id='person', score=self.leg_detection_score)
        primary.pose.pose.position = pt_map.point
        primary.pose.pose.orientation.w = 1.0
        det.results.append(primary)

        source = ObjectHypothesisWithPose()
        source.hypothesis = ObjectHypothesis(class_id='source_lidar_leg', score=1.0)
        source.pose.pose.position = pt_map.point
        source.pose.pose.orientation.w = 1.0
        det.results.append(source)

        det.bbox.center.position = pt_map.point
        det.bbox.center.orientation.w = 1.0

        # 라이다 트래커 자체의 트랙 ID (재시작 시 리셋될 수 있음). 재식별 노드가 map 기준
        # 지속 트랙 ID로 덮어쓴다 - oakd_detector_node의 'oakd_<id>'와 동일한 관례.
        det.id = f'leg_{int(track_id)}'
        return det

    def publish_leg_detections(self, detections, header):
        out_msg = Detection3DArray()
        out_msg.header = header
        out_msg.header.frame_id = self.map_frame
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
