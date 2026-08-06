#!/usr/bin/env python3
"""
oakd_detector_node.py

[역할]
AMR(터틀봇4) 탑재 OAK-D PRO 카메라 기반 근접(약 1.5~2m 이하) 사람 검출 노드.
전체 비전 파이프라인 문서의 "4번 위치추정 - OAKD PRO" + 변경된 "OAK-D 컨버터 노드" 역할을
합친 노드로, 검출 자체(YOLO-pose 추론)부터 이 노드에서 직접 수행한다.

[처리 흐름]
RGB(compressed) + Depth(compressedDepth) + CameraInfo 구독
  -> YOLO-pose로 사람 bbox + 발끝(ankle) keypoint 추출
  -> 발끝 픽셀 좌표의 depth 값을 읽어 camera_info 기반 3D 역투영 (camera_link 기준)
  -> tf2로 base_link -> map 변환 (재식별/트래킹 노드가 고정캠 스트림과 동일 스키마로
     합류시킬 수 있도록, 프레임 변환까지 이 노드에서 끝내고 내보낸다)
  -> vision_msgs/Detection3DArray로 발행

[입력 토픽]
  - <namespace>/oakd/rgb/image_raw/compressed        (sensor_msgs/CompressedImage)
  - <namespace>/oakd/stereo/image_raw/compressedDepth (sensor_msgs/CompressedImage)
  - <namespace>/oakd/stereo/camera_info               (sensor_msgs/CameraInfo)

[출력 토픽]
  - <namespace>/vision/detections_3d (vision_msgs/Detection3DArray, frame_id=map)

[참고]
  - 하위 노드(재식별/트래킹, 예측 회피)는 출력 스키마(Detection3DArray)만 보고 동작하므로,
    검출 로직 내부 구현이 바뀌어도 이 노드 밖으로는 영향이 가지 않는다.
  - 로봇<->노트북 대역폭 절약을 위해 모든 입력 토픽은 compressed 포맷으로만 구독한다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import CompressedImage, CameraInfo
from vision_msgs.msg import Detection3DArray

import tf2_ros


class OakdDetectorNode(Node):

    def __init__(self):
        super().__init__('oakd_detector_node')

        self.declare_parameter('namespace', 'robot5')
        self.declare_parameter('rgb_topic', '/robot5/oakd/rgb/image_raw/compressed')
        self.declare_parameter('depth_topic', '/robot5/oakd/stereo/image_raw/compressedDepth')
        self.declare_parameter('camera_info_topic', '/robot5/oakd/stereo/camera_info')
        self.declare_parameter('detections_topic', '/robot5/vision/detections_3d')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('sync_queue_size', 10)
        # 근접 안전모드 전환 임계값 (m). 이 거리 이하로 접근 시 IR 근접센서 값과 함께 확인
        self.declare_parameter('min_z_safety_threshold', 0.5)

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        slop = self.get_parameter('sync_slop').value
        queue_size = self.get_parameter('sync_queue_size').value

        self.rgb_sub = Subscriber(self, CompressedImage, rgb_topic, qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, CompressedImage, depth_topic, qos_profile=qos_profile_sensor_data)
        self.camera_info_sub = Subscriber(self, CameraInfo, camera_info_topic, qos_profile=qos_profile_sensor_data)

        self.time_sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.camera_info_sub],
            queue_size=queue_size,
            slop=slop,
        )
        self.time_sync.registerCallback(self.sync_callback)

        self.detections_pub = self.create_publisher(Detection3DArray, detections_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # To-do: YOLO-pose 모델 로드 (가중치 경로 파라미터화, GPU/CPU 디바이스 선택)
        self.pose_model = None

        self.get_logger().info(
            f'oakd_detector_node 시작 | rgb={rgb_topic} depth={depth_topic} -> {detections_topic}'
        )

    def sync_callback(self, rgb_msg: CompressedImage, depth_msg: CompressedImage, camera_info_msg: CameraInfo):
        # To-do: rgb_msg(JPEG)를 cv2.imdecode로 BGR 이미지로 디코딩
        # To-do: depth_msg(compressedDepth, 앞 12바이트는 depthQuantA/B 헤더)를 PNG 디코딩해 16UC1 depth 이미지 획득
        # To-do: YOLO-pose로 프레임 내 모든 사람의 bbox + keypoint 추론, 발끝(ankle) keypoint 픽셀좌표 추출
        # To-do: 발끝 keypoint가 검출되지 않은 경우 등급 결정
        #        (각도만 추정 / 무릎 keypoint로 보정 / 발끝 직접 검출) -> 아래 3D 역투영 신뢰도 플래그로 기록
        # To-do: camera_info_msg.k(내부 파라미터)와 발끝 픽셀의 depth 값으로 camera_link 기준 3D 좌표 역투영
        # To-do: 역투영된 3D 좌표가 min_z_safety_threshold 이하로 접근한 경우, 추적 중인 방향에 해당하는
        #        IR 근접센서 value가 임계값을 넘는지 확인 후 근접 안전모드 전환 신호를 별도로 처리
        # To-do: tf_buffer.lookup_transform('map', <camera/base_link frame>, ...)으로 map 프레임 변환
        # To-do: 변환된 3D 좌표 + 신뢰도 플래그를 Detection3DArray(frame_id=map)로 채워 발행
        self.publish_detections([], rgb_msg.header)

    def publish_detections(self, detections, header):
        msg = Detection3DArray()
        msg.header = header
        msg.header.frame_id = self.get_parameter('map_frame').value
        msg.detections = detections
        self.detections_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OakdDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
