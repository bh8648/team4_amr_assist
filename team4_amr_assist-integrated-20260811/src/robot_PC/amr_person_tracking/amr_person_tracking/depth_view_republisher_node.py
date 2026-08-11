#!/usr/bin/env python3
"""
depth_view_republisher_node.py

[역할]
RViz의 기본 Image 디스플레이는 표준 image_transport 플러그인으로 "compressed"를 알아서
디코딩할 수 있지만, 이 로봇의 OAK-D compressedDepth 스트림은 양자화 미적용 커스텀 포맷이라
(vision_utils.decode_compressed_depth 참고) 표준 compressed_depth_image_transport 플러그인이
올바르게 디코딩한다는 보장이 없다. 이미 검증된 우리 디코더로 직접 디코딩+컬러맵 입혀서 평범한
bgr8 jpeg CompressedImage로 재발행한다 - RViz가 특수 처리 없이 그냥 보여줄 수 있다.

증거영상(RGB/Depth 교차검증) 녹화용으로만 쓰고, 평소 파이프라인 동작에는 필요 없다
(launch의 publish_depth_view가 true일 때만 조건부로 뜬다).

[입력 토픽] <namespace>/oakd/stereo/image_raw/compressedDepth (sensor_msgs/CompressedImage)
[출력 토픽] <namespace>/oakd/stereo/depth_view/compressed (sensor_msgs/CompressedImage, bgr8 jpeg)
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage

from amr_person_tracking.vision_utils import decode_compressed_depth


class DepthViewRepublisherNode(Node):

    def __init__(self):
        super().__init__('depth_view_republisher_node')

        self.declare_parameter('depth_topic', '/robot5/oakd/stereo/image_raw/compressedDepth')
        self.declare_parameter('depth_view_topic', '/robot5/oakd/stereo/depth_view/compressed')

        depth_topic = self.get_parameter('depth_topic').value
        depth_view_topic = self.get_parameter('depth_view_topic').value

        self.pub = self.create_publisher(CompressedImage, depth_view_topic, qos_profile_sensor_data)
        self.create_subscription(
            CompressedImage, depth_topic, self.on_depth, qos_profile_sensor_data)

        self.get_logger().info(f'depth_view_republisher_node 시작 | {depth_topic} -> {depth_view_topic}')

    def on_depth(self, msg: CompressedImage):
        depth = decode_compressed_depth(msg.data)
        if depth is None:
            return
        vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        colorized = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
        ok, jpg = cv2.imencode('.jpg', colorized)
        if not ok:
            return

        out = CompressedImage()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.format = 'jpeg'
        out.data = jpg.tobytes()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthViewRepublisherNode()
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
