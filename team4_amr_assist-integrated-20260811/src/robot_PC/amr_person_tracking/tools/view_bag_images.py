#!/usr/bin/env python3
"""
rgb(compressed)/stereo(compressedDepth) 원본 토픽을 파이프라인 없이 직접 구독해서
나란히 띄우는 진단용 스크립트. oakd_detector_node/vision_utils.decode_compressed_depth와
동일한 디코딩 로직을 쓴다 (이 로봇의 compressedDepth는 양자화 미적용, 12바이트 헤더 스킵).

사용법:
  python3 view_bag_images.py \
      --rgb /robot5/oakd/rgb/image_raw/compressed \
      --depth /robot5/oakd/stereo/image_raw/compressedDepth
"""
import argparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from amr_person_tracking.vision_utils import decode_compressed_depth


class BagImageViewer(Node):
    def __init__(self, rgb_topic, depth_topic):
        super().__init__('bag_image_viewer_debug')
        self.rgb_frame = None
        self.depth_frame = None
        self.rgb_count = 0
        self.depth_count = 0
        self.last_rgb_stamp = None
        self.last_depth_stamp = None

        self.create_subscription(CompressedImage, rgb_topic, self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, depth_topic, self.on_depth, qos_profile_sensor_data)

        cv2.namedWindow('rgb | depth', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('rgb | depth', 1400, 700)

        self.create_timer(0.03, self.pump)
        self.create_timer(2.0, self.report)
        self.get_logger().info(f'구독: rgb={rgb_topic} depth={depth_topic}')

    def on_rgb(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            self.rgb_frame = frame
            self.rgb_count += 1
            self.last_rgb_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_depth(self, msg):
        depth = decode_compressed_depth(msg.data)
        if depth is not None:
            vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            self.depth_frame = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
            self.depth_count += 1
            self.last_depth_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def report(self):
        self.get_logger().info(
            f'[진행] rgb 수신={self.rgb_count}장 depth 수신={self.depth_count}장 '
            f'(rgb_stamp={self.last_rgb_stamp}, depth_stamp={self.last_depth_stamp})'
        )

    def pump(self):
        h, w = 700, 700
        canvas = np.zeros((h, w * 2, 3), dtype=np.uint8)
        if self.rgb_frame is not None:
            canvas[:, :w] = cv2.resize(self.rgb_frame, (w, h))
        if self.depth_frame is not None:
            canvas[:, w:] = cv2.resize(self.depth_frame, (w, h))
        cv2.putText(canvas, f'RGB x{self.rgb_count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(canvas, f'DEPTH x{self.depth_count}', (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('rgb | depth', canvas)
        cv2.waitKey(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rgb', default='/robot5/oakd/rgb/image_raw/compressed')
    parser.add_argument('--depth', default='/robot5/oakd/stereo/image_raw/compressedDepth')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = BagImageViewer(args.rgb, args.depth)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
