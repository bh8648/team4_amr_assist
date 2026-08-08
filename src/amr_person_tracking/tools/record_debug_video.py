#!/usr/bin/env python3
"""
record_debug_video.py

oakd_detector_node의 디버그 이미지 토픽을 mp4로 직접 녹화한다.

화면 녹화(ffmpeg x11grab)로 뷰어 창을 찍는 방식은 창이 가려지거나 포커스를 잃으면 정지
화면만 남는 등 실패가 잦았다(실제로 여러 번 겪음). 토픽을 직접 받아 프레임을 쓰면
디스플레이 유무와 무관하게 항상 같은 결과가 나온다 - 증거영상은 재현 가능해야 하므로
이 방식을 기본으로 쓴다.

[사용법]
  export ROS_DISCOVERY_SERVER="" ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77
  python3 record_debug_video.py out.mp4 --fps 10    # 다른 터미널에서 ros2 bag play
  # 재생이 끝나면 Ctrl-C (또는 kill -INT) -> 그 시점까지 받은 프레임으로 mp4 생성
"""

import argparse
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out', help='출력 mp4 경로')
    ap.add_argument('--topic', default='/robot5/vision/oakd_detector/debug/compressed')
    ap.add_argument('--fps', type=float, default=10.0,
                    help='출력 영상 fps. bag을 배속 재생했다면 그만큼 올려 잡으면 실시간처럼 보인다')
    args = ap.parse_args()

    frames = []

    class Recorder(Node):
        def __init__(self):
            super().__init__('record_debug_video')
            self.create_subscription(
                CompressedImage, args.topic, self.on_image, qos_profile_sensor_data)

        def on_image(self, msg):
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                frames.append(frame)

    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        # rclpy가 SIGINT를 먼저 처리하면 KeyboardInterrupt 대신 이게 올라온다. 안 잡으면
        # 아래 영상 저장이 통째로 건너뛰어진다.
        pass

    if frames:
        h, w = frames[0].shape[:2]
        writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (w, h))
        for f in frames:
            writer.write(f)
        writer.release()
        print(f'{len(frames)}프레임 -> {args.out} ({w}x{h}, {args.fps}fps)', file=sys.stderr)
    else:
        print('수신한 프레임이 없습니다 - publish_debug_image가 켜져 있는지 확인하세요',
              file=sys.stderr)

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
