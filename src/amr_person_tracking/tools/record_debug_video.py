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
import shutil
import subprocess
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def _write_video(frames, out_path, fps, width, height):
    """H.264(mp4)로 저장한다.

    cv2.VideoWriter는 이 빌드에서 H.264 태그(avc1/H264/X264)를 열지 못하고 mpeg4(mp4v)로
    떨어지는데, 그렇게 만든 파일은 브라우저·일부 뷰어에서 재생이 안 된다(실제로 증거영상이
    "깨져서 확인 불가"로 반려됨). 그래서 원시 프레임을 ffmpeg에 파이프로 넘겨 libx264로
    인코딩한다. ffmpeg이 없으면 최후수단으로 mp4v로 떨어지되 경고한다.
    """
    if shutil.which('ffmpeg'):
        cmd = [
            'ffmpeg', '-y', '-loglevel', 'error',
            '-f', 'rawvideo', '-pix_fmt', 'bgr24',
            '-s', f'{width}x{height}', '-r', str(fps), '-i', 'pipe:0',
            '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-preset', 'veryfast',
            out_path,
        ]
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        for frame in frames:
            proc.stdin.write(frame.tobytes())
        proc.stdin.close()
        proc.wait()
        return

    print('경고: ffmpeg이 없어 mp4v로 저장합니다 - 재생 안 되는 뷰어가 있을 수 있습니다',
          file=sys.stderr)
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
    for frame in frames:
        writer.write(frame)
    writer.release()


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
    except Exception as exc:
        # 컨텍스트가 내려가는 중에 spin이 돌면 RCLError('failed to initialize wait set')가
        # 나기도 한다. 여기서 죽으면 그때까지 모은 프레임을 전부 잃으므로(실측으로 영상 하나를
        # 통째로 날림) 수집만 멈추고 저장은 계속 진행한다.
        print(f'수집 중단({type(exc).__name__}: {exc}) - 지금까지 프레임으로 저장합니다',
              file=sys.stderr)

    if frames:
        h, w = frames[0].shape[:2]
        _write_video(frames, args.out, args.fps, w, h)
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
