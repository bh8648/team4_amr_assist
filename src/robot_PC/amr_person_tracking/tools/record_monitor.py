#!/usr/bin/env python3
"""
ros2 bag record와 동시에 켜놓고, 지정한 토픽들이 실제로 계속 들어오는지
1초마다 카운트/최근 수신 후 경과시간을 찍어주는 모니터. ros2 topic hz를 반복 실행하는
것보다 이 방식이 훨씬 안정적이다 (공유 Discovery Server 환경에서 매번 새 CLI 노드를
띄우는 방식은 discovery가 불안정해서 신뢰할 수 없음을 확인함).

사용법:
  python3 record_monitor.py /robot5/oakd/rgb/image_raw/compressed \
      /robot5/oakd/stereo/image_raw/compressedDepth \
      /robot5/oakd/rgb/camera_info /robot5/scan
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, CameraInfo, LaserScan, Image
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

# 그래프 discovery(get_topic_names_and_types)가 공유 Discovery Server 환경에서 느려서
# 못 믿는다 - 토픽 이름 패턴으로 바로 타입을 정한다. 필요하면 목록에 줄 추가.
NAME_PATTERNS = [
    ('camera_info', CameraInfo),
    ('compressed', CompressedImage),   # compressed / compressedDepth 둘 다 CompressedImage 메시지
    ('image_raw', Image),
    ('scan', LaserScan),
    ('odom', Odometry),
    ('tf', TFMessage),
]


def guess_type(topic):
    for pattern, cls in NAME_PATTERNS:
        if pattern in topic:
            return cls
    return None


class Monitor(Node):
    def __init__(self, topics):
        super().__init__('record_monitor')
        self.counts = {t: 0 for t in topics}
        self.last_seen = {t: None for t in topics}
        self.start = time.time()

        for topic in topics:
            msg_cls = guess_type(topic)
            if msg_cls is None:
                self.get_logger().warn(f'{topic}: 이름으로 타입 추론 실패 - 건너뜀 (스크립트의 NAME_PATTERNS에 추가 필요)')
                continue
            self.create_subscription(
                msg_cls, topic,
                (lambda t: lambda msg: self._on_msg(t))(topic),
                qos_profile_sensor_data)

        self.create_timer(1.0, self.report)

    def _on_msg(self, topic):
        self.counts[topic] += 1
        self.last_seen[topic] = time.time()

    def report(self):
        elapsed = time.time() - self.start
        line = [f'[{elapsed:5.1f}s]']
        for t, c in self.counts.items():
            age = '-' if self.last_seen[t] is None else f'{time.time()-self.last_seen[t]:.1f}s전'
            flag = '' if (self.last_seen[t] is not None and time.time() - self.last_seen[t] < 3.0) else '  <== 끊김!'
            line.append(f'{t.split("/")[-1]}={c}({age}){flag}')
        self.get_logger().info(' | '.join(line))


def main():
    topics = sys.argv[1:]
    if not topics:
        print('사용법: record_monitor.py <topic1> <topic2> ...')
        sys.exit(1)
    rclpy.init()
    node = Monitor(topics)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
