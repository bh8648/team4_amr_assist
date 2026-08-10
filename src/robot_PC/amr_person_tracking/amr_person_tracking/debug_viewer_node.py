#!/usr/bin/env python3
"""
debug_viewer_node.py

[역할]
oakd_detector_node가 `publish_debug_image=true`일 때 내보내는 오버레이(발끝 픽셀·등급·거리·
동기 시간차)를 `cv2.imshow`로 화면에 띄우는 순수 뷰어. 노드 자체에 로직은 없다 —
"실증"(실제로 검출이 맞는지 눈으로 확인)을 위해 별도 프로세스로 분리해뒀다.

ros2 topic echo로는 이미지를 볼 수 없고 rqt_image_view는 매번 수동으로 띄워야 하므로,
launch 파일에 이 노드를 조건부(publish_debug_image가 true일 때만)로 넣어 창이 자동으로 뜨게 한다.

[입력 토픽]
  - <namespace>/vision/oakd_detector/debug/compressed (sensor_msgs/CompressedImage)

[주의]
  - cv2.imshow는 디스플레이(X11/Wayland)가 있는 환경에서만 동작한다. headless(SSH -X 없는 원격,
    컨테이너 등)에서 실행하면 창을 못 띄우고 예외를 내며 종료한다.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage


class DebugViewerNode(Node):

    def __init__(self):
        super().__init__('debug_viewer_node')

        self.declare_parameter('image_topic', '/robot5/vision/oakd_detector/debug/compressed')
        self.declare_parameter('window_name', 'oakd_detector debug')
        # 로스백 재생처럼 원본 프레임 간격이 넓을 때(sync_slop 매칭 실패로 초당 1장 미만) 창이
        # 멈춘 것처럼 보이는 걸 막기 위한 최소 창 크기. 원본이 더 크면 원본 크기를 따른다.
        self.declare_parameter('window_width', 960)
        self.declare_parameter('window_height', 960)

        self.window_name = self.get_parameter('window_name').value
        image_topic = self.get_parameter('image_topic').value
        window_width = self.get_parameter('window_width').value
        window_height = self.get_parameter('window_height').value
        self.latest_frame = None

        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, window_width, window_height)
        except cv2.error as exc:
            self.get_logger().error(
                f'디스플레이에 창을 띄울 수 없습니다 ({exc}). '
                'X11/Wayland 세션에서 실행 중인지 확인하세요 (SSH라면 -X/-Y 필요).'
            )
            raise

        self.create_subscription(
            CompressedImage, image_topic, self.on_image, qos_profile_sensor_data)

        # 새 프레임 도착 여부와 무관하게 일정 주기로 GUI 이벤트 루프를 돌린다. on_image에서만
        # waitKey를 부르면 프레임 간격이 넓을 때(로스백 sync 매칭 희소) 그 사이 동안 창이
        # OS에 "응답 없음"으로 보여 마치 멈춘 것처럼 보인다.
        self.create_timer(0.03, self._pump_gui)

        self.get_logger().info(f'debug_viewer_node 시작 | {image_topic} -> 창 "{self.window_name}"')

    def on_image(self, msg: CompressedImage):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        self.latest_frame = frame

    def _pump_gui(self):
        if self.latest_frame is not None:
            cv2.imshow(self.window_name, self.latest_frame)
        # imshow가 실제로 그리고 창이 살아있게 하려면 GUI 이벤트 루프를 계속 돌려줘야 한다.
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DebugViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
