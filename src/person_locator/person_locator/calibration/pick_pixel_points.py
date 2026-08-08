# pick_pixel_points.py
#
# calibrate_homography.py의 "클릭 -> 그 자리에서 바로 실좌표 입력" 흐름은
# 클릭하는 사람과 실좌표(AMR 위치)를 읽는 사람이 같은 세션/터미널에 있다는
# 걸 전제로 함. 그런데 지금 구성은 웹캠 PC에서 클릭하는 사람과, AMR을
# teleop으로 몰면서 amcl_pose를 읽는 사람(이 PC)이 물리적으로 분리돼 있어서
# 그 흐름을 그대로 못 씀.
#
# 그래서 이 도구는 실좌표 입력 없이 "몇 번째 점을 어느 픽셀에서 찍었는지"만
# 기록함. 같은 순서로 log_amr_positions.py(다른 PC에서 실행)가 AMR 실좌표를
# 기록해두면, 나중에 compute_homography_from_points.py로 두 파일을 순서대로
# 짝지어서 homography를 계산함.
#
# 사용법 (웹캠이 붙은 PC에서, camera_publisher가 이미 돌고 있어야 함):
#   ros2 run person_locator pick_pixel_points --ros-args -p output:=pixel_points.yaml
#
# 스티커 지점에 AMR이 도착했다고 (다른 PC를 담당하는 사람에게 무전/음성으로)
# 확인받은 다음, 영상 창에서 그 지점을 클릭. 'u'로 마지막 점 취소,
# 'q'로 저장 후 종료.

import os

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import yaml

from person_locator.vision_utils import decode_jpeg

WINDOW_NAME = 'pick_pixel_points [click=add point  u=undo  q=save+quit]'


class FrameGrabber(Node):
    """calibrate_homography.py의 FrameGrabber와 동일 - 최신 프레임만 들고 있음."""

    def __init__(self):
        super().__init__('pick_pixel_points')

        self.declare_parameter('image_topic', 'camera/image_raw/compressed')
        self.declare_parameter('output', 'pixel_points.yaml')

        image_topic = self.get_parameter('image_topic').value
        self.output_path = self.get_parameter('output').value

        self.latest_frame = None
        self.subscription = self.create_subscription(
            CompressedImage, image_topic, self.image_callback, 10
        )
        self.get_logger().info(f'pick_pixel_points: "{image_topic}"에서 프레임 기다리는 중...')

    def image_callback(self, msg):
        self.latest_frame = decode_jpeg(msg)


def draw(frame, pixel_points):
    vis = frame.copy()
    for i, (u, v) in enumerate(pixel_points):
        cv2.circle(vis, (int(u), int(v)), 6, (0, 255, 0), -1)
        cv2.putText(vis, str(i + 1), (int(u) + 8, int(v) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(vis, f'points: {len(pixel_points)}  [click=add  u=undo  q=save+quit]',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    return vis


def main(args=None):
    rclpy.init(args=args)
    grabber = FrameGrabber()

    print('camera/image_raw/compressed에서 첫 프레임을 기다리는 중...')
    print('(`ros2 run person_locator camera_publisher`가 실행 중인지 확인하세요)')
    while grabber.latest_frame is None and rclpy.ok():
        rclpy.spin_once(grabber, timeout_sec=0.1)

    pixel_points = []
    pending_click = None

    def on_mouse(event, x, y, flags, param):
        nonlocal pending_click
        if event == cv2.EVENT_LBUTTONDOWN:
            pending_click = (x, y)

    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

    print('AMR이 스티커 지점에 도착하면, 영상 창에서 그 지점을 클릭하세요.')
    print("클릭할 때마다 몇 번째 점인지 이 터미널에 찍힙니다 - AMR 쪽")
    print("(log_amr_positions.py)과 반드시 같은 순서로 찍어야 나중에 짝이 맞습니다.")
    print("'u' = 마지막 점 취소, 'q' = 저장 후 종료")

    try:
        while rclpy.ok():
            rclpy.spin_once(grabber, timeout_sec=0.0)
            frame = grabber.latest_frame
            if frame is None:
                continue

            if pending_click is not None:
                pixel_points.append(pending_click)
                print(f'[{len(pixel_points)}] pixel=({pending_click[0]}, {pending_click[1]}) 기록됨')
                pending_click = None

            cv2.imshow(WINDOW_NAME, draw(frame, pixel_points))
            key = cv2.waitKey(1) & 0xFF

            if key == ord('u') and pixel_points:
                removed = pixel_points.pop()
                print(f'취소됨: {removed}')

            if key == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()

        if pixel_points:
            output_path = grabber.output_path
            os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
            with open(output_path, 'w') as f:
                yaml.safe_dump(
                    {'pixel_points': [[float(u), float(v)] for u, v in pixel_points]},
                    f, sort_keys=False,
                )
            print(f'픽셀 {len(pixel_points)}개를 {output_path}에 저장했습니다.')
        else:
            print('기록된 점이 없어 저장하지 않습니다.')

        grabber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
