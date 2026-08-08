# calibrate_homography.py
#
# 1회성(또는 "카메라가 움직일 때마다 다시 실행하는") 캘리브레이션 툴.
# rc_car_chase/calibrate_webcam_homography.py를 가져와서 고쳤는데, 핵심 차이가
# 하나 있음: 그 버전은 cv2.VideoCapture로 웹캠을 직접 여는데, 이건 이 툴을
# 카메라가 물리적으로 붙어 있는 바로 그 머신에서 실행할 때만 됨.
# 여기서는 카메라가 이 툴을 실행하는 머신과 *다른* 머신에 붙어있을 수도
# 있으므로(예: 추론용 PC에서 이 툴을 실행하지만 웹캠은 이 개발 PC에 붙어있음)
# 대신 pose_locator_node가 구독하는 것과 같은 JPEG 압축 이미지 토픽
# ("camera/image_raw/compressed")을 구독하는 방식으로 만듦 - 비압축
# Image로 하면 네트워크 부담이 너무 커서(camera_publisher.py 참고) 처음부터
# 압축 토픽 기준으로 맞춤.
#
# 사용법:
#   ros2 run person_locator calibrate_homography
#
# 실측으로 알고 있는 지면 기준점을 4개 이상 정해서(예: 바닥에 붙인 테이프
# 표시, 또는 줄자로 잰 위치 / 로봇의 AMCL pose를 "map" 프레임 기준으로
# 읽은 값), 영상 창에서 각 점의 픽셀 위치를 클릭하고(또는 빨간 크로스헤어를
# w/a/s/d로 옮긴 뒤 SPACE), 터미널에 뜨는 프롬프트에 그 점의 실제 (x, y)를
# 미터 단위, "map" 프레임 기준으로 입력함. 점을 4개 이상 모으면 'c'를 눌러
# homography 행렬을 계산+저장; 'u'로 마지막 점 취소; 'q'로 저장 없이 종료.

import datetime
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import yaml

from person_locator.vision_utils import decode_jpeg


class FrameGrabber(Node):
    """camera/image_raw/compressed에서 온 최신 프레임을 보관해두는 최소 노드.
    main()의 순수 OpenCV 캘리브레이션 UI 루프가 이걸 읽어서 씀."""

    def __init__(self):
        super().__init__('calibrate_homography')

        self.declare_parameter('image_topic', 'camera/image_raw/compressed')
        self.declare_parameter(
            'output',
            os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'config', 'person_homography.yaml',
            ),
        )

        image_topic = self.get_parameter('image_topic').value
        self.output_path = self.get_parameter('output').value

        # 새 프레임이 도착할 때마다 갱신됨; 첫 메시지가 오기 전까지는 None
        # (camera_publisher.py가 아직 안 켜져 있을 수도 있음)
        self.latest_frame = None

        self.subscription = self.create_subscription(
            CompressedImage, image_topic, self.image_callback, 10
        )
        self.get_logger().info(
            f'calibrate_homography: "{image_topic}"에서 프레임 기다리는 중...'
        )

    def image_callback(self, msg):
        self.latest_frame = decode_jpeg(msg)


class Calibrator:
    """캘리브레이션 창의 클릭/키보드 상태를 들고 있는 클래스 -
    rc_car_chase/calibrate_webcam_homography.py 걸 그대로 가져옴. 이 부분
    (마우스 처리, 오버레이 그리기)은 프레임이 어디서 왔는지랑 상관없음."""

    def __init__(self, window_name):
        self.window_name = window_name
        self.pixel_points = []
        self.pending_click = None
        self.crosshair = None  # (u, v), 첫 프레임 크기를 알고 나면 설정됨

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pending_click = (x, y)

    def ensure_crosshair(self, frame):
        if self.crosshair is None:
            h, w = frame.shape[:2]
            self.crosshair = [w // 2, h // 2]

    def move_crosshair(self, dx, dy, frame):
        h, w = frame.shape[:2]
        self.crosshair[0] = max(0, min(w - 1, self.crosshair[0] + dx))
        self.crosshair[1] = max(0, min(h - 1, self.crosshair[1] + dy))

    def draw(self, frame):
        vis = frame.copy()
        for i, (u, v) in enumerate(self.pixel_points):
            cv2.circle(vis, (int(u), int(v)), 6, (0, 255, 0), -1)
            cv2.putText(vis, str(i + 1), (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if self.crosshair is not None:
            cu, cv_ = self.crosshair
            cv2.drawMarker(vis, (cu, cv_), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        # 화면에 그리는 안내 문구는 영어로 둠 - OpenCV 기본 Hershey 폰트가
        # 한글을 못 그려서(깨지거나 안 나옴) 여기만 예외
        cv2.putText(vis, f'points: {len(self.pixel_points)} (need >= 4)  '
                          f'[click OR wasd+space=add  u=undo  c=compute+save  q=quit]',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        return vis


def main(args=None):
    rclpy.init(args=args)
    grabber = FrameGrabber()

    # 첫 프레임이 도착할 때까지 여기서 대기(노드를 spin해서 image_callback이
    # 계속 호출되게 함) - 안 그러면 창이 검은 프레임으로 열려서
    # ensure_crosshair()가 크기를 잘못 잡음
    print('camera/image_raw/compressed에서 첫 프레임을 기다리는 중...')
    print('(`ros2 run person_locator camera_publisher`가 실행 중인지 확인하세요)')
    while grabber.latest_frame is None and rclpy.ok():
        rclpy.spin_once(grabber, timeout_sec=0.1)

    window_name = 'person_homography_calibration'
    calib = Calibrator(window_name)
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, calib.on_mouse)

    world_points = []
    print('영상 창에서 지면 기준점을 클릭하거나(또는 빨간 크로스헤어를')
    print('w/a/s/d로, 크게 움직이려면 W/A/S/D로 옮긴 뒤 SPACE로 추가),')
    print('여기 터미널에 뜨는 프롬프트에 map 프레임 기준 실제 (x, y)를')
    print('미터 단위로 입력하세요.')
    print("'u'로 마지막 점 취소, 점이 4개 이상 모이면 'c'로 계산+저장,")
    print("'q'로 저장 없이 종료.")
    print('참고: 클릭했는데 아무 반응이 없어 보이면 이 터미널을 확인하세요 - ')
    print('실제 좌표 입력을 기다리는 텍스트 프롬프트가 여기 뜹니다.')

    small_step, big_step = 5, 25
    move_keys = {
        ord('w'): (0, -small_step), ord('s'): (0, small_step),
        ord('a'): (-small_step, 0), ord('d'): (small_step, 0),
        ord('W'): (0, -big_step), ord('S'): (0, big_step),
        ord('A'): (-big_step, 0), ord('D'): (big_step, 0),
    }

    try:
        while rclpy.ok():
            # 구독 콜백이 계속 돌아서 self.latest_frame이 최신 상태를
            # 유지하도록 함, 대신 영영 블록되지는 않게(짧은 timeout으로
            # OpenCV 창도 계속 반응하게 유지)
            rclpy.spin_once(grabber, timeout_sec=0.0)

            frame = grabber.latest_frame
            if frame is None:
                continue
            calib.ensure_crosshair(frame)

            if calib.pending_click is not None:
                u, v = calib.pending_click
                # input()이 다른 곳에서 대기하는 동안 창이 그냥 멈춘 것처럼
                # 보이지 않도록, 터미널 입력을 블로킹으로 기다리기 *전에*
                # 화면에 명확한 안내를 먼저 띄움
                vis = calib.draw(frame)
                cv2.putText(vis, f'Point at ({u},{v}) - CHECK THE TERMINAL WINDOW for input prompt!',
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow(window_name, vis)
                cv2.waitKey(1)

                calib.pending_click = None
                try:
                    raw = input(f'{len(calib.pixel_points) + 1}번 점, 픽셀 ({u},{v}) '
                                f'-> 실제 좌표 "x y" 입력 (미터, map 프레임): ')
                    x_str, y_str = raw.strip().split()
                    wx, wy = float(x_str), float(y_str)
                except (ValueError, EOFError):
                    print('입력이 잘못됨, 이 점은 버립니다.')
                    continue
                calib.pixel_points.append((u, v))
                world_points.append((wx, wy))

            cv2.imshow(window_name, calib.draw(frame))
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print('저장하지 않고 종료합니다.')
                break

            if key == ord('u') and calib.pixel_points:
                calib.pixel_points.pop()
                world_points.pop()
                print('마지막 점을 제거했습니다.')

            if key == ord(' '):
                calib.pending_click = tuple(calib.crosshair)

            if key in move_keys:
                dx, dy = move_keys[key]
                calib.move_crosshair(dx, dy, frame)

            if key == ord('c'):
                if len(calib.pixel_points) < 4:
                    print(f'최소 4개의 점이 필요합니다 (현재 {len(calib.pixel_points)}개).')
                    continue
                pixel_arr = np.array(calib.pixel_points, dtype=np.float32)
                world_arr = np.array(world_points, dtype=np.float32)
                # findHomography가 pixel_arr -> world_arr로 가장 잘 맞는
                # 3x3 행렬 H를 풀어줌 (4개보다 많은 점을 주면 최소자승 fit)
                H, _ = cv2.findHomography(pixel_arr, world_arr)
                if H is None:
                    print('homography 계산 실패 - 점들이 일직선상에 있는지 확인하세요.')
                    continue
                out = {
                    'homography_matrix': H.flatten().tolist(),
                    'frame_id': 'map',
                    'calibrated_at': datetime.datetime.now().isoformat(),
                    'num_points': len(calib.pixel_points),
                    'pixel_points': [[float(u), float(v)] for u, v in calib.pixel_points],
                    'world_points': [[float(x), float(y)] for x, y in world_points],
                    'note': ('이 homography는 이 카메라가 map 프레임에 대해 고정된 '
                             '위치/각도로 있을 때만 유효함. 카메라를 옮기거나 map '
                             '원점이 바뀌면 다시 캘리브레이션해야 함.'),
                }
                output_path = grabber.output_path
                os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
                with open(output_path, 'w') as f:
                    yaml.safe_dump(out, f, sort_keys=False)
                print(f'homography ({len(calib.pixel_points)}개 점)를 {output_path}에 저장했습니다')
                break
    finally:
        cv2.destroyAllWindows()
        grabber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
