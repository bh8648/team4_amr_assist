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

import datetime  # 캘리브레이션 시각 기록
import os  # 경로 조합/디렉터리 생성용

import cv2  # 영상 창/마우스 클릭/homography 계산
import numpy as np  # 점 배열 변환
import rclpy  # ROS2 파이썬 클라이언트 라이브러리
from rclpy.node import Node  # 노드 베이스 클래스
from sensor_msgs.msg import CompressedImage  # 카메라 프레임 메시지 타입
import yaml  # 결과 저장용

from person_locator.vision_utils import decode_jpeg  # JPEG 디코딩 유틸


class FrameGrabber(Node):
    """camera/image_raw/compressed에서 온 최신 프레임을 보관해두는 최소 노드.
    main()의 순수 OpenCV 캘리브레이션 UI 루프가 이걸 읽어서 씀."""

    def __init__(self):
        super().__init__('calibrate_homography')  # ROS2 노드 이름 등록

        self.declare_parameter('image_topic', 'camera/image_raw/compressed')
        self.declare_parameter(
            'output',
            os.path.join(
                # calibration/calibrate_homography.py -> calibration -> person_locator(패키지) ->
                # person_locator(패키지 루트, config/가 있는 곳) 이렇게 3단계 위로 올라가야 함 -
                # calibration/ 서브패키지로 옮기면서 한 단계 늘어남
                os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                'config', 'person_homography.yaml',
            ),
        )

        image_topic = self.get_parameter('image_topic').value
        self.output_path = self.get_parameter('output').value

        # 새 프레임이 도착할 때마다 갱신됨; 첫 메시지가 오기 전까지는 None
        # (camera_publisher.py가 아직 안 켜져 있을 수도 있음)
        self.latest_frame = None

        self.subscription = self.create_subscription(
            CompressedImage, image_topic, self.image_callback, 10  # 프레임 수신 콜백 등록
        )
        self.get_logger().info(
            f'calibrate_homography: "{image_topic}"에서 프레임 기다리는 중...'
        )

    def image_callback(self, msg):
        self.latest_frame = decode_jpeg(msg)  # 최신 프레임으로 갱신


class Calibrator:
    """캘리브레이션 창의 클릭/키보드 상태를 들고 있는 클래스 -
    rc_car_chase/calibrate_webcam_homography.py 걸 그대로 가져옴. 이 부분
    (마우스 처리, 오버레이 그리기)은 프레임이 어디서 왔는지랑 상관없음."""

    def __init__(self, window_name):
        self.window_name = window_name
        self.pixel_points = []  # 지금까지 찍은 (u, v) 픽셀 좌표 목록
        self.pending_click = None  # 아직 처리 안 한 최근 클릭/스페이스 좌표
        self.crosshair = None  # (u, v), 첫 프레임 크기를 알고 나면 설정됨

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pending_click = (x, y)  # 메인 루프가 다음 tick에 소비하도록 저장만 해둠

    def ensure_crosshair(self, frame):
        if self.crosshair is None:
            h, w = frame.shape[:2]
            self.crosshair = [w // 2, h // 2]  # 첫 프레임 크기를 알고 나면 화면 중앙으로 초기화

    def move_crosshair(self, dx, dy, frame):
        h, w = frame.shape[:2]
        self.crosshair[0] = max(0, min(w - 1, self.crosshair[0] + dx))  # 프레임 밖으로 못 나가게 클램프
        self.crosshair[1] = max(0, min(h - 1, self.crosshair[1] + dy))

    def draw(self, frame):
        vis = frame.copy()  # 원본 프레임을 훼손하지 않도록 복사본에 그림
        for i, (u, v) in enumerate(self.pixel_points):
            cv2.circle(vis, (int(u), int(v)), 6, (0, 255, 0), -1)  # 찍은 점을 초록 원으로 표시
            cv2.putText(vis, str(i + 1), (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)  # 점 옆에 순번 표시
        if self.crosshair is not None:
            cu, cv_ = self.crosshair
            cv2.drawMarker(vis, (cu, cv_), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)  # 현재 크로스헤어 위치 표시
        # 화면에 그리는 안내 문구는 영어로 둠 - OpenCV 기본 Hershey 폰트가
        # 한글을 못 그려서(깨지거나 안 나옴) 여기만 예외
        cv2.putText(vis, f'points: {len(self.pixel_points)} (need >= 4)  '
                          f'[click OR wasd+space=add  u=undo  c=compute+save  q=quit]',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)  # 상단 안내 텍스트
        return vis


def main(args=None):
    rclpy.init(args=args)  # rclpy 컨텍스트 초기화
    grabber = FrameGrabber()  # 노드 생성

    # 첫 프레임이 도착할 때까지 여기서 대기(노드를 spin해서 image_callback이
    # 계속 호출되게 함) - 안 그러면 창이 검은 프레임으로 열려서
    # ensure_crosshair()가 크기를 잘못 잡음
    print('camera/image_raw/compressed에서 첫 프레임을 기다리는 중...')
    print('(`ros2 run person_locator camera_publisher`가 실행 중인지 확인하세요)')
    while grabber.latest_frame is None and rclpy.ok():
        rclpy.spin_once(grabber, timeout_sec=0.1)  # 첫 프레임이 올 때까지 짧게 spin 반복

    window_name = 'person_homography_calibration'
    calib = Calibrator(window_name)  # 클릭/키보드 상태를 들고 있는 캘리브레이터 생성
    cv2.namedWindow(window_name)  # 영상 표시용 창 생성
    cv2.setMouseCallback(window_name, calib.on_mouse)  # 클릭 이벤트 콜백 등록

    world_points = []  # pixel_points와 순서가 대응하는 실제 (x, y) 목록
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
                continue  # 아직 프레임이 없으면 스킵
            calib.ensure_crosshair(frame)  # 크로스헤어 초기 위치 확정 (최초 1회만 동작)

            if calib.pending_click is not None:
                u, v = calib.pending_click
                # input()이 다른 곳에서 대기하는 동안 창이 그냥 멈춘 것처럼
                # 보이지 않도록, 터미널 입력을 블로킹으로 기다리기 *전에*
                # 화면에 명확한 안내를 먼저 띄움
                vis = calib.draw(frame)
                cv2.putText(vis, f'Point at ({u},{v}) - CHECK THE TERMINAL WINDOW for input prompt!',
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow(window_name, vis)
                cv2.waitKey(1)  # 위에서 그린 안내 문구가 실제로 화면에 반영되도록 짧게 대기

                calib.pending_click = None  # 소비했으니 비움
                try:
                    raw = input(f'{len(calib.pixel_points) + 1}번 점, 픽셀 ({u},{v}) '
                                f'-> 실제 좌표 "x y" 입력 (미터, map 프레임): ')
                    x_str, y_str = raw.strip().split()
                    wx, wy = float(x_str), float(y_str)
                except (ValueError, EOFError):
                    print('입력이 잘못됨, 이 점은 버립니다.')
                    continue
                calib.pixel_points.append((u, v))  # 픽셀 좌표 기록
                world_points.append((wx, wy))  # 대응하는 실제 좌표 기록

            cv2.imshow(window_name, calib.draw(frame))  # 이번 루프의 최신 상태를 창에 표시
            key = cv2.waitKey(1) & 0xFF  # 키 입력 폴링 (1ms 대기)

            if key == ord('q'):
                print('저장하지 않고 종료합니다.')
                break

            if key == ord('u') and calib.pixel_points:
                calib.pixel_points.pop()  # 마지막 픽셀 점 취소
                world_points.pop()  # 짝이 되는 실제 좌표도 같이 취소
                print('마지막 점을 제거했습니다.')

            if key == ord(' '):
                calib.pending_click = tuple(calib.crosshair)  # 크로스헤어 위치를 클릭한 것처럼 처리

            if key in move_keys:
                dx, dy = move_keys[key]
                calib.move_crosshair(dx, dy, frame)  # w/a/s/d(또는 대문자)로 크로스헤어 이동

            if key == ord('c'):
                if len(calib.pixel_points) < 4:
                    print(f'최소 4개의 점이 필요합니다 (현재 {len(calib.pixel_points)}개).')
                    continue
                pixel_arr = np.array(calib.pixel_points, dtype=np.float32)  # findHomography 입력 형태로 변환
                world_arr = np.array(world_points, dtype=np.float32)
                # findHomography가 pixel_arr -> world_arr로 가장 잘 맞는
                # 3x3 행렬 H를 풀어줌 (4개보다 많은 점을 주면 최소자승 fit)
                H, _ = cv2.findHomography(pixel_arr, world_arr)
                if H is None:
                    print('homography 계산 실패 - 점들이 일직선상에 있는지 확인하세요.')
                    continue
                out = {
                    'homography_matrix': H.flatten().tolist(),  # 3x3 행렬을 9개짜리 리스트로 flatten해서 저장
                    'frame_id': 'map',
                    'calibrated_at': datetime.datetime.now().isoformat(),  # 캘리브레이션 시각 기록
                    'num_points': len(calib.pixel_points),
                    'pixel_points': [[float(u), float(v)] for u, v in calib.pixel_points],  # 원본 점들도 보관
                    'world_points': [[float(x), float(y)] for x, y in world_points],
                    'note': ('이 homography는 이 카메라가 map 프레임에 대해 고정된 '
                             '위치/각도로 있을 때만 유효함. 카메라를 옮기거나 map '
                             '원점이 바뀌면 다시 캘리브레이션해야 함.'),
                }
                output_path = grabber.output_path
                os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)  # 출력 디렉터리가 없으면 생성
                with open(output_path, 'w') as f:
                    yaml.safe_dump(out, f, sort_keys=False)  # 입력한 순서 그대로 저장 (가독성)
                print(f'homography ({len(calib.pixel_points)}개 점)를 {output_path}에 저장했습니다')
                break
    finally:
        cv2.destroyAllWindows()  # 영상 창 정리
        grabber.destroy_node()  # 노드 리소스 정리
        rclpy.shutdown()  # rclpy 컨텍스트 종료


if __name__ == '__main__':
    main()
