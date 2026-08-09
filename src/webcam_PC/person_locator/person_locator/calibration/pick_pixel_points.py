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

import os  # 출력 디렉터리 생성용

import cv2  # 영상 창/마우스 클릭/그리기
import rclpy  # ROS2 파이썬 클라이언트 라이브러리
from rclpy.node import Node  # 노드 베이스 클래스
from sensor_msgs.msg import CompressedImage  # 카메라 프레임 메시지 타입
import yaml  # 결과 저장용

from person_locator.vision_utils import decode_jpeg  # JPEG 디코딩 유틸

WINDOW_NAME = 'pick_pixel_points [click=add point  u=undo  q=save+quit]'


class FrameGrabber(Node):
    """calibrate_homography.py의 FrameGrabber와 동일 - 최신 프레임만 들고 있음."""

    def __init__(self):
        super().__init__('pick_pixel_points')  # ROS2 노드 이름 등록

        self.declare_parameter('image_topic', 'camera/image_raw/compressed')
        self.declare_parameter('output', 'pixel_points.yaml')

        image_topic = self.get_parameter('image_topic').value
        self.output_path = self.get_parameter('output').value

        self.latest_frame = None  # 아직 프레임을 한 번도 못 받음
        self.subscription = self.create_subscription(
            CompressedImage, image_topic, self.image_callback, 10  # 프레임 수신 콜백 등록
        )
        self.get_logger().info(f'pick_pixel_points: "{image_topic}"에서 프레임 기다리는 중...')

    def image_callback(self, msg):
        self.latest_frame = decode_jpeg(msg)  # 최신 프레임으로 갱신


def draw(frame, pixel_points):
    vis = frame.copy()  # 원본 프레임을 훼손하지 않도록 복사본에 그림
    for i, (u, v) in enumerate(pixel_points):
        cv2.circle(vis, (int(u), int(v)), 6, (0, 255, 0), -1)  # 찍은 점을 초록 원으로 표시
        cv2.putText(vis, str(i + 1), (int(u) + 8, int(v) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)  # 점 옆에 순번 표시
    cv2.putText(vis, f'points: {len(pixel_points)}  [click=add  u=undo  q=save+quit]',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)  # 상단 안내 텍스트
    return vis


def main(args=None):
    rclpy.init(args=args)  # rclpy 컨텍스트 초기화
    grabber = FrameGrabber()  # 노드 생성

    print('camera/image_raw/compressed에서 첫 프레임을 기다리는 중...')
    print('(`ros2 run person_locator camera_publisher`가 실행 중인지 확인하세요)')
    while grabber.latest_frame is None and rclpy.ok():
        rclpy.spin_once(grabber, timeout_sec=0.1)  # 첫 프레임이 올 때까지 짧게 spin 반복

    pixel_points = []  # 지금까지 찍은 (u, v) 목록
    pending_click = None  # 아직 처리 안 한 최근 클릭 좌표

    def on_mouse(event, x, y, flags, param):
        nonlocal pending_click
        if event == cv2.EVENT_LBUTTONDOWN:
            pending_click = (x, y)  # 메인 루프가 다음 tick에 소비하도록 저장만 해둠

    cv2.namedWindow(WINDOW_NAME)  # 영상 표시용 창 생성
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)  # 클릭 이벤트 콜백 등록

    print('AMR이 스티커 지점에 도착하면, 영상 창에서 그 지점을 클릭하세요.')
    print("클릭할 때마다 몇 번째 점인지 이 터미널에 찍힙니다 - AMR 쪽")
    print("(log_amr_positions.py)과 반드시 같은 순서로 찍어야 나중에 짝이 맞습니다.")
    print("'u' = 마지막 점 취소, 'q' = 저장 후 종료")

    try:
        while rclpy.ok():
            rclpy.spin_once(grabber, timeout_sec=0.0)  # 논블로킹으로 최신 프레임/콜백만 처리
            frame = grabber.latest_frame
            if frame is None:
                continue  # 아직 프레임이 없으면 스킵

            if pending_click is not None:
                pixel_points.append(pending_click)  # 대기 중인 클릭을 점 목록에 반영
                print(f'[{len(pixel_points)}] pixel=({pending_click[0]}, {pending_click[1]}) 기록됨')
                pending_click = None  # 소비했으니 비움

            cv2.imshow(WINDOW_NAME, draw(frame, pixel_points))  # 점이 찍힌 프레임을 창에 표시
            key = cv2.waitKey(1) & 0xFF  # 키 입력 폴링 (1ms 대기)

            if key == ord('u') and pixel_points:
                removed = pixel_points.pop()  # 마지막 점 취소
                print(f'취소됨: {removed}')

            if key == ord('q'):
                break  # 종료 후 finally에서 저장
    finally:
        cv2.destroyAllWindows()  # 영상 창 정리

        if pixel_points:
            output_path = grabber.output_path
            os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)  # 출력 디렉터리가 없으면 생성
            with open(output_path, 'w') as f:
                yaml.safe_dump(
                    {'pixel_points': [[float(u), float(v)] for u, v in pixel_points]},
                    f, sort_keys=False,  # 찍은 순서 그대로 저장 (world_points와 순서 맞춰야 함)
                )
            print(f'픽셀 {len(pixel_points)}개를 {output_path}에 저장했습니다.')
        else:
            print('기록된 점이 없어 저장하지 않습니다.')

        grabber.destroy_node()  # 노드 리소스 정리
        rclpy.shutdown()  # rclpy 컨텍스트 종료


if __name__ == '__main__':
    main()
