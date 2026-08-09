# camera_publisher.py
#
# 웹캠이 물리적으로 붙어 있는 머신(지금 구성에서는 이 개발 PC)에서 실행.
# 프레임을 잡아서 JPEG로 압축한 뒤 ROS2 sensor_msgs/CompressedImage 토픽으로
# publish함으로써, (여기서는 YOLO-pose를 못 돌릴 만큼 사양이 안 되는) 스펙
# 되는 별도 머신에서 돌아가는 pose_locator_node가 카메라 장치에 직접 접근할
# 필요 없이 네트워크 너머로 영상 피드를 구독할 수 있게 해줌.
#
# 처음엔 비압축 sensor_msgs/Image로 만들었었는데, 640x480 BGR 기준 프레임당
# ~900KB라 15Hz만 해도 초당 13MB - 두 PC 사이 네트워크로 보내기엔 너무
# 무거워서 JPEG 압축(vision_utils.encode_jpeg)으로 바꿈.
#
# yolo_pub/camera_publisher.py랑 기본 구조는 같고, device index/fps/화질을
# ROS 파라미터로 조절할 수 있게 좀 더 유연하게 만들고 디버깅용 주석을 추가함.

import rclpy                              # ROS2 파이썬 클라이언트 라이브러리
from rclpy.node import Node               # 모든 ROS2 노드의 베이스 클래스
from sensor_msgs.msg import CompressedImage  # JPEG 압축 이미지 메시지 타입
import cv2                                # OpenCV: 카메라 캡처

from person_locator.vision_utils import encode_jpeg  # JPEG 인코딩 유틸


class CameraPublisher(Node):

    def __init__(self):
        # "camera_publisher"라는 이름으로 rclpy에 노드 등록
        # - `ros2 node list`에 이 이름으로 뜸
        super().__init__('camera_publisher')

        # --- 파라미터 (launch 파일이나 실행 시 override 가능,
        # 예: `ros2 run person_locator camera_publisher --ros-args -p device:=/dev/video2`) ---
        self.declare_parameter('device', 2)          # cv2.VideoCapture 인덱스 또는 /dev/videoN 경로
        self.declare_parameter('capture_width', 640)  # 요청할 프레임 가로 크기
        self.declare_parameter('capture_height', 480)  # 요청할 프레임 세로 크기
        self.declare_parameter('fps', 15.0)            # 프레임을 얼마나 자주 잡아서 publish할지
        # "/compressed" 접미사는 image_transport 진영에서 압축 이미지 토픽에
        # 관용적으로 쓰는 이름 - rqt_image_view 등 표준 툴이 이 이름 규칙을 인식함
        self.declare_parameter('topic', 'camera/image_raw/compressed')
        self.declare_parameter('jpeg_quality', 80)     # 0~100, 낮을수록 더 압축(더 가벼움, 화질 저하)

        device = self.get_parameter('device').value
        capture_width = self.get_parameter('capture_width').value
        capture_height = self.get_parameter('capture_height').value
        fps = self.get_parameter('fps').value
        topic = self.get_parameter('topic').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # publisher: 큐 깊이 10은 구독자가 잠깐 느려질 경우를 대비한 작은 버퍼
        # - 이 정도 프레임레이트에서는 사실상 크게 안 쓰이지만 ROS2 기본/관용적인
        # 큐 크기라 맞춰둠
        self.publisher = self.create_publisher(CompressedImage, topic, 10)

        # 카메라 열기. `device`는 int(웹캠 인덱스)로 올 수도 있고
        # 문자열(예: 특정 USB 카메라를 가리키는 "/dev/video2")로 올 수도 있는데
        # 둘 다 cv2.VideoCapture에 그대로 넣을 수 있는 유효한 인자임
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, capture_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_height)
        if not self.cap.isOpened():
            # 나중에 "왜 토픽이 비어있지?" 하고 헤매는 것보다,
            # 시작할 때 바로 크게 실패하는 게 디버깅하기 쉬움
            raise RuntimeError(f'카메라 장치를 열 수 없음: {device!r}')

        # 타이머 기반 캡처 루프: 1/fps 초마다 프레임 하나를 잡아서 publish함.
        # (꽉 찬 while 루프 대신) 타이머를 쓰면 rclpy executor가 다른
        # 콜백/파라미터 처리도 같이 계속 돌릴 수 있음
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'camera_publisher: device={device} -> topic "{topic}", {fps} Hz'
        )

    def timer_callback(self):
        # 카메라에서 다음 프레임 가져오기
        ret, frame = self.cap.read()

        if not ret:
            # 카메라 순간 오류(예: USB 신호 끊김) - 죽지 말고 경고만 하고
            # 다음 타이머 tick에서 다시 시도
            self.get_logger().warn('camera_publisher: 프레임 가져오기 실패', throttle_duration_sec=2.0)
            return

        # OpenCV BGR numpy 프레임을 JPEG로 압축해서 CompressedImage로 publish.
        # 비압축 대비 프레임당 용량이 수십 배 줄어들어 네트워크 부담이 적음
        msg = encode_jpeg(frame, quality=self.jpeg_quality)
        self.publisher.publish(msg)

    def destroy_node(self):
        # 노드가 죽기 전에 OS 카메라 핸들을 꼭 release해줘야 함
        # - 안 그러면 재시작할 때 장치가 계속 잠겨서 못 열릴 수 있음
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)  # rclpy 컨텍스트 초기화
    node = CameraPublisher()  # 노드 생성 (여기서 카메라를 염)
    try:
        rclpy.spin(node)  # 타이머 콜백을 계속 처리하며 대기
    except KeyboardInterrupt:
        pass  # Ctrl+C는 정상 종료 경로로 처리
    finally:
        node.destroy_node()  # 카메라 release 등 리소스 정리
        rclpy.shutdown()  # rclpy 컨텍스트 종료


if __name__ == '__main__':
    main()
