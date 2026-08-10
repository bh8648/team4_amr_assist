# pose_locator_node.py
#
# YOLO-pose를 돌릴 만한 사양이 되는 머신에서 실행 (이 개발 PC가 아닐 수 있음
# - 이미지가 로컬 cv2.VideoCapture 대신 ROS2 토픽으로 들어오는 이유는
# camera_publisher.py 주석 참고).
#
# 카메라 프레임 하나가 들어올 때마다 하는 일:
#   1. 프레임에 대해 YOLO-pose 트래킹 실행 (사람마다 지속적인 트래커 id를
#      붙여줘서, 여러 프레임에 걸쳐 한 사람을 "락온"할 수 있게 함).
#   2. 이번 프레임에서 누구를 계속 따라갈지 고름 (vision_utils.select_person).
#   3. 그 사람의 발목 keypoint들을 뽑아서 하나의 "서 있는 픽셀"로 만듦
#      (vision_utils.extract_standing_pixel).
#   4. 그 픽셀을 캘리브레이션된 homography 행렬에 통과시켜서 map 프레임 기준
#      실제 (x, y)를 얻음 (vision_utils.apply_homography).
#   5. 그 결과를 geometry_msgs/PointStamped로 "person/position"에 publish함.

import json  # 파라미터 기본값 json 로딩
from pathlib import Path  # config 경로 조합용

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory  # 설치된 share 디렉터리 경로 조회
import rclpy  # ROS2 파이썬 클라이언트 라이브러리
from rclpy.node import Node  # 노드 베이스 클래스
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # 토픽 QoS 설정용
from sensor_msgs.msg import CompressedImage  # 카메라/디버그/ROI 이미지 메시지 타입
from geometry_msgs.msg import PointStamped  # map 프레임 좌표 메시지 타입
from std_msgs.msg import Empty  # 호출 트리거 메시지 타입
from ultralytics import YOLO  # YOLO-pose 모델 로딩/추론

# 로컬 헬퍼 함수들 - 실제 계산/로직은 vision_utils.py 참고
from person_locator.vision_utils import (
    apply_homography,
    crop_person_bbox,
    crop_square_roi,
    decode_jpeg,
    encode_jpeg,
    extract_standing_pixel,
    extract_wrist_pixel,
    load_homography_yaml,
    select_person,
)

PACKAGE_NAME = 'person_locator'


def _load_default_params():
    """config/params.json에서 파라미터 기본값을 불러온다.

    colcon build로 설치된 share 디렉터리에서 우선 찾고, 아직 설치되지
    않은 소스 트리에서 바로 실행하는 경우(예: IDE에서 직접 실행)에는
    이 파일 기준 상대경로의 config/params.json으로 fallback한다.
    """
    try:
        config_dir = Path(get_package_share_directory(PACKAGE_NAME)) / 'config'
    except PackageNotFoundError:
        config_dir = Path(__file__).resolve().parent.parent / 'config'
    with open(config_dir / 'params.json', encoding='utf-8') as f:
        return json.load(f)


class PoseLocatorNode(Node):

    def __init__(self):
        super().__init__('pose_locator_node')  # ROS2 노드 이름 등록

        # --- 파라미터, 전부 launch 파일이나 --ros-args -p로 override 가능 ---
        # 기본값은 config/params.json에서 불러옴(_load_default_params 참고).
        # 각 키의 의미:
        #   model_path: YOLO-pose weight 경로. 경로 없이 파일명만 주면
        #     Ultralytics가 처음 실행할 때 자기 weights 캐시로 자동
        #     다운로드함; 이미 이 머신에 .pt 파일을 올려뒀으면(예:
        #     /home/soo/corecode/COCO_WholeBody/yolov8n-pose.pt를 복사)
        #     절대 경로로 override
        #   conf_threshold: 박스를 "사람"으로 볼 최소 YOLO 검출 신뢰도
        #   ankle_conf_threshold: 발목 keypoint를 믿을 최소 신뢰도 -
        #     vision_utils.extract_standing_pixel로 그대로 전달됨
        #   tracker: Ultralytics 내장 multi-object tracker 설정 - 검출된
        #     사람마다 프레임 간에 안정적인 id를 붙여줘서, select_person이
        #     같은 사람을 계속 따라갈 수 있게 해줌
        #   homography_yaml_path: 캘리브레이션된 homography 행렬을 어디서
        #     불러올지 - `ros2 run person_locator calibrate_homography`로 생성됨
        #   image_topic: 입력 이미지 토픽 (camera_publisher.py가 JPEG로
        #     압축해서 publish하며, 이 노드와 다른 머신일 수도 있음 -
        #     camera_publisher.py 주석 참고)
        #   target_topic: 출력 토픽 - map 프레임 기준 사람의 위치
        #   publish_overlay: true면 박스+스켈레톤이 그려진 디버그용 이미지도
        #     같이 publish함 - rqt_image_view로 검출이 잘 되는지 눈으로
        #     확인할 수 있음. 이것도 camera_publisher와 같은 이유로 JPEG
        #     압축해서 내보냄
        #
        # --- 손 제스처 호출(hand_gesture_caller 패키지) 연동 파라미터 ---
        #   wrist_conf_threshold: 손목 keypoint를 믿을 최소 신뢰도 - 이보다
        #     낮으면 이번 프레임은 wrist ROI를 아예 안 잘라서 안 보냄
        #     (extract_wrist_pixel 참고)
        #   wrist_roi_half_size: wrist ROI 정사각형의 반변 길이(px) -
        #     실제로 잘리는 영역은 (2*half)x(2*half), 손목 중심 기준
        #   call_trigger_topic: wrist_gesture_node가 "쥐었다 폈다" 제스처를
        #     확정하면 이 토픽으로 신호를 보냄 - 받으면 마지막으로
        #     계산해둔 사람 위치를 call_position_topic으로 쏨
        #   call_position_topic: AMR이 구독하는 "지금 이 사람이 부름"
        #     이벤트 좌표
        #
        # --- 하이바 판별(hardhat_detector 패키지) 연동 파라미터 ---
        #   hardhat_roi_padding_ratio: 사람 bounding box를 그대로 크롭해서
        #     넘김(crop_person_bbox 참고) - 처음엔 얼굴 keypoint만 좁게
        #     크롭했는데, 오버헤드/광각 카메라처럼 사람이 화면에서 작게
        #     잡히면 얼굴 keypoint 신뢰도가 안 나와서 크롭 자체가 계속
        #     스킵되는 문제가 있었음 + 학습 데이터도 전체 장면이었어서
        #     bbox 크롭이 스케일상 더 잘 맞음
        #   hardhat_roi_top_ratio: 1.0이면 기존과 동일하게 전신을 그대로
        #     씀 - 오탐 튜닝을 위해 위에서부터 이 비율만큼(예: 0.4 =
        #     상반신/머리)만 남기려면 1.0 미만으로 낮춤
        default_params = _load_default_params()
        for name, value in default_params.items():
            self.declare_parameter(name, value)

        model_path = self.get_parameter('model_path').value  # YOLO-pose 가중치 경로
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.ankle_conf_threshold = self.get_parameter('ankle_conf_threshold').value
        self.tracker_cfg = self.get_parameter('tracker').value
        homography_yaml_path = self.get_parameter('homography_yaml_path').value
        image_topic = self.get_parameter('image_topic').value  # 구독할 카메라 토픽
        target_topic = self.get_parameter('target_topic').value  # map 좌표 publish 토픽
        self.publish_overlay = self.get_parameter('publish_overlay').value
        overlay_topic = self.get_parameter('overlay_topic').value
        self.overlay_jpeg_quality = self.get_parameter('overlay_jpeg_quality').value

        self.wrist_conf_threshold = self.get_parameter('wrist_conf_threshold').value
        self.wrist_roi_half_size = self.get_parameter('wrist_roi_half_size').value
        self.publish_wrist_roi = self.get_parameter('publish_wrist_roi').value
        wrist_roi_topic = self.get_parameter('wrist_roi_topic').value  # 손목 ROI publish 토픽
        self.wrist_roi_jpeg_quality = self.get_parameter('wrist_roi_jpeg_quality').value
        call_trigger_topic = self.get_parameter('call_trigger_topic').value  # 호출 트리거 구독 토픽
        call_position_topic = self.get_parameter('call_position_topic').value  # 호출 좌표 publish 토픽

        self.hardhat_roi_padding_ratio = self.get_parameter('hardhat_roi_padding_ratio').value
        self.hardhat_roi_top_ratio = self.get_parameter('hardhat_roi_top_ratio').value
        self.publish_hardhat_roi = self.get_parameter('publish_hardhat_roi').value
        hardhat_roi_topic = self.get_parameter('hardhat_roi_topic').value  # 하이바 ROI publish 토픽
        self.hardhat_roi_jpeg_quality = self.get_parameter('hardhat_roi_jpeg_quality').value

        # homography 행렬은 시작할 때 미리 로드함 - 파일이 없거나 형식이
        # 이상하면 나중에 조용히 이상한 좌표를 publish하는 대신
        # 시작 시점에 바로 실패하게 함
        try:
            self.homography = load_homography_yaml(homography_yaml_path)  # 3x3 homography 행렬 로드
        except (OSError, KeyError, ValueError) as exc:
            raise RuntimeError(
                f'{homography_yaml_path}에서 homography를 불러오지 못함: {exc}. '
                f'먼저 `ros2 run person_locator calibrate_homography`를 실행하세요.'
            ) from exc

        self.get_logger().info(f'YOLO-pose 모델 로딩 중: {model_path}')
        self.model = YOLO(model_path)  # YOLO-pose 모델 로드

        # QoS: 작고, reliable하고, 최신 것만 남기는 큐. 뭔가 잠깐 밀려도
        # 오래된 프레임/포인트가 쌓이길 원하는 게 아니라 항상 가장 최신
        # 것만 원함 - rc_car_chase의 webcam_locator_node와 동일한 설정
        qos = QoSProfile(depth=1)  # 큐 길이 1 (최신 값만 유지)
        qos.reliability = ReliabilityPolicy.RELIABLE  # 유실 없이 전달 보장
        qos.history = HistoryPolicy.KEEP_LAST  # 오래된 메시지는 버림

        self.image_sub = self.create_subscription(
            CompressedImage, image_topic, self.image_callback, qos  # 카메라 프레임 수신 콜백 등록
        )
        self.target_pub = self.create_publisher(PointStamped, target_topic, qos)  # map 좌표 퍼블리셔
        self.overlay_pub = (
            self.create_publisher(CompressedImage, overlay_topic, 1)
            if self.publish_overlay else None  # 옵션 꺼져 있으면 퍼블리셔 자체를 안 만듦
        )
        self.wrist_roi_pub = (
            self.create_publisher(CompressedImage, wrist_roi_topic, 1)
            if self.publish_wrist_roi else None
        )
        self.hardhat_roi_pub = (
            self.create_publisher(CompressedImage, hardhat_roi_topic, 1)
            if self.publish_hardhat_roi else None
        )
        self.call_position_pub = self.create_publisher(
            PointStamped, call_position_topic, qos  # 호출 좌표 퍼블리셔
        )
        # std_msgs/Empty: 트리거는 "지금 이 순간"이라는 사실 자체가 페이로드라서
        # 별도 데이터가 필요 없음 - wrist_gesture_node가 제스처를 확정하는
        # 순간에만 한 번 publish함
        # 이전 트리거 씹히는거 예방 Depth=10, Empty는 빈 데이터기 때문에 부담 적음
        self.call_trigger_sub = self.create_subscription(
            Empty, call_trigger_topic, self.call_trigger_callback, 10  # 호출 트리거 수신 콜백 등록
        )

        # 지금 "락온"해서 따라가고 있는 사람의 트래커 id
        # (vision_utils.select_person 참고) - None이면 아직 아무도 락온 안 한 것
        self.locked_track_id = None
        # 지금 락온해서 크롭하고 있는 손목 쪽('left'/'right') -
        # (vision_utils.extract_wrist_pixel 참고) - None이면 아직 안 정해짐.
        # 사람 락이 풀리면 같이 리셋해서, 다음에 새로 락온되는 사람에 대해
        # 처음부터 다시 고르게 함
        self.locked_wrist_side = None
        self.frame_count = 0
        # call_trigger_callback이 쓸 "마지막으로 계산된 map (x, y)" -
        # 프레임 콜백과 트리거 콜백이 비동기로 들어오므로 여기 저장해뒀다가 씀.
        # 아무도 안 보이면(락 풀림) None으로 되돌려서, 트리거가 와도 낡은
        # 위치를 잘못 쏘지 않게 함
        self.last_person_point = None

        self.get_logger().info(
            f'pose_locator_node: "{image_topic}" 구독, "{target_topic}"에 publish'
        )

    def image_callback(self, msg):
        # 들어온 JPEG 압축 메시지를 Ultralytics/OpenCV가 다룰 수 있는
        # OpenCV BGR numpy 프레임으로 압축 해제
        frame = decode_jpeg(msg)

        self.frame_count += 1  # 처리한 프레임 수 누적 (로그 주기 판단용)
        if self.frame_count % 30 == 1:
            # 매 프레임마다 로그를 찍으면 너무 시끄러우니, 노드가 살아있고
            # 실제로 프레임을 받고 있는지 확인할 수 있게 주기적으로만 로그
            self.get_logger().info(f'pose_locator_node: {self.frame_count} 프레임 처리함')

        # 트래킹을 켠 채로 YOLO-pose 실행. `persist=True`는 Ultralytics한테
        # 트래킹 상태를 호출 사이(즉, 프레임 사이)에도 유지하라는 뜻이고,
        # 이게 있어야 같은 사람에 대해 .id가 안정적으로 유지됨.
        # verbose=False는 Ultralytics가 매 프레임마다 콘솔에 찍는 로그를 끔
        results = self.model.track(
            frame,
            persist=True,  # 프레임 간 트래킹 상태 유지 (같은 사람은 같은 id 유지)
            conf=self.conf_threshold,  # 이 신뢰도 미만 검출은 버림
            tracker=self.tracker_cfg,  # bytetrack 등 트래커 설정 파일
            verbose=False,  # Ultralytics 콘솔 로그 끔
        )
        result = results[0]  # 감지된 객체의 Bbox
        boxes = result.boxes  # 이번 프레임의 모든 검출 박스

        # 이번 프레임에 어떤 사람을 따라갈지 결정
        index, track_id = select_person(boxes, self.locked_track_id)

        if index is not None:
            # 다음 프레임의 select_person 호출이 같은 사람을 계속
            # 우선적으로 따라가도록 이 사람의 트래커 id를 기억해둠
            self.locked_track_id = track_id

            # 이 검출의 keypoint를 꺼냄. result.keypoints.xy는
            # (검출 개수, 17, 2) 형태고, result.keypoints.conf는
            # (검출 개수, 17) 형태인데 모델 설정에 따라 None일 수도 있음
            keypoints_xy = result.keypoints.xy[index].cpu().numpy()  # 이 사람의 17개 keypoint 픽셀 좌표
            keypoints_conf = (
                # NumPy 및 일반 파이썬 라이브러리는 GPU 메모리에 직접 접근 불가능하므로 cpu 사용
                result.keypoints.conf[index].cpu().numpy()
                if result.keypoints.conf is not None
                else None
            )
            box_xyxy = boxes.xyxy[index].tolist()  # 이 사람의 bounding box 좌표

            # 발목 keypoint(또는 bbox fallback)를 이미지 좌표계의
            # "서 있는 픽셀" (u, v) 하나로 변환
            u, v = extract_standing_pixel(
                keypoints_xy, keypoints_conf, box_xyxy, self.ankle_conf_threshold
            )

            # 그 픽셀을 캘리브레이션된 homography에 통과시켜서
            # map 프레임 기준 실제 (x, y)(미터 단위)를 얻음
            x, y = apply_homography(u, v, self.homography)

            # 결과를 map 프레임 기준 PointStamped로 publish
            point_msg = PointStamped()  # 퍼블리시할 메시지 객체 생성
            point_msg.header.stamp = self.get_clock().now().to_msg()  # 현재 시각 스탬프
            point_msg.header.frame_id = 'map'  # 좌표계 명시
            point_msg.point.x = x
            point_msg.point.y = y
            point_msg.point.z = 0.0  # 2D 지면 위치라 z는 항상 0
            self.target_pub.publish(point_msg)

            # call_trigger_callback이 나중에(비동기로) 이 값을 그대로
            # call_position_topic에 실어 보낼 수 있도록 저장해둠
            self.last_person_point = (x, y)

            if self.publish_wrist_roi:
                wrist_pixel = extract_wrist_pixel(
                    keypoints_xy, keypoints_conf, self.wrist_conf_threshold,
                    self.locked_wrist_side,
                )
                if wrist_pixel is not None:
                    wu, wv, side = wrist_pixel  # 이번에 고른 손목 픽셀과 그 쪽(left/right)
                    self.locked_wrist_side = side  # 다음 프레임에도 같은 쪽을 우선하도록 저장
                    roi = crop_square_roi(frame, wu, wv, self.wrist_roi_half_size)  # 손목 중심 정사각형 크롭
                    # 손목이 프레임 가장자리 바로 바깥이면 크롭 결과가
                    # 비어있을 수 있음(crop_square_roi 참고) - 그러면 그냥 건너뜀
                    if roi.size > 0:
                        roi_msg = encode_jpeg(roi, quality=self.wrist_roi_jpeg_quality)  # ROI를 JPEG로 압축
                        self.wrist_roi_pub.publish(roi_msg)

            if self.publish_hardhat_roi:
                hardhat_roi = crop_person_bbox(
                    frame, box_xyxy, self.hardhat_roi_padding_ratio, self.hardhat_roi_top_ratio
                )  # 사람 bbox 전체(+패딩)를 하이바 판별용으로 크롭
                if hardhat_roi.size > 0:
                    hardhat_roi_msg = encode_jpeg(hardhat_roi, quality=self.hardhat_roi_jpeg_quality)  # JPEG 압축
                    self.hardhat_roi_pub.publish(hardhat_roi_msg)
        else:
            # 이번 프레임에 아무도 검출 안 됨 - 다음에 검출되는 사람이
            # 누구든 새로 잡을 수 있도록 락을 풀고, 오래된/마지막으로
            # 알던 위치는 일부러 publish하지 않음
            self.locked_track_id = None
            # 사람이 바뀔 수 있으니 손목 락도 같이 풀어서, 새 사람에 대해
            # 처음부터 다시 고르게 함
            self.locked_wrist_side = None
            # 락이 풀렸으니 트리거가 와도 더 이상 유효하지 않은 옛 위치를
            # 쏘지 않도록 같이 비움
            self.last_person_point = None

        if self.publish_overlay:
            # result.plot()이 박스+스켈레톤 keypoint를 프레임 복사본에
            # 그려줌 - `ros2 run rqt_image_view rqt_image_view`로 검출/트래킹이
            # 잘 되는지 눈으로 확인할 때 유용함
            overlay = result.plot()  # 박스+스켈레톤이 그려진 디버그용 프레임 생성
            overlay_msg = encode_jpeg(overlay, quality=self.overlay_jpeg_quality)  # JPEG로 압축
            self.overlay_pub.publish(overlay_msg)

    def call_trigger_callback(self, _msg):
        # wrist_gesture_node가 "쥐었다 폈다"를 확정한 순간 호출됨. 무거운
        # 재계산 없이, image_callback이 매 프레임 갱신해둔 마지막 위치를
        # 그대로 실어 보냄 - 이 콜백 시점과 그 위치를 계산한 프레임 사이에
        # 최대 한 프레임(수십ms) 정도의 지연은 있을 수 있지만, 사람을
        # 부르는 용도로는 문제 없는 수준
        if self.last_person_point is None:
            self.get_logger().warn(
                'call_trigger를 받았지만 현재 락온된 사람이 없어PersonTfBroadcasterNode 무시함'
            )
            return

        x, y = self.last_person_point  # 마지막으로 계산해둔 map 좌표를 그대로 사용
        point_msg = PointStamped()  # 퍼블리시할 메시지 객체 생성
        point_msg.header.stamp = self.get_clock().now().to_msg()  # 트리거 발생 시각으로 스탬프
        point_msg.header.frame_id = 'map'  # 좌표계 명시
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.0  # 2D 지면 위치라 z는 항상 0
        self.call_position_pub.publish(point_msg)  # AMR이 구독하는 호출 좌표 토픽으로 publish
        self.get_logger().info(f'call_trigger 수신 -> call_position ({x:.2f}, {y:.2f}) publish')


def main(args=None):
    rclpy.init(args=args)  # rclpy 컨텍스트 초기화
    node = PoseLocatorNode()  # 노드 생성 (여기서 모델/homography 로드)
    try:
        rclpy.spin(node)  # 콜백을 계속 처리하며 대기
    except KeyboardInterrupt:
        pass  # Ctrl+C는 정상 종료 경로로 처리
    finally:
        node.destroy_node()  # 노드 리소스 정리
        rclpy.shutdown()  # rclpy 컨텍스트 종료


if __name__ == '__main__':
    main()
