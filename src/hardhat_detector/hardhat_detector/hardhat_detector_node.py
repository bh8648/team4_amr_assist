# hardhat_detector_node.py
#
# person_locator(pose_locator_node)가 락온한 사람의 bounding box를 잘라서
# publish하는 "person/hardhat_roi/compressed"를 구독해서, 학습된 detect
# 모델(cart/helmet/truck)로 "하이바를 쓰고 있는지"를 판별한 뒤
# "person/hardhat_status"(std_msgs/Bool)로 AMR에 넘겨주는 노드.
#
# 처음엔 얼굴 keypoint로 좁게 crop했었는데, 오버헤드/광각 카메라에서는
# 사람이 화면에서 작게 잡혀서 얼굴 keypoint 신뢰도가 안 나와 crop 자체가
# 계속 스킵되는 문제가 있었음 - 그래서 지금은 pose_locator_node가 사람
# bounding box를 그대로 넘겨줌(crop_person_bbox, person_locator/vision_utils.py
# 참고). 학습 데이터(datasets/detect_warn)도 얼굴만 잘라낸 게 아니라 카메라
# 전체 장면이었어서, 이렇게 넓게 주는 편이 학습 때 스케일/구도와도 더 비슷함.
#
# wrist_gesture_node(hand_gesture_caller 패키지)와 동일한 구조: 이 노드는
# "누가 어디 있는지", "crop을 어떻게 만들었는지"는 전혀 모른다 - 그건
# pose_locator_node가 YOLO-pose로 이미 하고 있는 일이라 여기서 다시 하지
# 않음(카메라 전체 프레임을 안 받는 이유). 오직 "지금 크롭 안에 하이바가
# 있는지"만 판단함.
#
# 한 프레임의 오분류로 AMR 판단이 흔들리는 걸 막기 위해, 최근 window_size
# 프레임의 판정을 모아 다수결로 최종 상태를 정하고, 상태가 실제로 바뀔
# 때만 publish한다(매 프레임 publish하지 않음). 또한 일정 시간(stale_timeout_sec)
# 동안 새 크롭이 안 들어오면(예: 사람이 프레임을 벗어남) 쌓아둔 투표를
# 버려서, 사람이 다시 나타났을 때 예전 프레임들과 섞여 판단하지 않게 한다.
#
# 주의: std_msgs/Bool은 "아직 판단 전" 또는 "지금 아무도 안 보임" 상태를
# 표현할 수 없다 - 사람이 프레임에서 사라져도 마지막으로 확정된 상태를
# 계속 유지한 채 재publish하지 않는다. AMR 쪽에서 "최근에 갱신됐는지"까지
# 확인해야 한다면 이 토픽의 timestamp가 없다는 점을 감안해서 별도 처리가
# 필요할 수 있음(예: 이 노드에 person/hardhat_status_stamped 같은 토픽을
# 추가하거나, AMR 쪽에서 마지막 수신 시각을 직접 추적).
#
# 사용법:
#   ros2 launch hardhat_detector hardhat_detector.launch.py
# (pose_locator_node가 먼저 돌면서 person/hardhat_roi/compressed를 publish하고
#  있어야 함 - 그러려면 그 노드도 락온된 사람이 있어야 함)

import time
from collections import deque

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from ultralytics import YOLO

from hardhat_detector.vision_utils import classify_hardhat, decode_jpeg, encode_jpeg


class HardhatDetectorNode(Node):

    def __init__(self):
        super().__init__('hardhat_detector_node')

        # --- 파라미터 ---
        # pose_locator_node의 hardhat_roi_topic 기본값과 일치해야 함
        self.declare_parameter('hardhat_roi_topic', 'person/hardhat_roi/compressed')
        # AMR이 구독할 최종 판정 토픽
        self.declare_parameter('hardhat_status_topic', 'person/hardhat_status')
        # 경로 없이 파일명만 주면 Ultralytics가 자기 weights 캐시에서 찾음;
        # 실제 학습된 하이바 분류 모델은 절대 경로로 override해서 사용
        self.declare_parameter('model_path', 'hardhat_cls.pt')
        # 모델의 클래스 이름 중 "하이바 착용"에 해당하는 이름 - 학습 시 정한
        # 클래스 이름에 맞게 override (classify_hardhat 주석 참고)
        self.declare_parameter('positive_class_name', 'helmet')
        # classify 모델의 top1 confidence 게이팅 기준(이 미만이면 애매한
        # 프레임으로 보고 투표 제외) 겸, detect 모델의 predict() conf 임계값
        self.declare_parameter('min_confidence', 0.6)
        # 최종 상태를 정할 때 참고하는 최근 프레임 개수
        self.declare_parameter('window_size', 5)
        # window 안에서 이 비율 이상 한쪽으로 쏠려야 상태를 확정/변경함 -
        # 애매하게 섞이면(과반 미달) 직전 상태를 그대로 유지
        self.declare_parameter('min_majority_ratio', 0.7)
        # 새 크롭이 이 시간(초) 이상 안 들어오면 쌓아둔 투표를 리셋
        self.declare_parameter('stale_timeout_sec', 2.0)
        self.declare_parameter('publish_debug_overlay', True)
        self.declare_parameter('debug_overlay_topic', 'person/hardhat_debug_image/compressed')

        hardhat_roi_topic = self.get_parameter('hardhat_roi_topic').value
        hardhat_status_topic = self.get_parameter('hardhat_status_topic').value
        model_path = self.get_parameter('model_path').value
        self.positive_class_name = self.get_parameter('positive_class_name').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.window_size = self.get_parameter('window_size').value
        self.min_majority_ratio = self.get_parameter('min_majority_ratio').value
        self.stale_timeout_sec = self.get_parameter('stale_timeout_sec').value
        self.publish_debug_overlay = self.get_parameter('publish_debug_overlay').value
        debug_overlay_topic = self.get_parameter('debug_overlay_topic').value

        self.get_logger().info(f'하이바 분류 모델 로딩 중: {model_path}')
        self.model = YOLO(model_path)

        self.recent_votes = deque(maxlen=self.window_size)
        self.current_status = None  # 아직 확정된 상태 없음 - 첫 확정 전까지 publish 안 함
        self.last_seen_at = None

        # QoS: pose_locator_node의 target_topic과 동일한 설정 - 작고,
        # reliable하고, 최신 것만 남기는 큐
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.roi_sub = self.create_subscription(
            CompressedImage, hardhat_roi_topic, self.roi_callback, 10
        )
        self.status_pub = self.create_publisher(Bool, hardhat_status_topic, qos)
        self.debug_pub = (
            self.create_publisher(CompressedImage, debug_overlay_topic, 1)
            if self.publish_debug_overlay else None
        )

        # roi_callback은 새 크롭이 들어올 때만 도는데, 사람이 사라지면 크롭
        # 자체가 안 들어와서 콜백이 멈춘다 - staleness는 콜백 밖에서
        # 주기적으로 확인해야 감지할 수 있어서 타이머로 따로 둠
        self.create_timer(0.5, self._check_stale)

        self.get_logger().info(
            f'hardhat_detector_node: "{hardhat_roi_topic}" 구독, '
            f'상태 바뀔 때 "{hardhat_status_topic}"에 publish'
        )

    def roi_callback(self, msg):
        roi = decode_jpeg(msg)
        if roi is None or roi.size == 0:
            return

        is_wearing, confidence = classify_hardhat(
            self.model, roi, self.positive_class_name, self.min_confidence
        )
        self.last_seen_at = time.monotonic()

        # detect 모델의 "못 찾음"(confidence == 0.0)은 이미 predict()의
        # conf_threshold로 걸러진 확정적 음성 신호라서 게이팅 없이 그대로
        # 투표에 넣음 - classify 모델만 top1 confidence로 애매한 프레임을 거름
        if self.model.task == 'detect' or confidence >= self.min_confidence:
            self.recent_votes.append(is_wearing)

        self._update_status()

        if self.publish_debug_overlay:
            self._publish_debug_overlay(roi, is_wearing, confidence)

    def _update_status(self):
        if len(self.recent_votes) < self.window_size:
            return  # 아직 판단할 만큼 샘플이 안 모임

        true_ratio = sum(self.recent_votes) / len(self.recent_votes)
        if true_ratio >= self.min_majority_ratio:
            new_status = True
        elif (1.0 - true_ratio) >= self.min_majority_ratio:
            new_status = False
        else:
            return  # 애매하게 섞임(과반 미달) - 직전 상태 유지, publish 안 함

        if new_status != self.current_status:
            self.current_status = new_status
            self.status_pub.publish(Bool(data=new_status))
            self.get_logger().info(
                f'하이바 상태 변경 -> {"착용" if new_status else "미착용"}'
            )

    def _check_stale(self):
        if self.last_seen_at is None:
            return
        if time.monotonic() - self.last_seen_at > self.stale_timeout_sec:
            self.recent_votes.clear()
            self.last_seen_at = None

    def _publish_debug_overlay(self, roi, is_wearing, confidence):
        overlay = roi.copy()
        label = f'{"HARDHAT" if is_wearing else "NO HARDHAT"} {confidence:.2f}'
        color = (0, 255, 0) if is_wearing else (0, 0, 255)
        cv2.putText(overlay, label, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        self.debug_pub.publish(encode_jpeg(overlay, quality=80))


def main(args=None):
    rclpy.init(args=args)
    node = HardhatDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
