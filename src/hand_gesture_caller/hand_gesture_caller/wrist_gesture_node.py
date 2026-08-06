# wrist_gesture_node.py
#
# person_locator(pose_locator_node)가 락온한 사람의 손목 주변을 잘라서 publish하는
# "person/wrist_roi/compressed"를 구독해서, 그 작은 크롭 안에서 MediaPipe Hands로
# 손 모양을 본 뒤 "폈다(OPEN) -> 쥐었다(CLOSED) -> 폈다(OPEN)"이 정해진 시간 안에
# 순서대로 나오면 한 번의 "호출 제스처"로 확정하고 person_locator에 트리거를 보냄.
#
# 이 노드는 "누가 어디 있는지"는 전혀 모른다 - 그건 pose_locator_node가
# YOLO-pose로 이미 하고 있는 일이라 여기서 다시 하지 않음(카메라 전체 프레임을
# 안 받는 이유). 오직 "지금 크롭 안의 손이 쥐었는지/폈는지"만 판단하고,
# person/call_trigger(std_msgs/Empty)를 쏘는 역할만 함 - 좌표 계산과 AMR로의
# 실제 publish는 pose_locator_node.call_trigger_callback이 담당(person_locator
# 패키지 참고).
#
# 사용법:
#   ros2 run hand_gesture_caller wrist_gesture_node
# (pose_locator_node가 먼저 돌면서 person/wrist_roi/compressed를 publish하고
#  있어야 함 - 그러려면 그 노드도 락온된 사람이 있어야 함)

import time

import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Empty

from hand_gesture_caller.vision_utils import decode_jpeg, encode_jpeg

# MediaPipe Hands 21개 landmark 중 이 판정에 쓰는 것만 골라둠
# (전체 목록: https://developers.google.com/mediapipe/solutions/vision/hand_landmarker)
WRIST = 0
MIDDLE_MCP = 9  # 중지 손허리손가락관절 - "손바닥 크기" 기준 스케일로 씀
FINGERTIPS = (8, 12, 16, 20)  # 검지/중지/약지/새끼 손끝 (엄지 제외 - 주먹 쥘 때 거동이 달라서)


def compute_openness_ratio(landmarks):
    """MediaPipe 손 landmark로부터 "손이 얼마나 펴져 있는지" 비율을 계산한다.

    손끝(WRIST 기준 거리 평균)을 "손바닥 한 뼘" 크기(WRIST-MIDDLE_MCP 거리)로
    나눠서 정규화함 - 이렇게 하면 손이 카메라에서 멀든 가깝든, ROI가 어떻게
    잘렸든 상관없이(스케일 불변) 순수하게 "편 정도"만 남는다.
    편 손은 대략 2.5~3.0, 주먹은 대략 1.0~1.3 근처로 나옴(경험적 범위).
    """
    palm_scale = _dist(landmarks[WRIST], landmarks[MIDDLE_MCP])
    if palm_scale < 1e-6:
        return None  # landmark가 겹쳐 나오는 등 이상치 - 신뢰 불가

    tip_distances = [_dist(landmarks[WRIST], landmarks[i]) for i in FINGERTIPS]
    avg_tip_distance = sum(tip_distances) / len(tip_distances)
    return avg_tip_distance / palm_scale


def _dist(a, b):
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5


class GestureStateMachine:
    """"OPEN -> CLOSED -> OPEN" 시퀀스를 시간 제한 안에서 추적하는 상태머신.

    노드/ROS와 무관한 순수 로직이라 따로 테스트하기 쉽게 클래스로 분리함.
    """

    IDLE = 'idle'                    # 편 손(기준선)을 기다리는 중
    ARMED_WAIT_CLOSE = 'wait_close'  # 편 손을 봤음, 이제 쥐는 걸 기다림
    ARMED_WAIT_OPEN = 'wait_open'    # 쥔 손을 봤음, 이제 다시 펴는 걸 기다림

    def __init__(self, open_threshold, closed_threshold, phase_timeout_sec,
                 stale_timeout_sec, cooldown_sec):
        self.open_threshold = open_threshold
        self.closed_threshold = closed_threshold
        self.phase_timeout_sec = phase_timeout_sec
        self.stale_timeout_sec = stale_timeout_sec
        self.cooldown_sec = cooldown_sec

        self.phase = self.IDLE
        self.phase_started_at = None
        self.last_seen_at = None
        self.cooldown_until = 0.0

    def update(self, ratio, now):
        """이번 프레임의 openness ratio(손을 못 찾았으면 None)를 반영하고,
        이번 호출로 제스처가 막 확정됐으면 True를 반환한다."""
        if now < self.cooldown_until:
            return False

        if ratio is None:
            # 손을 이번 프레임에 못 찾음 - 너무 오래 못 찾으면 진행 중이던
            # 시퀀스를 리셋(예: 손이 ROI 밖으로 나가버림)
            if self.last_seen_at is not None and now - self.last_seen_at > self.stale_timeout_sec:
                self._reset()
            return False
        self.last_seen_at = now

        # 단계 사이 시간이 너무 오래 걸리면(느긋하게 편 채로 가만있다가 나중에
        # 쥐는 것 같은 경우) 의도된 제스처로 보지 않고 리셋 - 새로 OPEN부터 다시 시작
        if self.phase_started_at is not None and now - self.phase_started_at > self.phase_timeout_sec:
            self._reset()

        is_open = ratio >= self.open_threshold
        is_closed = ratio <= self.closed_threshold
        # open_threshold와 closed_threshold 사이(애매한 중간 손 모양)는 무시 -
        # 히스테리시스 밴드라서 살짝 흔들리는 정도로는 단계가 안 튐

        if self.phase == self.IDLE:
            if is_open:
                self.phase = self.ARMED_WAIT_CLOSE
                self.phase_started_at = now
            return False

        if self.phase == self.ARMED_WAIT_CLOSE:
            if is_closed:
                self.phase = self.ARMED_WAIT_OPEN
                self.phase_started_at = now
            return False

        if self.phase == self.ARMED_WAIT_OPEN:
            if is_open:
                # 시퀀스 완성 - 제스처 확정
                self._reset()
                self.cooldown_until = now + self.cooldown_sec
                return True
            return False

        return False

    def _reset(self):
        self.phase = self.IDLE
        self.phase_started_at = None
        self.last_seen_at = None


class WristGestureNode(Node):

    def __init__(self):
        super().__init__('wrist_gesture_node')

        # --- 파라미터 ---
        # pose_locator_node의 wrist_roi_topic 기본값과 일치해야 함
        self.declare_parameter('wrist_roi_topic', 'person/wrist_roi/compressed')
        # pose_locator_node의 call_trigger_topic 기본값과 일치해야 함
        self.declare_parameter('call_trigger_topic', 'person/call_trigger')

        # MediaPipe Hands 검출 최소 신뢰도 - ROI 자체가 매 프레임 위치가
        # 바뀌므로(사람이 움직임) tracking 모드보다 매 프레임 새로 검출하는
        # static_image_mode=True가 더 안정적
        self.declare_parameter('min_detection_confidence', 0.5)

        # openness ratio 히스테리시스 밴드 - compute_openness_ratio 주석 참고.
        # 기본값은 경험적 값이라 실측(디버그 오버레이로 실제 ratio 로그 찍어서)
        # 후 튜닝 필요할 수 있음
        self.declare_parameter('open_ratio_threshold', 1.8)
        self.declare_parameter('closed_ratio_threshold', 1.3)
        # OPEN->CLOSED, CLOSED->OPEN 각 단계 전환에 허용하는 최대 시간(초) -
        # 이보다 오래 걸리면 의도치 않은 손 모양 변화로 보고 리셋
        self.declare_parameter('phase_timeout_sec', 1.5)
        # 진행 중이던 시퀀스가 있을 때, 손을 이 시간(초) 이상 못 찾으면 리셋
        self.declare_parameter('stale_timeout_sec', 1.0)
        # 트리거 발행 직후 이 시간(초) 동안은 새 시퀀스를 아예 시작하지 않음 -
        # 손을 펴는 순간 자체가 다음 "OPEN 기준선"으로 잘못 인식되는 것 방지
        self.declare_parameter('cooldown_sec', 2.0)

        self.declare_parameter('publish_debug_overlay', True)
        self.declare_parameter('debug_overlay_topic', 'person/gesture_debug_image/compressed')

        wrist_roi_topic = self.get_parameter('wrist_roi_topic').value
        call_trigger_topic = self.get_parameter('call_trigger_topic').value
        min_detection_confidence = self.get_parameter('min_detection_confidence').value
        self.publish_debug_overlay = self.get_parameter('publish_debug_overlay').value
        debug_overlay_topic = self.get_parameter('debug_overlay_topic').value

        self.hands = mp.solutions.hands.Hands(
            static_image_mode=True,
            max_num_hands=1,
            min_detection_confidence=min_detection_confidence,
        )
        self.drawing_utils = mp.solutions.drawing_utils
        self.hand_connections = mp.solutions.hands.HAND_CONNECTIONS

        self.state_machine = GestureStateMachine(
            open_threshold=self.get_parameter('open_ratio_threshold').value,
            closed_threshold=self.get_parameter('closed_ratio_threshold').value,
            phase_timeout_sec=self.get_parameter('phase_timeout_sec').value,
            stale_timeout_sec=self.get_parameter('stale_timeout_sec').value,
            cooldown_sec=self.get_parameter('cooldown_sec').value,
        )

        self.roi_sub = self.create_subscription(
            CompressedImage, wrist_roi_topic, self.roi_callback, 10
        )
        self.trigger_pub = self.create_publisher(Empty, call_trigger_topic, 10)
        self.debug_pub = (
            self.create_publisher(CompressedImage, debug_overlay_topic, 1)
            if self.publish_debug_overlay else None
        )

        self.get_logger().info(
            f'wrist_gesture_node: "{wrist_roi_topic}" 구독, '
            f'제스처 확정 시 "{call_trigger_topic}"에 publish'
        )

    def roi_callback(self, msg):
        roi = decode_jpeg(msg)
        if roi is None or roi.size == 0:
            return

        # MediaPipe는 RGB를 기대하는데 카메라 프레임은 OpenCV 관례상 BGR
        rgb_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb_roi)

        ratio = None
        landmarks = None
        if result.multi_hand_landmarks:
            landmarks = result.multi_hand_landmarks[0].landmark
            ratio = compute_openness_ratio(landmarks)

        triggered = self.state_machine.update(ratio, time.monotonic())
        if triggered:
            self.get_logger().info('제스처 확정("폈다-쥐었다-폈다") -> call_trigger publish')
            self.trigger_pub.publish(Empty())

        if self.publish_debug_overlay:
            self._publish_debug_overlay(roi, landmarks, ratio)

    def _publish_debug_overlay(self, roi, landmarks, ratio):
        overlay = roi.copy()
        if landmarks is not None:
            # drawing_utils가 draw할 때 원본 mediapipe landmark 리스트 형태를
            # 기대하므로, 방금 꺼내 쓴 .landmark 대신 result 쪽 원본 객체가
            # 필요함 - 여기서는 landmark 좌표만으로 직접 원을 그려서 그 문제를 피함
            h, w = overlay.shape[:2]
            for lm in landmarks:
                cx, cy = int(lm.x * w), int(lm.y * h)
                cv2.circle(overlay, (cx, cy), 3, (0, 255, 0), -1)
        label = f'ratio={ratio:.2f} phase={self.state_machine.phase}' if ratio is not None \
            else f'no hand phase={self.state_machine.phase}'
        cv2.putText(overlay, label, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        self.debug_pub.publish(encode_jpeg(overlay, quality=80))

    def destroy_node(self):
        self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WristGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
