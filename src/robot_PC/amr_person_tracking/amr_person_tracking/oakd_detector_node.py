#!/usr/bin/env python3
"""
oakd_detector_node.py

[역할]
AMR(터틀봇4) 탑재 OAK-D PRO 카메라 기반 근접 사람 검출 노드.
전체 비전 파이프라인 문서의 "4번 위치추정 - OAKD PRO" + 변경된 "OAK-D 컨버터 노드" 역할을
합친 노드로, 검출 자체(YOLO-pose 추론)부터 이 노드에서 직접 수행한다.

[처리 흐름]
RGB(compressed) + Depth(compressedDepth) 시간동기 구독 (CameraInfo는 별도 캐시)
  -> YOLO-pose로 사람 bbox + 발끝(ankle) keypoint 추출
  -> 발끝 픽셀 좌표의 depth 값을 camera_info 기반 3D 역투영 (카메라 optical frame 기준)
  -> tf2로 base_link(근접 판정) / map(발행) 변환
  -> vision_msgs/Detection3DArray로 발행

[유효 거리 - 실측 기하 기준]
이 카메라는 틸트 없이 수평으로 달려 있고 FOV는 63.8°다.
따라서 거리별로 보이는 최대 높이가 1.0m->0.87m, 1.5m->1.18m, 2.0m->1.49m, 2.5m->1.80m이라
2.5m 이내에서는 사람 머리가 항상 잘린다. 반대로 지면은 0.39m 앞부터 화각에 들어와
발끝은 오히려 근접할수록 잘 보인다. 또 스테레오 최소 측정거리 때문에 0.7m 아래는 depth가 없다.
=> OAK-D 유효구간은 대략 1.0~2.5m이고, 그보다 가까우면 라이다 다리검출이 주도해야 한다.
   이 노드는 그 열화를 covariance / foot_<grade> / view_truncated_top 세 신호로 스스로 신고한다.
   (주의: view_truncated_top은 현재 **발행만 되고 소비하는 노드가 없다**. reid_tracking_node의
    라이다 주도권 전환은 이 신호가 아니라 자체 락온 로직으로 판단한다.)

[입력 토픽]
  - <namespace>/oakd/rgb/image_raw/compressed         (sensor_msgs/CompressedImage)
  - <namespace>/oakd/stereo/image_raw/compressedDepth (sensor_msgs/CompressedImage)
  - <namespace>/oakd/rgb/camera_info                  (sensor_msgs/CameraInfo)
  - <namespace>/ir_intensity                          (irobot_create_msgs/IrIntensityVector, 선택)
  - <namespace>/odom                                  (nav_msgs/Odometry, 선택)

[출력 토픽]
  - <namespace>/vision/detections_3d   (vision_msgs/Detection3DArray, frame_id=map)
  - <namespace>/vision/proximity_alert (std_msgs/Bool, TRANSIENT_LOCAL 래치)
  - <namespace>/diagnostics            (diagnostic_msgs/DiagnosticArray)

[참고]
  - 하위 노드(재식별/트래킹, 예측 회피)는 출력 스키마(Detection3DArray)만 보고 동작하므로,
    검출 로직 내부 구현이 바뀌어도 이 노드 밖으로는 영향이 가지 않는다.
  - 총괄 매니저 노드가 추후 붙을 것을 전제로, 커스텀 메시지를 만들지 않고 표준 스키마만 쓴다.
    매니저는 위 세 토픽을 그대로 구독하면 되고 이 노드는 수정할 필요가 없다.
  - 로봇<->노트북 대역폭 절약을 위해 이미지 입력은 compressed 포맷으로만 구독한다.
"""

import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from message_filters import ApproximateTimeSynchronizer, Subscriber
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (PointStamped tf2 변환 등록)
from ultralytics import YOLO

from amr_person_tracking.vision_utils import (
    GRADE_SCORE,
    GRADE_SIGMA,
    IR_SENSOR_YAW_FALLBACK,
    angular_diff,
    backproject_pixel,
    decode_compressed_depth,
    estimate_person_depth,
    foot_pixel_candidates,
    lower_body_visibility_rank,
    sample_depth_patch,
    truncation_ratio,
    yaw_from_quaternion,
)

# 사람 bbox 3D 크기 근사 상수 (m). 실측이 아니라 Detection3D.bbox를 비워두지 않기 위한 자리값이다.
PERSON_BBOX_SIZE = (0.5, 0.5, 1.7)


class OakdDetectorNode(Node):

    def __init__(self):
        super().__init__('oakd_detector_node')

        self._declare_parameters()

        namespace = self.get_parameter('namespace').value
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        slop = self.get_parameter('sync_slop').value
        queue_size = self.get_parameter('sync_queue_size').value

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout').value)
        self.tf_allow_latest_fallback = self.get_parameter('tf_allow_latest_fallback').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.max_detection_distance = self.get_parameter('max_detection_distance').value
        self.duplicate_merge_distance = self.get_parameter('duplicate_merge_distance').value
        self.require_lower_body = self.get_parameter('require_lower_body').value
        self.single_person_only = self.get_parameter('single_person_only').value
        self.tracker_conf_threshold = self.get_parameter('tracker_conf_threshold').value
        self.kp_conf_threshold = self.get_parameter('kp_conf_threshold').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.depth_patch_size = self.get_parameter('depth_patch_size').value
        self.depth_consistency_tolerance = self.get_parameter('depth_consistency_tolerance').value
        self.depth_person_percentile = self.get_parameter('depth_person_percentile').value
        self.process_every_n = max(1, int(self.get_parameter('process_every_n').value))
        self.min_z_safety_threshold = self.get_parameter('min_z_safety_threshold').value
        self.proximity_release_margin = self.get_parameter('proximity_release_margin').value
        self.proximity_lost_release_sec = self.get_parameter('proximity_lost_release_sec').value
        self.enable_ir_safety_check = self.get_parameter('enable_ir_safety_check').value
        self.ir_proximity_threshold = self.get_parameter('ir_proximity_threshold').value
        self.ir_max_age = self.get_parameter('ir_max_age').value
        self.input_timeout = self.get_parameter('input_timeout').value
        self.truncation_margin_px = self.get_parameter('truncation_margin_px').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        self.follow_target_max_age = self.get_parameter('follow_target_max_age').value
        self.follow_target_match_distance = self.get_parameter('follow_target_match_distance').value
        self.log_follow_overlay_misses = self.get_parameter('log_follow_overlay_misses').value

        # ---- 캐시 상태 (콜백이 갱신, 동기 콜백/타이머가 소비) ----
        self.camera_k = None          # (fx, fy, cx, cy)
        self.camera_frame_id = None
        self.ir_readings = None
        self.ir_stamp = None
        self.ir_yaw_cache = {}
        # reid_tracking_node가 발행하는 "실제 추종 대상" - 디버그 이미지에 어느 검출이 AMR의
        # 최종 추종 대상인지 표시하는 용도로만 쓴다(제어에는 관여하지 않음).
        self.latest_follow_target = None  # (x, y, stamp_sec)
        self.robot_speed = 0.0
        self.proximity_active = False
        self._proximity_lost_since = None
        self.frame_index = 0
        self.last_frame_time = None
        self.last_rgb_depth_dt = 0.0

        # ---- 1주기 진단 카운터 ----
        self._stat_frames = 0
        # 상류 트래커가 이 프레임에 id를 하나도 못 낸 횟수. reid_tracking_node의 상류 id 구제가
        # 무력화되는 조건이라(det.id가 빈 문자열로 나감) 트랙 churn의 직접 원인 후보다.
        self._stat_no_track_id = 0
        self._stat_too_far = 0
        self._stat_duplicates = 0
        self._stat_no_lower_body = 0
        self._stat_single_person_suppressed = 0
        self._stat_detections = 0
        self._stat_depth_invalid = 0
        self._stat_depth_substituted = 0
        self._stat_tf_failures = 0
        self._stat_truncated = 0
        self._stat_grades = {g: 0 for g in GRADE_SIGMA}
        self._stat_nearest = None
        self._stat_ir_check = 'idle'
        self._stat_ir_value = 0
        self._diag_last_time = time.monotonic()

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        model_path = self.get_parameter('pose_model_path').value
        device = self.get_parameter('device').value
        self.tracker_config_path = self.get_parameter('tracker_config_path').value or None
        self.get_logger().info(f'YOLO-pose 모델 로드: {model_path}')
        if self.tracker_config_path:
            self.get_logger().info(f'커스텀 트래커 설정 사용: {self.tracker_config_path}')
        try:
            self.pose_model = YOLO(model_path)
            if device:
                self.pose_model.to(device)
        except Exception as exc:
            self.pose_model = None
            self.get_logger().error(f'YOLO-pose 모델 로드 실패: {exc}')

        # 외형 재식별 인코더. 로드에 실패해도 검출 자체는 계속돼야 하므로(임베딩은 보조 신호)
        # None으로 두고 넘어간다 - reid_tracking_node도 임베딩이 없으면 위치 기반으로 degrade한다.
        reid_path = self.get_parameter('reid_model_path').value
        self.reid_encoder = None
        if reid_path:
            try:
                from ultralytics.trackers.utils.reid import ReID
                self.reid_encoder = ReID(reid_path)
                self.get_logger().info(f'ReID 인코더 로드: {reid_path}')
            except Exception as exc:
                self.get_logger().error(f'ReID 인코더 로드 실패(임베딩 없이 계속): {exc}')

        # ---- 입력 ----
        # RGB와 Depth만 동기화한다. CameraInfo는 정적이라 동기화 대상에 넣으면
        # 매칭 실패로 프레임을 통째로 버릴 위험만 커진다.
        self.rgb_sub = Subscriber(self, CompressedImage, rgb_topic, qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, CompressedImage, depth_topic, qos_profile=qos_profile_sensor_data)
        self.time_sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=queue_size,
            slop=slop,
        )
        self.time_sync.registerCallback(self.sync_callback)

        self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_callback, qos_profile_sensor_data)

        if self.enable_ir_safety_check:
            # irobot_create_msgs는 터틀봇4 환경에만 있으므로, 기능을 끈 경우엔 import조차 하지 않는다.
            from irobot_create_msgs.msg import IrIntensityVector
            self.create_subscription(
                IrIntensityVector,
                self.get_parameter('ir_intensity_topic').value,
                self.ir_intensity_callback,
                qos_profile_sensor_data,
            )

        # ---- 출력 ----
        # 고빈도 스트림(검출, 디버그 이미지)은 best_effort로 - 유실보다 최신 프레임 지연 없이
        # 넘기는 게 더 중요하다. 저빈도 이벤트(진단, 근접 경보)는 reliable을 유지한다.
        streaming_qos = QoSProfile(depth=10)
        streaming_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        streaming_qos.history = HistoryPolicy.KEEP_LAST
        streaming_qos.durability = DurabilityPolicy.VOLATILE

        self.detections_pub = self.create_publisher(Detection3DArray, detections_topic, streaming_qos)

        # 검출별 외형 임베딩 동반 토픽. vision_msgs/Detection3D에 임베딩 필드가 없고 이
        # 패키지가 ament_python이라 커스텀 메시지를 만들기 어려워, sensor_msgs/Image를
        # 부동소수 행렬 운반체로 쓴다: encoding=32FC1, height=검출수 N, width=임베딩차원 D.
        # [규약] header.stamp가 같은 순간의 detections_topic 메시지와 동일하고, 행 i가
        # 그 메시지의 detections[i]에 대응한다(같은 콜백에서 연달아 발행하므로 순서 보장).
        self.reid_pub = (
            self.create_publisher(
                Image, self.get_parameter('reid_embeddings_topic').value, streaming_qos)
            if self.reid_encoder is not None else None
        )

        # 근접 경보는 나중에 뜨는 매니저 노드가 구독 즉시 현재 상태를 받아야 하므로 래치한다.
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.history = HistoryPolicy.KEEP_LAST
        self.proximity_pub = self.create_publisher(
            Bool, self.get_parameter('proximity_alert_topic').value, latched_qos)
        self.proximity_pub.publish(Bool(data=False))

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, self.get_parameter('diagnostics_topic').value, 10)

        self.debug_pub = (
            self.create_publisher(
                CompressedImage, self.get_parameter('debug_image_topic').value,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE))
            if self.publish_debug_image else None
        )
        if self.publish_debug_image:
            self.create_subscription(
                PoseStamped, self.get_parameter('follow_target_topic').value,
                self.follow_target_callback, qos_profile_sensor_data)

        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

        self.get_logger().info(
            f'oakd_detector_node 시작 | ns={namespace} rgb={rgb_topic} depth={depth_topic} '
            f'-> {detections_topic} (slop={slop}s, process_every_n={self.process_every_n})'
        )

    def _declare_parameters(self):
        self.declare_parameter('namespace', 'robot5')
        self.declare_parameter('rgb_topic', '/robot5/oakd/rgb/image_raw/compressed')
        self.declare_parameter('depth_topic', '/robot5/oakd/stereo/image_raw/compressedDepth')
        # depth가 RGB에 정렬(i_align_depth)돼 나오고 두 CameraInfo가 동일하므로 rgb 쪽을 쓴다.
        self.declare_parameter('camera_info_topic', '/robot5/oakd/rgb/camera_info')
        self.declare_parameter('detections_topic', '/robot5/vision/detections_3d')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # 동기 신뢰도 노브. slop을 조이면 짝의 시간차가 보장되는 대신 매칭되는 프레임 수가 줄어든다.
        # 실측(bag rosbag2_2026_08_06-12_27_59) 기준 slop=0.05에서 RGB의 49%가 매칭돼 약 4.9fps가 나온다.
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('sync_queue_size', 10)
        # 추론 부하 노브. 동기 품질과 무관하게 콜백에서 프레임을 건너뛴다. CPU 추론이면 2 이상.
        self.declare_parameter('process_every_n', 1)

        self.declare_parameter('pose_model_path', 'yolo11n-pose.pt')
        self.declare_parameter('device', '')
        # 빈 문자열이면 Ultralytics 기본 BoT-SORT 설정을 사용한다. 운영 런치는
        # config/botsort_reid.yaml의 절대 설치 경로를 전달해 ReID와 확장된 track buffer를 쓴다.
        self.declare_parameter('tracker_config_path', '')
        # 전용 ReID 모델 경로(예: yolo26n-reid.onnx). 빈 문자열이면 임베딩 기능 자체를 끄고
        # 기존 동작을 그대로 유지한다. 사람이 화면 밖으로 나갔다 재등장할 때 신원을 잇기 위해
        # reid_tracking_node가 쓰는 외형 특징을 여기서 계산해 동반 토픽으로 내보낸다.
        # 기술자 선택 근거는 tools/reid_embedding_eval.py 참고(전용 ReID만 임계가 견고함).
        self.declare_parameter('reid_model_path', '')
        self.declare_parameter('reid_embeddings_topic',
                               '/robot5/vision/detection_embeddings')
        # 근접 구간에서는 상반신이 잘려 COCO 학습 검출기의 person 점수가 낮게 나오므로 기본값을 낮춘다.
        # 주의: 이 0.3은 위 [유효 거리] 기하로부터 추론한 값이고 아직 실측으로 확정하지 못했다.
        # 검증에 쓴 bag(rosbag2_2026_08_06-12_27_59)에는 사람이 찍히지 않아 분포를 볼 수 없었다.
        # 사람이 나오는 근접 주행 bag이 생기면 person conf 분포를 보고 다시 정해야 한다.
        self.declare_parameter('conf_threshold', 0.3)
        # base_link 기준 이 거리를 넘는 검출은 발행하지 않는다(0 이하면 끔).
        # 원거리 검출은 depth 오차가 커서 map 좌표가 프레임마다 수 m씩 튀는데, 그런 유령
        # 검출이 만든 트랙이 추종 트랙을 끌고 가 실제로 대상을 잃게 만들었다(실기 16:29 로그:
        # 3~6m에 상류id oakd_7 하나가 (-3.54,2.41)/(-2.83,2.06)/(-4.66,3.52)로 튀는 동안
        # 추종 track 1이 y -0.28 -> +1.51로 1.8m 끌려간 뒤 소멸). 사람 추종에 원거리 검출은
        # 쓰이지 않는다 - 원거리 획득은 웹캠 호출좌표 담당이고, 라이다도
        # scan_range_limit=5.0으로 같은 상한을 쓴다.
        self.declare_parameter('max_detection_distance', 5.0)
        # map 좌표가 이 거리 안에서 겹치는 검출은 한 사람으로 보고 신뢰도 높은 쪽만 남긴다.
        # gating_min_gate(0.3m)보다 작게 잡아, 게이팅이 구분할 수 있는 거리는 건드리지 않는다.
        self.declare_parameter('duplicate_merge_distance', 0.25)
        # 단일 사용자 전용 운영: 얼굴/상체만 보이는 원거리 검출은 버리고, 좌우 중 적어도
        # 한쪽 무릎+발목이 함께 보이는 사람만 3D 추적 입력으로 사용한다.
        self.declare_parameter('require_lower_body', True)
        # 조건을 통과한 사람이 여러 명이어도 하체 가시성이 가장 좋은 한 명만 발행한다.
        self.declare_parameter('single_person_only', True)
        # 트래커에 넣을 때만 쓰는 낮은 임계. YOLO의 .track()에 conf_threshold를 그대로 주면
        # 저신뢰 검출이 트래커에 도달하기도 전에 잘려, TrackTrack/ByteTrack 계열의 2단계
        # 연결(track_low_thresh=0.25로 저신뢰 검출을 기존 트랙에 이어붙여 끊김을 막는 장치)이
        # 쓸 후보가 아예 없어진다. 이 값을 track_low_thresh보다 낮게 줘서 그 장치를 살리고,
        # 발행은 위 conf_threshold로 따로 거른다(_build_detections) - 하류가 받는 검출 품질은
        # 그대로 두면서 트랙 연속성만 얻는다. new_track_thresh(0.7)가 있어 저신뢰 검출이
        # 함부로 새 트랙을 만들지도 않는다.
        self.declare_parameter('tracker_conf_threshold', 0.1)
        self.declare_parameter('kp_conf_threshold', 0.5)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('depth_patch_size', 5)
        # 발끝 depth가 bbox 안 '사람 표면 깊이'와 이보다 더 어긋나면 사람 깊이로 대체한다.
        # 0 이하면 검증을 끈다. 실측 근거는 estimate_person_depth 독스트링 참고.
        self.declare_parameter('depth_consistency_tolerance', 0.4)
        self.declare_parameter('depth_person_percentile', 20.0)
        self.declare_parameter('truncation_margin_px', 4)

        self.declare_parameter('tf_timeout', 0.2)
        self.declare_parameter('tf_allow_latest_fallback', False)
        self.declare_parameter('odom_topic', '/robot5/odom')

        # 근접 안전모드 전환 임계값 (m, base_link 기준 수평거리).
        # 스테레오 최소 측정거리가 약 0.7m라 그보다 낮게 잡으면 depth로는 도달할 수 없다.
        self.declare_parameter('min_z_safety_threshold', 0.8)
        self.declare_parameter('proximity_release_margin', 0.1)
        # 근접 경보가 켜진 상태에서 검출이 사라졌을 때, 이 시간이 지나면 해제한다.
        # 0 이하로 두면 해제하지 않는다(옛 동작 = 무기한 래치).
        self.declare_parameter('proximity_lost_release_sec', 1.0)
        self.declare_parameter('proximity_alert_topic', '/robot5/vision/proximity_alert')
        self.declare_parameter('enable_ir_safety_check', True)
        self.declare_parameter('ir_intensity_topic', '/robot5/ir_intensity')
        # IR 원시값 임계. 실기에서 사람/벽 반사율에 맞춰 튜닝이 필요하다.
        self.declare_parameter('ir_proximity_threshold', 100)
        self.declare_parameter('ir_max_age', 0.5)

        self.declare_parameter('diagnostics_topic', '/robot5/diagnostics')
        self.declare_parameter('input_timeout', 3.0)
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_topic', '/robot5/vision/oakd_detector/debug/compressed')
        # 디버그 이미지에 "실제로 AMR이 추종 중인 대상"을 표시하기 위한 구독 - oakd_id는 이
        # 카메라 하나의 내부 트래커 라벨일 뿐, 최종적으로 로봇이 따라가는 대상은
        # reid_tracking_node가 융합해 발행하는 target_person_pose다. 순수 시각화 목적이라
        # publish_debug_image가 꺼져 있으면 구독하지 않는다.
        self.declare_parameter('follow_target_topic', '/robot5/target_person_pose')
        self.declare_parameter('follow_target_max_age', 1.0)
        # 추종 대상 좌표와 검출의 map 거리가 이보다 멀면 "그 검출은 추종 대상이 아니다"로 보고
        # FOLLOWING 표시를 하지 않는다. 이 상한이 없으면 화면의 유일한 검출이 무조건 칠해져
        # 오버레이가 착시를 준다.
        self.declare_parameter('follow_target_match_distance', 0.8)
        # 화면에 검출이 있는데 FOLLOWING이 안 칠해진 이유를 로그로 남긴다(2초 throttle).
        self.declare_parameter('log_follow_overlay_misses', False)

    # ------------------------------------------------------------------ 캐시 콜백

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx, fy, cx, cy
        self.camera_frame_id = msg.header.frame_id

    def odom_callback(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.robot_speed = math.hypot(v.x, v.y)

    def ir_intensity_callback(self, msg):
        self.ir_readings = msg.readings
        self.ir_stamp = self.get_clock().now()

    def follow_target_callback(self, msg: PoseStamped):
        stamp = _stamp_to_sec(msg.header.stamp)
        self.latest_follow_target = (msg.pose.position.x, msg.pose.position.y, stamp)

    # ------------------------------------------------------------------ 메인 파이프라인

    def sync_callback(self, rgb_msg: CompressedImage, depth_msg: CompressedImage):
        self.frame_index += 1
        if self.frame_index % self.process_every_n != 0:
            return

        self.last_frame_time = time.monotonic()

        if self.pose_model is None:
            return
        if self.camera_k is None:
            self.get_logger().warn('CameraInfo 미수신 - 역투영 불가', throttle_duration_sec=2.0)
            return

        frame = cv2.imdecode(np.frombuffer(rgb_msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('RGB 디코딩 실패', throttle_duration_sec=2.0)
            return

        depth_img = decode_compressed_depth(depth_msg.data)
        if depth_img is None:
            self.get_logger().warn('depth 디코딩 실패', throttle_duration_sec=2.0)
            return

        # ApproximateTimeSynchronizer가 짝지어준 두 프레임의 실제 시간차.
        # slop 이내가 보장되지만 그 안에서의 편차는 아래 covariance에 반영한다.
        self.last_rgb_depth_dt = abs(_stamp_to_sec(rgb_msg.header.stamp) - _stamp_to_sec(depth_msg.header.stamp))

        results = self.pose_model.track(
            frame, persist=True, classes=[0], conf=self.tracker_conf_threshold, verbose=False,
            # 설치된 Ultralytics가 공식 지원하는 BoT-SORT 설정을 사용한다.
            tracker=self.tracker_config_path or 'botsort.yaml')
        detections, nearest, overlay_items = self._build_detections(results[0], depth_img, rgb_msg)

        self._update_proximity(nearest)
        # 임베딩을 detections보다 **먼저** 발행한다. 구독자(reid_tracking_node)는 detections를
        # 받은 그 콜백 안에서 같은 stamp의 임베딩을 찾는데, detections를 먼저 보내면 임베딩이
        # 아직 도착하지 않아 매번 위치 기반으로 폴백해버린다(실제로 겪음 - 외형 매칭이 통째로
        # 무시됐다). 같은 stamp, 같은 순서라는 규약은 그대로다.
        self._publish_embeddings(frame, overlay_items, rgb_msg.header)
        self.publish_detections(detections, rgb_msg.header)

        self._stat_frames += 1
        if results and getattr(results[0].boxes, 'id', None) is None and len(results[0].boxes):
            self._stat_no_track_id += 1
        self._stat_detections += len(detections)
        if nearest is not None:
            dist = nearest[0]
            self._stat_nearest = dist if self._stat_nearest is None else min(self._stat_nearest, dist)

        if self.debug_pub is not None:
            self._publish_debug_image(results[0], overlay_items, rgb_msg.header.stamp)

    def _build_detections(self, result, depth_img, rgb_msg):
        """YOLO 결과 하나를 Detection3D 목록으로 바꾼다.

        반환: (detections, nearest, overlay_items).
        nearest는 최근접 대상의 (거리 m, base_link 기준 방위각 rad) 또는 None.
        방위각은 근접 안전모드에서 어느 IR 센서를 볼지 고르는 데 쓴다.
        """
        detections = []
        overlay_items = []
        confs = []          # detections와 같은 순서. 중복 병합 시 어느 쪽을 남길지 고른다.
        nearest = None

        boxes = getattr(result, 'boxes', None)
        if boxes is None or len(boxes) == 0:
            return detections, nearest, overlay_items

        keypoints = getattr(result, 'keypoints', None)
        kp_xy_all = keypoints.xy.cpu().numpy() if keypoints is not None and keypoints.xy is not None else None
        kp_conf_all = (
            keypoints.conf.cpu().numpy()
            if keypoints is not None and getattr(keypoints, 'conf', None) is not None else None
        )

        eligible = []
        for i in range(len(boxes)):
            bbox = boxes.xyxy[i].cpu().numpy()
            box_conf = float(boxes.conf[i])
            if box_conf < self.conf_threshold:
                continue
            kp_xy = kp_xy_all[i] if kp_xy_all is not None and i < len(kp_xy_all) else None
            kp_conf = kp_conf_all[i] if kp_conf_all is not None and i < len(kp_conf_all) else None
            lower_rank = lower_body_visibility_rank(
                kp_xy, kp_conf, self.kp_conf_threshold)
            if self.require_lower_body and lower_rank is None:
                self._stat_no_lower_body += 1
                continue
            eligible.append((i, lower_rank or (0, 0, 0.0), box_conf))

        if self.single_person_only and len(eligible) > 1:
            best = max(eligible, key=lambda item: (*item[1], item[2]))
            self._stat_single_person_suppressed += len(eligible) - 1
            eligible = [best]

        for i, _lower_rank, box_conf in eligible:
            bbox = boxes.xyxy[i].cpu().numpy()
            track_id = int(boxes.id[i]) if boxes.id is not None else -1
            kp_xy = kp_xy_all[i] if kp_xy_all is not None and i < len(kp_xy_all) else None
            kp_conf = kp_conf_all[i] if kp_conf_all is not None and i < len(kp_conf_all) else None

            # [접지점 후보 중 depth로 고르기] 좌우 발목을 평균내면 그 중점이 두 다리 사이
            # 허공이라 뒤쪽 바닥/벽 거리가 찍힌다(실측 22.1%). 그래서 각 발목을 후보로 받아
            # 몸통 깊이에 가장 가까운 후보를 채택한다 - 실측상 배경 오샘플이 0%가 된다.
            # 몸통 깊이를 못 구하면(측정 실패 등) 첫 후보를 그대로 쓴다.
            candidates = foot_pixel_candidates(kp_xy, kp_conf, bbox, self.kp_conf_threshold)
            person_z = estimate_person_depth(
                depth_img, bbox, self.depth_scale, self.depth_person_percentile)

            sampled = []
            for cu, cv, cgrade in candidates:
                cz = sample_depth_patch(
                    depth_img, cu, cv, self.depth_patch_size, self.depth_scale)
                if cz is not None and cz > 0.0:
                    sampled.append((cu, cv, cz, cgrade))
            if not sampled:
                self._stat_depth_invalid += 1
                continue

            # 방향(u, v)은 후보들의 **중점**을 쓴다. 사람의 좌우 중심이라 안정적이다.
            # 거리(z)는 **각 발목에서 실제로 잰 값의 평균**을 쓴다.
            # 예전에는 중점 픽셀에서 depth까지 같이 읽었는데, 걸을 때 그 지점은 두 다리 사이
            # 허공이라 뒤쪽 벽/바닥이 찍혔다(실측 22.1%). 반대로 발목 하나만 골라 쓰면 앞발/뒷발이
            # 프레임마다 번갈아 선택돼 보폭만큼 진동한다. 방향은 중점, 거리는 발목 - 이 조합이
            # 실측에서 가장 좋았다(0.5m 초과 점프: 중점 12회 / 발목 택1 6회 / 이 방식 2회).
            u = sum(s[0] for s in sampled) / len(sampled)
            v = sum(s[1] for s in sampled) / len(sampled)
            z = sum(s[2] for s in sampled) / len(sampled)
            grade = sampled[0][3]

            # [발끝 depth 검증] 발끝 픽셀 주변이 사람이 아니라 뒤쪽 벽/바닥을 찍는 일이 잦다
            # (실측: my_new_bag5에서 발끝 depth로 배경 거리 2.51m가 반복 등장). 그러면 역투영
            # 에서 거리 오차가 그대로 위치 오차가 돼 추종 좌표가 1m 넘게 순간이동한다.
            # bbox 안쪽에서 구한 '사람 표면 깊이'와 크게 어긋나면 그 값으로 대체한다
            # (실측: 원시 측정 0.5m 초과 점프 12회 -> 4회).
            # 위에서 후보를 골랐는데도 몸통 깊이와 크게 어긋나면(두 발목이 다 가려진 경우 등)
            # 최후 안전망으로 몸통 깊이를 쓴다. 후보 선택이 잘 되면 거의 발동하지 않아야 한다
            # (depth_substituted 진단값으로 확인).
            if self.depth_consistency_tolerance > 0.0:
                if person_z is not None and abs(z - person_z) > self.depth_consistency_tolerance:
                    z = person_z
                    self._stat_depth_substituted += 1
                    # 발끝을 직접 못 재고 몸통 깊이로 대체한 것이므로 신뢰 등급을 낮춘다.
                    # (이 등급이 covariance로 나가 하류가 불확실도를 알 수 있어야 한다)
                    if grade == 'toe_direct':
                        grade = 'knee_corrected'

            fx, fy, cx, cy = self.camera_k
            x_cam, y_cam, z_cam = backproject_pixel(u, v, z, fx, fy, cx, cy)

            pt_cam = PointStamped()
            pt_cam.header.frame_id = self.camera_frame_id
            pt_cam.header.stamp = rgb_msg.header.stamp
            pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = x_cam, y_cam, z_cam

            pt_base = self._transform(pt_cam, self.base_frame)
            pt_map = self._transform(pt_cam, self.map_frame)
            if pt_base is None or pt_map is None:
                self._stat_tf_failures += 1
                continue

            distance = math.hypot(pt_base.point.x, pt_base.point.y)
            if 0.0 < self.max_detection_distance < distance:
                self._stat_too_far += 1
                continue
            bearing = math.atan2(pt_base.point.y, pt_base.point.x)
            if nearest is None or distance < nearest[0]:
                nearest = (distance, bearing)

            trunc = truncation_ratio(bbox, self.truncation_margin_px)
            if trunc > 0.0:
                self._stat_truncated += 1
            self._stat_grades[grade] = self._stat_grades.get(grade, 0) + 1

            detections.append(self._make_detection3d(pt_map, box_conf, grade, trunc, track_id, rgb_msg.header))
            overlay_items.append(
                (u, v, grade, distance, track_id, pt_map.point.x, pt_map.point.y, bbox))
            confs.append(box_conf)

        detections, overlay_items = self._merge_duplicates(detections, overlay_items, confs)
        return detections, nearest, overlay_items

    def _merge_duplicates(self, detections, overlay_items, confs):
        """map 좌표가 거의 겹치는 검출을 하나로 줄인다(신뢰도 높은 쪽을 남김).

        한 사람이 한 프레임에 두 번 검출되면(겹치는 bbox) 하류 reid_tracking_node의 배타적
        배정 때문에 **반드시** 하나는 새 트랙이 된다 - 위치 게이팅을 아무리 손봐도 막을 수
        없다. 실기(evidence/live_run_0809_1706.log)에서 같은 시각에 2cm 떨어진 트랙 6/7이
        생겼고 7은 수명 0.00s로 즉사했다. 그런 검출쌍은 같은 사람으로 보는 게 맞다.

        임계는 게이팅 반경(gating_min_gate 0.3m)보다 작게 잡아, 게이팅이 정상적으로 구분할
        수 있는 거리(서로 다른 사람)는 병합하지 않는다.
        """
        if self.duplicate_merge_distance <= 0.0 or len(detections) < 2:
            return detections, overlay_items
        order = sorted(range(len(detections)), key=lambda i: confs[i], reverse=True)
        kept = []
        for i in order:
            xi, yi = overlay_items[i][5], overlay_items[i][6]
            if any(math.hypot(xi - overlay_items[k][5], yi - overlay_items[k][6])
                   <= self.duplicate_merge_distance for k in kept):
                self._stat_duplicates += 1
                continue
            kept.append(i)
        if len(kept) == len(detections):
            return detections, overlay_items
        kept.sort()   # 원래 검출 순서를 유지한다(임베딩 인덱스 규약과 맞춰야 한다)
        return [detections[i] for i in kept], [overlay_items[i] for i in kept]

    def _make_detection3d(self, pt_map, box_conf, grade, trunc, track_id, header):
        det = Detection3D()
        # header를 통째로 대입(참조 공유)하면 안 된다 - leg_detector_bridge_node/
        # reid_tracking_node에서 이미 고친 것과 같은 버그: 아래 frame_id 변경이 호출자가
        # 넘긴 원본 rgb_msg.header까지 오염시켜, 같은 콜백에서 이후에 header를 다시 쓰는
        # 코드(publish_detections)가 이미 바뀐 값을 읽게 된다. stamp만 값으로 복사한다.
        det.header.stamp = header.stamp
        det.header.frame_id = self.map_frame

        # 등급이 주는 위치 불확실성에, 동기 시간차 동안 로봇이 움직인 거리를 더한다.
        # 하위 칼만필터가 이 covariance를 관측 노이즈로 그대로 쓰기 때문에,
        # 동기 품질 열화가 파이프라인 전체에 자동으로 전파된다.
        sigma = GRADE_SIGMA.get(grade, GRADE_SIGMA['angle_only'])
        variance = sigma ** 2 + (self.robot_speed * self.last_rgb_depth_dt) ** 2

        primary = ObjectHypothesisWithPose()
        primary.hypothesis = ObjectHypothesis(class_id='person', score=box_conf)
        primary.pose.pose.position = pt_map.point
        primary.pose.pose.orientation.w = 1.0
        primary.pose.covariance[0] = variance
        primary.pose.covariance[7] = variance
        primary.pose.covariance[14] = variance
        det.results.append(primary)

        # 좌표 산출 방식 등급. 'person'만 보는 하위 노드는 이 항목을 무시하면 된다.
        foot = ObjectHypothesisWithPose()
        foot.hypothesis = ObjectHypothesis(
            class_id=f'foot_{grade}', score=GRADE_SCORE.get(grade, 0.0))
        foot.pose.pose.position = pt_map.point
        foot.pose.pose.orientation.w = 1.0
        det.results.append(foot)

        # 상반신이 잘린 검출임을 알리는 신호. 재식별/트래킹 노드가 라이다로 주도권을
        # 넘길지 판단하는 근거가 된다.
        if trunc > 0.0:
            truncated = ObjectHypothesisWithPose()
            truncated.hypothesis = ObjectHypothesis(class_id='view_truncated_top', score=trunc)
            truncated.pose.pose.position = pt_map.point
            truncated.pose.pose.orientation.w = 1.0
            det.results.append(truncated)

        det.bbox.center.position = pt_map.point
        det.bbox.center.orientation.w = 1.0
        det.bbox.size = Vector3(
            x=PERSON_BBOX_SIZE[0], y=PERSON_BBOX_SIZE[1], z=PERSON_BBOX_SIZE[2])

        # 이미지 평면 트래커(ByteTrack) ID. 프레임 간 단기 연속성만 보장한다.
        # 재식별/트래킹 노드가 map 기준 지속 트랙 ID로 덮어쓴다.
        det.id = f'oakd_{track_id}' if track_id >= 0 else ''
        return det

    def _transform(self, pt_cam, target_frame):
        try:
            return self.tf_buffer.transform(pt_cam, target_frame, timeout=self.tf_timeout)
        except Exception as exc:
            if self.tf_allow_latest_fallback:
                try:
                    pt_latest = PointStamped()
                    pt_latest.header.frame_id = pt_cam.header.frame_id
                    pt_latest.point = pt_cam.point  # stamp를 비우면 최신 변환을 쓴다
                    return self.tf_buffer.transform(pt_latest, target_frame, timeout=self.tf_timeout)
                except Exception:
                    pass
            self.get_logger().warn(
                f'{target_frame} 변환 실패: {exc}', throttle_duration_sec=2.0)
            return None

    # ------------------------------------------------------------------ 근접 안전모드

    def _update_proximity(self, nearest):
        """거리 + IR 근접센서 교차확인으로 근접 안전모드 상태를 갱신하고, 변할 때만 발행한다."""
        if nearest is None:
            self._stat_ir_check = 'idle'
            # 검출이 없다고 그냥 돌아가면, 켜져 있던 경보가 영영 해제되지 않는다 - 퍼블리셔가
            # TRANSIENT_LOCAL 래치라 마지막 True가 계속 남고, 더 먼 사람을 다시 검출해야만
            # 풀렸다. 사람이 초근접에서 프레임 밖으로 나가거나 depth가 무효가 되는 상황이라
            # 실제로 도달 가능한 상태다. 다만 한두 프레임 유실로 깜빡이면 안 되므로
            # proximity_lost_release_sec 동안 유지한 뒤 해제한다.
            if not self.proximity_active or self.proximity_lost_release_sec <= 0.0:
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._proximity_lost_since is None:
                self._proximity_lost_since = now
                return
            if now - self._proximity_lost_since < self.proximity_lost_release_sec:
                return
            self.proximity_active = False
            self._proximity_lost_since = None
            self.proximity_pub.publish(Bool(data=False))
            self.get_logger().warn(
                f'근접 경보 해제: 검출이 {self.proximity_lost_release_sec:.1f}s 이상 없음')
            return

        self._proximity_lost_since = None

        distance, bearing = nearest
        if self.proximity_active:
            # 히스테리시스: 켤 때보다 조금 더 멀어져야 해제해 경계에서 깜빡이지 않게 한다.
            should_alert = distance <= self.min_z_safety_threshold + self.proximity_release_margin
        else:
            should_alert = distance <= self.min_z_safety_threshold
            if should_alert and self.enable_ir_safety_check:
                should_alert = self._ir_confirms(bearing)

        if should_alert != self.proximity_active:
            self.proximity_active = should_alert
            self.proximity_pub.publish(Bool(data=should_alert))
            self.get_logger().info(
                f'근접 안전모드 {"진입" if should_alert else "해제"} (최근접 {distance:.2f}m)')

    def _ir_confirms(self, bearing):
        """추적 방향에 해당하는 IR 근접센서가 실제로 반응하는지 확인한다.

        IR 토픽이 없거나 오래됐으면 교차확인을 포기하고 거리 조건만으로 판정한다.
        (센서 미발행 때문에 안전 기능이 통째로 죽는 것보다 낫다.)
        """
        if not self.ir_readings or self.ir_stamp is None:
            self._stat_ir_check = 'unavailable'
            return True
        age = (self.get_clock().now() - self.ir_stamp).nanoseconds * 1e-9
        if age > self.ir_max_age:
            self._stat_ir_check = 'stale'
            return True

        value = self._nearest_ir_value(bearing)
        if value is None:
            # 센서 프레임의 방위각을 tf에서도 폴백 테이블에서도 못 찾은 경우
            self._stat_ir_check = 'unavailable'
            return True

        self._stat_ir_value = value
        confirmed = value >= self.ir_proximity_threshold
        self._stat_ir_check = 'confirmed' if confirmed else 'rejected'
        return confirmed

    def _ir_yaw(self, frame_id):
        """IR 센서 프레임의 base_link 기준 방위각(rad). tf_static에서 1회 조회 후 캐시."""
        if frame_id in self.ir_yaw_cache:
            return self.ir_yaw_cache[frame_id]
        yaw = IR_SENSOR_YAW_FALLBACK.get(frame_id)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.1))
            yaw = yaw_from_quaternion(tf.transform.rotation)
        except Exception:
            pass
        if yaw is not None:
            self.ir_yaw_cache[frame_id] = yaw
        return yaw

    def _nearest_ir_value(self, bearing):
        """base_link 기준 방위각 bearing에 가장 가까운 IR 센서의 원시값. 없으면 None."""
        if not self.ir_readings:
            return None
        best = None
        for reading in self.ir_readings:
            yaw = self._ir_yaw(reading.header.frame_id)
            if yaw is None:
                continue
            diff = angular_diff(bearing, yaw)
            if best is None or diff < best[0]:
                best = (diff, int(reading.value))
        return None if best is None else best[1]

    # ------------------------------------------------------------------ 출력

    def publish_detections(self, detections, header):
        msg = Detection3DArray()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.map_frame
        msg.detections = detections
        self.detections_pub.publish(msg)

    def _publish_embeddings(self, frame, overlay_items, header):
        """발행한 검출들의 외형 임베딩을 32FC1 Image(N x D)로 내보낸다.

        overlay_items는 detections와 1:1 같은 순서라, 여기서 나오는 행 순서가 곧
        Detection3DArray의 검출 순서다. 검출이 하나도 없으면 발행하지 않는다(구독자는
        임베딩이 없으면 위치 기반으로 degrade하므로 빈 메시지를 보낼 필요가 없다).
        """
        if self.reid_pub is None or not overlay_items:
            return
        # ReID 인코더는 원본 프레임과 xywh 박스를 받아 크롭을 내부에서 수행한다.
        # 미리 크롭해서 넘기면 안 된다.
        xywh = np.array(
            [[(b[0] + b[2]) * 0.5, (b[1] + b[3]) * 0.5, b[2] - b[0], b[3] - b[1]]
             for *_, b in overlay_items], dtype=np.float32)
        try:
            embeddings = self.reid_encoder(frame, xywh)
        except Exception as exc:
            self.get_logger().warn(f'ReID 임베딩 계산 실패: {exc}', throttle_duration_sec=5.0)
            return

        rows = []
        for emb in embeddings:
            v = np.asarray(emb, dtype=np.float32).reshape(-1)
            norm = float(np.linalg.norm(v))
            # 코사인 유사도로 비교할 것이므로 여기서 미리 정규화해 보낸다.
            rows.append(v / norm if norm > 0.0 else v)
        if not rows or len({r.shape[0] for r in rows}) != 1:
            return
        matrix = np.ascontiguousarray(np.stack(rows), dtype=np.float32)

        msg = Image()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.map_frame
        msg.height, msg.width = matrix.shape
        msg.encoding = '32FC1'
        msg.is_bigendian = 0
        msg.step = msg.width * 4
        msg.data = matrix.tobytes()
        self.reid_pub.publish(msg)

    def _plot_published_only(self, result):
        """발행 기준(conf_threshold)을 통과한 검출만 그린 오버레이를 만든다.

        Results를 불리언 마스크로 인덱싱해 걸러낸 뒤 plot한다. 마스크 인덱싱이 안 되는
        버전이면 원본을 그대로 그려(기능 저하만 있고 동작은 유지) 예외로 죽지 않게 한다.
        """
        boxes = getattr(result, 'boxes', None)
        if boxes is None or len(boxes) == 0:
            return result.plot()
        keep = boxes.conf >= self.conf_threshold
        if bool(keep.all()):
            return result.plot()
        if not bool(keep.any()):
            # 발행할 게 하나도 없으면 상자 없는 원본 프레임을 쓴다.
            return result.orig_img.copy()
        try:
            return result[keep].plot()
        except Exception:
            return result.plot()

    def _publish_debug_image(self, result, overlay_items, frame_stamp):
        try:
            # 트래커에는 tracker_conf_threshold(0.1)로 저신뢰 검출까지 넣기 때문에, result에는
            # 우리가 발행하지 않는 후보까지 들어 있다. 그대로 plot하면 허공에 'person 0.20'
            # 같은 오검출 상자가 그려져 증거영상을 오독하게 만든다(사용자 지적). 실제로
            # 발행한 것과 같은 기준(conf_threshold)으로 걸러서 그린다.
            overlay = self._plot_published_only(result)
        except Exception:
            return

        # 이 프레임의 검출들 중 실제 AMR 추종 대상(target_person_pose)에 가장 가까운 것을
        # 찾는다 - oakd_id는 이 카메라 하나의 내부 라벨일 뿐이라, "AMR이 실제로 따라가는
        # 사람이 어느 것인가"는 reid_tracking_node가 융합한 결과와 map 좌표를 대조해야만
        # 알 수 있다. follow_target이 오래돼(follow_target_max_age 초과) 신뢰할 수 없으면
        # 아무것도 강조하지 않는다. 신선도 비교는 반드시 이 프레임 자체의 타임스탬프
        # (rgb_msg.header.stamp, 발행 쪽과 동일한 시간 기준)로 해야 한다 - self.get_clock()
        # 의 벽시계는 로스백 재생(use_sim_time 미설정) 시 메시지 타임스탬프와 기준이 완전히
        # 달라 신선도 검사가 항상 실패하는 버그가 있었다(실측 확인: FOLLOWING이 전혀 안 뜸).
        follow_idx = None
        why = None   # FOLLOWING을 못 칠한 이유(진단 로그용)
        if not overlay_items:
            why = None            # 그릴 검출 자체가 없으면 설명할 것도 없다
        elif self.latest_follow_target is None:
            why = '추종 좌표 미수신(reid가 아직 대상을 못 정했거나 토픽 미연결)'
        if self.latest_follow_target is not None and overlay_items:
            fx, fy, f_stamp = self.latest_follow_target
            frame_sec = _stamp_to_sec(frame_stamp)
            if abs(frame_sec - f_stamp) > self.follow_target_max_age:
                why = (f'추종 좌표가 낡음 {abs(frame_sec - f_stamp):.2f}s '
                       f'(> {self.follow_target_max_age:.2f}s)')
            if abs(frame_sec - f_stamp) <= self.follow_target_max_age:
                best_d = None
                for idx, item in enumerate(overlay_items):
                    map_x, map_y = item[5], item[6]
                    d = math.hypot(map_x - fx, map_y - fy)
                    if best_d is None or d < best_d:
                        best_d, follow_idx = d, idx
                # [중요] 거리 상한이 없으면 "가장 가까운 검출"이 무조건 뽑혀, 화면에 사람이
                # 하나만 있으면 추종 대상과 몇 미터 떨어져 있든 항상 FOLLOWING으로 칠해진다.
                # 그러면 오버레이가 "추종이 잘 되고 있다"는 착시를 주고 증거영상이 못 쓰게 된다
                # (실제로 그렇게 잘못 표시된 영상을 만들었다). 매칭이 안 되면 아무것도 칠하지
                # 않고 아래에서 'TARGET NOT VISIBLE'을 표시한다.
                if best_d is None or best_d > self.follow_target_match_distance:
                    why = (f'가장 가까운 검출이 추종 좌표에서 {best_d:.2f}m '
                           f'(> {self.follow_target_match_distance:.2f}m) '
                           f'| 추종좌표 ({fx:.2f},{fy:.2f})')
                    follow_idx = None
        if why is not None and self.log_follow_overlay_misses:
            # 사람이 화면에 있는데 FOLLOWING이 안 뜨는 이유를 한 줄로 남긴다. 원인이
            # "대상 미선정 / 좌표 낡음 / 매칭 거리 초과" 셋으로 갈려 로그 없이는 구분되지 않는다.
            self.get_logger().info(f'[follow] 미표시: {why}', throttle_duration_sec=2.0)

        for i, (u, v, grade, distance, track_id, map_x, map_y, bbox) in enumerate(overlay_items):
            if i == follow_idx:
                # AMR이 실제로 추종 중인 대상 - 사람 하나하나 위치를 텍스트로 대조하는 것보다
                # bbox 전체를 반투명하게 채우는 쪽이 한눈에 들어온다(사용자 피드백 반영).
                x1, y1, x2, y2 = (int(coord) for coord in bbox)
                fill = overlay.copy()
                cv2.rectangle(fill, (x1, y1), (x2, y2), (255, 0, 255), -1)
                cv2.addWeighted(fill, 0.35, overlay, 0.65, 0, overlay)
                cv2.rectangle(overlay, (x1, y1), (x2, y2), (255, 0, 255), 3)
                # 라벨은 bbox '아래쪽 안'에 그린다. 위쪽은 (a) 초근접이라 bbox 상단이 화면
                # 밖으로 잘리는 게 기본 상황이고(문서 [유효 거리]), (b) result.plot()의
                # "person 0.81" 라벨과 좌상단 dt= 표시가 이미 자리를 차지해 겹쳐 읽기 힘들다.
                # 반면 발끝 쪽은 이 카메라 기하상 가까울수록 잘 보여 라벨 자리로 안정적이다.
                label_y = min(max(y2 - 12, 24), overlay.shape[0] - 6)
                cv2.putText(
                    overlay, 'FOLLOWING', (max(x1 + 6, 4), label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2, cv2.LINE_AA)
            cv2.circle(overlay, (int(u), int(v)), 6, (0, 255, 255), -1)
            cv2.putText(
                overlay, f'{grade} {distance:.2f}m', (int(u) + 8, int(v)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
            # map 좌표(3D 역투영 + tf 결과)도 발끝 점 위에 덧그린다. 화면 좌상단 같은 고정
            # 위치에 두는 것도 고려했지만, 사람이 여럿이면 고정 위치 하나로는 "이 좌표가
            # 누구 것인지" 알 수 없다 - track_id와 같은 이유로 검출된 좌표를 따라다니게
            # 그려야 한다.
            cv2.putText(
                overlay, f'map({map_x:.2f},{map_y:.2f})', (int(u) + 8, int(v) + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            # YOLO track_id(.track(persist=True)가 부여)를 발끝 점 위에 덧그린다. result.plot()도
            # bbox 위쪽에 id를 이미 그리지만, 이 로봇은 초근접 구간에서 사람 머리~bbox 상단이
            # 화면 밖으로 잘리는 게 기본 상황이라(문서 참고) 그 라벨이 안 보이는 경우가 흔하다.
            # 발끝 점은 오히려 근접할수록 잘 보이므로(같은 이유), 여기다 고정 위치가 아니라
            # 검출된 좌표를 따라다니게 그려야 "이 track_id가 실제로 이 사람을 계속 추적 중"이라는
            # 게 시각적으로 분명해진다 - 화면 구석 고정 표시는 얼어붙은 값인지 실시간 추적인지
            # 구분이 안 된다.
            cv2.putText(
                overlay, f'id:{track_id}', (int(u) + 8, int(v) - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 140, 255), 2, cv2.LINE_AA)
        if follow_idx is None:
            # 추종 대상이 화면 안 어느 검출과도 매칭되지 않았다. 아무 표시가 없는 것과
            # "표시할 대상이 없다"를 구분해줘야 영상만 보고 오판하지 않는다.
            cv2.putText(
                overlay, 'TARGET NOT VISIBLE', (10, overlay.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(
            overlay, f'dt={self.last_rgb_depth_dt * 1000:.0f}ms', (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        ok, encoded = cv2.imencode('.jpg', overlay)
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        self.debug_pub.publish(msg)

    def publish_diagnostics(self):
        """매니저 노드/진단 애그리게이터가 이 로봇의 비전 상태를 파악하는 표준 채널."""
        now = time.monotonic()
        elapsed = max(now - self._diag_last_time, 1e-6)
        fps = self._stat_frames / elapsed
        age = None if self.last_frame_time is None else now - self.last_frame_time

        status = DiagnosticStatus()
        status.name = f'{self.get_parameter("namespace").value}: oakd_detector_node'
        status.hardware_id = str(self.get_parameter('namespace').value)

        if self.pose_model is None:
            status.level = DiagnosticStatus.ERROR
            status.message = 'YOLO-pose 모델 미로드'
        elif age is None or age > self.input_timeout:
            status.level = DiagnosticStatus.STALE
            status.message = '이미지 입력 끊김'
        elif self._stat_tf_failures > 0 or self._stat_depth_invalid > 0:
            status.level = DiagnosticStatus.WARN
            status.message = 'depth 무효 또는 tf 변환 실패 발생'
        else:
            status.level = DiagnosticStatus.OK
            status.message = '정상'

        status.values = [
            KeyValue(key='fps', value=f'{fps:.2f}'),
            KeyValue(key='detections', value=str(self._stat_detections)),
            KeyValue(key='nearest_distance_m',
                     value='n/a' if self._stat_nearest is None else f'{self._stat_nearest:.2f}'),
            KeyValue(key='rgb_depth_dt_ms', value=f'{self.last_rgb_depth_dt * 1000:.1f}'),
            KeyValue(key='depth_invalid', value=str(self._stat_depth_invalid)),
            # 발끝 depth가 배경을 찍어 사람 깊이로 대체된 횟수. 이 값이 크면 스테레오가
            # 바닥/실루엣 경계에서 약하다는 뜻이라 지면평면(IPM) 도입을 검토할 근거가 된다.
            KeyValue(key='depth_substituted', value=str(self._stat_depth_substituted)),
            KeyValue(key='tf_failures', value=str(self._stat_tf_failures)),
            KeyValue(key='truncated_count', value=str(self._stat_truncated)),
            KeyValue(key='frames_without_track_id', value=str(self._stat_no_track_id)),
            KeyValue(key='dropped_too_far', value=str(self._stat_too_far)),
            KeyValue(key='merged_duplicates', value=str(self._stat_duplicates)),
            KeyValue(key='dropped_no_lower_body', value=str(self._stat_no_lower_body)),
            KeyValue(
                key='single_person_suppressed',
                value=str(self._stat_single_person_suppressed)),
            KeyValue(key='proximity_alert', value=str(self.proximity_active).lower()),
            KeyValue(key='ir_check', value=self._stat_ir_check),
            KeyValue(key='ir_value', value=str(self._stat_ir_value)),
            KeyValue(key='last_frame_age_s', value='n/a' if age is None else f'{age:.1f}'),
        ] + [
            KeyValue(key=f'grade_{g}', value=str(c)) for g, c in sorted(self._stat_grades.items())
        ]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diagnostics_pub.publish(msg)

        if status.level == DiagnosticStatus.STALE:
            self.get_logger().warn(
                f'이미지 입력 {self.input_timeout}s 이상 없음', throttle_duration_sec=10.0)

        self._reset_diagnostic_counters(now)

    def _reset_diagnostic_counters(self, now):
        self._diag_last_time = now
        self._stat_frames = 0
        # 상류 트래커가 이 프레임에 id를 하나도 못 낸 횟수. reid_tracking_node의 상류 id 구제가
        # 무력화되는 조건이라(det.id가 빈 문자열로 나감) 트랙 churn의 직접 원인 후보다.
        self._stat_no_track_id = 0
        self._stat_too_far = 0
        self._stat_duplicates = 0
        self._stat_no_lower_body = 0
        self._stat_single_person_suppressed = 0
        self._stat_detections = 0
        self._stat_depth_invalid = 0
        self._stat_depth_substituted = 0
        self._stat_tf_failures = 0
        self._stat_truncated = 0
        self._stat_grades = {g: 0 for g in GRADE_SIGMA}
        self._stat_nearest = None


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = OakdDetectorNode()
    # tf2_ros.TransformListener는 자신의 구독을 ReentrantCallbackGroup에 등록해두지만,
    # 기본 rclpy.spin()이 쓰는 SingleThreadedExecutor는 스레드가 하나뿐이라 그룹을 나눠봐야
    # 소용이 없다 - sync_callback 안에서 tf_buffer.transform()이 대기하는 동안 같은 스레드가
    # /tf 구독 콜백도 처리해야 해서, 실제로는 새 tf가 안 들어오고 매번 tf_timeout을 거의
    # 다 채우고서야 풀린다. MultiThreadedExecutor로 스레드를 분리해야 tf 콜백이 그 사이에
    # 끼어들어 처리될 수 있다.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
