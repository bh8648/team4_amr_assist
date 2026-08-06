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
이 카메라는 base_link 기준 z=0.244m에 틸트 없이 수평으로 달려 있고 FOV는 63.8°다.
따라서 거리별로 보이는 최대 높이가 1.0m->0.87m, 1.5m->1.18m, 2.0m->1.49m, 2.5m->1.80m이라
2.5m 이내에서는 사람 머리가 항상 잘린다. 반대로 지면은 0.39m 앞부터 화각에 들어와
발끝은 오히려 근접할수록 잘 보인다. 또 스테레오 최소 측정거리 때문에 0.7m 아래는 depth가 없다.
=> OAK-D 유효구간은 대략 1.0~2.5m이고, 그보다 가까우면 라이다 다리검출이 주도해야 한다.
   이 노드는 그 열화를 covariance / foot_<grade> / view_truncated_top 세 신호로 스스로 신고한다.

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
from geometry_msgs.msg import PointStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, CompressedImage
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
    estimate_foot_pixel,
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
        self.kp_conf_threshold = self.get_parameter('kp_conf_threshold').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.depth_patch_size = self.get_parameter('depth_patch_size').value
        self.process_every_n = max(1, int(self.get_parameter('process_every_n').value))
        self.min_z_safety_threshold = self.get_parameter('min_z_safety_threshold').value
        self.proximity_release_margin = self.get_parameter('proximity_release_margin').value
        self.enable_ir_safety_check = self.get_parameter('enable_ir_safety_check').value
        self.ir_proximity_threshold = self.get_parameter('ir_proximity_threshold').value
        self.ir_max_age = self.get_parameter('ir_max_age').value
        self.input_timeout = self.get_parameter('input_timeout').value
        self.truncation_margin_px = self.get_parameter('truncation_margin_px').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value

        # ---- 캐시 상태 (콜백이 갱신, 동기 콜백/타이머가 소비) ----
        self.camera_k = None          # (fx, fy, cx, cy)
        self.camera_frame_id = None
        self.ir_readings = None
        self.ir_stamp = None
        self.ir_yaw_cache = {}
        self.robot_speed = 0.0
        self.proximity_active = False
        self.frame_index = 0
        self.last_frame_time = None
        self.last_rgb_depth_dt = 0.0

        # ---- 1주기 진단 카운터 ----
        self._stat_frames = 0
        self._stat_detections = 0
        self._stat_depth_invalid = 0
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
        self.get_logger().info(f'YOLO-pose 모델 로드: {model_path}')
        try:
            self.pose_model = YOLO(model_path)
            if device:
                self.pose_model.to(device)
        except Exception as exc:
            self.pose_model = None
            self.get_logger().error(f'YOLO-pose 모델 로드 실패: {exc}')

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
        self.detections_pub = self.create_publisher(Detection3DArray, detections_topic, 10)

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
                CompressedImage, self.get_parameter('debug_image_topic').value, 1)
            if self.publish_debug_image else None
        )

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
        # 근접 구간에서는 상반신이 잘려 COCO 학습 검출기의 person 점수가 낮게 나오므로 기본값을 낮춘다.
        # 주의: 이 0.3은 위 [유효 거리] 기하로부터 추론한 값이고 아직 실측으로 확정하지 못했다.
        # 검증에 쓴 bag(rosbag2_2026_08_06-12_27_59)에는 사람이 찍히지 않아 분포를 볼 수 없었다.
        # 사람이 나오는 근접 주행 bag이 생기면 person conf 분포를 보고 다시 정해야 한다.
        self.declare_parameter('conf_threshold', 0.3)
        self.declare_parameter('kp_conf_threshold', 0.5)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('depth_patch_size', 5)
        self.declare_parameter('truncation_margin_px', 4)

        self.declare_parameter('tf_timeout', 0.2)
        self.declare_parameter('tf_allow_latest_fallback', False)
        self.declare_parameter('odom_topic', '/robot5/odom')

        # 근접 안전모드 전환 임계값 (m, base_link 기준 수평거리).
        # 스테레오 최소 측정거리가 약 0.7m라 그보다 낮게 잡으면 depth로는 도달할 수 없다.
        self.declare_parameter('min_z_safety_threshold', 0.8)
        self.declare_parameter('proximity_release_margin', 0.1)
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
            frame, persist=True, classes=[0], conf=self.conf_threshold, verbose=False)
        detections, nearest, overlay_items = self._build_detections(results[0], depth_img, rgb_msg)

        self._update_proximity(nearest)
        self.publish_detections(detections, rgb_msg.header)

        self._stat_frames += 1
        self._stat_detections += len(detections)
        if nearest is not None:
            dist = nearest[0]
            self._stat_nearest = dist if self._stat_nearest is None else min(self._stat_nearest, dist)

        if self.debug_pub is not None:
            self._publish_debug_image(results[0], overlay_items)

    def _build_detections(self, result, depth_img, rgb_msg):
        """YOLO 결과 하나를 Detection3D 목록으로 바꾼다.

        반환: (detections, nearest, overlay_items).
        nearest는 최근접 대상의 (거리 m, base_link 기준 방위각 rad) 또는 None.
        방위각은 근접 안전모드에서 어느 IR 센서를 볼지 고르는 데 쓴다.
        """
        detections = []
        overlay_items = []
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

        for i in range(len(boxes)):
            bbox = boxes.xyxy[i].cpu().numpy()
            box_conf = float(boxes.conf[i])
            track_id = int(boxes.id[i]) if boxes.id is not None else -1

            kp_xy = kp_xy_all[i] if kp_xy_all is not None and i < len(kp_xy_all) else None
            kp_conf = kp_conf_all[i] if kp_conf_all is not None and i < len(kp_conf_all) else None

            u, v, grade = estimate_foot_pixel(kp_xy, kp_conf, bbox, self.kp_conf_threshold)

            z = sample_depth_patch(depth_img, u, v, self.depth_patch_size, self.depth_scale)
            if z is None or z <= 0.0:
                self._stat_depth_invalid += 1
                continue

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
            bearing = math.atan2(pt_base.point.y, pt_base.point.x)
            if nearest is None or distance < nearest[0]:
                nearest = (distance, bearing)

            trunc = truncation_ratio(bbox, self.truncation_margin_px)
            if trunc > 0.0:
                self._stat_truncated += 1
            self._stat_grades[grade] = self._stat_grades.get(grade, 0) + 1

            detections.append(self._make_detection3d(pt_map, box_conf, grade, trunc, track_id, rgb_msg.header))
            overlay_items.append((u, v, grade, distance))

        return detections, nearest, overlay_items

    def _make_detection3d(self, pt_map, box_conf, grade, trunc, track_id, header):
        det = Detection3D()
        det.header = header
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
            return

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
        msg.header = header
        msg.header.frame_id = self.map_frame
        msg.detections = detections
        self.detections_pub.publish(msg)

    def _publish_debug_image(self, result, overlay_items):
        try:
            overlay = result.plot()
        except Exception:
            return
        for u, v, grade, distance in overlay_items:
            cv2.circle(overlay, (int(u), int(v)), 6, (0, 255, 255), -1)
            cv2.putText(
                overlay, f'{grade} {distance:.2f}m', (int(u) + 8, int(v)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
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
            KeyValue(key='tf_failures', value=str(self._stat_tf_failures)),
            KeyValue(key='truncated_count', value=str(self._stat_truncated)),
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
        self._stat_detections = 0
        self._stat_depth_invalid = 0
        self._stat_tf_failures = 0
        self._stat_truncated = 0
        self._stat_grades = {g: 0 for g in GRADE_SIGMA}
        self._stat_nearest = None


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = OakdDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
