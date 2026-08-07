#!/usr/bin/env python3
"""
leg_detector_bridge_node.py

[역할]
터틀봇4는 카메라 장착 높이가 낮아, 사람에게 가까이 접근하면(초근접 구간) OAK-D RGB/Depth
프레임에 사람의 다리만 잡히는 경우가 많다. 이 구간에서는 YOLO-pose 기반 발끝 검출보다 2D 라이다
다리(leg) 검출이 더 신뢰도가 높다 — RPLIDAR 스캔 평면 자체가 다리 높이와 맞아떨어지고, 라이다는
range+bearing(거리+각도)을 직접 측정하는 센서라 호모그래피나 depth 역투영 같은 변환이 필요 없기
때문이다.

[외부 패키지 의존 제거 - Foxy/Humble 호환 문제]
원래는 mowito/ros2_leg_detector(leg_detector_msgs/PersonArray)의 출력을 oakd_detector_node와
동일한 스키마(vision_msgs/Detection3DArray)로 변환하는 얇은 컨버터로 설계했다. 하지만
ros2_leg_detector는 ROS2 Foxy 기준 코드에 OpenCV 3.4.12 명시 의존까지 있어 이 워크스페이스
(Humble)에서 빌드가 되지 않았다 (leg_detector_msgs/leg_detector 클론+빌드 실패).

그래서 외부 패키지를 워크스페이스에 추가로 설치하지 않고, LiDAR 드라이버가 표준으로 내보내는
sensor_msgs/LaserScan을 이 노드가 직접 구독해 다리쌍을 검출하는 경량 휴리스틱으로 대체했다
(알고리즘은 leg_detection_utils.py 참고 - jump-distance 클러스터링 + 폭 필터 + 그리디 페어링).
트래킹(라이다 트랙 ID 부여)도 원래 ros2_leg_detector가 내부에서 하던 역할이라, 이 노드가
reid_tracking_node와 동일한 방식(tracking_utils.Track/match_track)으로 직접 수행한다 - 그래야
reid_tracking_node의 5~6단계 락온/스왑검증 로직(같은 raw leg id가 연속 프레임 최고 후보여야
락온 확정되는 구조)이 코드 변경 없이 그대로 맞물린다.

이 노드가 검출/추적한 다리쌍의 신원(누구인지)은 이 노드가 판단하지 않는다 - 위치와 이 노드가
매긴 raw 트랙 ID만 실어 보내고, 신원 매칭(웹캠이 추종 중이던 타겟과의 대조)은 reid_tracking_node가
담당한다. 이 경계와 출력 스키마/토픽은 원래 설계에서 바뀌지 않았다.

[검출기 성격]
학습 기반 분류기가 아니라 거리/폭 임계값만 쓰는 단순 검출기다. 실제 라이다의 각해상도, 노이즈,
사람 다리 두께(옷 두께 포함)에 맞춰 아래 파라미터(cluster_distance_threshold, leg_diameter_min/max,
leg_pair_max_distance 등)를 현장에서 튜닝해야 한다.

[입력 토픽]
  - <namespace>/scan (sensor_msgs/msg/LaserScan) - LiDAR 드라이버가 laser 프레임으로 발행

[출력 토픽]
  - <namespace>/vision/leg_detections_3d (vision_msgs/Detection3DArray, frame_id=map)
  - <namespace>/vision/leg_detections/markers (visualization_msgs/MarkerArray, frame_id=map, 선택)
    RViz로 눈으로 확인하기 위한 디버그 시각화. 라이다 스캔에는 카메라 이미지 같은 게 없어서
    oakd_detector_node처럼 프레임에 오버레이를 그릴 수 없다 - 원본 ros2_leg_detector도 같은
    이유로 /visualization_marker(Marker)를 발행하며, 이 노드는 그 관례를 따른다.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesis, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (PointStamped tf2 변환 등록)

from amr_person_tracking.leg_detection_utils import (
    StaticBackgroundFilter, scan_to_points, cluster_points, filter_leg_clusters, pair_legs)
from amr_person_tracking.tracking_utils import Track, assign_tracks, stamp_to_sec

MARKER_NS_SPHERES = 'leg_detections'
MARKER_NS_LABELS = 'leg_detections_labels'


class LegDetectorBridgeNode(Node):

    def __init__(self):
        super().__init__('leg_detector_bridge_node')

        self.declare_parameter('scan_topic', '/robot5/scan')
        self.declare_parameter('leg_detections_topic', '/robot5/vision/leg_detections_3d')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tf_timeout', 0.2)
        # 실물 로봇에선 odom/tf가 스캔보다 항상 앞서 있어 필요 없지만, 로스백 재생 시
        # tf가 스캔 타임스탬프보다 뒤처지는 구간(원본 녹화의 실제 공백 포함)이 있으면
        # "extrapolation into the future"로 매 프레임 실패한다. true면 정확한 시각 대신
        # 최신 tf로 폴백한다 (oakd_detector_node의 동일 파라미터와 같은 목적).
        self.declare_parameter('tf_allow_latest_fallback', False)
        # 이 노드는 학습 기반 신뢰도를 못 내므로 고정값을 싣는다 - 실제 검출 신뢰도가 아니라
        # "라이다 다리검출 출처"라는 표시에 가깝다.
        self.declare_parameter('leg_detection_score', 0.7)

        # 검출 파라미터 (leg_detection_utils.detect_persons) - 현장 라이다 특성에 맞춰 튜닝 필요
        self.declare_parameter('scan_range_limit', 5.0)
        self.declare_parameter('cluster_distance_threshold', 0.08)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('leg_diameter_min', 0.05)
        self.declare_parameter('leg_diameter_max', 0.25)
        self.declare_parameter('leg_pair_max_distance', 0.5)
        self.declare_parameter('allow_single_leg_detection', True)
        # 폭(첫-끝 포인트 직선거리)만으로는 볼록한 다리 원호와 오목하게 꺾인 벽 모서리를
        # 구분 못 해 실제 로스백에서 벽 모서리가 다리로 오검출되는 걸 확인했다. 원형적합으로
        # 곡률까지 검사해 걸러낸다 (leg_detection_utils.fit_circle_kasa).
        self.declare_parameter('leg_circularity_filter_enabled', True)
        self.declare_parameter('leg_circle_fit_radius_max', 0.20)
        self.declare_parameter('leg_circle_fit_rms_max', 0.01)

        # 원형적합만으론 책상/의자 다리(진짜 원통형이라 형상으로 구별 불가)를 못 거른다 -
        # "계속 같은 자리에 있다"는 시간적 근거로 추가 제외한다 (StaticBackgroundFilter).
        # 짝짓기(pair_legs) 이전의 개별 다리 위치 기준으로 판단한다 - 짝지어진 중심점은
        # 스캔마다 어떤 후보끼리 묶이느냐에 따라 튀어서(실측으로 확인) "정지"와 "사람의 미세
        # 이동"을 구분할 수 없었다(leg_detection_utils.StaticBackgroundFilter 문서 참고).
        self.declare_parameter('background_filter_enabled', True)
        self.declare_parameter('background_confirm_duration_sec', 3.0)
        self.declare_parameter('background_max_gap_sec', 1.0)
        self.declare_parameter('background_cell_size', 0.05)
        self.declare_parameter('background_exclusion_radius', 0.10)

        # 검출된 다리쌍(사람 중심점)에 raw 트랙 ID를 매기는 단순 최근접 트래커.
        # reid_tracking_node의 게이팅과 동일한 방식(tracking_utils) - 스캔이 빠르게(보통 5~15Hz)
        # 들어오므로 track_timeout은 reid_tracking_node보다 짧게 잡는다.
        self.declare_parameter('track_gating_max_speed', 2.0)
        self.declare_parameter('track_gating_min_gate', 0.3)
        self.declare_parameter('track_velocity_alpha', 0.5)
        self.declare_parameter('track_timeout', 1.0)

        # RViz 디버그 시각화 (검증 단계라 기본 on - oakd_detector_node의 publish_debug_image와 동일한 취지)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_topic', '/robot5/vision/leg_detections/markers')
        self.declare_parameter('marker_scale', 0.15)
        # 노드가 멈추거나 해당 프레임에 발행이 없어도 RViz에 마커가 유령처럼 남지 않도록 자동 만료
        self.declare_parameter('marker_lifetime', 0.5)

        scan_topic = self.get_parameter('scan_topic').value
        leg_detections_topic = self.get_parameter('leg_detections_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout').value)
        self.tf_allow_latest_fallback = self.get_parameter('tf_allow_latest_fallback').value
        self.leg_detection_score = self.get_parameter('leg_detection_score').value

        self.scan_range_limit = self.get_parameter('scan_range_limit').value
        self.cluster_distance_threshold = self.get_parameter('cluster_distance_threshold').value
        self.min_cluster_points = self.get_parameter('min_cluster_points').value
        self.leg_diameter_min = self.get_parameter('leg_diameter_min').value
        self.leg_diameter_max = self.get_parameter('leg_diameter_max').value
        self.leg_pair_max_distance = self.get_parameter('leg_pair_max_distance').value
        self.allow_single_leg_detection = self.get_parameter('allow_single_leg_detection').value
        self.leg_circularity_filter_enabled = self.get_parameter('leg_circularity_filter_enabled').value
        self.leg_circle_fit_radius_max = self.get_parameter('leg_circle_fit_radius_max').value
        self.leg_circle_fit_rms_max = self.get_parameter('leg_circle_fit_rms_max').value

        self.background_filter_enabled = self.get_parameter('background_filter_enabled').value
        self.background_filter = StaticBackgroundFilter(
            cell_size=self.get_parameter('background_cell_size').value,
            exclusion_radius=self.get_parameter('background_exclusion_radius').value,
            confirm_duration_sec=self.get_parameter('background_confirm_duration_sec').value,
            max_gap_sec=self.get_parameter('background_max_gap_sec').value,
        ) if self.background_filter_enabled else None

        self.track_gating_max_speed = self.get_parameter('track_gating_max_speed').value
        self.track_gating_min_gate = self.get_parameter('track_gating_min_gate').value
        self.track_velocity_alpha = self.get_parameter('track_velocity_alpha').value
        self.track_timeout = self.get_parameter('track_timeout').value

        self.publish_markers = self.get_parameter('publish_markers').value
        self.marker_scale = self.get_parameter('marker_scale').value
        self.marker_lifetime = Duration(seconds=self.get_parameter('marker_lifetime').value)

        self.leg_detections_pub = self.create_publisher(Detection3DArray, leg_detections_topic, 10)
        self.marker_pub = (
            self.create_publisher(MarkerArray, self.get_parameter('marker_topic').value, 10)
            if self.publish_markers else None
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # raw 트랙 ID(int) -> Track (map 좌표계). reid_tracking_node와 동일한 목적의 로컬 트래커.
        self.tracks = {}
        self._next_track_id = 1

        # LiDAR 드라이버는 보통 best_effort로 발행한다 (oakd_detector_node의 detections_pub과 동일한 사정).
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'leg_detector_bridge_node 시작 (자체 LaserScan 다리검출) | {scan_topic} -> {leg_detections_topic}'
        )

    def scan_callback(self, msg: LaserScan):
        stamp = stamp_to_sec(msg.header.stamp)

        points = scan_to_points(msg, self.scan_range_limit)
        clusters = cluster_points(points, self.cluster_distance_threshold)
        legs_laser = filter_leg_clusters(
            clusters, self.min_cluster_points, self.leg_diameter_min, self.leg_diameter_max,
            circularity_filter_enabled=self.leg_circularity_filter_enabled,
            leg_circle_fit_radius_max=self.leg_circle_fit_radius_max,
            leg_circle_fit_rms_max=self.leg_circle_fit_rms_max)

        # 짝짓기(pair_legs) 전에 개별 다리 후보를 map 프레임으로 변환한다 - 배경 판정은
        # 로봇이 움직여도 세계 기준으로 성립해야 하고, StaticBackgroundFilter는 짝짓기 이전
        # 개별 다리 위치를 입력으로 기대한다(클래스 docstring 참고 - 짝지어진 중심점은
        # 스캔마다 어떤 후보끼리 묶이느냐에 따라 튀어서 "정지" 판정에 쓸 수 없었다).
        legs_map = []
        for lx, ly in legs_laser:
            pt = PointStamped()
            pt.header.frame_id = msg.header.frame_id
            pt.header.stamp = msg.header.stamp
            pt.point.x = lx
            pt.point.y = ly
            pt.point.z = 0.0

            pt_map = self._transform_to_map(pt)
            if pt_map is not None:
                legs_map.append((pt_map.point.x, pt_map.point.y))

        if self.background_filter_enabled:
            for x, y in legs_map:
                self.background_filter.observe(x, y, stamp)
            legs_map = [p for p in legs_map if not self.background_filter.is_background(*p)]

        persons = pair_legs(legs_map, self.leg_pair_max_distance, self.allow_single_leg_detection)

        # 트랙 배정을 후보별로 따로 하면(예전 방식) 같은 트랙을 두 후보가 중복으로 차지할 수
        # 있어서, 이번 콜백의 후보 전체를 놓고 한 번에 배타적으로 한다.
        track_ids = assign_tracks(
            self.tracks, persons, stamp, self.track_gating_max_speed, self.track_gating_min_gate)

        detections = []
        markers = []
        for (x, y), track_id in zip(persons, track_ids):
            track_id = self._update_track(x, y, stamp, track_id)

            pt_map = PointStamped()
            pt_map.header.frame_id = self.map_frame
            pt_map.header.stamp = msg.header.stamp
            pt_map.point.x = x
            pt_map.point.y = y
            pt_map.point.z = 0.0

            detections.append(self._make_detection3d(pt_map, track_id, msg.header))
            if self.publish_markers:
                markers.extend(self._make_markers(pt_map, track_id, msg.header))

        self._prune_stale_tracks(stamp)
        self.publish_leg_detections(detections, msg.header)
        if self.publish_markers:
            self.publish_markers_array(markers, msg.header)

    def _transform_to_map(self, pt: PointStamped):
        try:
            return self.tf_buffer.transform(pt, self.map_frame, timeout=self.tf_timeout)
        except Exception as exc:
            if self.tf_allow_latest_fallback:
                try:
                    pt_latest = PointStamped()
                    pt_latest.header.frame_id = pt.header.frame_id
                    pt_latest.point = pt.point  # stamp를 비우면 최신 변환을 쓴다
                    return self.tf_buffer.transform(pt_latest, self.map_frame, timeout=self.tf_timeout)
                except Exception:
                    pass
            self.get_logger().warn(
                f'{self.map_frame} 변환 실패: {exc}', throttle_duration_sec=2.0)
            return None

    def _update_track(self, x, y, stamp, track_id):
        """assign_tracks()가 이번 콜백에 대해 배타적으로 배정한 track_id(없으면 None)로
        트랙을 새로 만들거나 갱신한다."""
        if track_id is None:
            track_id = self._next_track_id
            self._next_track_id += 1
            self.tracks[track_id] = Track(track_id, x, y, stamp, source='lidar_leg')
        else:
            self.tracks[track_id].update(
                x, y, stamp, source='lidar_leg',
                velocity_alpha=self.track_velocity_alpha, max_speed=self.track_gating_max_speed)
        return track_id

    def _prune_stale_tracks(self, now):
        stale = [tid for tid, tr in self.tracks.items() if now - tr.last_stamp > self.track_timeout]
        for tid in stale:
            del self.tracks[tid]

    def _make_detection3d(self, pt_map, track_id, header):
        det = Detection3D()
        # header를 통째로 대입(참조 공유)하면 안 된다 - 아래 frame_id 변경이 호출자가 넘긴
        # 원본 header 객체(예: msg.header)까지 오염시켜서, 같은 콜백의 다음 후보가 이미
        # 바뀐 frame_id를 읽게 되는 버그가 있었다. stamp만 값으로 복사한다.
        det.header.stamp = header.stamp
        det.header.frame_id = self.map_frame

        primary = ObjectHypothesisWithPose()
        primary.hypothesis = ObjectHypothesis(class_id='person', score=self.leg_detection_score)
        primary.pose.pose.position = pt_map.point
        primary.pose.pose.orientation.w = 1.0
        det.results.append(primary)

        source = ObjectHypothesisWithPose()
        source.hypothesis = ObjectHypothesis(class_id='source_lidar_leg', score=1.0)
        source.pose.pose.position = pt_map.point
        source.pose.pose.orientation.w = 1.0
        det.results.append(source)

        det.bbox.center.position = pt_map.point
        det.bbox.center.orientation.w = 1.0

        # 이 노드가 매긴 raw 트랙 ID (노드 재시작 시 리셋됨). 재식별 노드가 map 기준 지속 트랙
        # ID로 덮어쓴다 - oakd_detector_node의 'oakd_<id>'와 동일한 관례.
        det.id = f'leg_{int(track_id)}'
        return det

    def publish_leg_detections(self, detections, header):
        out_msg = Detection3DArray()
        out_msg.header.stamp = header.stamp
        out_msg.header.frame_id = self.map_frame
        out_msg.detections = detections
        self.leg_detections_pub.publish(out_msg)

    def _make_markers(self, pt_map, track_id, header):
        """검출 다리쌍 하나당 구체(위치) + 텍스트(라이다 트랙 ID) 마커 한 쌍을 만든다.

        marker.id를 트랙 ID로 고정해 같은 사람이 프레임마다 새 마커가 아니라
        RViz에서 하나의 마커가 매끄럽게 이동하는 것처럼 보이게 한다.
        """
        sphere = Marker()
        sphere.header.stamp = header.stamp
        sphere.header.frame_id = self.map_frame
        sphere.ns = MARKER_NS_SPHERES
        sphere.id = int(track_id)
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = pt_map.point
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = self.marker_scale
        sphere.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.85)
        sphere.lifetime = self.marker_lifetime.to_msg()

        label = Marker()
        label.header.stamp = header.stamp
        label.header.frame_id = self.map_frame
        label.ns = MARKER_NS_LABELS
        label.id = int(track_id)
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = pt_map.point.x
        label.pose.position.y = pt_map.point.y
        label.pose.position.z = pt_map.point.z + 0.25
        label.pose.orientation.w = 1.0
        label.scale.z = 0.15
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        label.text = f'leg_{int(track_id)}'
        label.lifetime = self.marker_lifetime.to_msg()

        return [sphere, label]

    def publish_markers_array(self, markers, header):
        """매 프레임 이전 마커를 전부 지우고 다시 그린다 - 트랙 ID가 사라졌을 때 RViz에
        유령 마커가 남는 것을 방지하는 표준적인 방식이다 (lifetime 만료에만 기대지 않는다)."""
        out_msg = MarkerArray()
        for ns in (MARKER_NS_SPHERES, MARKER_NS_LABELS):
            delete_all = Marker()
            delete_all.header.stamp = header.stamp
            delete_all.header.frame_id = self.map_frame
            delete_all.ns = ns
            delete_all.action = Marker.DELETEALL
            out_msg.markers.append(delete_all)
        out_msg.markers.extend(markers)
        self.marker_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LegDetectorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
