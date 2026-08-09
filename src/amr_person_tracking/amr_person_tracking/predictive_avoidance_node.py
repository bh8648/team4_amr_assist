#!/usr/bin/env python3
"""
predictive_avoidance_node.py

[역할]
전체 비전 파이프라인 문서의 "6번 예측적 회피" 노드. 빠르게 접근하는 대상(사람/장애물)에
로봇이 미리 반응할 수 있도록, 트래킹된 대상의 위치 시계열로부터 속도를 추정하고 예측 위치를
로봇의 local_costmap에 반영한다.

reid_tracking_node의 자체 트랙 상태는 지수평활로 단순화했지만(그 노드 docstring 참고), 이
노드는 문서가 실제 칼만필터 담당으로 지정한 곳이라 predictive_utils.ConstantVelocityKalman2D로
표준적인 등속도 모델 칼만필터를 그대로 쓴다.

[처리 흐름]
재식별/트래킹 노드의 트랙 시계열(id 포함 Detection3DArray) 구독
  -> 트랙별 등속도 모델 칼만필터로 속도 추정
     (주의: 프레임 간 Δt는 수신 시각이 아니라 메시지 header.stamp 기준으로 계산.
      네트워크 지연으로 프레임 간격이 불규칙해질 수 있음을 감안한 설계)
  -> 예측된 미래 위치 주변에 속도에 비례한 범위의 가상 포인트 생성
  -> (방식 A) 가상 포인트를 PointCloud2로 발행해 voxel_layer가 실제 장애물처럼 마킹하도록 함
  -> (방식 B) 로봇 쪽으로의 접근 속도 성분(tf로 조회한 로봇 위치 기준)에 비례해 각 로봇
     local_costmap의 inflation 파라미터를 SetParameters 서비스로 동적 조정
  둘 중 하나 또는 병행 (파라미터로 선택)

[입력 토픽]
  - <namespace>/vision/tracked_detections_3d (vision_msgs/Detection3DArray, id 포함)

[출력]
  - <namespace>/vision/predicted_obstacle_points (sensor_msgs/PointCloud2)  [방식 A]
  - rcl_interfaces/SetParameters 서비스 호출 (각 로봇 local_costmap 노드 대상) [방식 B]
  - <namespace>/vision/motion_vector_markers (visualization_msgs/MarkerArray) [rviz2 시각화]
    publish_markers 파라미터로 켠다. 회피 동작에는 관여하지 않는다.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from amr_person_tracking.predictive_utils import ConstantVelocityKalman2D, approach_speed, ring_points
from amr_person_tracking.tracking_utils import extract_person_position, stamp_to_sec


def _point(x, y, z):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


class PredictiveAvoidanceNode(Node):

    def __init__(self):
        super().__init__('predictive_avoidance_node')

        self.declare_parameter('tracked_detections_topic', '/robot5/vision/tracked_detections_3d')
        self.declare_parameter('predicted_points_topic', '/robot5/vision/predicted_obstacle_points')
        self.declare_parameter('local_costmap_param_service', '/robot5/local_costmap/local_costmap/set_parameters')
        # 가상 포인트 발행 / costmap 파라미터 조정 중 사용할 방식 선택: 'pointcloud' | 'costmap_params' | 'both'
        #
        # 주의: 'pointcloud' 방식이 실제로 동작하려면 Nav2 voxel_layer의 observation_sources에
        # 아래 predicted_points_topic이 등록돼 있어야 한다. 그 costmap 설정은 포인트클라우드 기반
        # 장애물 탐색을 담당하는 팀원 소유이므로, 이 모드를 쓰기 전에 등록 여부를 협의해야 한다.
        # (이 노드가 만드는 건 장애물 탐지 결과가 아니라 트래킹 결과로부터 예측한 가상 포인트다.)
        self.declare_parameter('avoidance_mode', 'pointcloud')
        # rviz2에서 이동 벡터를 눈으로 확인하기 위한 마커. 기본은 꺼둔다 - 회피 동작과
        # 무관한 부가 발행이라 실주행에서 대역폭만 쓴다.
        self.declare_parameter('publish_markers', False)
        self.declare_parameter('markers_topic', '/robot5/vision/motion_vector_markers')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_timeout', 0.2)
        # 어느 트랙이든 이만큼 갱신이 없으면 칼만필터 상태를 버린다
        self.declare_parameter('track_timeout', 2.0)

        # 칼만필터 튜닝
        self.declare_parameter('kf_process_noise', 1.0)
        self.declare_parameter('kf_measurement_noise', 0.1)
        self.declare_parameter('kf_initial_velocity_variance', 4.0)

        # 방식 A: 가상 포인트
        # 이 시간(초) 뒤의 위치를 예측해 가상 포인트를 그 주변에 뿌린다
        self.declare_parameter('predict_horizon', 1.0)
        self.declare_parameter('virtual_points_per_track', 8)
        self.declare_parameter('virtual_point_radius_base', 0.15)
        # 속도(m/s) 1당 반경에 더해지는 값(m) - 빠를수록 더 넓게 마킹
        self.declare_parameter('virtual_point_radius_gain', 0.2)
        self.declare_parameter('point_z', 0.0)
        # 이 속도(m/s) 미만이면(거의 정지) 예측 포인트를 만들지 않는다 - 정지한 사람까지
        # 매번 부풀려 마킹할 필요는 없다
        self.declare_parameter('min_track_speed_for_prediction', 0.1)

        # 방식 B: costmap inflation 동적 조정
        # Nav2 local_costmap에 등록된 inflation layer 플러그인 인스턴스 이름
        self.declare_parameter('inflation_layer_name', 'inflation_layer')
        self.declare_parameter('base_inflation_radius', 0.55)
        self.declare_parameter('max_inflation_radius', 1.5)
        self.declare_parameter('inflation_radius_gain', 0.3)
        self.declare_parameter('base_cost_scaling_factor', 3.0)
        self.declare_parameter('min_cost_scaling_factor', 1.0)
        self.declare_parameter('cost_scaling_factor_gain', 0.5)
        # 이 접근속도(m/s) 이하면 조정 없이 기준값(base_*) 그대로 사용
        self.declare_parameter('approach_speed_min', 0.2)

        tracked_topic = self.get_parameter('tracked_detections_topic').value
        predicted_points_topic = self.get_parameter('predicted_points_topic').value
        costmap_param_service = self.get_parameter('local_costmap_param_service').value

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout').value)
        self.track_timeout = self.get_parameter('track_timeout').value

        self.kf_process_noise = self.get_parameter('kf_process_noise').value
        self.kf_measurement_noise = self.get_parameter('kf_measurement_noise').value
        self.kf_initial_velocity_variance = self.get_parameter('kf_initial_velocity_variance').value

        self.predict_horizon = self.get_parameter('predict_horizon').value
        self.virtual_points_per_track = self.get_parameter('virtual_points_per_track').value
        self.virtual_point_radius_base = self.get_parameter('virtual_point_radius_base').value
        self.virtual_point_radius_gain = self.get_parameter('virtual_point_radius_gain').value
        self.point_z = self.get_parameter('point_z').value
        self.min_track_speed_for_prediction = self.get_parameter('min_track_speed_for_prediction').value

        self.inflation_layer_name = self.get_parameter('inflation_layer_name').value
        self.base_inflation_radius = self.get_parameter('base_inflation_radius').value
        self.max_inflation_radius = self.get_parameter('max_inflation_radius').value
        self.inflation_radius_gain = self.get_parameter('inflation_radius_gain').value
        self.base_cost_scaling_factor = self.get_parameter('base_cost_scaling_factor').value
        self.min_cost_scaling_factor = self.get_parameter('min_cost_scaling_factor').value
        self.cost_scaling_factor_gain = self.get_parameter('cost_scaling_factor_gain').value
        self.approach_speed_min = self.get_parameter('approach_speed_min').value

        self.avoidance_mode = self.get_parameter('avoidance_mode').value
        self.publish_markers = self.get_parameter('publish_markers').value

        self.tracked_sub = self.create_subscription(
            Detection3DArray, tracked_topic, self.tracked_detections_callback, 10)

        self.predicted_points_pub = self.create_publisher(PointCloud2, predicted_points_topic, 10)
        self.markers_pub = None
        if self.publish_markers:
            self.markers_pub = self.create_publisher(
                MarkerArray, self.get_parameter('markers_topic').value, 10)
        self.costmap_param_client = self.create_client(SetParameters, costmap_param_service)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 트랙 id(str) -> ConstantVelocityKalman2D
        self.kalman_states = {}

        self.get_logger().info(
            f'predictive_avoidance_node 시작 | {tracked_topic} -> {predicted_points_topic} / '
            f'{costmap_param_service} (mode={self.avoidance_mode})'
        )

    def tracked_detections_callback(self, msg: Detection3DArray):
        stamp = stamp_to_sec(msg.header.stamp)
        for detection in msg.detections:
            # 검출마다 자기 관측 시각을 들고 온다(reid_tracking_node가 트랙의 last_stamp를
            # 찍어 보낸다). 배열 헤더 시각을 일괄로 쓰면 이번 프레임에 갱신되지 않은 트랙까지
            # 새 관측으로 취급돼 속도가 0으로 끌려간다. 시각이 비어 있는 상류(구버전 등)에는
            # 배열 헤더로 폴백한다.
            det_stamp = stamp_to_sec(detection.header.stamp) or stamp
            self.update_kalman(detection, det_stamp)
        self._prune_stale(stamp)

        if self.avoidance_mode in ('pointcloud', 'both'):
            self.publish_predicted_points(msg.header)
        if self.avoidance_mode in ('costmap_params', 'both'):
            self.adjust_costmap_inflation(stamp)
        if self.markers_pub is not None:
            self.publish_motion_vector_markers(msg.header)

    def update_kalman(self, detection, stamp):
        pos = extract_person_position(detection)
        if pos is None:
            return
        x, y, _z = pos
        kf = self.kalman_states.get(detection.id)
        if kf is None:
            self.kalman_states[detection.id] = ConstantVelocityKalman2D(
                x, y, stamp,
                process_noise=self.kf_process_noise,
                measurement_noise=self.kf_measurement_noise,
                initial_velocity_variance=self.kf_initial_velocity_variance,
            )
        else:
            kf.update(x, y, stamp)

    def _prune_stale(self, now):
        stale = [tid for tid, kf in self.kalman_states.items() if now - kf.last_stamp > self.track_timeout]
        for tid in stale:
            del self.kalman_states[tid]

    # ------------------------------------------------------------------ 방식 A: 가상 포인트

    def publish_predicted_points(self, header):
        points = []
        for kf in self.kalman_states.values():
            speed = kf.speed()
            if speed < self.min_track_speed_for_prediction:
                continue
            px, py = kf.predict_future(self.predict_horizon)
            radius = self.virtual_point_radius_base + self.virtual_point_radius_gain * speed
            points.extend(ring_points(px, py, self.point_z, radius, self.virtual_points_per_track))

        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.predicted_points_pub.publish(cloud)

    # ------------------------------------------------------------------ rviz2 시각화

    def publish_motion_vector_markers(self, header):
        """트랙별 속도 추정을 rviz2 MarkerArray로 발행한다(트랙당 화살표 1 + 라벨 1).

        마커 lifetime을 track_timeout으로 줘서 트랙이 사라지면 저절로 지워지게 한다 -
        DELETEALL을 매 프레임 보내면 rviz2에서 깜빡인다.
        """
        markers = MarkerArray()
        lifetime = Duration(seconds=self.track_timeout).to_msg()
        for slot, (tid, kf) in enumerate(sorted(self.kalman_states.items())):
            x, y = kf.position()
            vx, vy = kf.velocity()
            speed = kf.speed()
            # 화살표 끝 = 실제로 회피에 쓰는 예측 위치. 가상 포인트 링과 같은 지점이라
            # rviz2에서 둘이 겹치는지로 교차검증이 된다.
            px, py = kf.predict_future(self.predict_horizon)

            arrow = Marker()
            arrow.header.frame_id = self.map_frame
            arrow.header.stamp = header.stamp
            arrow.ns = 'motion_vector'
            arrow.id = slot
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.05   # 자루 지름
            arrow.scale.y = 0.12   # 머리 지름
            arrow.scale.z = 0.15   # 머리 길이
            # 예측에 반영되는 속도(초록) / 문턱 미만이라 무시되는 속도(회색)를 구분한다.
            moving = speed >= self.min_track_speed_for_prediction
            arrow.color.r = 0.3 if moving else 0.6
            arrow.color.g = 1.0 if moving else 0.6
            arrow.color.b = 0.3 if moving else 0.6
            arrow.color.a = 0.9
            arrow.points = [_point(x, y, self.point_z), _point(px, py, self.point_z)]
            arrow.lifetime = lifetime
            markers.markers.append(arrow)

            label = Marker()
            label.header = arrow.header
            label.ns = 'motion_vector_label'
            label.id = slot
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = _point(x, y, self.point_z + 0.3)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.2
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 0.9
            label.text = f'{tid} {speed:.2f} m/s'
            label.lifetime = lifetime
            markers.markers.append(label)

        self.markers_pub.publish(markers)

    # ------------------------------------------------------------------ 방식 B: costmap inflation

    def adjust_costmap_inflation(self, stamp):
        robot_xy = self._lookup_robot_position()
        best_speed = 0.0
        if robot_xy is not None:
            rx, ry = robot_xy
            for kf in self.kalman_states.values():
                x, y = kf.position()
                vx, vy = kf.velocity()
                speed = approach_speed(rx, ry, x, y, vx, vy)
                if speed > best_speed:
                    best_speed = speed

        effective = max(0.0, best_speed - self.approach_speed_min)
        inflation_radius = min(
            self.max_inflation_radius,
            self.base_inflation_radius + self.inflation_radius_gain * effective)
        cost_scaling_factor = max(
            self.min_cost_scaling_factor,
            self.base_cost_scaling_factor - self.cost_scaling_factor_gain * effective)

        self._call_set_parameters(inflation_radius, cost_scaling_factor)

    def _lookup_robot_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(), timeout=self.tf_timeout)
        except Exception as exc:
            self.get_logger().warn(f'로봇 위치(tf) 조회 실패: {exc}', throttle_duration_sec=2.0)
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def _call_set_parameters(self, inflation_radius, cost_scaling_factor):
        if not self.costmap_param_client.service_is_ready():
            self.get_logger().warn(
                'local_costmap set_parameters 서비스가 아직 응답하지 않음 (Nav2 미기동?)',
                throttle_duration_sec=5.0)
            return

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name=f'{self.inflation_layer_name}.inflation_radius',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=inflation_radius)),
            Parameter(
                name=f'{self.inflation_layer_name}.cost_scaling_factor',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=cost_scaling_factor)),
        ]
        future = self.costmap_param_client.call_async(request)
        future.add_done_callback(self._on_set_parameters_response)

    def _on_set_parameters_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'costmap set_parameters 호출 실패: {exc}', throttle_duration_sec=5.0)
            return
        for result in response.results:
            if not result.successful:
                self.get_logger().warn(f'costmap 파라미터 적용 거부됨: {result.reason}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = PredictiveAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
