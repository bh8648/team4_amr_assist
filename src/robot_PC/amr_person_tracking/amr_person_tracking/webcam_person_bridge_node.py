#!/usr/bin/env python3
"""
webcam_person_bridge_node.py

[역할]
별도 PC에서 도는 외부 웹캠 로컬라이저(person_locator 패키지)의 출력을 이 파이프라인이
쓸 수 있는 형태로 변환해 준다. 지금 담당하는 것은 **호출 좌표 한 갈래**다.

  person_locator                     이 노드                      reid_tracking_node
  ------------------------------     -----------------------      --------------------
  /person/call_position          ->  프레임 변환 + 재스탬프   ->  <ns>/vision/webcam/
  (PointStamped, frame_id=map)       + 유효성 검사                call_position
                                                                  -> 추종 대상 지정

[왜 별도 노드인가]
1. tf2를 reid_tracking_node 밖에 둔다. reid는 지금 tf2를 전혀 쓰지 않고 launch에도
   tf remap이 없다. 터틀봇4는 tf를 <ns>/tf로 발행하므로 reid에 tf 조회를 넣으면
   조용히 실패한다(launch 파일의 tf_remappings 주석 참고). 이 노드만 remap을 받는다.
2. 전역 토픽 -> 네임스페이스 토픽 변환 지점을 한 곳으로 모은다. /person/call_position은
   현장에 하나뿐인 전역 토픽이라, 로봇마다 reid가 직접 구독하면 제스처 한 번에
   **모든 로봇이 동시에 재지정**된다. 이 노드가 자기 로봇 것으로만 중계한다.

[클럭]
원격 PC의 스탬프를 기본적으로 신뢰하지 않는다. 이유는 webcam_bridge_utils 모듈
docstring 참고 - 어긋난 방향에 따라 "조용히 무시" 또는 "트랙 테이블 전멸"로 갈린다.

[입력]
  - <call_input_topic> (geometry_msgs/PointStamped, 기본 /person/call_position)

[출력]
  - <call_output_topic> (geometry_msgs/PointStamped, output_frame으로 변환됨)

[미구현]
  연속 검출 융합(person/position -> Detection3DArray)은 아직 하지 않는다. 웹캠은
  트랙 id를 발행하지 않고 사람당 스트림도 아니라, 상시 융합하면 트랙 churn과 라이다
  락온 오작동 위험이 크다. publish_detections 파라미터 자리만 남겨 둔다.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PointStamped

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (PointStamped용 tf2 변환 등록)

from amr_person_tracking.webcam_bridge_utils import (
    SkewTracker,
    is_message_fresh,
    is_valid_point,
    resolve_stamp,
)


class WebcamPersonBridgeNode(Node):
    """외부 웹캠의 호출 좌표를 이 로봇의 좌표계/클럭으로 중계한다."""

    def __init__(self):
        super().__init__('webcam_person_bridge_node')

        self.declare_parameter('call_input_topic', '/person/call_position')
        # 상대 토픽. 노드 네임스페이스 아래로 해석되므로 -r __ns:=/robot6 하나만으로
        # 로봇별 분리가 된다(절대 경로 기본값은 다른 로봇을 가리키는 함정이 된다).
        self.declare_parameter('call_output_topic', 'vision/webcam/call_position')
        # 웹캠이 신고하는 좌표계. person_locator는 'map'을 하드코딩해 발행한다.
        self.declare_parameter('expected_source_frame', 'map')
        # 이 파이프라인이 쓰는 좌표계. launch의 map_frame과 같아야 한다.
        self.declare_parameter('output_frame', 'map')
        # tf     : tf2로 변환한다(기본). AMCL이 없어 map 프레임이 없으면 변환이 실패하고
        #          아무것도 발행하지 않는다 - 조용히 틀린 좌표를 내보내는 것보다 낫다.
        # reject : 프레임이 다르면 버린다.
        # assume : 프레임 이름만 바꿔 그대로 믿는다. 벤치 테스트 전용, 실기 금지.
        self.declare_parameter('frame_policy', 'tf')
        self.declare_parameter('tf_timeout', 0.2)
        self.declare_parameter('tf_allow_latest_fallback', False)
        # receive_time | passthrough | offset_corrected (webcam_bridge_utils 참고)
        self.declare_parameter('stamp_policy', 'receive_time')
        self.declare_parameter('max_message_age_sec', 1.0)
        self.declare_parameter('clock_skew_warn_sec', 0.5)
        # 발행자가 RELIABLE이라 기본을 맞춘다. Wi-Fi에서 재전송이 밀리면 best_effort로.
        self.declare_parameter('input_reliability', 'reliable')
        # 연속 검출 융합. 아직 구현하지 않았다(모듈 docstring 참고).
        self.declare_parameter('publish_detections', False)

        self.expected_source_frame = self.get_parameter('expected_source_frame').value
        self.output_frame = self.get_parameter('output_frame').value
        self.frame_policy = self.get_parameter('frame_policy').value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout').value)
        self.tf_allow_latest_fallback = self.get_parameter('tf_allow_latest_fallback').value
        self.stamp_policy = self.get_parameter('stamp_policy').value
        self.max_message_age_sec = self.get_parameter('max_message_age_sec').value
        self.clock_skew_warn_sec = self.get_parameter('clock_skew_warn_sec').value

        if self.get_parameter('publish_detections').value:
            self.get_logger().warn(
                'publish_detections는 아직 구현되지 않았다 - 호출 좌표만 중계한다')

        self.skew = SkewTracker()
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        reliability = (ReliabilityPolicy.BEST_EFFORT
                       if self.get_parameter('input_reliability').value == 'best_effort'
                       else ReliabilityPolicy.RELIABLE)
        input_topic = self.get_parameter('call_input_topic').value
        output_topic = self.get_parameter('call_output_topic').value
        self.call_pub = self.create_publisher(PointStamped, output_topic, 10)
        self.create_subscription(
            PointStamped, input_topic, self.call_callback,
            QoSProfile(depth=10, reliability=reliability))

        self.get_logger().info(
            f'webcam_person_bridge_node 시작 | {input_topic} -> {output_topic} '
            f'(frame {self.expected_source_frame} -> {self.output_frame}, '
            f'policy={self.frame_policy}, stamp={self.stamp_policy})')
        # 호모그래피는 특정 map에서 카메라를 고정한 채 AMCL 기준으로 맞춘 값이다.
        # 프레임 처리가 맞아도 카메라를 옮겼거나 map을 다시 만들었으면 좌표는 틀린다.
        self.get_logger().info(
            '주의: 웹캠 좌표는 카메라 고정 + 동일 map 전제에서만 유효하다')

    def call_callback(self, msg: PointStamped):
        """호출 좌표 한 건을 검증/변환해 자기 로봇 토픽으로 중계한다."""
        local_sec = self.get_clock().now().nanoseconds * 1e-9
        remote_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not is_valid_point(msg.point.x, msg.point.y):
            self.get_logger().warn('호출 좌표에 NaN/inf - 폐기')
            return

        offset = self.skew.update(remote_sec, local_sec)
        if abs(offset) > self.clock_skew_warn_sec:
            self.get_logger().warn(
                f'웹캠 PC와 클럭 편차 {offset:+.2f}s (> {self.clock_skew_warn_sec:.2f}s). '
                f'stamp_policy={self.stamp_policy}로 보정 중이나 두 PC 시각 동기를 권장한다',
                throttle_duration_sec=10.0)
        if not is_message_fresh(remote_sec, local_sec, offset, self.max_message_age_sec):
            self.get_logger().warn(
                f'호출 좌표가 너무 늦게 도착 (지연 {local_sec - remote_sec - offset:.2f}s) - 폐기')
            return

        transformed = self._to_output_frame(msg)
        if transformed is None:
            return

        out = PointStamped()
        out.header.frame_id = self.output_frame
        stamp_sec = resolve_stamp(self.stamp_policy, remote_sec, local_sec, offset)
        out.header.stamp.sec = int(stamp_sec)
        out.header.stamp.nanosec = int(round((stamp_sec - int(stamp_sec)) * 1e9))
        out.point = transformed.point
        self.call_pub.publish(out)
        self.get_logger().info(
            f'호출 좌표 중계: ({out.point.x:.2f}, {out.point.y:.2f}) '
            f'[{self.output_frame}] 원본 프레임={msg.header.frame_id or "(빈값)"}')

    def _to_output_frame(self, msg: PointStamped):
        """frame_policy에 따라 output_frame으로 옮긴다. 실패하면 None."""
        src = msg.header.frame_id
        if src == self.output_frame:
            return msg
        if src != self.expected_source_frame:
            self.get_logger().warn(
                f'예상치 못한 프레임 "{src or "(빈값)"}" '
                f'(기대 {self.expected_source_frame}) - 폐기', throttle_duration_sec=2.0)
            return None
        if self.frame_policy == 'assume':
            return msg
        if self.frame_policy == 'reject':
            self.get_logger().warn(
                f'{src} != {self.output_frame} 이고 frame_policy=reject - 폐기',
                throttle_duration_sec=2.0)
            return None
        try:
            return self.tf_buffer.transform(msg, self.output_frame, timeout=self.tf_timeout)
        except Exception as exc:
            if self.tf_allow_latest_fallback:
                try:
                    latest = PointStamped()
                    latest.header.frame_id = src   # stamp를 비우면 최신 변환을 쓴다
                    latest.point = msg.point
                    return self.tf_buffer.transform(
                        latest, self.output_frame, timeout=self.tf_timeout)
                except Exception:
                    pass
            self.get_logger().warn(
                f'{self.output_frame} 변환 실패: {exc}', throttle_duration_sec=2.0)
            return None


def main():
    rclpy.init()
    node = WebcamPersonBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
