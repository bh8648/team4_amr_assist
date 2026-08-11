#!/usr/bin/env python3
"""저장한 dock pose를 Nav2 AMCL에 자동으로 설정한다."""

import math
import os

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node

from robot_bridge.initial_pose_utils import (
    DEFAULT_COVARIANCE_XY,
    DEFAULT_COVARIANCE_YAW,
    DEFAULT_POSE_FILE,
    load_initial_pose,
    validate_initial_pose,
)
from robot_bridge.pose_utils import quaternion_to_yaw


class InitialPosePublisherNode(Node):
    """AMCL이 확인할 때까지 한 번 캡처한 dock pose를 재전송한다."""

    def __init__(self, robot_id='', pose_file=''):
        super().__init__('initial_pose_publisher_node')
        default_pose_file = pose_file or os.path.expanduser(DEFAULT_POSE_FILE)
        self.declare_parameter('robot_id', robot_id)
        self.declare_parameter('pose_file', default_pose_file)
        self.declare_parameter('initial_pose_x', '')
        self.declare_parameter('initial_pose_y', '')
        self.declare_parameter('initial_pose_yaw', '')
        self.declare_parameter('initial_pose_covariance_xy', DEFAULT_COVARIANCE_XY)
        self.declare_parameter('initial_pose_covariance_yaw', DEFAULT_COVARIANCE_YAW)
        self.declare_parameter('initial_pose_delay_sec', 2.0)
        self.declare_parameter('initial_pose_retry_sec', 1.0)
        self.declare_parameter('initial_pose_max_attempts', 10)
        self.declare_parameter('initial_pose_reload_sec', 2.0)
        self.declare_parameter('initial_pose_confirm_position_tolerance', 0.75)
        self.declare_parameter('initial_pose_confirm_yaw_tolerance', 0.8)

        self.robot_id = str(self.get_parameter('robot_id').value).strip()
        if self.robot_id not in ('robot5', 'robot11'):
            raise ValueError('robot_id는 robot5 또는 robot11이어야 합니다.')
        self.pose_file = os.path.expanduser(str(self.get_parameter('pose_file').value))
        self.delay_sec = max(
            0.0, float(self.get_parameter('initial_pose_delay_sec').value))
        self.retry_sec = float(self.get_parameter('initial_pose_retry_sec').value)
        self.max_attempts = int(self.get_parameter('initial_pose_max_attempts').value)
        self.reload_sec = float(self.get_parameter('initial_pose_reload_sec').value)
        self.confirm_position_tolerance = float(
            self.get_parameter('initial_pose_confirm_position_tolerance').value)
        self.confirm_yaw_tolerance = float(
            self.get_parameter('initial_pose_confirm_yaw_tolerance').value)
        if (self.retry_sec <= 0.0 or self.reload_sec <= 0.0
                or self.max_attempts <= 0):
            raise ValueError('initial pose 재전송 주기/횟수가 올바르지 않습니다.')
        if self.confirm_position_tolerance <= 0.0 or self.confirm_yaw_tolerance <= 0.0:
            raise ValueError('initial pose 확인 허용오차는 0보다 커야 합니다.')

        topic_prefix = f'/{self.robot_id}'
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, f'{topic_prefix}/initialpose', 10)
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f'{topic_prefix}/amcl_pose',
            self.amcl_pose_callback,
            10,
        )
        self.attempts = 0
        self.confirmed = False
        self.start_timer = None
        self.retry_timer = None
        self.reload_timer = None
        self.pose = self._load_configured_pose()
        if self.pose is None:
            self.get_logger().error(
                f'{self.robot_id} dock pose가 없습니다: {self.pose_file}. '
                f'RViz에서 1회 위치 설정 후 capture_initial_pose를 실행하세요.')
            self.reload_timer = self.create_timer(self.reload_sec, self._try_reload_pose)
            return
        self._schedule_begin()

    def _schedule_begin(self):
        self.start_timer = self.create_timer(max(self.delay_sec, 0.01), self._begin)

    def _try_reload_pose(self):
        """누락된 dock pose가 재측정되면 launch 재시작 없이 읽어들인다."""
        pose = self._load_configured_pose()
        if pose is None:
            return
        self.pose = pose
        if self.reload_timer is not None:
            self.reload_timer.cancel()
        self.get_logger().info(f'{self.robot_id} 재측정 dock pose 로드 완료')
        self._schedule_begin()

    def _load_configured_pose(self):
        explicit_names = ('initial_pose_x', 'initial_pose_y', 'initial_pose_yaw')
        explicit = [str(self.get_parameter(name).value).strip() for name in explicit_names]
        if any(explicit):
            if not all(explicit):
                raise ValueError('initial_pose_x/y/yaw는 모두 지정해야 합니다.')
            return validate_initial_pose({
                'x': explicit[0],
                'y': explicit[1],
                'yaw': explicit[2],
                'covariance_xy': self.get_parameter(
                    'initial_pose_covariance_xy').value,
                'covariance_yaw': self.get_parameter(
                    'initial_pose_covariance_yaw').value,
            })
        try:
            return load_initial_pose(self.pose_file, self.robot_id)
        except (FileNotFoundError, KeyError):
            if self.robot_id == 'robot5':
                self.get_logger().info(
                    'robot5 저장 dock pose가 없어 기본값 x=0, y=0, yaw=0을 사용합니다.')
                return validate_initial_pose({'x': 0.0, 'y': 0.0, 'yaw': 0.0})
            return None
        except (OSError, ValueError):
            return None

    def _begin(self):
        if self.start_timer is not None:
            self.start_timer.cancel()
        self.publish_initial_pose()
        if not self.confirmed and self.attempts < self.max_attempts:
            self.retry_timer = self.create_timer(self.retry_sec, self.publish_initial_pose)

    def build_message(self):
        """저장 pose를 RViz 2D Pose Estimate와 같은 메시지로 변환한다."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.pose['x']
        msg.pose.pose.position.y = self.pose['y']
        half_yaw = self.pose['yaw'] / 2.0
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)
        msg.pose.covariance[0] = self.pose['covariance_xy']
        msg.pose.covariance[7] = self.pose['covariance_xy']
        msg.pose.covariance[35] = self.pose['covariance_yaw']
        return msg

    def publish_initial_pose(self):
        """Dock pose를 발행하고 최대 시도 횟수에서 멈춘다."""
        if self.pose is None or self.confirmed:
            return
        if self.attempts >= self.max_attempts:
            if self.retry_timer is not None:
                self.retry_timer.cancel()
            self.get_logger().error(
                f'{self.robot_id} AMCL이 initial pose를 확인하지 않았습니다.')
            return
        self.initial_pose_pub.publish(self.build_message())
        self.attempts += 1
        self.get_logger().info(
            f'{self.robot_id} dock initial pose 전송 '
            f'({self.attempts}/{self.max_attempts}): '
            f'x={self.pose["x"]:.3f}, y={self.pose["y"]:.3f}, '
            f'yaw={self.pose["yaw"]:.3f}')

    def amcl_pose_callback(self, msg):
        """Initial pose 전송 후 AMCL이 dock pose 근처를 반환하면 재전송을 종료한다."""
        if self.attempts <= 0 or self.confirmed:
            return
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        position_error = math.hypot(
            position.x - self.pose['x'], position.y - self.pose['y'])
        current_yaw = quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w)
        yaw_delta = current_yaw - self.pose['yaw']
        yaw_error = abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
        if (position_error > self.confirm_position_tolerance
                or yaw_error > self.confirm_yaw_tolerance):
            return
        self.confirmed = True
        if self.retry_timer is not None:
            self.retry_timer.cancel()
        self.get_logger().info(f'{self.robot_id} AMCL initial pose 확인')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
