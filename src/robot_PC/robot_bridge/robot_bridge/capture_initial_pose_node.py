#!/usr/bin/env python3
"""RViz로 잡은 현재 AMCL pose를 dock 초기 pose YAML에 1회 저장한다."""

import math
import os

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node

from robot_bridge.initial_pose_utils import (
    DEFAULT_COVARIANCE_XY,
    DEFAULT_COVARIANCE_YAW,
    DEFAULT_POSE_FILE,
    save_initial_pose,
)
from robot_bridge.pose_utils import quaternion_to_yaw


class CaptureInitialPoseNode(Node):
    """Map frame AMCL pose 한 건을 로봇별 dock pose로 저장한다."""

    def __init__(self, robot_id='', pose_file=''):
        super().__init__('capture_initial_pose_node')
        default_pose_file = pose_file or os.path.expanduser(DEFAULT_POSE_FILE)
        self.declare_parameter('robot_id', robot_id)
        self.declare_parameter('pose_file', default_pose_file)
        self.declare_parameter('initial_pose_covariance_xy', DEFAULT_COVARIANCE_XY)
        self.declare_parameter('initial_pose_covariance_yaw', DEFAULT_COVARIANCE_YAW)
        self.robot_id = str(self.get_parameter('robot_id').value).strip()
        if self.robot_id not in ('robot5', 'robot11'):
            raise ValueError('robot_id는 robot5 또는 robot11이어야 합니다.')
        self.pose_file = os.path.expanduser(str(self.get_parameter('pose_file').value))
        self.covariance_xy = float(
            self.get_parameter('initial_pose_covariance_xy').value)
        self.covariance_yaw = float(
            self.get_parameter('initial_pose_covariance_yaw').value)
        self.done = False
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{self.robot_id}/amcl_pose',
            self.pose_callback,
            10,
        )
        self.get_logger().info(
            f'/{self.robot_id}/amcl_pose 대기 중. RViz 2D Pose Estimate를 '
            '정확히 설정한 후 실행해야 합니다.')

    def pose_callback(self, msg):
        """유효한 map pose 첫 한 건을 저장한다."""
        if self.done:
            return
        frame = str(msg.header.frame_id).strip().lstrip('/')
        if frame != 'map' and not frame.endswith('/map'):
            self.get_logger().warn(f'map frame이 아닌 pose 무시: {msg.header.frame_id}')
            return
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        values = (position.x, position.y, *quaternion)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn('NaN/inf AMCL pose 무시')
            return
        if sum(value * value for value in quaternion) < 1e-12:
            self.get_logger().warn('유효하지 않은 AMCL quaternion 무시')
            return
        yaw = quaternion_to_yaw(*quaternion)
        saved_path = save_initial_pose(self.pose_file, self.robot_id, {
            'x': position.x,
            'y': position.y,
            'yaw': yaw,
            'covariance_xy': self.covariance_xy,
            'covariance_yaw': self.covariance_yaw,
        })
        self.done = True
        self.get_logger().info(
            f'{self.robot_id} dock pose 저장 완료: {saved_path} '
            f'(x={position.x:.3f}, y={position.y:.3f}, yaw={yaw:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = CaptureInitialPoseNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
