#!/usr/bin/env python3
"""
predictive_avoidance_node.py

[역할]
전체 비전 파이프라인 문서의 "6번 예측적 회피" 노드. 빠르게 접근하는 대상(사람/장애물)에
로봇이 미리 반응할 수 있도록, 트래킹된 대상의 위치 시계열로부터 속도를 추정하고 예측 위치를
로봇의 local_costmap에 반영한다.

[처리 흐름]
재식별/트래킹 노드의 트랙 시계열(id 포함 Detection3DArray) 구독
  -> 트랙별 등속도 모델 칼만필터로 속도 추정
     (주의: 프레임 간 Δt는 수신 시각이 아니라 메시지 header.stamp 기준으로 계산.
      네트워크 지연으로 프레임 간격이 불규칙해질 수 있음을 감안한 설계)
  -> 예측된 미래 위치 주변에 속도에 비례한 범위의 가상 포인트 생성
  -> (방식 A) 가상 포인트를 PointCloud2로 발행해 voxel_layer가 실제 장애물처럼 마킹하도록 함
  -> (방식 B) 접근 속도에 비례해 각 로봇 local_costmap의 inflation 파라미터를
     SetParameters 서비스로 동적 조정
  둘 중 하나 또는 병행 (파라미터로 선택)

[입력 토픽]
  - <namespace>/vision/tracked_detections_3d (vision_msgs/Detection3DArray, id 포함)

[출력]
  - <namespace>/vision/predicted_obstacle_points (sensor_msgs/PointCloud2)  [방식 A]
  - rcl_interfaces/SetParameters 서비스 호출 (각 로봇 local_costmap 노드 대상) [방식 B]
"""

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection3DArray
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.srv import SetParameters


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

        tracked_topic = self.get_parameter('tracked_detections_topic').value
        predicted_points_topic = self.get_parameter('predicted_points_topic').value
        costmap_param_service = self.get_parameter('local_costmap_param_service').value

        self.tracked_sub = self.create_subscription(
            Detection3DArray, tracked_topic, self.tracked_detections_callback, 10)

        self.predicted_points_pub = self.create_publisher(PointCloud2, predicted_points_topic, 10)
        self.costmap_param_client = self.create_client(SetParameters, costmap_param_service)

        # To-do: 트랙 id -> 칼만필터 상태(위치, 속도, 마지막 header.stamp) 저장소
        self.kalman_states = {}

        self.get_logger().info(
            f'predictive_avoidance_node 시작 | {tracked_topic} -> {predicted_points_topic} / {costmap_param_service}'
        )

    def tracked_detections_callback(self, msg: Detection3DArray):
        # To-do: msg.header.stamp를 기준으로 트랙별 이전 stamp와의 Δt 계산 (수신 시각 사용 금지)
        # To-do: 트랙 id별 칼만필터(등속도 모델) 업데이트 -> 속도 추정
        # To-do: 추정 속도로 예측 위치 계산, 속도에 비례한 범위의 가상 포인트 집합 생성
        for detection in msg.detections:
            self.update_kalman(detection, msg.header)

        mode = self.get_parameter('avoidance_mode').value
        if mode in ('pointcloud', 'both'):
            self.publish_predicted_points(msg.header)
        if mode in ('costmap_params', 'both'):
            self.adjust_costmap_inflation()

    def update_kalman(self, detection, header):
        # To-do: header.stamp 기반 Δt로 등속도 모델 칼만필터 predict/update 수행
        pass

    def publish_predicted_points(self, header):
        # To-do: 가상 포인트들을 PointCloud2로 인코딩해 발행 (voxel_layer의 observation_source로 등록됨)
        cloud = PointCloud2()
        cloud.header = header
        self.predicted_points_pub.publish(cloud)

    def adjust_costmap_inflation(self):
        # To-do: 접근 속도에 비례한 inflation_radius/cost_scaling_factor 파라미터 값 계산 후
        #        SetParameters 요청 생성 및 costmap_param_client.call_async 호출
        pass


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
