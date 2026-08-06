#!/usr/bin/env python3
"""
reid_tracking_node.py

[역할]
전체 비전 파이프라인 문서의 "7번 재식별/트래킹" 노드. 지금 단계에서는 "틀만 만들어두고
나중에 디벨롭"하는 것이 목표이며, 실제 게이팅/매칭 알고리즘은 To-do로 남겨둔다.

고정캠(웹캠) 로컬라이제이션 노드와 oakd_detector_node가 각각 발행하는
vision_msgs/Detection3DArray(둘 다 frame_id=map, 동일 스키마)를 통합 입력으로 받아,
가려짐/프레임 이탈 후에도 같은 사람을 같은 트랙 ID로 유지시킨 뒤 재발행한다.

여기에 더해, 터틀봇4가 사람에게 아주 가까이 접근해 카메라에 다리만 보이는 초근접 구간에서는
leg_detector_bridge_node가 라이다 다리검출 결과를 같은 스키마로 편입시켜 준다. 다만 라이다
다리검출은 "다리쌍이 여기 있다"는 위치/속도만 알 뿐 신원을 모르므로, 웹캠이 추종하던 타겟과
라이다가 검출한 다리쌍이 같은 사람인지는 이 노드가 별도로 판별해야 한다 (아래 leg_detections_callback
참고).

[처리 흐름]
웹캠 로컬라이제이션 스트림 + oakd_detector_node 스트림 구독 (통합)
  -> 3D 위치 기반 게이팅 (마지막 위치 + 속도로 근접 매칭)
  -> (다인원이 자주 겹치는 경우) appearance 임베딩(OSNet) 보조 매칭
  -> 원거리(웹캠)->근접(OAK-D) 전환 시, 동일 인물의 소스가 바뀌는 지점에서
     칼만필터로 좌표를 블렌딩해 좌표 불연속 완화
  -> 표준 id 필드에 지속 트랙 ID를 기록해 Detection3DArray 재발행
  -> 추종 대상 트랙 하나를 선정해 2D map 좌표(PoseStamped)로 별도 발행
     (Nav2 goal 발행/액션 호출 로직 자체는 이 노드의 범위가 아님 - 좌표만 표준 메시지로 제공)

leg_detector_bridge_node 스트림은 별도 콜백(leg_detections_callback)에서 다음 순서로 처리한다:
  1. 시간 정렬  - followed_track_id의 최근 위치+속도(칼만필터 추정치)로 라이다 메시지의
     header.stamp 시점까지 위치를 외삽(predict)
  2. 게이팅 매칭 - 외삽된 예측 위치와 각 라이다 검출 위치 간 거리를 계산해 임계값 이내 후보만 필터링
  3. 모호성 해소 - 후보가 여럿이면 위치 거리뿐 아니라 속도 벡터(방향+크기) 유사도까지 결합해 하나 선정
  4. 락온 확정  - 같은 라이다 트랙 ID가 연속 N프레임 동안 게이트 안에 들어오면 확정, 그 전까지는
     웹캠/OAK-D 값만 사용
  5. 추종 유지  - 락온 중에는 재매칭하지 않고 락온된 라이다 트랙 ID를 그대로 추종
  6. 백그라운드 검증 - 락온 중에도 웹캠 추정 위치와 계속 비교해서, 차이가 임계값 이상 벌어지는
     상태가 이어지면 "ID 스왑 의심"으로 락온을 해제하고 2~4단계를 재트리거
  (전제조건: 이 매칭 전체가 map 좌표계 일치, 즉 로봇 localization(AMCL 등) 정확도를 전제로 한다.
   매칭이 계속 실패하면 라이다/웹캠 임계값 튜닝보다 로봇 localization 드리프트를 먼저 의심할 것)

[입력 토픽]
  - <namespace>/vision/detections_3d          (vision_msgs/Detection3DArray, oakd_detector_node)
  - <webcam_namespace>/vision/detections_3d    (vision_msgs/Detection3DArray, 웹캠 로컬라이제이션 노드, 외부)
  - <namespace>/vision/leg_detections_3d       (vision_msgs/Detection3DArray, leg_detector_bridge_node)

[출력 토픽]
  - <namespace>/vision/tracked_detections_3d  (vision_msgs/Detection3DArray, id 필드에 지속 트랙 ID)
  - <namespace>/target_person_pose            (geometry_msgs/PoseStamped, frame_id=map, 추종 대상 2D 좌표)
"""

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped


class ReidTrackingNode(Node):

    def __init__(self):
        super().__init__('reid_tracking_node')

        self.declare_parameter('oakd_detections_topic', '/robot5/vision/detections_3d')
        self.declare_parameter('webcam_detections_topic', '/vision/webcam/detections_3d')
        self.declare_parameter('leg_detections_topic', '/robot5/vision/leg_detections_3d')
        self.declare_parameter('tracked_detections_topic', '/robot5/vision/tracked_detections_3d')
        self.declare_parameter('target_pose_topic', '/robot5/target_person_pose')
        self.declare_parameter('map_frame', 'map')
        # 게이팅 시 동일 인물로 간주할 최대 이동 거리(m/s 기준 속도 게이트)
        self.declare_parameter('gating_max_speed', 2.0)
        # 웹캠-라이다 신원 매칭: 예측 위치 대비 라이다 검출 후보를 인정할 최대 거리(m)
        self.declare_parameter('lidar_gating_position_threshold', 0.6)
        # 웹캠-라이다 신원 매칭: 같은 라이다 트랙 ID가 이만큼 연속으로 게이트 안에 들어오면 락온 확정
        self.declare_parameter('lidar_lock_confirm_frames', 7)
        # 웹캠-라이다 신원 매칭: 락온 중 웹캠 위치와 이 이상 벌어지면 ID 스왑 의심, 재매칭 트리거
        self.declare_parameter('lidar_lock_swap_threshold', 0.3)

        oakd_topic = self.get_parameter('oakd_detections_topic').value
        webcam_topic = self.get_parameter('webcam_detections_topic').value
        leg_topic = self.get_parameter('leg_detections_topic').value
        tracked_topic = self.get_parameter('tracked_detections_topic').value
        target_pose_topic = self.get_parameter('target_pose_topic').value

        self.oakd_sub = self.create_subscription(
            Detection3DArray, oakd_topic, self.oakd_detections_callback, 10)
        self.webcam_sub = self.create_subscription(
            Detection3DArray, webcam_topic, self.webcam_detections_callback, 10)
        self.leg_sub = self.create_subscription(
            Detection3DArray, leg_topic, self.leg_detections_callback, 10)

        self.tracked_pub = self.create_publisher(Detection3DArray, tracked_topic, 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)

        # To-do: 트랙 상태 저장소 (track_id -> 마지막 위치/속도/출처(webcam|oakd|lidar_leg)/appearance 임베딩 등)
        self.tracks = {}
        # To-do: 현재 추종 중인 트랙 ID (최초 지정 후 유지, 유실 시 재선정 정책 필요)
        self.followed_track_id = None

        # 웹캠-라이다 신원 매칭 상태
        # To-do: 락온된 라이다 트랙 ID. None이면 아직 라이다 쪽과 매칭 확정 전 (웹캠/OAK-D 값만 사용)
        self.locked_lidar_track_id = None
        # To-do: 라이다 트랙 ID별 연속 게이팅 성공 프레임 수 (lidar_lock_confirm_frames 이상이면 락온)
        self.lock_candidate_streak = {}

        self.get_logger().info(
            f'reid_tracking_node 시작 | oakd={oakd_topic} webcam={webcam_topic} leg={leg_topic} '
            f'-> {tracked_topic}, {target_pose_topic}'
        )

    def oakd_detections_callback(self, msg: Detection3DArray):
        # To-do: oakd_detector_node에서 들어온 근접 검출을 트랙과 게이팅 매칭
        # To-do: 동일 트랙이 webcam 출처 -> oakd 출처로 막 전환된 시점이면 칼만필터로 좌표 블렌딩
        self.update_tracks(msg, source='oakd')

    def webcam_detections_callback(self, msg: Detection3DArray):
        # To-do: 웹캠 로컬라이제이션 노드에서 들어온 원거리 검출을 트랙과 게이팅 매칭
        self.update_tracks(msg, source='webcam')

    def leg_detections_callback(self, msg: Detection3DArray):
        """웹캠(또는 OAK-D)이 추종 중이던 타겟과 라이다 다리검출 결과의 신원을 대조한다.

        라이다 다리검출은 위치/속도만 줄 뿐 "누구인지"는 모르므로, followed_track_id의
        위치를 라이다 프레임 시각으로 외삽해 대조하는 절차가 필요하다.
        """
        if self.locked_lidar_track_id is not None:
            # To-do: [5. 추종 유지] 이미 락온된 상태 - 재매칭하지 않고 locked_lidar_track_id에
            #        해당하는 detection을 그대로 이 프레임의 followed 트랙 위치로 사용
            # To-do: [6. 백그라운드 검증] 락온된 detection 위치와 웹캠/OAK-D 쪽 최신 추정 위치를
            #        계속 비교. 차이가 lidar_lock_swap_threshold 이상인 상태가 몇 프레임
            #        연속되면 "ID 스왑 의심" -> self.locked_lidar_track_id = None, streak 초기화
            #        후 아래 재매칭 절차로 다시 진입
            pass
        else:
            # To-do: [1. 시간 정렬] followed_track_id의 최근 칼만필터 위치+속도로
            #        msg.header.stamp 시점까지 위치 외삽(predict). 수신 시각이 아니라
            #        라이다 메시지의 header.stamp(스캔 시각) 기준으로 Δt 계산
            # To-do: [2. 게이팅 매칭] 외삽된 예측 위치와 msg.detections 각각의 위치 간 거리 계산,
            #        lidar_gating_position_threshold 이내 후보만 필터링
            # To-do: [3. 모호성 해소] 후보가 여럿이면 위치 거리 + 속도 벡터(방향/크기) 유사도를
            #        결합해 최적 후보 하나 선정
            # To-do: [4. 락온 확정] 선정된 라이다 트랙 ID의 lock_candidate_streak를 +1,
            #        나머지 트랙 ID는 0으로 리셋. streak가 lidar_lock_confirm_frames 이상이면
            #        self.locked_lidar_track_id로 확정
            pass

        # To-do: (전제조건) 이 매칭은 라이다/웹캠 검출이 모두 동일 map 좌표계로 변환되어 있다는
        #        전제 하에 동작한다. 매칭이 계속 실패하면 로봇 localization(AMCL 등) 드리프트를
        #        먼저 의심할 것 — 게이팅 임계값 튜닝으로 해결되는 문제가 아닐 수 있다.

    def update_tracks(self, msg: Detection3DArray, source: str):
        # To-do: 3D 위치 기반 게이팅 (마지막 위치 + 속도로 근접 매칭)
        # To-do: 다인원이 자주 겹치는 경우 appearance 임베딩(OSNet) 비교로 보조 매칭
        # To-do: 매칭 결과로 트랙 갱신 후 id 필드를 채운 Detection3DArray 재발행
        self.publish_tracked([], msg.header)
        self.publish_target_pose(msg.header)

    def publish_tracked(self, detections, header):
        msg = Detection3DArray()
        msg.header = header
        msg.detections = detections
        self.tracked_pub.publish(msg)

    def publish_target_pose(self, header):
        # To-do: self.followed_track_id에 해당하는 트랙의 map 좌표(x, y)를 조회해
        #        z를 0으로 눌러 PoseStamped로 채운 뒤 발행 (Nav2 쪽 goal 발행 로직은 별도 노드 담당)
        pose = PoseStamped()
        pose.header = header
        pose.header.frame_id = self.get_parameter('map_frame').value
        self.target_pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = ReidTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
