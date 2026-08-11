#!/usr/bin/env python3
"""
reid_tracking_node.py

[역할]
전체 비전 파이프라인 문서의 "7번 재식별/트래킹" 노드.

고정캠(웹캠) 로컬라이제이션 노드와 oakd_detector_node가 각각 발행하는
vision_msgs/Detection3DArray(둘 다 frame_id=map, 동일 스키마)를 통합 입력으로 받아,
가려짐/프레임 이탈 후에도 같은 사람을 같은 트랙 ID로 유지시킨 뒤 재발행한다.

여기에 더해, 터틀봇4가 사람에게 아주 가까이 접근해 카메라에 다리만 보이는 초근접 구간에서는
leg_detector_bridge_node가 라이다 다리검출 결과를 같은 스키마로 편입시켜 준다. 다만 라이다
다리검출은 "다리쌍이 여기 있다"는 위치만 알 뿐 신원을 모르므로, 웹캠/OAK-D가 추종하던 타겟과
라이다가 검출한 다리쌍이 같은 사람인지는 이 노드가 별도로 판별해야 한다 (leg_detections_callback).

[의도적 단순화 - tracking_utils.py 참고]
문서는 출처 전환 시 "칼만필터로 좌표 블렌딩"을 말하지만, 여기서는 완전한 칼만필터 행렬 대신
지수평활 기반 등속도 모델(Track.update)을 쓴다. predictive_avoidance_node가 이 파이프라인의
실제 칼만필터 담당으로 문서에 이미 지정돼 있어, 이 노드는 그보다 가벼운 방식으로 충분하다.

[웹캠/OAK-D 게이팅 + 출처 전환 블렌딩]
update_tracks(oakd_detections_callback/webcam_detections_callback 공용)가 처리한다:
  1. 검출 위치 -> 기존 트랙에 위치+속도 게이팅으로 비용행렬을 만들고 헝가리안 전역 할당
     (tracking_utils.assign_tracks). 게이팅이 실패해도 상류 트래커 id 힌트로 구제한다
  2. 매칭 실패 시 새 트랙 생성, 지속 트랙 ID 부여
  3. 매칭된 트랙의 직전 출처와 이번 출처가 다르면(webcam<->oakd 전환) 이번 한 프레임만
     position_alpha를 낮춰 위치를 블렌딩해 좌표 불연속을 완화
  4. 단, 그 트랙이 현재 라이다 다리검출에 락온된 추종 대상이면 위치/속도를 덮어쓰지 않는다
     (아래 leg_detections_callback의 5단계 "추종 유지"와 충돌 - 두 출처가 매 프레임 번갈아
     쓰면 유한차분 속도 추정이 실제로 없는 왕복 운동으로 튄다). last_independent_update만
     기록해 6단계 백그라운드 검증에 쓴다
  5. 다인원이 자주 겹치는 경우를 위한 appearance 임베딩 보조 매칭은 **구현돼 있고 기본만 꺼져
     있다**(appearance_weight=0). oakd_detector_node가 reid_model_path로 임베딩을 발행할 때만
     동작하며, 켜면 assign_tracks의 비용에 코사인 거리가 가중 합산된다. 기본을 끈 이유는
     실측에서 점프가 오히려 늘었기 때문이다(evidence/evidence_reid_embedding_log.txt)

[웹캠/OAK-D <-> 라이다 다리검출 신원 매칭]
leg_detections_callback이 다음 순서로 처리한다 (추종 대상 트랙 하나에 대해서만 동작):
  1. 시간 정렬  - followed_track_id의 최근 위치+속도로 라이다 메시지의 header.stamp 시점까지
     위치를 외삽(Track.predict)
  2. 게이팅 매칭 - 외삽된 예측 위치와 각 라이다 검출 위치 간 거리를 계산해 lidar_gating_position_threshold
     이내 후보만 필터링
  3. 모호성 해소 - 후보가 여럿이면 위치 거리뿐 아니라 속도 벡터(방향+크기) 유사도까지 결합해 점수화
     (속도는 라이다 쪽에 없는 필드라 같은 라이다 트랙 id의 연속 프레임 위치를 직접 미분해 구한다)
  4. 락온 확정  - 같은 라이다 트랙 ID가 연속 lidar_lock_confirm_frames 프레임 동안 최고 후보로
     선정되면 확정, 그 전까지는 웹캠/OAK-D 값만 사용. 다만 "이번 스캔에 게이팅을 통과하는
     후보가 아예 없음"은 스트릭을 리셋하지 않는다 - 원시 다리검출기가 스캔마다 검출/미검출을
     오가는 게 흔해서(실측 확인), 빈 스캔마다 리셋하면 연속 확정 자체가 사실상 불가능해진다.
     리셋은 "다른 후보가 새로 최고점이 됐을 때"만 발생한다.
  5. 추종 유지  - 락온 중에는 재매칭하지 않고 락온된 라이다 트랙 ID를 그대로 추종. 해당 id가
     잠깐 안 보이면 leg_lock_grace_period 동안은 유지, 그 이상 안 보이면 락온 해제
  6. 백그라운드 검증 - 락온 중에도, 웹캠/OAK-D가 "마지막으로 독립적으로" 갱신한 추정 위치와
     라이다 위치를 계속 비교한다(라이다로 덮어쓴 값끼리 비교하면 항상 일치하므로 의미가 없다).
     차이가 lidar_lock_swap_threshold 이상인 상태가 lidar_swap_confirm_frames 프레임 연속되면
     "ID 스왑 의심"으로 락온을 해제하고 1~4단계를 재트리거. 웹캠/OAK-D 값이 너무 오래됐으면
     (swap_check_max_age 초과) 검증할 근거가 없으므로 이번 프레임은 판단을 보류한다
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
from rclpy.qos import qos_profile_sensor_data

import math
import array

from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesis, ObjectHypothesisWithPose
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image

from amr_person_tracking.predictive_utils import ConstantVelocityKalman2D
from amr_person_tracking.tracking_utils import (
    Track,
    assign_tracks,
    resolve_call_designation,
    sec_to_stamp,
    cosine_similarity,
    distance,
    extract_person_position,
    extract_person_variance,
    stamp_to_sec,
    velocity_similarity,
)


class ReidTrackingNode(Node):

    def __init__(self):
        super().__init__('reid_tracking_node')

        self.declare_parameter('oakd_detections_topic', '/robot5/vision/detections_3d')
        self.declare_parameter('webcam_detections_topic', '/vision/webcam/detections_3d')
        self.declare_parameter('leg_detections_topic', '/robot5/vision/leg_detections_3d')
        # 트랙 생성/소멸을 좌표·상류id·최근접 기존트랙 거리와 함께 찍는다. 한 사람이 서 있는데
        # 트랙이 여러 개 만들어지는(=추종이 깜빡이는) 원인을 가리기 위한 진단용이라 기본은 꺼둔다.
        self.declare_parameter('log_track_lifecycle', False)
        self.declare_parameter('tracked_detections_topic', '/robot5/vision/tracked_detections_3d')
        self.declare_parameter('target_pose_topic', '/robot5/target_person_pose')
        self.declare_parameter('map_frame', 'map')

        # 웹캠/OAK-D 게이팅
        # 게이팅 시 동일 인물로 간주할 최대 이동 거리(m/s 기준 속도 게이트)
        self.declare_parameter('gating_max_speed', 2.0)
        # dt가 아주 작을 때 게이트가 0에 가까워지지 않도록 하는 최소 게이팅 반경(m)
        self.declare_parameter('gating_min_gate', 0.3)
        # 트랙 속도 추정의 지수평활 계수 (클수록 최신 관측을 더 신뢰)
        self.declare_parameter('velocity_alpha', 0.5)
        # 트랙 출처가 바뀌는 첫 프레임에만 적용하는 위치 블렌딩 계수 (1.0이면 블렌딩 없음)
        self.declare_parameter('source_switch_blend_alpha', 0.4)
        # 어느 출처로부터도 이만큼 갱신이 없으면 트랙을 삭제한다 (가려짐/프레임 이탈 허용 시간)
        self.declare_parameter('track_timeout', 3.0)
        # 위치 게이팅이 실패해도, 상류(oakd_detector_node의 YOLO track(persist=True) 등)가
        # 이전에 같은 id를 이 내부 트랙에 매핑해준 적이 있으면 그 힌트로 구제한다 - 실측으로
        # 상류 트래킹이 이 노드의 순수 위치 게이팅보다 occlusion에 훨씬 강함을 확인했다. 다만
        # 상류 id가 재사용돼 다른 사람에게 붙는 경우를 막기 위해 절대 거리 상한을 둔다.
        self.declare_parameter('id_rescue_max_distance', 1.0)
        self.declare_parameter('id_rescue_max_extrapolation', 1.0)

        # 외형(ReID) 기반 재식별. oakd_detector_node가 reid_model_path로 켜져 있을 때만
        # 임베딩이 들어오고, 없으면 아래 값들은 모두 무시돼 기존 위치 기반 동작이 된다.
        self.declare_parameter('reid_embeddings_topic', '/robot5/vision/detection_embeddings')
        # 게이트를 통과한 후보들 사이의 선택에만 더해지는 외형 비용 가중치(미터 단위 거리와
        # 더해지므로 "유사도 1 차이 = 몇 m 차이로 볼지"에 해당). 0이면 외형 미사용.
        self.declare_parameter('appearance_weight', 0.5)
        # 트랙 외형의 EMA 유지 계수 (1.0에 가까울수록 과거를 더 신뢰)
        self.declare_parameter('embedding_alpha', 0.9)
        # track_timeout으로 사라진 트랙의 신원을 이만큼 더 보관했다가 재등장 시 되살린다.
        # 실측(my_new_bag4/5)에서 사람이 화면 밖으로 나갔다 10초 이상 뒤 재등장하는 일이
        # 흔해, track_timeout(3초)만으로는 매번 새 신원이 발급됐다.
        self.declare_parameter('dormant_ttl', 30.0)
        # 갤러리에서 신원을 되살릴 코사인 유사도 하한. tools/reid_embedding_eval.py 실측
        # 최적 임계는 0.473이지만, 잘못된 병합(다른 사람을 같은 사람으로 이어붙임)이 잘못된
        # 분리보다 훨씬 위험하므로 보수적으로 높인다.
        self.declare_parameter('revival_similarity', 0.6)
        # 한 번 정해진 추종 대상이 사라져도 다른 사람으로 갈아타지 않는다(갤러리에서 돌아오길
        # 기다린다). 기다리는 동안 target_person_pose는 발행되지 않는다 - 엉뚱한 사람을
        # 따라가게 하는 것보다 안전하다. dormant_ttl이 상한 역할을 한다.
        # [웹캠 호출좌표 기반 추종 대상 지정]
        # 외부 웹캠이 "이 사람이 불렀다"고 알려준 map 좌표. webcam_person_bridge_node가
        # 자기 로봇 네임스페이스로 중계해 준 것을 구독한다(전역 /person/call_position을
        # 직접 구독하면 제스처 한 번에 모든 로봇이 동시에 재지정된다).
        # 상대 토픽(브리지의 call_output_topic과 같은 규칙) - 네임스페이스 아래로 해석된다.
        self.declare_parameter('call_position_topic', 'vision/webcam/call_position')
        # 호출 좌표에서 이 반경 안의 트랙 중 가장 가까운 사람을 추종 대상으로 삼는다.
        # 호모그래피 오차와, 제스처 시점부터 로봇이 도착할 때까지 호출자가 움직인 거리를
        # 함께 흡수해야 하므로 넉넉히 잡는다. 실측 인접 간격(0.63m)은 반경을 좁힐 근거가
        # 아니다 - 반경 안에서 **가장 가까운** 트랙을 고르는 것이 실제 판별 기준이다.
        self.declare_parameter('call_designation_radius', 2.0)
        # 이 시간이 지나도록 반경 안에 아무도 안 나타나면 호출을 포기하고 기존 정책으로
        # 돌아간다. 없으면 오래된 호출이 영원히 재선정을 막는다(dormant_ttl과 같은 역할).
        self.declare_parameter('call_ttl', 30.0)
        # 호출이 현재 추종 대상과 sticky 대기를 선점할지. 사람이 명시적으로 부른 것이므로
        # 기본은 선점이다 - 갤러리에서 기다리던 신원 때문에 "이리 와"를 무시하는 쪽이
        # 더 나쁜 오류다.
        self.declare_parameter('call_overrides_follow', True)
        self.declare_parameter('sticky_follow', True)
        # sticky_follow가 기다리는 동안, 살아있는 트랙이 **정확히 하나뿐이면** 그 사람을 대상으로
        # 삼는다. 임베딩이 꺼져 있으면(기본) 갤러리 복원은 위치 근접만 보는데, "사라진 자리
        # 근처에 사람이 있나"는 "눈앞에 사람이 있나"와 사실상 같은 정보라 구별력이 없다.
        # 구별할 수 없는 것을 구별하겠다고 기다리면, 눈앞의 사람을 두고 아무도 따라가지 않는
        # 상태가 길게 이어진다(실기 15:36 로그에서 실측). 후보가 둘 이상일 때는 여전히
        # 기다린다 - 그때가 sticky_follow가 실제로 값을 하는 유일한 상황이다.
        # 기본을 끈다. 실기(16:06 로그 + 영상)에서 이 기능이 추종 대상을 다른 사람에게
        # 넘기는 것이 실측됐다: 트랙 churn이 심한 구간에서 "살아있는 유일한 트랙"이 원래
        # 대상이 아닌 옆 사람일 수 있고, 두 사람이 0.63m 거리로 붙어 있으면
        # revival_max_distance 가드(1.5m)가 아무 구별도 못 한다. 트랙 churn의 근본 원인
        # (상류 트래커 id 미수신)을 먼저 잡은 뒤 다시 평가할 것.
        self.declare_parameter('sticky_follow_adopt_when_alone', False)
        # 외형 임베딩이 없을 때 갤러리에서 신원을 되살릴 최대 거리(m). "마지막으로 보이던
        # 자리 근처로 돌아왔는가"로 판단한다. 너무 키우면 다른 사람을 같은 사람으로 잇는다.
        self.declare_parameter('revival_max_distance', 1.5)
        # 트랙 위치/속도를 등속도 칼만필터로 추정한다(끄면 기존 지수평활).
        # [실측 결과 - 기본값을 끈 이유] my_new_bag5에서 어떤 설정도 기준선을 명확히 이기지
        # 못했다. 점프는 줄지만 **추종 좌표 발행 커버리지가 함께 떨어지는 맞교환**이었다:
        #   설정                        커버리지  최대점프  0.5m초과
        #   칼만 없음(기준선)              55%     0.80m     6건
        #   KF R=0.04                    55%     1.13m     6건
        #   KF q=0.2 / R하한 0.09         44%     0.67m     4건
        #   KF q=0.2 + R하한 0.09         23%     0.54m     1건
        #   위 + 마할라노비스 게이트        44%     0.79m     3건
        # 커버리지가 떨어지는 건 필터 예측이 검출과 어긋나 게이팅이 실패 -> 트랙이 새로 생기고
        # (동시 트랙 3->4) -> 추종 대상을 잃어 sticky가 기다리기 때문으로 보인다. 추종 로봇에는
        # "좌표가 자주 없는 것"이 "가끔 튀는 것"보다 나쁠 수 있어 기본값은 꺼둔다.
        # 켜서 쓰려면 q=0.2 / kalman_min_variance=0.09 조합이 실측상 균형이 가장 나았다.
        self.declare_parameter('use_kalman', False)
        self.declare_parameter('kalman_process_noise', 0.2)
        # 검출기가 covariance를 안 줄 때 쓰는 기본 관측 분산(m^2). 0.05m 오차 상당.
        self.declare_parameter('kalman_default_variance', 0.0025)
        # 관측 분산의 하한(m^2). 검출기가 신고하는 covariance는 접지점 산출 '등급'만 반영해
        # 실제 오차를 크게 과소평가한다 - 실측에서 가장 큰 점프들이 전부 최고 신뢰 등급
        # (toe_direct, sigma=0.05m)에서 났고, 실제 프레임간 오차는 중앙값 0.15m 수준이었다.
        # 이 값을 그대로 R로 쓰면 필터가 관측을 과신해(K≈1) 평활이 사실상 사라진다.
        # 실측 오차 수준으로 바닥을 깔아준다(0.04 = sigma 0.2m).
        self.declare_parameter('kalman_min_variance', 0.09)
        # 마할라노비스 게이트(2자유도). sqrt(5.991)=2.448이 95%, sqrt(9.21)=3.035가 99%.
        # 0 이하면 기존 유클리드 고정반경 게이트를 쓴다.
        self.declare_parameter('mahalanobis_gate', 3.035)
        # 위치 게이트를 통과했더라도 외형이 이보다 안 닮았으면 매칭을 거부하고 새 트랙으로
        # 돌린다. 0 이하면 거부를 끈다(기본값=끔).
        # [실측 결과 - 기본값을 끈 이유] my_new_bag4에서 0.3으로 켜 봤더니 오히려 나빠졌다:
        # 0.5m 초과 점프 12건 -> 16건, 동시 트랙 최대 6 -> 8개. 오프라인 측정(동일인 5%분위
        # 0.447)은 "인접 프레임" 쌍이라 유사도가 높게 나오지만, 실제로는 트랙의 EMA 임베딩과
        # 수 초 뒤 자세/조명이 달라진 검출을 비교하게 돼 같은 사람도 0.3을 밑돈다. 그래서
        # 정상 트랙이 쪼개지고 새 트랙이 늘어 대상 재선정 churn만 커졌다.
        # 크롭 품질이 좋은 환경(정면, 근거리, 머리 안 잘림)에서 다시 시도해볼 여지는 있다.
        self.declare_parameter('appearance_reject_similarity', 0.0)

        # 웹캠-라이다 신원 매칭
        # 예측 위치 대비 라이다 검출 후보를 인정할 최대 거리(m)
        self.declare_parameter('lidar_gating_position_threshold', 0.6)
        # 같은 라이다 트랙 ID가 이만큼 연속으로 최고 후보면 락온 확정
        # 라이다 락온 확정에 필요한 연속 후보 프레임 수.
        #
        # [배경 학습과의 경주에 대해] 10Hz 기준 7프레임 = 0.7초라, 의자/책상 다리를 정적
        # 배경으로 확정하는 데 걸리는 3초(leg_detector_bridge_node의
        # background_confirm_duration_sec)보다 짧다. 한때 전역 워밍업 타이머로 락온을
        # 미뤄봤지만 두 가지로 틀렸다: (1) 실제 플로우에서는 웹캠 호출좌표로 이동하는 동안
        # 배경 필터가 이미 학습을 끝내는데 그 시간이 반영되지 않아 정작 필요한 첫 락온만
        # 늦췄고, (2) 세션 중 새 장소로 이동해 처음 보는 정적 물체를 만나는 - 원래 막으려던 -
        # 상황에는 타이머가 이미 만료돼 아무 보호도 못 했다. 학습 진행도는 후보 위치마다
        # 다르므로 전역 타이머로 근사할 수 없다.
        self.declare_parameter('lidar_lock_confirm_frames', 7)
        # 락온 중 웹캠/OAK-D 위치와 이 이상 벌어지면 ID 스왑 의심
        self.declare_parameter('lidar_lock_swap_threshold', 0.3)
        # 스왑 의심이 이만큼 연속돼야 실제로 락온 해제 (순간적인 튐으로 오해제되지 않도록)
        self.declare_parameter('lidar_swap_confirm_frames', 5)
        # 락온된 라이다 id가 잠깐 안 보여도 유지하는 유예 시간(초)
        self.declare_parameter('leg_lock_grace_period', 1.0)
        # 락온 전 후보 게이팅에서 followed 트랙 예측에 쓰는 최대 외삽 시간(초). 웹캠/OAK-D 갱신
        # 없이 후보를 여러 프레임 모으는 동안 오래된 속도로 계속 미래를 예측하면 오차가 누적된다.
        self.declare_parameter('leg_match_max_extrapolation', 0.5)
        # 웹캠/OAK-D의 마지막 독립 갱신이 이보다 오래되면 백그라운드 검증을 보류(판단 근거 부족)
        self.declare_parameter('swap_check_max_age', 1.0)

        oakd_topic = self.get_parameter('oakd_detections_topic').value
        webcam_topic = self.get_parameter('webcam_detections_topic').value
        leg_topic = self.get_parameter('leg_detections_topic').value
        tracked_topic = self.get_parameter('tracked_detections_topic').value
        target_pose_topic = self.get_parameter('target_pose_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.gating_max_speed = self.get_parameter('gating_max_speed').value
        self.gating_min_gate = self.get_parameter('gating_min_gate').value
        self.velocity_alpha = self.get_parameter('velocity_alpha').value
        self.source_switch_blend_alpha = self.get_parameter('source_switch_blend_alpha').value
        self.track_timeout = self.get_parameter('track_timeout').value
        self.id_rescue_max_distance = self.get_parameter('id_rescue_max_distance').value
        self.id_rescue_max_extrapolation = self.get_parameter('id_rescue_max_extrapolation').value
        self.lidar_gating_position_threshold = self.get_parameter('lidar_gating_position_threshold').value
        self.lidar_lock_confirm_frames = self.get_parameter('lidar_lock_confirm_frames').value
        self.lidar_lock_swap_threshold = self.get_parameter('lidar_lock_swap_threshold').value
        self.lidar_swap_confirm_frames = self.get_parameter('lidar_swap_confirm_frames').value
        self.leg_lock_grace_period = self.get_parameter('leg_lock_grace_period').value
        self.leg_match_max_extrapolation = self.get_parameter('leg_match_max_extrapolation').value
        self.swap_check_max_age = self.get_parameter('swap_check_max_age').value
        self.appearance_weight = self.get_parameter('appearance_weight').value
        self.embedding_alpha = self.get_parameter('embedding_alpha').value
        self.dormant_ttl = self.get_parameter('dormant_ttl').value
        self.revival_similarity = self.get_parameter('revival_similarity').value
        self.sticky_follow = self.get_parameter('sticky_follow').value
        self.call_designation_radius = self.get_parameter('call_designation_radius').value
        self.call_ttl = self.get_parameter('call_ttl').value
        self.call_overrides_follow = self.get_parameter('call_overrides_follow').value
        self.pending_call = None      # (x, y, stamp) - 아직 사람을 못 찾은 호출
        self.sticky_follow_adopt_when_alone = self.get_parameter(
            'sticky_follow_adopt_when_alone').value
        self.log_track_lifecycle = self.get_parameter('log_track_lifecycle').value
        self.revival_max_distance = self.get_parameter('revival_max_distance').value
        self.use_kalman = self.get_parameter('use_kalman').value
        self.kalman_process_noise = self.get_parameter('kalman_process_noise').value
        self.kalman_default_variance = self.get_parameter('kalman_default_variance').value
        self.kalman_min_variance = self.get_parameter('kalman_min_variance').value
        gate = self.get_parameter('mahalanobis_gate').value
        self.mahalanobis_gate = gate if (gate > 0.0 and self.use_kalman) else None
        reject = self.get_parameter('appearance_reject_similarity').value
        self.appearance_reject_similarity = reject if reject > 0.0 else None

        # oakd_detector_node의 detections_pub이 best_effort라 QoS를 맞춰야 구독이 연결된다
        # (reliable 구독은 best_effort 발행자와 호환되지 않는다).
        self.oakd_sub = self.create_subscription(
            Detection3DArray, oakd_topic, self.oakd_detections_callback, qos_profile_sensor_data)
        # 외형 임베딩 동반 토픽. oakd_detector_node가 detections와 동일한 header.stamp로,
        # 행 i가 detections[i]에 대응하도록 32FC1 (N x D) Image로 발행한다.
        self.reid_sub = self.create_subscription(
            Image, self.get_parameter('reid_embeddings_topic').value,
            self.embeddings_callback, qos_profile_sensor_data)
        self.create_subscription(
            PointStamped, self.get_parameter('call_position_topic').value,
            self.call_position_callback, 10)
        self.webcam_sub = self.create_subscription(
            Detection3DArray, webcam_topic, self.webcam_detections_callback, 10)
        self.leg_sub = self.create_subscription(
            Detection3DArray, leg_topic, self.leg_detections_callback, 10)

        self.tracked_pub = self.create_publisher(Detection3DArray, tracked_topic, 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)

        # 트랙 상태 저장소: track_id(int) -> Track
        self.tracks = {}
        self._next_track_id = 1
        # 현재 추종 중인 트랙 ID. 유실되면 None으로 돌아가 재선정 대상이 된다
        self.followed_track_id = None
        # 웹캠/OAK-D가 "라이다를 거치지 않고" 마지막으로 갱신한 (x, y, stamp) - 라이다 락온 중
        # 백그라운드 검증의 비교 기준. 라이다로 덮어쓴 값끼리 비교하면 항상 일치해 의미가 없어 분리했다.
        self.last_independent_update = {}
        # (source, 상류 det.id 문자열) -> 이 노드의 내부 트랙 id. 위치 게이팅 실패 시 구제용
        # 힌트로 쓴다 (update_tracks 참고). det.id가 빈 문자열인 검출(상류 트래커가 아직
        # id를 못 붙인 경우)은 여기 안 들어간다.
        self.source_id_to_track = {}
        # stamp(초, 반올림 키) -> 그 순간 검출들의 임베딩 행렬(list of list). 검출 콜백이
        # 같은 stamp로 조회해 인덱스로 대응시킨다. 짝을 못 찾으면 위치 기반으로 degrade.
        self.pending_embeddings = {}
        # [dormant identity gallery] track_timeout으로 사라진 신원을 dormant_ttl 동안 보관한다.
        # track_id -> (embedding, x, y, stamp). 사람이 화면 밖으로 나갔다 한참 뒤 재등장할 때
        # 위치만으로는 이을 수 없어(예측 위치가 이미 무의미), 외형으로 되살리기 위한 저장소다.
        self.dormant_gallery = {}
        # 갤러리로 들어간 "원래 추종하던" 신원. 되살아나면 추종을 이 사람에게 되돌린다.
        self.dormant_followed_id = None
        # 외형 매칭이 실제로 동작 중인지 눈으로 확인하기 위한 카운터. 임베딩이 조용히
        # 폴백돼(짝을 못 찾아) 외형이 통째로 무시되는 상황을 놓치지 않으려고 둔다.
        self._stat_emb_received = 0
        self._stat_emb_matched = 0
        self._stat_emb_missed = 0
        self._stat_revivals = 0
        self.create_timer(5.0, self._log_reid_stats)

        # 웹캠-라이다 신원 매칭 상태
        self.locked_leg_id = None                # 락온된 라이다 쪽 원본 id 문자열 (예: 'leg_7')
        self.lock_candidate_streak = {}          # raw leg id -> 연속 최고-후보 프레임 수
        self.leg_last_seen = {}                  # raw leg id -> (x, y, stamp), 후보 속도 추정용
        self.swap_check_streak = 0               # 백그라운드 검증에서 연속 불일치 프레임 수

        self.get_logger().info(
            f'reid_tracking_node 시작 | oakd={oakd_topic} webcam={webcam_topic} leg={leg_topic} '
            f'-> {tracked_topic}, {target_pose_topic}'
        )

    # ------------------------------------------------------------------ 웹캠/OAK-D

    def embeddings_callback(self, msg: Image):
        """검출별 외형 임베딩(32FC1, N x D)을 stamp로 캐시해 둔다.

        검출 콜백보다 먼저 도착하는 것이 보통이지만(발행 순서상 바로 뒤이므로 사실상 동시),
        순서를 가정하지 않고 캐시에 넣고 검출 쪽에서 조회한다. 짝을 못 찾은 오래된 항목은
        무한정 쌓이지 않게 정리한다.
        """
        if msg.encoding != '32FC1' or msg.height == 0 or msg.width == 0:
            return
        flat = array.array('f')
        flat.frombytes(bytes(msg.data))
        if len(flat) != msg.height * msg.width:
            return
        matrix = [list(flat[r * msg.width:(r + 1) * msg.width]) for r in range(msg.height)]
        stamp = stamp_to_sec(msg.header.stamp)
        self.pending_embeddings[round(stamp, 6)] = matrix
        self._stat_emb_received += 1
        if len(self.pending_embeddings) > 30:
            for key in sorted(self.pending_embeddings)[:-15]:
                del self.pending_embeddings[key]

    def _take_embeddings(self, stamp, count):
        """이 stamp의 임베딩 행렬을 꺼낸다(소비). 없거나 행수가 안 맞으면 None들의 리스트."""
        matrix = self.pending_embeddings.pop(round(stamp, 6), None)
        if matrix is None or len(matrix) != count:
            if count:
                self._stat_emb_missed += 1
            return [None] * count
        self._stat_emb_matched += 1
        return matrix

    def _log_reid_stats(self):
        """외형 매칭이 실제로 붙고 있는지 주기적으로 보고한다(전부 0이면 뭔가 끊긴 것)."""
        if not (self._stat_emb_received or self._stat_emb_matched or self._stat_emb_missed):
            return
        self.get_logger().info(
            f'[reid] 임베딩 수신 {self._stat_emb_received} / 검출과 매칭 {self._stat_emb_matched} '
            f'/ 짝없음 {self._stat_emb_missed} | 갤러리 {len(self.dormant_gallery)}개 '
            f'| 신원복원 누적 {self._stat_revivals}회')

    def oakd_detections_callback(self, msg: Detection3DArray):
        self.update_tracks(msg, source='oakd')

    def webcam_detections_callback(self, msg: Detection3DArray):
        self.update_tracks(msg, source='webcam')

    def _new_track(self, track_id, x, y, stamp, source, variance):
        """트랙 생성. use_kalman이면 등속도 칼만필터를 붙여 준다.

        관측 노이즈 초기값은 이 검출이 신고한 분산을 쓴다 - 첫 관측부터 신뢰도를 반영해야
        초기 공분산이 현실적이고, 그래야 마할라노비스 게이팅이 처음부터 제대로 작동한다.
        """
        kalman = None
        if self.use_kalman:
            kalman = ConstantVelocityKalman2D(
                x, y, stamp,
                process_noise=self.kalman_process_noise,
                measurement_noise=variance if variance else self.kalman_default_variance)
        return Track(track_id, x, y, stamp, source, kalman=kalman)

    def update_tracks(self, msg: Detection3DArray, source: str):
        entries = []
        for det in msg.detections:
            pos = extract_person_position(det)
            if pos is None:
                continue
            x, y, _z = pos
            # 발행 쪽(oakd_detector_node/leg_detector_bridge_node) 관례상 det.header는 항상
            # 배열 header와 동일하게 채워져 있다.
            stamp = stamp_to_sec(det.header.stamp)
            # 검출기가 신고한 위치 분산. 없으면 기본값을 써서 칼만필터가 항상 R을 갖게 한다.
            var = extract_person_variance(det)
            if var is None:
                var = self.kalman_default_variance
            entries.append((x, y, stamp, det.id, max(var, self.kalman_min_variance)))

        # 한 메시지 안 서로 다른 검출이 같은 트랙을 중복으로 차지하지 않도록, 이번 콜백에
        # 들어온 검출 전체를 놓고 한 번에 배타적으로 배정한다. 게이팅은 배열 header의 공통
        # 시점(batch_stamp) 기준으로 모든 기존 트랙을 일관되게 예측해 비교한다.
        batch_stamp = stamp_to_sec(msg.header.stamp)
        positions = [(x, y) for x, y, _, _, _ in entries]
        variances = [v for _, _, _, _, v in entries]
        # 이 배열과 같은 stamp로 온 외형 임베딩(인덱스가 검출 순서와 일치). 웹캠 등 임베딩을
        # 주지 않는 출처면 전부 None이라 아래 로직이 전부 위치 기반으로 degrade한다.
        embeddings = self._take_embeddings(batch_stamp, len(entries))
        track_ids = assign_tracks(
            self.tracks, positions, batch_stamp, self.gating_max_speed, self.gating_min_gate,
            embeddings=embeddings, appearance_weight=self.appearance_weight,
            appearance_reject=self.appearance_reject_similarity,
            mahalanobis_gate=self.mahalanobis_gate, measurement_noises=variances)

        # [상류 트래커 id 구제] 상류 id(예: oakd_detector_node의 YOLO track(persist=True) id)가
        # 예전에 이 내부 트랙에 매핑된 적이 있고 그 트랙이 아직 살아있으면, 위치 게이팅 결과가
        # 이미 있어도(다른 트랙으로 매칭됐어도) 이 매핑을 우선한다. 실측(2026-08-08 세션)으로
        # 상류 트래킹이 occlusion에 이 노드의 순수 위치 게이팅보다 훨씬 강함을 확인했고
        # (22.74초짜리 bag에서 YOLO id는 2번만 바뀐 반면 위치 게이팅만 쓰는 라이다 쪽 내부
        # 트랙 id는 훨씬 자주 바뀌었다), 게이팅 "성공"만으로는 부족하다는 것도 my_new_bag3로
        # 추가 확인했다: 갓 생성된 트랙이 다음 프레임에 자기 게이트를 살짝 벗어나 중복 트랙이
        # 생기면, 이후 두 트랙이 같은 상류 id(oakd_4)의 검출을 번갈아 "위치상 더 가깝다"는
        # 이유로 차지하며 추종 대상 좌표가 순간이동(최대 6.4m/s)했다 - 상류 tracker는 그동안
        # 쭉 같은 id(oakd_4)로 봤으므로, 게이팅 성공 여부와 무관하게 이 매핑을 우선하면 그
        # 순간이동이 사라진다(실측: 최대 점프 1.28m/6.4m/s -> 0.63m/3.15m/s). 상류 id가
        # 재사용돼 다른 사람에게 잘못 붙는 경우를 막기 위해 절대 거리 상한은 그대로 둔다.
        claimed = {tid for tid in track_ids if tid is not None}
        for i, (x, y, stamp, det_id, _var) in enumerate(entries):
            if not det_id:
                continue
            hint_id = self.source_id_to_track.get((source, det_id))
            if hint_id is None or hint_id == track_ids[i] or hint_id in claimed \
                    or hint_id not in self.tracks:
                continue
            px, py = self.tracks[hint_id].predict(
                batch_stamp, max_dt=self.id_rescue_max_extrapolation)
            if distance(px, py, x, y) <= self.id_rescue_max_distance:
                if track_ids[i] is not None:
                    claimed.discard(track_ids[i])
                track_ids[i] = hint_id
                claimed.add(hint_id)

        for idx, ((x, y, stamp, det_id, var), track_id) in enumerate(zip(entries, track_ids)):
            embedding = embeddings[idx] if idx < len(embeddings) else None
            if track_id is None:
                # [dormant gallery] 새 신원을 발급하기 직전에, 최근에 사라진 신원 중 외형이
                # 충분히 닮은 것이 있으면 그 신원을 되살린다. 이게 없으면 사람이 화면 밖으로
                # 나갔다 돌아올 때마다 새 id가 붙는다(실측: 3명인데 YOLO id 11개).
                revived = self._revive_from_gallery(embedding, batch_stamp, claimed, x, y)
                if revived is not None:
                    track_id = revived
                    claimed.add(track_id)
                    # 공백 동안의 낡은 속도로 외삽하면 안 되므로 위치는 새 관측으로 리셋하고
                    # 속도는 0에서 다시 추정하게 한다.
                    track = self._new_track(track_id, x, y, stamp, source, var)
                    track.embedding = self.dormant_gallery[track_id][0]
                    self.tracks[track_id] = track
                    del self.dormant_gallery[track_id]
                    self._stat_revivals += 1
                    if self.dormant_followed_id == track_id:
                        self.followed_track_id = track_id
                        self.dormant_followed_id = None
                        self._reset_leg_lock()
                        self.get_logger().info(
                            f'외형 재식별로 추종 대상 복귀: track {track_id}')
                    else:
                        self.get_logger().info(f'외형 재식별로 신원 복원: track {track_id}')
                else:
                    track_id = self._next_track_id
                    self._next_track_id += 1
                    self.tracks[track_id] = self._new_track(
                        track_id, x, y, stamp, source, var)
                    if self.log_track_lifecycle:
                        self._log_new_track(track_id, x, y, stamp, source, det_id,
                                            batch_stamp, len(entries), track_ids)
            elif track_id == self.followed_track_id and self.locked_leg_id is not None:
                # 락온 중에는 문서 5단계("재매칭 없이 락온된 라이다 트랙을 그대로 추종")대로
                # 웹캠/OAK-D가 트랙의 위치·속도를 직접 덮어쓰지 않는다. 두 출처가 서로 다른
                # 위치를 보고하는 락온 구간에서 매 프레임 번갈아 스냅하면, 유한차분 속도 추정이
                # 실제로는 없는 왕복 운동으로 튀어(수 m/s대) 다음 프레임 게이팅까지 깨뜨린다.
                # 아래에서 last_independent_update만 기록해 백그라운드 검증(6단계)에 쓴다.
                pass
            else:
                track = self.tracks[track_id]
                # 출처가 막 바뀐 첫 프레임만 위치를 블렌딩해 좌표 불연속을 완화한다.
                position_alpha = (
                    self.source_switch_blend_alpha if track.last_source != source else 1.0)
                track.update(x, y, stamp, source, velocity_alpha=self.velocity_alpha,
                             position_alpha=position_alpha, max_speed=self.gating_max_speed,
                             measurement_noise=var)

            if track_id in self.tracks:
                self.tracks[track_id].update_embedding(embedding, alpha=self.embedding_alpha)
            if det_id:
                self.source_id_to_track[(source, det_id)] = track_id
            # 배경검증 기준은 이 노드가 블렌딩한 트랙 상태가 아니라 웹캠/OAK-D의 원본 관측값이어야
            # 한다 - 블렌딩된 값을 기준으로 삼으면 스왑 판정 자체가 무뎌진다.
            self.last_independent_update[track_id] = (x, y, stamp)

        self._prune_tracks(stamp_to_sec(msg.header.stamp))
        self._maybe_select_followed_track(stamp_to_sec(msg.header.stamp))
        self.publish_tracked(msg.header)
        self.publish_target_pose(msg.header)

    def _revive_from_gallery(self, embedding, now, claimed, x=None, y=None):
        """갤러리에서 외형이 가장 닮은 신원을 찾는다. 기준 미만이면 None(=새 신원 발급).

        위치는 보지 않는다 - 갤러리에 있다는 것 자체가 "한동안 안 보여서 예측 위치가 이미
        의미 없어진" 상태라, 위치로 거르면 정작 되살려야 할 재등장을 놓친다. 대신 잘못된
        병합을 막기 위해 임계(revival_similarity)를 실측 최적치보다 보수적으로 잡는다.
        """
        if not self.dormant_gallery:
            return None
        best_id, best_score = None, None
        for tid, (emb, gx, gy, _stamp) in self.dormant_gallery.items():
            if tid in claimed or tid in self.tracks:
                continue
            sim = cosine_similarity(embedding, emb)
            if sim is not None:
                # 외형을 쓸 수 있으면 그쪽이 우선 - 위치와 달리 사람이 이동해도 유효하다.
                if sim < self.revival_similarity:
                    continue
                score = 1.0 + sim          # 외형 매칭을 위치 매칭보다 항상 우선
            else:
                # 외형이 없으면 "마지막으로 보이던 자리 근처로 돌아왔는가"로 판단한다.
                # 사람이 완전히 다른 곳으로 이동해 돌아오면 못 살리지만, 그 경우 새 신원을
                # 발급하는 게 다른 사람을 같은 사람으로 잘못 잇는 것보다 안전하다.
                if x is None or y is None:
                    continue
                d = distance(gx, gy, x, y)
                if d > self.revival_max_distance:
                    continue
                score = 1.0 - d / max(self.revival_max_distance, 1e-6)
            if best_score is None or score > best_score:
                best_id, best_score = tid, score
        return best_id

    def _prune_tracks(self, now):
        """가려짐/프레임 이탈 정도는 견디되, track_timeout을 넘겨 갱신이 없으면 삭제한다.

        단, 외형 임베딩이 있는 트랙은 바로 버리지 않고 dormant_ttl 동안 갤러리에 옮겨
        놓는다(사람이 화면 밖으로 나갔다 돌아올 때 같은 신원으로 이어붙이기 위해).
        """
        stale = [tid for tid, tr in self.tracks.items() if now - tr.last_stamp > self.track_timeout]
        for tid in stale:
            track = self.tracks[tid]
            # 임베딩이 없어도(외형 ReID를 안 쓰는 기본 구성) 갤러리에 넣는다. 예전에는
            # 임베딩이 있는 트랙만 보관해서, 기본 구성에서는 갤러리가 늘 비고 sticky_follow가
            # 통째로 무력화됐다(대상을 잃으면 곧바로 다른 사람으로 갈아탐). 외형이 없으면
            # 아래 _revive_from_gallery가 마지막 위치 근접으로 되살린다.
            if self.log_track_lifecycle:
                self.get_logger().info(
                    f'[lifecycle] 소멸 track {tid} @({track.x:.2f},{track.y:.2f}) '
                    f'마지막갱신 {now - track.last_stamp:.2f}s 전 / 수명 '
                    f'{track.last_stamp - track.created_stamp:.2f}s / '
                    f'추종중={self.followed_track_id == tid} / 출처={track.last_source}')
            self.dormant_gallery[tid] = (track.embedding, track.x, track.y, track.last_stamp)
            del self.tracks[tid]
            self.last_independent_update.pop(tid, None)
            if self.followed_track_id == tid:
                # 추종 대상이 사라졌다. 갤러리에 신원이 남아 있으면 "원래 따라가던 사람"으로
                # 기억해 두고, 나중에 외형으로 되살아나면 추종을 그 사람에게 되돌린다.
                # 이게 없으면 그 사이 _maybe_select_followed_track이 고른 다른 사람을 계속
                # 따라가게 돼, 정작 원래 대상이 돌아와도 추종이 옮겨가지 않는다.
                self.followed_track_id = None
                # "원래 따라가던 사람"을 기억한다. 대상을 잃은 뒤 _maybe_select_followed_track이
                # 임시로 고른 다른 사람이 또 만료될 때 이 값을 덮어쓰면, 정작 원래 대상이
                # 돌아와도 복귀하지 못한다(실측으로 이 덮어쓰기 때문에 복귀가 한 번도 발생하지
                # 않았다). 그래서 이미 기다리는 신원이 있으면 유지한다.
                if tid in self.dormant_gallery and self.dormant_followed_id is None:
                    self.dormant_followed_id = tid
                self._reset_leg_lock()
        if stale:
            stale_set = set(stale)
            dead_keys = [k for k, v in self.source_id_to_track.items() if v in stale_set]
            for k in dead_keys:
                del self.source_id_to_track[k]

        # 갤러리도 dormant_ttl이 지나면 버린다. 무한정 두면 아주 오래된 신원이 엉뚱하게
        # 되살아날 수 있고 메모리도 계속 늘어난다.
        expired = [tid for tid, (_e, _x, _y, s) in self.dormant_gallery.items()
                   if now - s > self.dormant_ttl]
        for tid in expired:
            del self.dormant_gallery[tid]
            if self.dormant_followed_id == tid:
                # 되살아나길 기다리던 대상이 TTL을 넘겼다 - 더는 기다리지 않는다.
                self.dormant_followed_id = None

    def _log_new_track(self, track_id, x, y, stamp, source, det_id,
                       batch_stamp=None, batch_n=None, batch_ids=None):
        """새 트랙이 왜 생겼는지 판단할 재료를 남긴다.

        한 사람이 서 있는데 트랙이 여러 개 생기는 원인 후보가 셋이라(상류 트래커 id 변경 /
        위치 게이팅 실패 / 상류 id 재사용 오구제), 생성 시점의 "가장 가까운 기존 트랙까지
        거리"와 "이 상류 id를 전에 본 적 있는지"를 함께 찍어야 구분된다.
        """
        # 게이팅은 저장된 위치가 아니라 batch_stamp로 **예측한** 위치와 비교하므로 둘 다 찍는다.
        # 저장 위치는 붙어 있는데 예측 위치가 멀면 원인은 속도 추정 오버슛이다.
        nearest, nearest_d, nearest_pred_d, nearest_v = None, None, None, None
        for tr in self.tracks.values():
            if tr.id == track_id:
                continue
            d = distance(tr.x, tr.y, x, y)
            if nearest_d is None or d < nearest_d:
                px, py = (tr.predict(batch_stamp, max_dt=self.leg_match_max_extrapolation)
                          if batch_stamp is not None else (tr.x, tr.y))
                nearest, nearest_d = tr.id, d
                nearest_pred_d = distance(px, py, x, y)
                nearest_v = math.hypot(tr.vx, tr.vy)
        seen = [k for k in self.source_id_to_track if k[1] == det_id]
        if nearest is not None:
            near_txt = (f'최근접 track {nearest} 저장 {nearest_d:.2f}m / 예측 {nearest_pred_d:.2f}m '
                        f'(v={nearest_v:.2f}m/s)')
        else:
            near_txt = '기존 트랙 없음'
        # 같은 배치에 검출이 여러 개면, 배타적 배정 때문에 가까운 트랙이 이미 다른 검출에
        # 선점됐을 수 있다 - 한 사람이 두 번 검출되는(중복 bbox) 경우가 여기 해당한다.
        batch_txt = ''
        if batch_n is not None:
            taken = [t for t in (batch_ids or []) if t is not None]
            batch_txt = f' / 배치검출 {batch_n}개(기존트랙배정 {len(taken)}건)'
        gallery_d = None
        for gid, (_e, gx, gy, _s) in self.dormant_gallery.items():
            d = distance(gx, gy, x, y)
            if gallery_d is None or d < gallery_d[1]:
                gallery_d = (gid, d)
        gal_txt = f'갤러리 최근접 {gallery_d[0]} {gallery_d[1]:.2f}m' if gallery_d else '갤러리 빔'
        self.get_logger().info(
            f'[lifecycle] 생성 track {track_id} @({x:.2f},{y:.2f}) 출처={source} '
            f'상류id={det_id!r}(기존매핑 {len(seen)}건) / {near_txt} / {gal_txt}{batch_txt}')

    def call_position_callback(self, msg: PointStamped):
        """웹캠 호출 좌표를 받아 추종 대상 지정을 예약한다.

        브리지가 이미 프레임을 맞춰 보내지만 여기서 한 번 더 검증한다 - 이벤트 토픽이라
        로그가 넘칠 일이 없고, 프레임이 어긋난 지정 좌표는 **엉뚱한 사람**을 고르게 만드는
        조용한 실패라 값이 싸다.
        """
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f'호출 좌표 프레임이 {msg.header.frame_id or "(빈값)"} - '
                f'{self.map_frame}가 아니라 무시한다')
            return
        stamp = stamp_to_sec(msg.header.stamp)
        self.pending_call = (msg.point.x, msg.point.y, stamp)
        self.get_logger().info(
            f'호출 좌표 수신: ({self.pending_call[0]:.2f}, {self.pending_call[1]:.2f}) '
            f'- 반경 {self.call_designation_radius:.1f}m 안에서 추종 대상을 고른다')
        if self.call_overrides_follow:
            # 사람이 명시적으로 지정했으므로 갤러리에서 기다리던 신원은 포기한다.
            self.dormant_followed_id = None
            self.followed_track_id = None
        self._maybe_select_followed_track(stamp)
        self.publish_target_pose(msg.header)

    def _maybe_select_followed_track(self, now=None):
        """추종 대상 선정 정책. 실제 대상 지정(제스처/호출좌표 등)은 아직 이 패키지 범위 밖이라
        초기 획득은 "가장 먼저 잡힌(가장 오래된) 트랙"이라는 단순한 기본값을 쓴다.

        [sticky_follow] 한 번 정해진 대상은 사라져도 다른 사람으로 갈아타지 않는다.
        예전에는 추종 트랙이 track_timeout(3초)으로 지워지면 곧바로 그 시점에 살아있는 아무
        사람(가장 오래된 트랙)을 골랐다. 그래서 추종 대상이 실제로 다른 사람에게 건너뛰는 게
        실측됐고(my_new_bag5에서 1.11m/0.71m/1.02m 점프), 애써 만든 외형 갤러리 복원도
        기다리는 사이에 다른 사람이 대상이 돼버려 무용지물이었다.
        이제는 갤러리에서 돌아오길 기다리는 신원이 있으면 아무도 고르지 않는다. 그동안
        followed_track_id는 None이라 target_person_pose가 발행되지 않는데, 이는 이 노드가
        "대상 없음"을 침묵으로 표현하는 기존 관례와 같고, 엉뚱한 사람 좌표를 내보내 로봇이
        따라가게 하는 것보다 안전하다. 무한정 기다리지 않도록 dormant_ttl이 지나 갤러리에서
        만료되면(_prune_tracks) dormant_followed_id가 풀려 아래 재선정이 다시 동작한다.
        """
        # [우선순위] 호출 지정 > sticky 대기 > 가장 오래된 트랙.
        # 호출 지정을 맨 위에 두는 이유: 로봇이 호출 좌표로 이동하는 동안 기존 정책이
        # 지나가던 사람을 잡아두면, 도착해서 추종을 시작할 때 엉뚱한 사람을 따라간다.
        if self.pending_call is not None and now is not None:
            action, tid = resolve_call_designation(
                self.tracks, self.pending_call, now,
                self.call_designation_radius, self.call_ttl)
            if action == 'designate':
                self.followed_track_id = tid
                self.pending_call = None
                self.dormant_followed_id = None
                self._reset_leg_lock()
                self.get_logger().info(f'호출 좌표로 추종 대상 지정: track {tid}')
                return
            if action == 'wait':
                # 아직 호출자를 못 찾았다. 여기서 반환해야 아래 폴백이 다른 사람을
                # 붙잡지 않는다.
                return
            self.pending_call = None
            self.get_logger().warn(
                f'호출 좌표 만료({self.call_ttl:.0f}s) - 기존 선정 정책으로 돌아간다')

        if self.followed_track_id is not None and self.followed_track_id in self.tracks:
            return
        if self.sticky_follow and self.dormant_followed_id is not None:
            # 후보가 정확히 하나면 기다릴 이유가 없다(위 파라미터 주석 참고).
            if not (self.sticky_follow_adopt_when_alone and len(self.tracks) == 1):
                return
            adopted = next(iter(self.tracks.values()))
            # 기다리던 신원이 아직 갤러리에 있으면, 그 신원이 사라진 자리에서 너무 멀리
            # 떨어진 사람은 받아들이지 않는다 - 다인원 현장에서 원래 대상이 나가고 **다른**
            # 사람만 남은 경우를 그대로 넘겨받는 것이 sticky_follow가 막으려던 바로 그
            # 상황이다. 갤러리에서 이미 만료됐다면 비교할 근거가 없으므로 그냥 받아들인다.
            waiting = self.dormant_gallery.get(self.dormant_followed_id)
            if waiting is not None:
                d = distance(waiting[1], waiting[2], adopted.x, adopted.y)
                if d > self.revival_max_distance:
                    self.get_logger().info(
                        f'sticky 대기 유지: 유일 후보 track {adopted.id}가 기다리던 신원 '
                        f'{self.dormant_followed_id}의 마지막 위치에서 {d:.2f}m '
                        f'(> {self.revival_max_distance:.2f}m) 떨어져 있다',
                        throttle_duration_sec=2.0)
                    return
            self.get_logger().info(
                f'sticky 대기 해제: 후보가 track {adopted.id} 하나뿐이라 추종 대상으로 삼는다 '
                f'(기다리던 신원 {self.dormant_followed_id})')
            self.dormant_followed_id = None
        if not self.tracks:
            self.followed_track_id = None
            return
        self.followed_track_id = min(self.tracks.values(), key=lambda tr: tr.created_stamp).id
        self._reset_leg_lock()

    # ------------------------------------------------------------------ 라이다 다리검출 신원 매칭

    def leg_detections_callback(self, msg: Detection3DArray):
        stamp = stamp_to_sec(msg.header.stamp)
        positions = {}
        for det in msg.detections:
            pos = extract_person_position(det)
            if pos is not None:
                positions[det.id] = (pos[0], pos[1])

        if self.followed_track_id is None or self.followed_track_id not in self.tracks:
            # 추종 대상 자체가 없으면 라이다 신원매칭도 의미가 없다 - 후보 이력만 갱신해둔다.
            self._update_leg_last_seen(positions, stamp)
            return

        followed = self.tracks[self.followed_track_id]

        if self.locked_leg_id is not None:
            self._handle_locked(followed, positions, stamp)
        else:
            self._handle_unlocked(followed, positions, stamp)

        self._update_leg_last_seen(positions, stamp)

        # 카메라(웹캠/OAK-D)가 끊긴 동안에도 라이다 단독 락온 갱신이 외부로 계속 나가야 한다 -
        # 이 발행이 없으면 update_tracks(카메라 콜백)에서만 발행되므로, 카메라가 안 보이는
        # 초근접/가려짐 구간에서 tracked_detections/target_pose가 조용히 멈춘 것처럼 보인다.
        # _maybe_select_followed_track()/_prune_tracks()는 여기서 부르지 않는다 - 그건 "아직
        # 추종 대상이 없을 때 초기 선정"을 위한 카메라 경로 전용 로직이라, 이미 followed_track_id가
        # 있다는 전제로 진입하는 이 라이다 경로에서 다시 실행하면 락온 로직과 충돌할 수 있다.
        # locked_leg_id가 None이면(이번 프레임에 유실/스왑으로 막 리셋됐거나 애초에 미락온)
        # 실제로 갱신된 새 위치가 없으므로 발행하지 않는다.
        if self.locked_leg_id is not None:
            self.publish_tracked(msg.header)
            self.publish_target_pose(msg.header)

        # (전제조건) 이 매칭은 라이다/웹캠 검출이 모두 동일 map 좌표계로 변환되어 있다는 전제
        # 하에 동작한다. 매칭이 계속 실패하면 로봇 localization(AMCL 등) 드리프트를 먼저
        # 의심할 것 - 게이팅 임계값 튜닝으로 해결되는 문제가 아닐 수 있다.

    def _handle_locked(self, followed, positions, stamp):
        if self.locked_leg_id not in positions:
            # [5. 추종 유지] 이번 프레임에 락온 id가 안 보임 - 일시 유실, 유예 시간 안이면 유지
            last_seen = self.leg_last_seen.get(self.locked_leg_id)
            if last_seen is not None and stamp - last_seen[2] > self.leg_lock_grace_period:
                self.get_logger().warn(
                    f'라이다 락온 id {self.locked_leg_id} 유실 - 재매칭 진입')
                self._reset_leg_lock(stamp)
            return

        x, y = positions[self.locked_leg_id]

        # [6. 백그라운드 검증] 라이다로 덮어쓰기 전, 웹캠/OAK-D가 마지막으로 "독립적으로" 갱신한
        # 위치와 비교한다. 그 값이 너무 오래됐으면(웹캠도 안 보이는 상황) 판단을 보류한다.
        independent = self.last_independent_update.get(self.followed_track_id)
        if independent is not None and stamp - independent[2] <= self.swap_check_max_age:
            discrepancy = distance(independent[0], independent[1], x, y)
            if discrepancy > self.lidar_lock_swap_threshold:
                self.swap_check_streak += 1
            else:
                self.swap_check_streak = 0
            if self.swap_check_streak >= self.lidar_swap_confirm_frames:
                self.get_logger().warn(
                    f'라이다 락온({self.locked_leg_id}) ID 스왑 의심 - 재매칭 진입')
                self._reset_leg_lock(stamp)
                return

        # [5. 추종 유지] 재매칭 없이 락온된 id를 그대로 반영
        followed.update(x, y, stamp, source='lidar_leg', velocity_alpha=self.velocity_alpha,
                        max_speed=self.gating_max_speed)

    def _handle_unlocked(self, followed, positions, stamp):
        pred_x, pred_y = followed.predict(stamp, max_dt=self.leg_match_max_extrapolation)  # [1. 시간 정렬]

        best_rid, best_score = None, None
        for rid, (x, y) in positions.items():
            d = distance(pred_x, pred_y, x, y)
            if d > self.lidar_gating_position_threshold:  # [2. 게이팅 매칭]
                continue
            # [3. 모호성 해소] 라이다는 속도를 안 주므로 같은 raw id의 직전 위치로 직접 미분
            prev = self.leg_last_seen.get(rid)
            if prev is not None and stamp > prev[2]:
                vx = (x - prev[0]) / (stamp - prev[2])
                vy = (y - prev[1]) / (stamp - prev[2])
            else:
                vx = vy = 0.0
            sim = velocity_similarity(followed.vx, followed.vy, vx, vy)
            score = sim - d  # 가까울수록 + 속도가 비슷할수록 높은 점수
            if best_score is None or score > best_score:
                best_rid, best_score = rid, score

        # 이번 스캔에 게이팅을 통과하는 후보가 아예 없으면(best_rid is None) 기존에 쌓아온
        # 스트릭은 그대로 둔다 - 실측(my_new_bag2)으로 원시 다리검출기가 스캔마다 검출/미검출을
        # 오가는(깜빡이는) 게 흔하다는 게 확인됐는데, "검출이 한 스캔 빈 것"과 "다른 후보가
        # 새로 최고점이 된 것"은 서로 다른 사건이다. 전자로 매번 스트릭을 리셋하면 락온 확정에
        # 필요한 연속 lidar_lock_confirm_frames를 채우기가 실질적으로 불가능해진다(시뮬레이션
        # 검증: 재획득까지 걸리는 시간이 5.66s->1.17s로 단축, 락온 성공 횟수도 늘어남). 이미
        # 리셋 로직을 후자(다른 후보로 교체됐을 때)로만 좁혀도 안전한 이유는, 어차피 매칭
        # 자체가 매 스캔 위치+속도 유사도로 다시 계산되므로 진짜로 다른 사람/물체가 끼어들면
        # best_rid가 바뀌면서 정상적으로 리셋되기 때문이다.
        if best_rid is None:
            return

        for rid in list(self.lock_candidate_streak.keys()):
            if rid != best_rid:
                self.lock_candidate_streak[rid] = 0

        self.lock_candidate_streak[best_rid] = self.lock_candidate_streak.get(best_rid, 0) + 1
        if self.lock_candidate_streak[best_rid] < self.lidar_lock_confirm_frames:
            return

        # [4. 락온 확정]
        self.locked_leg_id = best_rid
        self.swap_check_streak = 0
        x, y = positions[best_rid]
        followed.update(x, y, stamp, source='lidar_leg', velocity_alpha=self.velocity_alpha,
                        max_speed=self.gating_max_speed)
        self.get_logger().info(f'라이다 락온 확정: {best_rid} -> track {self.followed_track_id}')

    def _reset_leg_lock(self, stamp=None):
        """라이다 락온 상태를 푼다. stamp를 주면 추종 트랙을 카메라 관측 위치로 되돌린다.

        stamp는 "지금 살아있는 락온을 놓는 중"인 호출에서만 넘긴다(_handle_locked의 유실/스왑
        경로). 초기 대상 선정이나 트랙 삭제처럼 애초에 락온이 없던 경로는 되돌릴 것도 없다.
        """
        if stamp is not None and self.locked_leg_id is not None:
            self._restore_camera_position(stamp)
        self.locked_leg_id = None
        self.lock_candidate_streak = {}
        self.swap_check_streak = 0

    def _restore_camera_position(self, stamp):
        """락온을 놓는 순간, 추종 트랙을 카메라가 마지막으로 독립 관측한 위치로 되돌린다.

        락온 중에는 트랙 위치를 라이다 다리검출만 갱신한다(카메라 덮어쓰기 금지 - 위 분기 참고).
        그 좌표는 카메라가 보는 위치에서 수십 cm씩 벌어지는데, 락온이 끊긴 뒤 트랙을 그 자리에
        그대로 두면 카메라 검출이 위치 게이팅을 통과하지 못한다. 그러면 카메라가 사람을 계속
        보고 있는데도 그 검출은 **다른 트랙**으로 가고, 추종 트랙만 아무에게도 갱신되지 않아
        track_timeout 뒤에 굶어 죽는다.

        실기(evidence/live_run_0809_1648.log)에서 이 경로로 추종 대상을 두 번 잃었다:
          18.6s 락온 -> 24.5s 유실 -> 26.5s 소멸 track 1 수명 6.95s 추종중=True 출처=lidar_leg
          32.3s 락온 -> 34.0s 유실 -> 36.0s 소멸 track 1 수명 1.63s 추종중=True 출처=lidar_leg
        죽은 트랙의 마지막 출처가 둘 다 lidar_leg다.

        속도도 라이다 좌표 점프로 오염돼 있어(예측 위치가 날아가 게이팅을 또 깨뜨린다) 함께
        0으로 되돌린다. 카메라 관측이 swap_check_max_age보다 오래됐으면 되돌릴 근거가 없으므로
        그냥 둔다 - 그 경우는 카메라도 사람을 못 보고 있다는 뜻이다.
        """
        if self.followed_track_id is None:
            return
        track = self.tracks.get(self.followed_track_id)
        independent = self.last_independent_update.get(self.followed_track_id)
        if track is None or independent is None:
            return
        x, y, obs_stamp = independent
        if stamp - obs_stamp > self.swap_check_max_age:
            return
        moved = distance(track.x, track.y, x, y)
        track.x, track.y = x, y
        track.vx = track.vy = 0.0
        if track.kalman is not None:
            track.kalman = ConstantVelocityKalman2D(
                x, y, stamp,
                process_noise=self.kalman_process_noise,
                measurement_noise=self.kalman_default_variance)
        self.get_logger().info(
            f'락온 해제 - 추종 track {self.followed_track_id} 위치를 카메라 관측으로 되돌림 '
            f'({moved:.2f}m 이동, 관측 {stamp - obs_stamp:.2f}s 전)')

    def _update_leg_last_seen(self, positions, stamp):
        """이번 프레임에 보인 raw leg id만 갱신한다(통째로 덮어쓰지 않음) - 그래야 이번 프레임에
        안 보인 락온 id의 "마지막으로 본 시각"이 살아남아 유예시간(grace period) 판정이 된다.
        오래 안 보인 id는 leg_lock_grace_period보다 넉넉히 지나면 정리해 메모리가 무한정
        늘어나지 않게 한다."""
        for rid, (x, y) in positions.items():
            self.leg_last_seen[rid] = (x, y, stamp)
        prune_before = stamp - max(self.leg_lock_grace_period * 3, 5.0)
        stale = [rid for rid, (_, _, t) in self.leg_last_seen.items() if t < prune_before]
        for rid in stale:
            del self.leg_last_seen[rid]

    # ------------------------------------------------------------------ 출력

    def publish_tracked(self, header):
        msg = Detection3DArray()
        # header를 통째로 대입(참조 공유)하면 안 된다 - leg_detector_bridge_node에서 같은
        # 패턴이 원본 header 객체를 오염시키는 버그였던 것과 동일한 위험을 안고 있다. 지금은
        # 아래에서 바로 map_frame으로 다시 세팅해서 우연히 값이 맞아떨어지지만, 일관되게
        # 값만 복사한다.
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.map_frame
        msg.detections = [self._make_detection3d(tr, header) for tr in self.tracks.values()]
        self.tracked_pub.publish(msg)

    def _make_detection3d(self, track, header):
        det = Detection3D()
        # 배열 헤더(=이번 카메라 프레임 시각)가 아니라 **그 트랙이 마지막으로 실제 관측된
        # 시각**을 찍는다. 이번 프레임에 갱신되지 않은 트랙도 track_timeout 동안 계속
        # 발행되는데, 최신 시각을 달아 보내면 하류(predictive_avoidance_node)가 이를 새
        # 관측으로 오인해 "제자리에 멈춘 관측"을 반복 입력받고 속도 추정이 0으로 끌려간다.
        # 실측(my_new_bag5): 발행 엔트리의 77.5%가 좌표 미갱신, 최장 40프레임 연속.
        det.header.stamp = sec_to_stamp(track.last_stamp)
        det.header.frame_id = self.map_frame

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis = ObjectHypothesis(class_id='person', score=1.0)
        hyp.pose.pose.position.x = track.x
        hyp.pose.pose.position.y = track.y
        hyp.pose.pose.orientation.w = 1.0
        det.results.append(hyp)

        det.bbox.center.position.x = track.x
        det.bbox.center.position.y = track.y
        det.bbox.center.orientation.w = 1.0

        # 표준 id 필드에 이 노드가 부여한 지속 트랙 ID를 기록 - 예측 회피 노드가 이 값으로
        # 시계열을 이어붙인다.
        det.id = str(track.id)
        return det

    def publish_target_pose(self, header):
        """추종 대상이 있을 때만 발행한다.

        예전엔 대상이 없어도 위치(0,0,0)+방향(0,0,0,0, 유효하지 않은 쿼터니언)인 기본값
        Pose를 그대로 내보냈다 - 구독자가 이를 "map 원점에 있는 대상"으로 오해할 수 있는
        실제 버그였다. 이 패키지 범위 밖의 Nav2 목표 발행 노드가 아직 없어 지금은 아무도
        구독하지 않지만, "대상 없음"을 침묵으로 표현하는 게(이 노드의 다른 발행 경로들도
        전부 이 관례를 따른다 - 예: 라이다 단독 갱신은 락온 중일 때만 발행) 잘못된 좌표를
        내보내는 것보다 안전하다.
        """
        if self.followed_track_id is None or self.followed_track_id not in self.tracks:
            return
        track = self.tracks[self.followed_track_id]
        pose = PoseStamped()
        pose.header.stamp = header.stamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = track.x
        pose.pose.position.y = track.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
