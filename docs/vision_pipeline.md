# 비전 파이프라인 구조

작업 현장에서 사람의 위치·제스처·안전 상태를 인식해 다수의 AMR(TurtleBot4)을 안전하게
운용하기 위한 비전 파이프라인이다. 고정된 웹캠으로 넓은 작업장을 감시하고, 각 AMR에 탑재된
OAK-D PRO 카메라로 로봇 주변 근접 영역을 정밀하게 인식하는 두 축으로 구성된다.

> 이 저장소(`rokey_ws`)는 이 중 **AMR(TurtleBot4 + OAK-D PRO) 기반 사람 추종** 부분을
> `src/amr_person_tracking` 패키지로 구현한다. 고정캠 검출/로컬라이제이션 등 나머지 노드는
> 별도 저장소에서 구현되며, 여기서는 표준 메시지 스키마로 인터페이스만 맞춘다.

## 1. 노드 구성

### 1) 고정캠 사람 검출 및 상체 신호 인식
프레임 내 모든 사람의 bbox + keypoint + 상체 신호를 한 번에 처리한다. YOLO-pose로 COCO 17
keypoints를 multi-person single-pass로 추출한다. wrist keypoint는 손 세부 신호 노드로,
발끝 keypoint는 위치추정 노드로, bbox는 재식별/트래킹 노드로 전달된다.

### 2) 손 세부 신호 인식
손가락 단위 신호만 추가로 구분한다. MediaPipe Hands를 사용하며, 1)번이 준 wrist keypoint를
기준으로 ROI를 크롭해 입력한다. 결과는 1)번의 제스처 분류 결과에 세부 유형 필드로 병합된다.

### 3) 위치추정 - 웹캠
사람이 로봇/작업 반경에서 먼 경우 대략적인 world 좌표를 확보한다. 발끝 keypoint(1번 출력)와
사전 캘리브레이션된 호모그래피로 world XY를 계산하고, 작업자가 반경 N미터 안으로 들어오면
Nav2 goal을 트리거한다.

### 4) 위치추정 - OAK-D PRO (AMR 탑재, `amr_person_tracking` 구현)
로봇-작업자 거리가 대략 1.5~2m 이하로 좁혀지면 정밀 위치로 전환한다. `oakd_detector_node`가
OAK-D의 RGB/Depth 스트림에서 직접 YOLO-pose로 사람의 발끝 keypoint를 추출하고, depth 값을
camera_info 내부 파라미터로 역투영해 camera_link 기준 3D 좌표를 얻는다. 발끝이 가려져 직접
검출되지 않는 경우를 대비해 좌표 산출 방식에 등급(각도만 추정 / 무릎 keypoint로 보정 / 발끝
직접 검출)을 매겨 신뢰도 플래그로 함께 싣는다. depth가 MinZ 이하로 가까워지면 추적 방향에
해당하는 IR 근접센서 값을 같이 확인해 근접 안전모드로 전환한다.

**접지점 depth를 어디서 재는가** — 좌우 발목이 모두 보일 때 두 픽셀을 평균한 "중점"에서 depth를
읽으면 안 된다. 걸을 때 다리가 벌어지면 그 중점은 두 다리 사이 허공이라 뒤쪽 벽/바닥 거리가
찍힌다(실측: 두 발목이 모두 검출된 프레임의 22.1%가 몸통 깊이와 0.4m 이상 어긋남. 발목 간격
중앙값 77px, 90%분위 219px). 그래서 `foot_pixel_candidates()`가 각 발목을 **개별 후보**로
내보내고, 노드는 **방향(u,v)은 후보들의 중점, 거리(z)는 각 발목에서 실제로 잰 depth의 평균**을
쓴다. 중점 픽셀은 사람의 좌우 중심이라 안정적이고 depth는 실제 사람 표면에서 온다. 발목 하나만
고르는 방식도 검토했지만 걸을 때 앞발/뒷발이 프레임마다 번갈아 뽑혀 보폭만큼 진동했다.
그래도 몸통 깊이와 크게 어긋나면(`depth_consistency_tolerance`, 기본 0.4m) bbox 안쪽에서 구한
사람 표면 깊이로 대체하는 최후 안전망을 두는데, 위 방식 도입 후 이 안전망은 실측상 발동하지
않는다(22.2% → 0.0%).

**검출 신뢰도 임계는 두 단계로 분리**돼 있다. YOLO 트래커에는 `tracker_conf_threshold`(기본 0.1)로
저신뢰 검출까지 넣어 TrackTrack/ByteTrack 계열의 2단계 연결(저신뢰 검출을 기존 트랙에 이어붙여
끊김을 막는 장치)을 살리고, 실제로 발행하는 검출은 `conf_threshold`(기본 0.3)로 따로 거른다.
하류가 받는 검출 품질은 그대로면서 트랙 연속성만 얻는 구성이다(실측: 3인 교차 bag에서 YOLO
고유 track_id 11개 → 6개, 실제 인원 3명에 근접).

웹캠(3번)의 world XY에서 이 노드의 depth 3D XY로 전환되는 시점에는 좌표가 불연속으로 튈 수
있는데, 이는 재식별/트래킹 노드(7번)가 출처 전환 블렌딩으로 완화한다.

**처리 성능(실측, RTX 4070 노트북 + bag 재생 기준)** — 프레임당 약 27ms(외형 ReID 미사용 시,
기본 구성)로 이론상 37Hz까지 가능하다. 실제 달성 레이트는 **4.84Hz**인데 이는 연산 한계가 아니라
**depth 스트림이 5Hz**이기 때문이다(RGB는 이미 10Hz). 10Hz로 올리려면 로봇 쪽 depth 발행률을
올려야 하며 이 패키지 코드 변경은 필요 없다. 외형 ReID를 켜면 프레임당 54ms로 늘어난다(ReID
onnx가 CPU로 동작 — onnxruntime-gpu가 CUDA 12를 요구하는데 이 환경은 torch용 CUDA 13만 있어
GPU 초기화에 실패한다).

터틀봇4는 카메라 장착 높이가 낮아서, 사람에게 더 가까이 접근하는 초근접 구간에서는 OAK-D
프레임에 다리만 잡혀 depth 기반 발끝 검출 자체가 불안정해진다. 이 구간은 오히려 2D 라이다가
유리하다 — RPLIDAR 스캔 평면이 마침 다리 높이와 맞아떨어지고, 라이다는 range+bearing을 직접
측정하므로 호모그래피나 depth 역투영 같은 좌표 변환이 필요 없다. `leg_detector_bridge_node`가
`sensor_msgs/msg/LaserScan`을 직접 구독해 자체 구현한 경량 검출기로 다리쌍을 찾아 같은
`Detection3DArray` 스키마로 편입시켜, 원거리(웹캠) → 근접(OAK-D depth) → 초근접(라이다
다리검출) → 최근접(IR 안전모드)로 이어지는 전환 체인을 완성한다. 검출은 jump-distance
클러스터링으로 스캔 포인트를 묶은 뒤, 폭+원형적합(Kåsa 곡률 적합)으로 벽/모서리처럼 곡률이
없는 클러스터를 배제하고, 남은 다리 후보를 그리디로 페어링하는 순서로 동작한다. 곡률만으로는
책상·의자 다리처럼 실제로 원통형인 정적 물체를 구별할 수 없다는 한계가 있어, 개별 다리
후보마다 등속도 칼만필터로 속도를 추정해 "처음 관측된 뒤 누적 나이 3초 이상이고 추정 속도가
0.01m/s 이하"인 물체를 정적 배경으로 확정·제외하는 온라인 필터를 추가로 둔다(실측 기준 평균
검출 오탐 77%, occlusion이 반복되는 최악 구간은 93% 감소).

### 5) 라이다 사각지대 보완
2D 라이다 평면보다 낮거나 높은 장애물을 보완한다. OAK-D-PRO의 depth를 PointCloud2로 발행해
Nav2 voxel_layer의 observation_source로 라이다 obstacle_layer와 병렬 등록한다
(`src/oakd_pointcloud` 패키지, `depth_image_proc` 컴포저블 노드를 launch로 구성).

### 6) 예측적 회피 (AMR 탑재, `amr_person_tracking` 구현)
빠르게 접근하는 대상에 로봇이 미리 반응하도록 한다. `predictive_avoidance_node`가 트래킹된
대상(7번 출력)의 위치 시계열에 등속도 모델 칼만필터를 적용해 속도를 추정한다. 이때 프레임 간
Δt는 고정값이 아니라 항상 이미지 메시지의 `header.stamp`(촬영 시각, 수신 시각이 아님) 기준으로
계산한다 — 로봇↔노트북 사이 네트워크 지연으로 프레임 간격이 불규칙하기 때문이다. 추정된 속도로
예측 위치 주변에 속도에 비례한 범위의 가상 포인트를 만들어 기존 장애물 마킹 스트림에 추가
발행하면 voxel_layer가 이를 실제 장애물처럼 마킹하고, 그 위에 전역 inflation이 한 번 더
적용되어 결과적으로 그 물체 주변만 더 넓게 부풀려진다. 접근 속도에 비례해 local_costmap의
inflation 파라미터를 직접 조정하는 방식도 대안으로 지원한다.

### 7) 재식별/트래킹 (AMR 탑재, `amr_person_tracking` 구현)
가려짐·프레임 이탈 후에도 같은 사람을 같은 트랙으로 유지한다. `reid_tracking_node`가 웹캠
로컬라이제이션 스트림과 `oakd_detector_node`의 근접 검출 스트림을 함께 받아 통합 트랙을 관리한다.
동일 인물의 출처가 웹캠→OAK-D로 전환되는 첫 프레임만 위치를 블렌딩해 불연속을 완화한다.

검출 하나가 들어오면 **4단계를 순서대로** 거쳐 내부 트랙 ID를 정한다:

1. **위치+속도 게이팅 + Hungarian 전역 최적 배정** — 게이트 반경은 `max(gating_min_gate,
   gating_max_speed × dt)`. 예전에는 최근접부터 그리디로 확정했으나, 지역적으로 최선인 선택이
   전체로는 더 나쁜 조합을 만들 수 있어 `scipy.optimize.linear_sum_assignment`로 교체했다.
2. **상류 트래커 id 구제** — 상류(YOLO `track(persist=True)`) id가 이전에 어떤 내부 트랙에
   매핑됐고 그 트랙이 아직 살아있으면, 위치 게이팅 결과와 **무관하게** 그 매핑을 우선한다.
   갓 생성된 트랙이 다음 프레임에 자기 게이트를 살짝 벗어나 중복 트랙이 생기면 두 트랙이 같은
   상류 id의 검출을 번갈아 차지하며 추종 좌표가 순간이동했기 때문이다(실측 6.4m/s → 3.1m/s).
   상류 id 재사용으로 다른 사람에게 붙는 것을 막기 위해 절대 거리 상한을 둔다.
3. **dormant identity gallery 부활** — 새 ID를 발급하기 직전, 최근 사라진 신원 중 마지막으로
   보이던 자리에서 `revival_max_distance`(1.5m) 이내로 돌아온 것이 있으면 그 신원을 되살린다
   (위치는 새 관측으로 리셋하고 속도는 0에서 재추정 — 공백 동안의 낡은 속도로 외삽하면 안 된다).
4. 전부 실패하면 새 트랙 ID 발급.

**추종 대상 유지(sticky_follow)** — 한 번 정해진 대상은 사라져도 다른 사람으로 갈아타지 않는다.
예전에는 추종 트랙이 `track_timeout`(3초)으로 지워지면 곧바로 그 시점에 살아있는 아무 사람을
골라, 추종 대상이 실제로 다른 사람에게 건너뛰었다. 이제는 사라진 트랙을 `dormant_ttl`(30초)
동안 갤러리에 마지막 위치와 함께 보관하고, **기다리는 신원이 있는 동안에는 새 대상을 고르지
않는다.** 그 사이 `target_person_pose`는 발행되지 않는데, 이는 이 노드가 "대상 없음"을 침묵으로
표현하는 관례와 같고 엉뚱한 사람 좌표를 내보내는 것보다 안전하다. 그 사람이 마지막으로 보이던
자리에서 `revival_max_distance`(1.5m) 이내로 다시 나타나면 원래 신원으로 되살려 추종을 복구한다.
`dormant_ttl`이 상한이라 영영 아무도 안 따라가는 상태는 생기지 않는다.

**아직 임시인 부분** — 최초 대상 획득은 "가장 먼저 잡힌(가장 오래된) 트랙"이라는 자리표시자
정책이다. 제스처나 웹캠 호출좌표 기반의 실제 대상 지정이 붙으면 이 자리를 대체하면 되고,
sticky 로직은 그대로 재사용된다.

**검증했지만 기본으로 끈 기능** — 외형(ReID) 임베딩 매칭과 칼만필터/마할라노비스 게이팅은
구현·단위테스트까지 돼 있으나 이 현장 데이터에서 이득이 없어 파라미터로 꺼뒀다. 각각
`evidence/evidence_reid_embedding_log.txt`, `evidence/evidence_kalman_mahalanobis_log.txt`에
측정 근거와 켜는 법이 있다.

`leg_detector_bridge_node`가 편입시킨 라이다 다리검출 스트림은 위치/속도만 알 뿐 신원을
모르므로, 웹캠이 추종하던 타겟과 같은 사람인지는 이 노드가 별도로 판별한다. 절차는 다음과
같다: 추종 중인 타겟의 최근 위치+속도(칼만필터 추정치)를 라이다 메시지의 촬영 시각까지
외삽해 시간을 맞춘 뒤(시간 정렬), 예측 위치와 가까운 라이다 검출만 후보로 거르고(위치
게이팅), 후보가 여럿이면 속도 벡터의 방향·크기 유사도까지 함께 봐서 하나로 좁힌다. 같은
라이다 트랙 ID가 연속된 여러 프레임 동안 계속 후보로 뽑히면 그때 비로소 해당 트랙에
"락온"하고, 락온 이후에는 매 프레임 재매칭하지 않고 그 라이다 트랙 ID를 그대로 따라간다.
다만 사람들이 스쳐 지나가며 라이다 트랙 ID가 순간적으로 바뀌는(swap) 경우에 대비해, 락온
중에도 웹캠 쪽 추정 위치와 계속 대조하다가 차이가 벌어지면 락온을 풀고 재매칭을 다시
트리거한다. 이 매칭 전체는 라이다·웹캠 검출이 같은 map 좌표계로 정확히 변환되어 있다는
전제 위에서 동작하므로, 매칭이 계속 실패한다면 게이팅 임계값보다 로봇 localization(AMCL 등)
드리프트를 먼저 의심해야 한다.

트래킹된 대상마다 표준 id 필드에 지속 트랙 ID를 기록해 재발행하며, 이 중 추종 대상으로 선정된
트랙 하나는 z를 0으로 눌러 2D map 좌표(`geometry_msgs/PoseStamped`)로 별도 발행한다. 이
좌표를 실제 Nav2 goal로 소비하는 노드는 이 패키지의 범위가 아니다.

### 8) 위험/낙상 감지 (차순위, 메인 기능 아님)
쓰러짐, 낙하물 등 안전 이벤트를 감지한다. 1)번의 pose 파이프라인을 재사용하며, 자세가 급격히
수직→수평으로 바뀌는 패턴 등으로 판단한다. 감지되면 인근 로봇 + HMI + DB로 전달된다.

## 2. 인터페이스 표

| 노드 | 방향 | 인터페이스 (토픽/서비스 · 메시지 타입) | 상대 노드 / 시스템 | 비고 |
|---|---|---|---|---|
| 고정캠 검출 노드 | 수신 | `sensor_msgs/msg/Image`, `sensor_msgs/msg/CameraInfo` | 웹캠 드라이버 | YOLO-pose로 사람 검출+keypoint 추출 (제스처·트래킹·위험감지 통합) |
| 고정캠 검출 노드 | 송신 | `vision_msgs/msg/Detection2DArray` (class_id에 person/gesture_/hazard_ 라벨) | → 로컬라이제이션 노드 | |
| 고정캠 검출 노드 | 송신 | `geometry_msgs/msg/PoseArray` (keypoint 픽셀좌표, 임시) | → 로컬라이제이션 노드 | 표준 keypoint 메시지가 없어 절충한 유일한 부분 |
| 로컬라이제이션 노드 | 수신 | 위 `Detection2DArray` + `PoseArray` | 고정캠 검출 노드 | |
| 로컬라이제이션 노드 | 수신 | tf2 정적변환(map→webcam_frame) | 0단계 캘리브레이션 | 호모그래피 파라미터 포함 |
| 로컬라이제이션 노드 | 송신 | `vision_msgs/msg/Detection3DArray` (frame_id=map) | → 재식별/트래킹 노드 | pixel → world 변환 완료 시점 |
| 위험·장애물 브로드캐스트 노드 | 수신 | `Detection3DArray` 중 hazard/obstacle 클래스 필터 | 로컬라이제이션 노드 | |
| 위험·장애물 브로드캐스트 노드 | 송신 | `sensor_msgs/msg/PointCloud2`(`/vision/person_points`, `/vision/obstacle_points`) | → 각 로봇 `nav2_costmap_2d`(voxel_layer/obstacle_layer, observation_sources) | 라이다 사각지대 보완 겸용, 클래스별 안전마진 차등 적용 |
| `depthai_ros_driver`(기성 패키지) | 송신 | RGB/Depth/CameraInfo (compressed) | → `oakd_detector_node` | OAK-D-PRO 하드웨어 스트림, 대역폭 절약을 위해 compressed로 발행 |
| `oakd_detector_node` (`amr_person_tracking`) | 수신 | `sensor_msgs/msg/CompressedImage`(rgb, depth) + `sensor_msgs/msg/CameraInfo` | `depthai_ros_driver` | YOLO-pose 추론부터 3D 역투영까지 이 노드에서 직접 수행 |
| `oakd_detector_node` (`amr_person_tracking`) | 송신 | `vision_msgs/msg/Detection3DArray` (frame_id=map, tf2로 변환 완료) | → 재식별/트래킹 노드 | 고정캠 스트림과 동일 스키마로 합류 — 하위 노드가 출처 구분 불필요. `pose.covariance[0]`에 등급+동기시간차 기반 위치 분산을 실어 보냄 |
| `leg_detector_bridge_node` (`amr_person_tracking`) | 수신 | `sensor_msgs/msg/LaserScan` | LiDAR 드라이버 | 자체 구현 검출기 - 곡률필터+칼만필터 기반 정적배경 제외. range+bearing 센서라 depth 역투영/호모그래피 불필요, tf 변환만 수행 |
| `leg_detector_bridge_node` (`amr_person_tracking`) | 송신 | `vision_msgs/msg/Detection3DArray` (frame_id=map) | → 재식별/트래킹 노드 | 다른 두 소스와 동일 스키마로 합류. 신원은 모름 — id 필드엔 라이다 쪽 트랙 ID만 실림 |
| `reid_tracking_node` (`amr_person_tracking`) | 수신 | 로컬라이제이션 노드 + `oakd_detector_node` + `leg_detector_bridge_node`의 `Detection3DArray` (통합 스트림) | 상동 | 위치+속도 게이팅 → Hungarian 전역 배정 → 상류 id 구제 → 갤러리 부활 4단계. 라이다 스트림은 시간정렬 예측→게이팅→N프레임 확인 후 락온하는 별도 신원 매칭 절차를 거침 |
| `reid_tracking_node` (`amr_person_tracking`) | 송신 | `vision_msgs/msg/Detection3DArray` (표준 id 필드에 지속 트랙ID 기록해 재발행) | → 예측 회피 노드, 스케줄러(비전 범위 밖) | 커스텀 ID 메시지 대신 표준 필드 재사용 |
| `reid_tracking_node` (`amr_person_tracking`) | 송신 | `geometry_msgs/msg/PoseStamped` (frame_id=map) | → Nav2 goal 발행 노드 (범위 밖) | 추종 대상 트랙의 2D map 좌표만 제공, Nav2 액션 호출은 별도 노드 담당. **추종 대상이 있을 때만 발행**(없으면 침묵) |
| `predictive_avoidance_node` (`amr_person_tracking`) | 수신 | id 포함 `Detection3DArray` 시계열 | 재식별/트래킹 노드 | 칼만필터(등속도 모델)로 속도 추정+예측, Δt는 `header.stamp` 기준 |
| `predictive_avoidance_node` (`amr_person_tracking`) | 송신 | `rcl_interfaces/srv/SetParameters` | → 각 로봇 `local_costmap` 노드(nav2_costmap_2d) | 접근 속도에 비례해 inflation 파라미터 동적 조정 |
| `predictive_avoidance_node` (`amr_person_tracking`) | 송신(대안) | `sensor_msgs/msg/PointCloud2` (가상 포인트) | → `nav2_costmap_2d` voxel_layer | `avoidance_mode` 파라미터로 SetParameters 방식과 양자택일/병행 |

## 3. `amr_person_tracking` 패키지 구조

```
src/amr_person_tracking/
  package.xml
  setup.py / setup.cfg
  amr_person_tracking/
    oakd_detector_node.py        # 4번: YOLO-pose + depth 3D 역투영
    leg_detector_bridge_node.py  # 4번 보완: 초근접 구간 라이다 다리검출 편입
    reid_tracking_node.py        # 7번: 재식별/트래킹 + 웹캠-라이다 신원 매칭 + Nav2 좌표 발행
    predictive_avoidance_node.py # 6번: 칼만필터 예측 회피
    debug_viewer_node.py         # (검증용) 디버그 이미지 뷰어
    depth_view_republisher_node.py   # (검증용) depth를 컬러맵 jpeg로 재발행
    mock_webcam_publisher_node.py    # (검증용) 웹캠 목업
    vision_utils.py              # 접지점 후보/depth 샘플링/역투영 등 순수함수
    tracking_utils.py            # Track, Hungarian 배정, 코사인 유사도 등 순수 로직
    predictive_utils.py          # 등속도 칼만필터(+마할라노비스), 다리 정지판정
  config/tracktrack_reid.yaml    # (기본 미사용) 외형 ReID를 켤 때 쓰는 트래커 설정
  launch/amr_person_tracking.launch.py   # 노드 일괄 기동
  test/                          # 순수함수 단위테스트 45건
  tools/                         # 검증 도구 (아래 4절)
```

## 4. 검증 도구 (`tools/`)

파이프라인 변경의 효과를 감이 아니라 수치로 확인하기 위한 도구들이다. 모두 격리 도메인
(`ROS_DISCOVERY_SERVER="" ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=<실기와 다른 값>`)에서 쓴다 —
실기 접속 설정 그대로 bag을 재생하면 실시간 로봇 데이터가 섞여 측정이 오염된다.

| 도구 | 용도 |
|---|---|
| `capture_target_pose.py` | `target_person_pose` + `tracked_detections_3d`를 캡처하고 `--analyze`로 순간이동 점프·동시 트랙 수를 정량화 |
| `measure_detection_jitter.py` | reid 노드를 거치기 **전** 원시 검출 좌표의 프레임간 점프를 측정. "ID 매칭 문제인가 측정 문제인가"를 가르는 데 씀 |
| `reid_embedding_eval.py` | 외형 기술자 후보들의 신원 판별력(AUC·최적 임계)을 bag으로 측정. 수동 라벨링 없이 "동일 프레임 다른 검출=다른 사람"을 음성 GT로 씀 |
| `record_debug_video.py` | 디버그 이미지 토픽을 H.264 mp4로 직접 녹화(화면 캡처 방식은 창이 가려지면 정지화면만 남아 교체) |
| `record_monitor.py`, `qos_overrides.yaml`, `fastdds_profile.xml` | bag 녹화 시 토픽 유실 감시 및 QoS/버퍼 설정 |

| 노드 | 파일 | 대응 역할 |
|---|---|---|
| oakd_detector_node | `amr_person_tracking/oakd_detector_node.py` | 4번 (위치추정 - OAK-D PRO) |
| leg_detector_bridge_node | `amr_person_tracking/leg_detector_bridge_node.py` | 4번 보완 (초근접 구간 라이다 다리검출) |
| reid_tracking_node | `amr_person_tracking/reid_tracking_node.py` | 7번 (재식별/트래킹, 웹캠-라이다 신원 매칭) |
| predictive_avoidance_node | `amr_person_tracking/predictive_avoidance_node.py` | 6번 (예측적 회피) |

`src/oakd_pointcloud` 패키지(5번, 라이다 사각지대 보완)는 별도로 존재하며 이 패키지와는 독립적으로 동작한다.
