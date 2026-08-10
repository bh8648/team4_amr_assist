# amr_person_tracking → main 병합 설계

- 브랜치: `feature/amr_person_tracking_merge` (main에서 분기)
- 날짜: 2026-08-10

## 배경

`feature/amr_person_tracking` 브랜치에는 사람 검출·추적 파이프라인(`src/amr_person_tracking/`,
4개 노드: `oakd_detector_node`, `leg_detector_bridge_node`, `reid_tracking_node`,
`predictive_avoidance_node`)이 있다. 이 파이프라인은 `<namespace>/target_person_pose`
(geometry_msgs/PoseStamped, frame_id=map)를 발행하지만, FOLLOWING 상태 진입/이탈 신호나
작업자 감지·유실 신호 계약은 전혀 모른다.

main의 `src/robot_PC/robot_bridge/robot_bridge/robot_bridge_node.py`는 이미 다음을
전제로 짜여 있다 (기존 코드, 무수정):

- `{robot_id}/target_person_pose`를 구독해 Nav2 `NavigateToPose` goal로 변환·전송
  (1Hz 상한, 0.2m 최소 이동 게이팅 포함) — **이미 완비**
- FOLLOWING 관련 상태일 때 `{robot_id}/worker_tracking/enable` (std_msgs/Bool,
  depth=1/RELIABLE/TRANSIENT_LOCAL)을 발행해 로컬 인식·추적 노드를 켜고 끔
- `{robot_id}/worker_detected` (Bool)를 구독해 도착 대기 중 작업자 인식을
  중앙 TaskCommand로 전달
- `{robot_id}/worker_tracking/lost` (Bool)를 구독해 60초 재탐색 실패를
  중앙 RobotError로 전달

즉 **"트래킹 결과를 Nav2 goal로 넘기는 로직"은 이미 main에 있고, 빠진 것은
`worker_tracking/enable → worker_detected/worker_tracking_lost` 계약을 채우는
어댑터**다. 이번 병합의 핵심은 그 어댑터(`worker_tracking_bridge_node.py`) 하나를
새로 만들고, 검출·추적 파이프라인 자체는 무수정으로 가져오는 것이다.

## 목표

1. `src/amr_person_tracking/` 패키지를 `src/robot_PC/amr_person_tracking/`으로
   **무수정** 병합 (코드/launch/test. `evidence/`의 대용량 mp4는 제외, `docs/`는 포함).
2. `src/robot_PC/robot_bridge/robot_bridge/worker_tracking_bridge_node.py`를 신규
   작성해 enable/detected/lost 계약을 구현.
3. `robot_bridge/launch/robot.launch.py`에 새 노드 실행과 `amr_person_tracking.launch.py`
   include를 배선.
4. 기존 `robot_bridge_node.py`, `robot_bridge_node.py`의 테스트는 무수정.

## 비목표 (Non-goals)

- `amr_person_tracking` 패키지 내부 로직·파라미터 튜닝은 다루지 않는다 (그대로 가져옴).
- `worker_tracking/enable`이 꺼졌을 때 `oakd_detector_node`/`leg_detector_bridge_node`
  등 무거운 검출 자체를 멈추는 것은 이번 범위 밖이다 — 이번 어댑터는 **발행 게이팅**만
  하고, 업스트림 검출 프로세스는 계속 돈다 (문서화된 한계로 남김).
- `predictive_avoidance_node`(회피)는 이미 파이프라인에 포함되어 그대로 병합되지만,
  robot_bridge와의 신규 연동은 없다 (기존 그대로 costmap/voxel_layer 직결).

## 아키텍처

```
                         (namespace=robot_id로 launch)
[oakd_detector_node] ──┐
[leg_detector_bridge]  ├─▶ [reid_tracking_node] ──/target_person_pose_raw──▶┐
                        │      (amr_person_tracking, 무수정)                │
                        │                                                   │
                                                                             ▼
[robot_bridge_node] ──worker_tracking/enable──▶ [worker_tracking_bridge_node] (신규)
      ▲                                              │              │
      │                                   target_person_pose   worker_detected
      │                                   (재발행, enable시만)   worker_tracking/lost
      └──────────────────────────────────────────────┴──────────────┘
```

`SetRemap`(GroupAction으로 스코프 격리)이 `reid_tracking_node`의
`/{robot_id}/target_person_pose` 발행을 `/{robot_id}/target_person_pose_raw`로
바꿔치기하므로, `amr_person_tracking` 패키지 소스는 건드리지 않는다.
`worker_tracking_bridge_node`가 `target_person_pose` 토픽의 유일한 최종 발행자가 된다.

## 컴포넌트

### `worker_tracking_bridge_node.py` (신규)

파라미터: `robot_id` (robot5|robot11, robot_bridge_node와 동일 검증), `worker_lost_timeout`
(기본 60.0초).

구독:
- `{robot_id}/worker_tracking/enable` (Bool) — QoS `depth=1, RELIABLE, TRANSIENT_LOCAL`
  (robot_bridge_node의 발행 QoS와 동일하게 맞춤 — 늦게 뜬 이 노드도 현재 enable 상태를
  즉시 받아야 함)
- `{robot_id}/target_person_pose_raw` (PoseStamped) — 기본 QoS(depth=10, RELIABLE,
  VOLATILE), `reid_tracking_node`의 발행 QoS와 동일

발행:
- `{robot_id}/target_person_pose` (PoseStamped) — 기본 QoS. enable=True일 때만
  raw 수신 즉시 그대로 재발행 (변환/검증 없음 — 검증은 robot_bridge_node가 이미 함)
- `{robot_id}/worker_detected` (Bool) — enable 켜진 뒤 첫 raw 좌표 수신 시 1회만
  (latch, enable 재진입 시 리셋)
- `{robot_id}/worker_tracking/lost` (Bool) — enable 상태에서 마지막 raw 수신 후
  `worker_lost_timeout` 이상 경과 시 1회만 (latch, 재수신되거나 enable 재진입 시 리셋).
  1Hz 내부 타이머로 판정.

내부 상태: `enabled: bool`, `last_pose_time: Optional[Time]`, `detected_sent: bool`,
`lost_sent: bool`. enable이 False→True로 바뀌면 네 값 모두 초기화(리셋).

### launch 배선 (`robot_bridge/launch/robot.launch.py`)

기존 `_launch_nodes()`가 만드는 노드 리스트에 추가:

1. `worker_tracking_bridge_node` (robot_bridge 패키지, `namespace=robot_id`,
   `parameters=[{'robot_id': robot_id}]`)
2. `GroupAction(actions=[SetRemap(src=.../target_person_pose, dst=.../target_person_pose_raw),
   IncludeLaunchDescription(amr_person_tracking.launch.py, launch_arguments={'namespace': robot_id,
   'publish_debug_image': 'false'})])` — `publish_debug_image` 기본값을 false로 덮어써서
   실물 로봇에서 cv2 창이 자동으로 뜨지 않게 함 (원본 패키지 기본값은 true).

### 의존성

`amr_person_tracking/package.xml`이 주석으로만 명시하고 rosdep에 없는 pip 패키지
(`ultralytics`, `opencv-python`, `scipy`, `numpy`)와 rosdep 대상(`depthai_ros_driver`,
`irobot_create_msgs`)이 있다. 이 저장소엔 `requirements.txt`가 없으므로 빌드 전
수동 설치가 필요 — 구현 계획에 "pip/rosdep 설치 확인" 단계로 명시한다.
`robot_bridge/package.xml`에는 `<exec_depend>amr_person_tracking</exec_depend>` 추가
(launch에서 참조하므로).

## 테스트

`src/robot_PC/robot_bridge/test/test_worker_tracking_bridge_node.py` 신규
(`test_robot_bridge_node.py`와 동일한 rclpy 노드 단위 테스트 패턴):

1. enable=False 상태에서 raw pose 수신 → `target_person_pose` 재발행 안 됨
2. enable=True 후 첫 raw pose 수신 → `target_person_pose` 재발행 + `worker_detected` 1회
3. 같은 enable 구간에서 두 번째 raw pose 수신 → `worker_detected` 재발행 안 됨(latch)
4. 짧은 `worker_lost_timeout` 파라미터로 노드 생성 후 pose 없이 타이머 콜백 직접 호출 →
   `worker_tracking/lost` 1회
5. enable False→True 재진입 → latch/타이머 리셋 확인 (재수신 시 `worker_detected` 다시 발행)

기존 `robot_bridge_node.py`/`test_robot_bridge_node.py`는 무수정이므로 회귀 테스트는
그대로 통과해야 한다 (변경 없음 확인 차원).

## 변경 파일 목록

- 신규: `src/robot_PC/amr_person_tracking/**` (feature 브랜치에서 무수정 이식,
  `evidence/` 제외)
- 신규: `docs/amr_person_tracking_pipeline_flow.md`,
  `docs/session_backup_amr_person_tracking.md` (feature 브랜치에서 이식)
- 신규: `src/robot_PC/robot_bridge/robot_bridge/worker_tracking_bridge_node.py`
- 신규: `src/robot_PC/robot_bridge/test/test_worker_tracking_bridge_node.py`
- 수정: `src/robot_PC/robot_bridge/setup.py` (entry_point 추가)
- 수정: `src/robot_PC/robot_bridge/package.xml` (`amr_person_tracking` exec_depend 추가)
- 수정: `src/robot_PC/robot_bridge/launch/robot.launch.py` (노드 실행 + GroupAction/SetRemap
  + IncludeLaunchDescription 배선)
- 무수정: `robot_bridge_node.py`, 기존 테스트 전부

## 알려진 한계

- enable=False여도 `amr_person_tracking`의 검출 노드들은 계속 실행된다 (발행만 게이팅됨).
  자원 절약이 필요해지면 별도 후속 작업.
- `amr_person_tracking`의 웹캠 검출 경로는 (원본 문서에 명시된 대로) 스키마 불일치로
  아직 실제 연결되지 않음 — 이번 병합으로도 해결되지 않는, 원본 브랜치의 기존 한계.
