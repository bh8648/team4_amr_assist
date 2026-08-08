# 웹캠 PC - 중앙 PC - AMR PC 통합 설계

## 배경

`main` 브랜치는 웹캠 PC(사람 인식/호출)와 중앙 PC(배정/작업관리/HMI/DB/교착방지)를 더미 로봇 상태로 통합 테스트했다. 현재 브랜치 `feature/hju/robot11_bridge_node`(이하 HEAD)는 중앙 PC와 AMR PC(robot11 실물 `robot_bridge`)를 더미 웹캠(`webcam_pc_cli`)으로 통합 테스트했다.

이 문서는 두 브랜치를 하나로 합쳐 웹캠 PC - 중앙 PC - AMR PC 3대가 실제로 연결되는 전체 파이프라인을 완성하기 위한 설계다. 이 프로젝트는 **실제 물리 로봇(iRobot Create3 기반 AMR)을 움직인다.**

## PC별 최종 구성

| PC | 담당 패키지 | 상태 |
|---|---|---|
| 웹캠 PC | `person_locator`(사람 위치추정+호출좌표), `hand_gesture_caller`(손짓 트리거), `hardhat_detector`(안전모 감지, 독립 기능) | main에서 그대로 반입 (실물) |
| 중앙 PC | `robot_manager` 5개 노드 (assignment/task/hmi/db/deadlock) | 이번에 통합 |
| AMR PC2 (robot11) | `robot_bridge`(robot11, 실물 검증됨) | 실물, 이번에 인터페이스 조정 |
| AMR PC1 (robot5) | `robot_bridge`(robot5용, 미제작) | 범위 밖 — robot5는 당분간 상태 없음(배정 후보에서 자연히 제외) |

`person_locator`/`hand_gesture_caller`/`hardhat_detector`는 main과 동일하게 저장소 최상위 `src/`(웹캠 PC 전용 별도 워크스페이스)에 유지한다. `real_project/src/`(중앙 PC + AMR PC)와 물리적으로 다른 머신이므로 구조를 분리한 상태를 그대로 가져온다.

## 1. 토픽/메시지 인터페이스

### 로봇 상태
- **main 방식 채택**: `/{robot_id}/robot_status` (로봇별 개별 토픽).
- `robot_bridge/robot11_bridge_node.py`가 현재 발행 중인 `/robot_status`(단일 토픽)를 `/robot11/robot_status`로 변경한다.
- `robot_bridge`의 `ROBOT_ID`(현재 모듈 상수 `'robot11'`)를 ROS 파라미터(`declare_parameter('robot_id', 'robot11')`)로 변경한다. 토픽 이름과 `RobotStatus.robot_id` 값 모두 이 파라미터를 사용하도록 고친다. robot5용 브릿지 자체는 이번 범위가 아니지만, 같은 코드를 파라미터만 바꿔 재사용할 수 있게 해 둔다.
- `RobotStatus.msg`의 `current_task_id` 필드(HEAD 전용 추가분)는 유지한다.
- `robot_assignment_node.py`, `db_manager_node.py`, `deadlock_prevention_node.py`는 main의 per-robot 구독 방식(`status_callback(expected_robot_id, msg)`로 토픽-robot_id 불일치 검증)을 채택한다.

### 호출좌표
- **main 방식 채택**: `geometry_msgs/PointStamped` → `/person/call_position` (frame_id == 'map' 검증 포함).
- HEAD 전용 `robot_status/msg/AssignmentGoal.msg`(x, y만 있는 커스텀 메시지)와 `/assignment_goal` 토픽은 폐기한다. `CMakeLists.txt`에서 해당 라인도 제거한다.
- `robot_assignment_node.py`는 main 버전(PointStamped 구독, per-robot 상태 구독, frame_id/중복거리 검증)을 채택하고, HEAD가 추가한 `is_robot_busy(status)`(`RobotStatus.current_task_id` 기반) 판정 로직을 이식한다.

### HMI 백엔드
- `hmi_backend_node.py`는 main 버전을 그대로 채택한다(HEAD가 이 파일을 건드리지 않아 병합 충돌 없음).

## 2. 작업자 감지 / 배송확인 자동화

`task_manager_node.py`의 `handle_navigation_result()`를 다음과 같이 바꾼다:

- `TO_WORKER` 목표 도착 성공 → 기존처럼 "작업자 감지 대기" 상태로 멈추지 않고 **즉시 `ASSIGNED → FOLLOWING` 전환**한다.
- `TO_DESTINATION` 목표 도착 성공 → 기존처럼 "배송 확인 대기" 상태로 멈추지 않고 **즉시 `TRANSPORTING → RETURNING` 전환 + 도킹 좌표로 Nav2 goal 발행**한다(기존 `DELIVERY_CONFIRMED` 핸들러가 하던 일을 그대로 수행).

`command_callback()`의 `WORKER_DETECTED`, `DELIVERY_CONFIRMED` 분기는 제거한다. 더 이상 이 두 커맨드를 외부에서 보낼 주체가 없다(웹캠 PC의 실제 감지 알고리즘과 위치 연동은 추후 별도 구현 예정이며, 그전까지는 로봇이 배정된 위치에 물리적으로 도착한 것 자체를 "감지 완료"로 간주한다).

`START_TRANSPORT`(배송모드) 커맨드 처리는 그대로 유지하되, 발행 주체가 `webcam_pc_cli` → HMI로 바뀐다 (아래 4번 참고).

## 3. 도킹 상태 확인 후 진행 (신규)

AMR이 배정받는 시점에 이미 도킹돼 있을 수 있다. 배정 즉시 주행을 시작하기 전에 도킹 여부를 확인하고, 도킹돼 있으면 언도킹을 먼저 완료한 뒤 진행한다.

- `RobotStatus.msg`에 `bool is_docked` 필드를 추가한다(메시지 뒤쪽에 추가 — 필드 이름 기반 생성이라 기존 소비자에 영향 없음).
- `robot_bridge`가 `irobot_create_msgs/msg/DockStatus`로 추정되는 토픽을 구독해 `is_docked`를 채운다.
  - **⚠️ 확인 필요**: 실제 토픽명·메시지 타입·필드명은 가정이다. 사용자가 나중에 robot11 PC에서 `ros2 topic list | grep dock`, `ros2 interface show irobot_create_msgs/msg/DockStatus`로 직접 확인 후 수정하기로 함. 이 구독부는 한 곳에 격리해서 나중에 한 줄만 고치면 되도록 구현한다.
- `task_manager_node`가 `/{robot_id}/robot_status`를 신규 구독해 로봇별 최신 `is_docked`를 캐싱한다.
- `assignment_callback()`에서 Task 생성 시:
  - `is_docked == False`(또는 상태 미수신) → 기존처럼 즉시 `send_navigation_goal()`.
  - `is_docked == True` → `dock_publishers[robot_id]`(기존 `/{robot_id}/dock/request` Bool 토픽)에 `Bool(False)`를 **한 번만** 발행하고 Nav goal은 보류. `ManagedTask`에 `undock_requested: bool` 플래그를 둬서 중복 발행을 막는다(재시도 로직 없음 — 실패 시 사람이 개입).
  - 이후 해당 로봇의 `robot_status` 갱신에서 `is_docked`가 `False`로 바뀌는 걸 확인하면 그때 `send_navigation_goal()`을 실행한다.
- `robot_bridge`의 `dock_callback()`(이미 Dock/Undock 액션 클라이언트 구현돼 있음)에 in-flight 가드를 추가한다: 이미 진행 중인 Dock/Undock 액션이 있으면 새 요청을 무시해서, 자동 트리거든 HMI 수동 조작이든 실제 하드웨어에 중복 액션 콜이 나가지 않도록 한다.
- DB(`amr.db`)에 `is_docked`를 영속화하지는 않는다 — 기존 DB에 스키마 생성/마이그레이션 코드가 없고, 이번 기능에 필수도 아니라서 범위에서 제외했다. 필요해지면 팀원과 상의해서 별도로 진행한다.

## 4. 신규: HMI 배송모드

`hmi_backend_node.py`(main 버전 기반)에 배송모드 엔드포인트를 추가한다:

- `POST /api/robot/{robot_id}/transport {destination_id}` — `destinations` 테이블에서 좌표 조회 후 `TaskCommand(START_TRANSPORT, target_x/y/yaw=목적지 좌표)` 발행.
- 나중에 제거하기 쉽도록 메서드/엔드포인트를 파일 내 별도 블록으로 명확히 분리한다(다른 로직과 얽지 않음).
- React 프론트엔드에 FOLLOWING 상태일 때만 노출되는 목적지 선택 드롭다운 + "배송 시작" 버튼을 독립 컴포넌트로 추가한다. 이 UI는 로봇 부착 UI가 나중에 생기면 통째로 들어낼 수 있어야 한다.

## 5. 더미 제거

- `robot_manager/dummy_status_publisher.py` 완전 삭제, `setup.py`의 `dummy_publisher` entry_point도 함께 제거한다.
- `webcam_pc_cli.py` 축소:
  - 제거: `호출`(person_locator가 대체), `작업자감지`/`배송모드`/`배송확인`(자동화 또는 HMI로 이동), `목적지목록`.
  - `webcam_pc_cli_utils.py`에서 제거되는 유틸: `parse_call_args`, `select_destination`, `DELIVER_COMMAND` 관련 파싱 분기.
  - 유지: `추종시작`/`추종중지`(FOLLOWING mock 좌표 발행 — AMR의 oakd 카메라 미구현 구간을 임시로 메움), `상태`, `종료`.
- `robot_status/msg/AssignmentGoal.msg` 삭제, `robot_status/CMakeLists.txt`에서 해당 라인 제거.

## 6. 범위 밖 (일부러 손 안 댐)

- robot5용 실물 `robot_bridge` 없음 → robot5는 당분간 배정 후보에서 자연히 제외(RobotStatus 자체가 안 옴).
- AMR의 oakd 카메라 기반 실시간 추종좌표 미구현 → `webcam_pc_cli`의 FOLLOWING mock 좌표 발행으로 계속 대체.
- `CENTRAL_SYSTEM_NODE_FLOW.md`(아키텍처 문서) 갱신은 이번 작업 범위에서 제외한다(사용자 요청).
- `amr.db`에 `is_docked` 컬럼 영속화는 범위에서 제외한다(사용자 요청, 필요 시 팀 논의 후 별도 진행).

## 실제 로봇에서만 검증 가능한 항목 (구현 완료 후 사용자 확인 필수)

이 프로젝트는 실제 로봇을 움직이므로 시뮬레이션/유닛테스트만으로는 검증할 수 없는 항목이 있다. 아래는 구현 후 반드시 사용자가 실물 환경에서 확인해야 한다:

1. `irobot_create_msgs/msg/DockStatus`의 실제 토픽명·필드명 (현재 가정값으로 구현, 사용자가 나중에 수정하기로 함).
2. 도킹/언도킹 물리 동작의 안전성과 타이밍 — "1회만 언도킹 발행 후 `robot_status`의 `is_docked=False` 확인 후 진행" 로직이 실제 Create3 하드웨어에서 타이밍 경합 없이 동작하는지.
3. `robot11_dock_pose` 실측값(`[-2.3, -3.6, -π/2]`)이 새 도킹 감지 로직과 맞물릴 때도 유효한지.
4. 웹캠 PC + 중앙 PC + AMR PC 3대가 동시에 통신하는 ROS2 네트워크(도메인ID/멀티캐스트/DDS) 설정 — 기존엔 2PC씩만 테스트했음.
5. 자동전환(작업자 위치 도착 즉시 FOLLOWING, 배송 위치 도착 즉시 RETURNING)이 대기 없이 바로 다음 동작으로 넘어가는 게 실제 동선에서 안전한지.
6. 배터리/AMCL 센서 QoS(`BEST_EFFORT` 가정)가 실제 Create3 펌웨어와 호환되는지.
7. oakd 카메라 부재로 FOLLOWING 단계는 mock 좌표로만 대체되므로, 실제 사람 추종 시나리오는 이번 스펙 범위에서 검증되지 않는다.

## 영향받는 테스트

- `robot_bridge/test/test_robot11_bridge_node.py`: dock 관련 테스트(in-flight 가드로 동작 변경), `build_status_message` 관련 테스트(`is_docked` 필드 추가) 갱신 필요.
- `robot_manager/test/test_task_manager_node.py`: 신규 자동전환/도킹확인 로직에 대한 테스트 추가 필요.
- `robot_manager/test/test_webcam_pc_cli_node.py`: 제거되는 명령(`호출`/`작업자감지`/`배송모드`/`배송확인`/`목적지목록`) 관련 테스트 삭제/수정.
- `robot_manager/test/test_webcam_pc_cli_utils.py`: 제거되는 유틸(`parse_call_args`, `select_destination`, `DELIVER_COMMAND` 파싱) 관련 테스트 삭제.
