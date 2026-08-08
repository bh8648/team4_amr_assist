# 파이프라인 테스트용 webcam PC CLI 설계

## 배경

전체 파이프라인은 다음과 같다.

1. AMR은 도킹 상태 (DOCKED)
2. 웹캠 PC → 중앙 PC로 작업자 위치 전달
3. 중앙 노드가 적절한 AMR 배정
4. AMR이 작업자 위치로 이동 (ASSIGNED)
5. AMR이 작업자 위치 도착 후 오크디 카메라로 작업자 추적
6. 추적 완료 후 작업자를 따라다님 (FOLLOWING)
7. 터미널에 "배송 모드"를 칠 경우 배송모드 시작 (TRANSPORTING)
8. 목적지는 DB에 있는 특정 좌표들 중 하나로 이동
9. 목적지 도착 후 5초 대기 시 배송 완료 판정
10. 도킹스테이션 근처로 이동 (RETURN)
11. 도킹

전체 구조는 중앙 PC·웹캠 PC·로봇 PC로 나뉘지만, 현재는 하나의 PC에서 모두 개발 중이다. `robot_manager`(중앙 PC)는 5개 노드가 모두 구현되어 있고, `robot_bridge`(로봇 PC)는 robot11 전용 브릿지가 구현되어 있다. 하지만 **웹캠 PC 쪽은 이 브랜치에 전혀 구현되어 있지 않다** — 작업자 호출 좌표 전달, 추적 완료 감지, 배송 확인을 보낼 주체가 없다. `destinations` 테이블도 비어 있어 배송모드의 목적지를 고를 수 없다.

이번 작업은 이 파이프라인을 실물 robot11 + 실물 Nav2로 end-to-end 테스트하기 위해 웹캠 PC 역할을 대신할 대화형 CLI와 테스트용 목적지 데이터를 추가하는 것이다. Nav2 자체는 사용자가 기존 bringup으로 직접 실행하므로 이 작업 범위에 포함하지 않는다 — "mock"이 필요한 것은 Nav2 시스템이 아니라, 웹캠 파이프라인이 없어 비어 있는 목표 좌표값이다.

## 목표

1. 웹캠 PC 역할(작업자 호출 좌표 전달, 추적 완료 감지, 배송 확인)을 대화형 CLI로 대체해 전체 상태 머신(DOCKED → ASSIGNED → FOLLOWING → TRANSPORTING → RETURNING → DOCKED)을 실물 robot11 + 실물 Nav2로 테스트한다.
2. FOLLOWING 상태에서 `robot_bridge`의 `target_person_pose` → Nav2 goal 중계 로직도 mock 좌표로 실물 검증한다.
3. `destinations` 테이블에 배송모드 테스트용 목적지를 채운다.

## 범위

**포함**: `robot_manager` 패키지에 새 CLI 노드 추가, `destinations` 테이블 시드, CLI의 순수 로직 유닛 테스트.

**제외**: Nav2 자체 구현(사용자가 기존 bringup으로 실행), robot5, `central_system.launch.py` 통합, 실제 물리 실행·검증(사용자 담당 — 아래 "안전 수칙" 참조).

## 패키지/파일

- 새 파일: `real_project/src/robot_manager/robot_manager/webcam_pc_cli.py`
- executable: `webcam_pc_cli` (`setup.py`의 `entry_points`에 추가, 기존 `dummy_publisher`와 같은 방식)
- `central_system.launch.py`에는 포함하지 않는다 — 대화형 stdin이 필요해 launch로 백그라운드 구동하면 입력을 받을 수 없다. 사용자가 별도 터미널에서 `ros2 run robot_manager webcam_pc_cli`로 직접 실행한다.
- 동작 방식: rclpy를 데몬 스레드에서 spin, 메인 스레드는 stdin 명령 루프 (`hmi_backend_node`의 spin-thread 패턴과 동일).

## 인터페이스

### 발행

| 토픽 | 타입 | 트리거 |
|---|---|---|
| `/assignment_goal` | `robot_status/AssignmentGoal` | `호출 <x> <y>` |
| `/task/command` | `robot_status/TaskCommand` | `작업자감지`(WORKER_DETECTED), `배송모드`(START_TRANSPORT), `배송확인`(DELIVERY_CONFIRMED) |
| `/robot11/target_person_pose` | `geometry_msgs/PoseStamped` (frame_id=`map`) | `추종시작`으로 시작되는 고정 간격 자동 순차 발행(10점), `추종중지`로 취소 |

### 구독

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/robot_assignment` | `robot_status/RobotAssignment` | 배정 성공/실패 여부를 터미널에 출력 |
| `/task/state` | `robot_status/TaskState` | 로봇별 최신 `(task_id, state)` 캐싱 — 이후 명령의 robot_id/task_id 자동 채움에 사용 |
| `/robot_error` | `robot_status/RobotError` | 실패·오류 사유를 그대로 터미널에 출력 |

## 명령어

| 명령 | 동작 |
|---|---|
| `호출 <x> <y>` | `AssignmentGoal(x, y)` 발행 |
| `목적지목록` | `destinations` 테이블 조회·출력 |
| `작업자감지` | `WORKER_DETECTED` 발행 (활성 로봇 자동 채움) |
| `추종시작 [간격초=3]` | mock 좌표 10개를 간격초마다 하나씩 `/robot11/target_person_pose`로 순차 발행 시작. 간격초는 양수여야 하며, 생략·0 이하·숫자가 아니면 에러 출력 후 미시작. 이미 진행 중인 타이머가 있으면 (재시작하지 않고) "이미 진행 중, 먼저 추종중지" 에러만 출력 |
| `추종중지` | 진행 중인 순차 발행 타이머 취소. 진행 중인 타이머가 없으면 조용히 무시 |
| `배송모드 [목적지id]` | `START_TRANSPORT` 발행. 명령어 매칭은 입력 전체에서 공백을 제거한 뒤 "배송모드"로 시작하는지만 검사하고, **원본 입력에서 그 부분을 제거한 나머지를 공백 기준으로 split한 첫 토큰**을 목적지id 인자로 쓴다(즉 정규화는 명령어 키워드에만 적용, 인자는 원본 그대로 파싱). 목적지id 생략 시: `destinations`가 1개면 자동 선택, 여러 개면 목록 출력 후 재입력 요구, 0개면 "등록된 목적지 없음" 에러. 존재하지 않는 목적지id를 지정하면 "목적지 없음" 에러 출력 후 미발행 |
| `배송확인` | `DELIVERY_CONFIRMED` 발행 |
| `상태` | 캐싱된 로봇별 최신 `TaskState` 출력 |
| `종료` | 노드 종료 |

## robot_id/task_id 자동 채움 로직

- `/task/state`로 로봇별 최신 메시지를 `{robot_id: TaskState}` 딕셔너리에 캐싱한다.
- "정확히 1개"의 판정 기준은 **`task_manager_node.ACTIVE_STATES`(`ASSIGNED`, `FOLLOWING`, `TRANSPORTING`, `RETURNING`)에 속한 로봇만 후보로 삼는다** — TaskState를 한 번이라도 받은 모든 로봇을 후보로 삼으면 로봇이 `DOCKED`/`ERROR`로 끝난 뒤에도 계속 "로봇 지정 요구" 상태에 머무르게 되므로, 반드시 활성 상태 필터를 거친다.
- `작업자감지`/`배송모드`/`배송확인` 실행 시: 활성 상태인 로봇이 정확히 1개면 그 `robot_id`/`task_id`로 발행한다. 0개면 "활성 작업 없음"을 출력하고 발행하지 않는다. 2개 이상이면 로봇 지정을 요구한다(예: `작업자감지 robot11`).
- **상태 전이 유효성은 CLI가 판단하지 않는다.** `task_manager_node`가 이미 상태 머신을 검증하므로, 잘못된 시점에 명령을 보내면 `task_manager_node`가 `/robot_error`로 `INVALID_TRANSITION_*` 등을 발행하고 CLI는 이를 그대로 출력한다. 규칙을 중복 구현하지 않는다.

## 하드코딩 좌표 (placeholder — 실행 전 사용자가 실제 값으로 교체 필수)

지도 범위: `-5.4 ≤ x < 0.6`, `-5.65 ≤ y < 1.5` (`map2.yaml`의 origin/resolution과 `map2.pgm` 크기로 `robot_assignment_node.load_map_bounds()`와 동일하게 계산한 값. `dummy_status_publisher.py`의 여유를 둔 파라미터 값과는 다르니 혼동하지 말 것)

- **FOLLOWING mock 10점** (`webcam_pc_cli.py` 상단 상수, yaw 없이 직선 경로):
  `(-4.0,-4.0), (-3.7,-3.8), (-3.4,-3.6), (-3.1,-3.4), (-2.8,-3.2), (-2.5,-3.0), (-2.2,-2.8), (-1.9,-2.6), (-1.6,-2.4), (-1.3,-2.2)`
- **배송 목적지 2점** (`destinations` 테이블 INSERT): `DEST_A`(-4.5, -4.5, 0.0), `DEST_B`(-1.0, -1.0, 1.57)
- **도킹 복귀 위치**: `task_manager_node`의 `robot11_dock_pose` 파라미터, 현재 기본값 `[0,0,0]`을 그대로 유지한다 — 이 스펙에서 새로 만들지 않으며, 실행 전 사용자가 실제 도킹 스테이션 근처 좌표로 launch 인자를 지정해야 한다.
- **작업자 호출 위치**는 하드코딩하지 않고 `호출 <x> <y>`로 매번 실행 시 입력한다.

이 값들은 전부 "지도 범위 안"이라는 것만 보장된 placeholder이며, 실제로 빈 공간인지·장애물이 없는지는 검증되지 않았다. **사용자가 실행 전 실측값으로 교체해야 한다.**

## DB 시드

`amr.db`의 `destinations` 테이블에 위 2행을 INSERT한다. 별도 마이그레이션 체계 없이 1회성 INSERT로 처리한다.

`목적지목록`/`배송모드` 조회는 `hmi_backend_node.py`와 동일하게 요청마다 짧게 `sqlite3.connect()`로 열고 즉시 닫는 패턴을 따른다 — `db_manager_node`가 동시에 같은 파일에 쓰기 작업을 하고 있어도(WAL 미설정) 짧은 조회 트랜잭션이므로 기존 코드베이스와 동일한 수준의 동시성 처리로 충분하다. 별도 재시도·락 처리는 추가하지 않는다.

## 안전 수칙 (하드웨어 관련 — 반드시 사용자 주도)

이 테스트는 실물 robot11이 실제로 Nav2 목표를 향해 주행하고 도킹 스테이션에 도킹하는 물리 테스트다. 세션이 로봇과 같은 ROS2 네트워크에 연결되어 있어도, AI는 다음 항목을 원격에서 단독으로 실행·판단하지 않는다:

- Nav2 bringup 실행 (사용자가 로봇 PC에서 직접)
- `robot11_bridge_node` 실행 및 `/robot_status`에 실측값이 찍히는지 확인 (육안 확인)
- 좌표 placeholder → 실측값 교체 (사용자가 실제 공간을 알고 있어야 함)
- 이동을 유발하는 모든 CLI 명령 — 사용자가 로봇 옆에서 비상정지를 쥔 채 직접 타이핑한다:
  - `호출 <x> <y>` — **배정 성공 즉시 `task_manager_node.assignment_callback()`이 TO_WORKER Nav2 목표를 바로 전송해 로봇이 주행을 시작한다.** 단순히 배정 결과를 조회하는 명령이 아니므로 반드시 이 목록에 포함한다.
  - `추종시작`, `배송모드`, `배송확인`
- 도킹/언도킹 액션 트리거와 정렬 확인
- `추종시작`의 간격초 기본값(3초)은 미검증 초기값이며, 실제 주행을 보며 사용자와 함께 튜닝한다

**AI가 자율로 실행해도 되는 것은 CLI 노드 실행(`ros2 run robot_manager webcam_pc_cli`) 자체와, 이동을 전혀 유발하지 않는 순수 조회 명령(`목적지목록`, `상태`)뿐이다.** `호출`을 포함해 위에 나열된 모든 이동 유발 명령은 예외 없이 사용자가 직접 트리거한다.

## 테스트 방침

- 순수 로직(목적지 조회/선택, 명령 파싱, robot_id/task_id 자동 채움, "배송모드"/"배송 모드" 정규화)은 ROS 스핀 없이 pytest로 검증한다.
- 실제 ROS 그래프 동작(robot11 실물 주행, Nav2, 도킹)은 위 안전 수칙에 따른 수동 체크리스트로 대체한다.

## 수동 체크리스트 (사용자 주도)

1. 로봇 PC에서 Nav2 bringup + `robot11_bridge_node` 실행 → `/robot_status`에 실제 위치·배터리 확인
2. `webcam_pc_cli` 실행 → `호출 <x> <y>`로 배정 유발 → `/robot_assignment` 성공 확인 (DOCKED → ASSIGNED)
3. `작업자감지`로 ASSIGNED → FOLLOWING 전환 확인
4. `추종시작` 실행하며 로봇 옆에서 실제 주행 관찰, 이상 시 `추종중지`
5. `배송모드 DEST_A`로 FOLLOWING → TRANSPORTING 전환, 실제 목적지 도착 확인
6. 목적지 도착 후 `배송확인`으로 TRANSPORTING → RETURNING 전환, 도킹 위치로 이동 확인
7. 도킹 액션 성공 후 RETURNING → DOCKED 전환 확인

## 범위 밖 (다음 작업)

- Nav2 자체 구현/mock 서버
- robot5 지원
- `target_person_pose` 실 발행 쪽(`reid_tracking_node`, 별도 브랜치)과의 병합
- `central_system.launch.py`에 CLI 통합
- 도킹 스테이션 정렬 좌표 확정
