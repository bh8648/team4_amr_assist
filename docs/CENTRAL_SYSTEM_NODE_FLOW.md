# 중앙 PC 5개 노드 구조 및 데이터 흐름

## 1. 중앙 시스템 전체 구조

```mermaid
flowchart LR
    subgraph VisionPC[Vision PC]
        Gesture[wrist_gesture_node\n손 제스처 확정] -->|Empty\nperson/call_trigger| Vision[pose_locator_node\n마지막 위치 저장]
    end
    Vision -->|PointStamped\n/person/call_position| Allocation[amr_allocation_node\nrobot_assignment_node]
    Robots[robot5 · robot11] -->|RobotStatus\n/robot_status| Allocation
    Allocation -->|RobotAssignment\n/robot_assignment| Task[task_manager_node]
    Allocation -->|RobotAssignment\n/robot_assignment| DB[db_manager_node]
    Allocation -->|RobotError\n/robot_error| DB

    Browser[React HMI\n:5173] <-->|HTTP JSON\n:8000| HMI[hmi_manager_node\nhmi_backend_node]
    HMI -->|TaskCommand\n/task/command| Task
    HMI -->|SELECT| Database[(amr.db)]
    HMI -->|YAML + PGM 읽기| Map[map2]

    Task -->|TaskState\n/task/state| DB
    Task -->|TaskState\n/task/state| Allocation
    Task -->|NavigateToPose Action| Nav5[robot5 Nav2]
    Task -->|NavigateToPose Action| Nav11[robot11 Nav2]
    Nav5 -->|Action Result| Task
    Nav11 -->|Action Result| Task
    Task -->|Pause · Dock Bool| Robots

    Robots -->|RobotStatus\n/robot_status| Deadlock[deadlock_prevention_node]
    Task -->|TaskState\n/task/state| Deadlock
    Deadlock -->|DeadlockPermission\n/deadlock/permission| Task

    Robots -->|RobotStatus · RobotError| DB
    DB -->|INSERT · UPDATE| Database
    Database -->|최신 상태 · Task · 오류| HMI
```

## 2. 핵심 데이터 이동표

| 송신 노드 | 데이터 | ROS 이름 | 수신 노드 | 용도 |
|---|---|---|---|---|
| wrist_gesture_node | `Empty` | `/person/call_trigger` | pose_locator_node (Vision PC 내부) | 손 제스처 확정 순간을 트리거해 마지막 위치를 발행하게 함 |
| pose_locator_node | `PointStamped` | `/person/call_position` | Allocation | 제스처 확정 시점에 저장해둔 작업자 호출 좌표 전달 |
| robot5, robot11 | `RobotStatus` | `/robot_status` | Allocation, DB, Deadlock | 위치·방향·배터리·작업 ID 전달 |
| Allocation | `RobotAssignment` | `/robot_assignment` | Task, DB | 배정 성공 여부·로봇 ID·목표 좌표 전달 |
| Allocation, Task, Robot | `RobotError` | `/robot_error` | Task, DB | 오류 상태 전환 및 DB 기록 |
| HMI | `TaskCommand` | `/task/command` | Task | PAUSE, RESUME, DOCK, UNDOCK 등 사용자 명령 |
| Task | `TaskState` | `/task/state` | Allocation, DB, Deadlock | 재배정 차단, DB 기록, 교착 판단 |
| Task | `NavigateToPose.Goal` | `/robot5/navigate_to_pose`, `/robot11/navigate_to_pose` | 각 AMR Nav2 | 작업자·배송지·도킹 위치 이동 |
| Nav2/로봇 PC | Action Result 또는 `NavigationResult` | Nav2 Action, `/navigation/result` | Task | 이동 성공·실패 결과 전달 |
| Deadlock | `DeadlockPermission` | `/deadlock/permission` | Task | robot11 정지·재개 결정 전달 |
| DB | SQLite 행 | `amr.db` | HMI | 화면에 표시할 최신 데이터 제공 |

## 3. Task 상태 머신

```mermaid
stateDiagram-v2
    [*] --> DOCKED
    DOCKED --> ASSIGNED: AMR 배정 성공
    ASSIGNED --> FOLLOWING: WORKER_DETECTED
    FOLLOWING --> TRANSPORTING: START_TRANSPORT + 배송 좌표
    TRANSPORTING --> RETURNING: DELIVERY_CONFIRMED 또는 CANCEL
    RETURNING --> DOCKED: 도킹 위치 이동 성공

    ASSIGNED --> PAUSED: HMI 정지 또는 교착 대기
    FOLLOWING --> PAUSED: HMI 정지 또는 교착 대기
    TRANSPORTING --> PAUSED: HMI 정지 또는 교착 대기
    RETURNING --> PAUSED: HMI 정지 또는 교착 대기
    PAUSED --> ASSIGNED: 이전 상태 복구
    PAUSED --> FOLLOWING: 이전 상태 복구
    PAUSED --> TRANSPORTING: 이전 상태 복구
    PAUSED --> RETURNING: 이전 상태 복구

    ASSIGNED --> ERROR: 오류 또는 Nav2 실패
    FOLLOWING --> ERROR: 오류
    TRANSPORTING --> ERROR: 오류 또는 Nav2 실패
    RETURNING --> ERROR: 오류 또는 Nav2 실패
```

## 4. amr_allocation_node Flow Chart

실제 실행 파일은 `robot_assignment_node.py`이며 ROS 노드 이름은 `amr_allocation_node`이다.

```mermaid
flowchart TD
    A[/PointStamped 수신/] --> B[x, y 숫자 변환]
    B --> C{NaN 또는 무한대?}
    C -- 예 --> C1[INVALID_COORDINATE 발행]
    C -- 아니오 --> D{map2 범위 안인가?}
    D -- 아니오 --> D1[OUT_OF_MAP 발행]
    D -- 예 --> E{이미 대기 중인 Goal이 있는가?}
    E -- 같은 위치 --> E1[중복 요청 무시]
    E -- 다른 위치 --> E2[PENDING_ASSIGNMENT_EXISTS 발행]
    E -- 없음 --> F[/robot5 · robot11 상태 확인/]
    F --> G[오래된 상태·오류·RobotStatus 작업 ID·TaskState 작업 상태·저배터리 제외]
    G --> H[거리 + 배터리 패널티 + 방향 패널티 계산]
    H --> I{후보 AMR 존재?}
    I -- 예 --> J[최저 점수 AMR 선택]
    J --> K[/RobotAssignment 성공 발행/]
    I -- 아니오 --> L[실패 원인 계산]
    L --> M[/RobotAssignment 실패 + RobotError 발행/]
    M --> N[Goal 저장 후 주기적 재시도]
    N --> O{60초 이내 배정 성공?}
    O -- 예 --> K
    O -- 아니오 --> P[ASSIGNMENT_TIMEOUT 발행 후 대기 정보 제거]
```

지도 경계는 YAML과 PGM 헤더에서 계산한다.

```text
-5.4 ≤ x < 0.6
-5.65 ≤ y < 1.5
```

## 5. task_manager_node Flow Chart

```mermaid
flowchart TD
    A{입력 종류} -->|RobotAssignment| B{배정 성공?}
    A -->|TaskCommand| C{명령 종류}
    A -->|Nav2 Result| D{이동 성공?}
    A -->|DeadlockPermission| E{허가 여부}
    A -->|RobotError| F[Task를 ERROR로 전환]

    B -- 아니오 --> B1[처리 종료]
    B -- 예 --> B2[Task ID 생성]
    B2 --> B3[ASSIGNED 상태 발행]
    B3 --> B4[작업자 좌표로 Nav2 Goal 전송]

    C -->|WORKER_DETECTED| C1[ASSIGNED → FOLLOWING]
    C -->|START_TRANSPORT| C2[FOLLOWING → TRANSPORTING]
    C2 --> C3[배송지 Nav2 Goal 전송]
    C -->|DELIVERY_CONFIRMED| C4[TRANSPORTING → RETURNING]
    C -->|CANCEL| C4
    C4 --> C5[도킹 위치 Nav2 Goal 전송]
    C -->|PAUSE| C6[현재 상태와 정지 원인 저장]
    C6 --> C7[Nav2 Goal 취소 + 로봇 정지]
    C -->|RESUME| C8[이전 상태 복구 + 필요 시 Goal 재전송]
    C -->|DOCK · UNDOCK| C9[선택 로봇 도킹 토픽 발행]

    D -- 실패 --> D1[ERROR 전환 + RobotError 발행]
    D -- 작업자 위치 도착 --> D2[작업자 감지 대기]
    D -- 배송지 도착 --> D3[배송 확인 대기]
    D -- 도킹 위치 도착 --> D4[도킹 요청 + DOCKED 전환]

    E -- 거부 --> E1[robot11 PAUSED · DEADLOCK_WAIT 저장]
    E -- 허가 --> E2{정지 원인이 DEADLOCK_WAIT인가?}
    E2 -- 예 --> C8
    E2 -- 아니오 --> E3[수동 정지 유지]

    C1 --> Z[/TaskState 발행/]
    C2 --> Z
    C4 --> Z
    C6 --> Z
    C8 --> Z
    D1 --> Z
    D4 --> Z
    F --> Z
```

## 6. hmi_manager_node Flow Chart

실제 실행 파일은 `hmi_backend_node.py`이고 Launch에서 노드 이름을 `hmi_manager_node`로 지정한다.

```mermaid
flowchart TD
    A[React가 HTTP 요청] --> B{API 종류}
    B -->|로그인| C[rokey / 1234 확인]
    C --> D{성공?}
    D -- 예 --> D1[세션 토큰 반환]
    D -- 아니오 --> D2[HTTP 401]

    B -->|GET /api/robots| E[amr.db 최신 Robot 상태와 Task 조회]
    E --> E1[HMI JSON 형식으로 변환]
    E1 --> E2[React에 반환]

    B -->|GET /api/map| F[YAML + PGM 읽기]
    F --> F1[지도 cell 배열 반환]

    B -->|GET /api/database/...| G[허용된 DB 테이블 SELECT]
    G --> G1[최대 100개 행 반환]

    B -->|일시정지·재개| H[TaskCommand PAUSE 또는 RESUME 생성]
    B -->|도킹·언도킹| I[TaskCommand DOCK 또는 UNDOCK 생성]
    H --> J[/task/command 발행]
    I --> J

    B -.->|작업 취소·텔레옵| K[프론트 버튼만 존재]
    K -.-> K1[백엔드와 ROS 연결은 추후 구현]
```

## 7. db_manager_node Flow Chart

```mermaid
flowchart TD
    A{수신 데이터} -->|RobotStatus| B[로봇별 최신 메시지와 수신 시각 저장]
    A -->|RobotAssignment| C{배정 성공?}
    A -->|TaskState| D[Task 상태 캐시 갱신]
    A -->|RobotError| E[error_logs 즉시 INSERT]

    B --> F[1초 타이머]
    F --> G[5초 기준 ONLINE · OFFLINE 계산]
    G --> H[TaskState와 결합]
    H --> I[robot_status_logs INSERT]

    C -- 아니오 --> C1[Task 생성하지 않음]
    C -- 예 --> C2[Task ID와 목적지 검색]
    C2 --> C3[tasks INSERT 또는 UPSERT]

    D --> D1{DOCKED 또는 ERROR?}
    D1 -- 예 --> D2[완료 시각과 결과 기록]
    D1 -- 아니오 --> D3[현재 Task state 기록]
    D2 --> D4[tasks UPSERT]
    D3 --> D4

    E --> DB[(amr.db)]
    I --> DB
    C3 --> DB
    D4 --> DB
```

## 8. deadlock_prevention_node Flow Chart

```mermaid
flowchart TD
    A[/robot5 · robot11 RobotStatus 수신/] --> B[최신 좌표 저장]
    C[/TaskState 수신/] --> D[각 로봇 작업 상태 저장]
    B --> E[0.2초마다 거리 계산]
    D --> E

    E --> F{활성 교착이 이미 있는가?}
    F -- 예 --> G{robot5가 DOCKED · PAUSED · ERROR인가?}
    G -- 예 --> G1[robot11 교착 정지 해제]
    G -- 아니오 --> H{상태 메시지가 정상이고 거리 ≥ 0.9m인가?}
    H -- 예 --> H1[robot11 재개 허가]
    H -- 아니오 --> H2[robot11 정지 유지]

    F -- 아니오 --> I{두 로봇 모두 이동 작업 상태인가?}
    I -- 아니오 --> I1[교착 제어하지 않음]
    I -- 예 --> J{중심 간 거리 < 0.7m인가?}
    J -- 아니오 --> J1[계속 관찰]
    J -- 예 --> K[robot5 진행 허가]
    K --> L[robot11 정지 명령]
    L --> M[활성 교착 상태 저장]

    N[통신 유실 또는 상태 오래됨] --> O{활성 교착 중인가?}
    O -- 예 --> O1[Fail-safe: robot11 정지 유지]
    O -- 아니오 --> O2[판단 보류]

    P[사용자가 교착 중 일시정지] --> Q[정지 원인을 HMI_PAUSE로 변경]
    Q --> R[거리 회복 후에도 자동 재개하지 않음]
```

거리 기준은 로봇 반지름 약 `0.23m`를 가정한 초깃값이다.

```text
접촉 중심 거리: 0.23 + 0.23 = 0.46m
교착 시작 거리: 0.70m
교착 해제 거리: 0.90m
```

`DOCKED` 로봇은 중앙 우선순위 경쟁에서 제외한다. 도킹된 로봇의 물리적 충돌 회피는 각 AMR의 Nav2 Costmap과 장애물 센서가 담당해야 한다.

## 9. 현재 구현과 추후 연결 항목

현재 구현됨:

- AMR 자동 배정과 지도 범위 검사
- Task 상태 머신과 정지 이전 상태 복구
- robot5, robot11 Nav2 Action Client 초안
- HMI 일시정지·재개·도킹·언도킹의 `/task/command` 연결
- Task 상태 DB UPSERT
- 거리 기반 robot5 우선 교착 제어
- 통신 유실 시 robot11 정지 유지

추후 구현 필요:

- 실제 도킹 좌표 설정: 현재 두 로봇 모두 `(0, 0, 0)`
- HMI 작업 취소 API와 `TaskCommand.CANCEL` 연결
- HMI 텔레옵 API와 각 로봇 속도 명령 연결
- 작업자 감지 노드와 `WORKER_DETECTED` 연결
- 로봇 부착 UI와 `START_TRANSPORT`, `DELIVERY_CONFIRMED` 연결
- 실제 로봇 footprint 확인 후 `0.7m / 0.9m` 현장 튜닝
- 도킹된 AMR이 Nav2 Costmap에서 장애물로 인식되는지 검증
