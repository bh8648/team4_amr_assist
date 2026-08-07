# robot5 브릿지 노드 설계

## 배경

중앙 시스템(`real_project/src/robot_manager`)은 5개 노드로 구현되어 있고, robot5·robot11이 각자 다른 컴퓨터에서 Nav2를 띄운다는 전제로 설계되어 있다 (`CENTRAL_SYSTEM_NODE_FLOW.md`). 하지만 로봇/네비게이션 노트북 쪽에서 중앙과 통신할 노드는 이 레포에 존재하지 않는다. `task_manager_node.py`가 기대하는 토픽·액션을 실제로 채워주는 로봇 쪽 브릿지 노드가 필요하다.

이번 범위는 **robot5 전용**이다. robot11은 추후 별도로 작업한다.

## 목표

1. `task_manager_node`가 구독하는 `/robot_status`를 실제 로봇 상태(위치·배터리)로 채워 발행한다.
2. 중앙의 `pause`·`dock` 명령을 실제 로봇 동작(Nav2 goal 취소, Create3 Dock/Undock 액션)으로 연결한다.
3. FOLLOWING 상태에서 사람 추종 목표(`/robot5/target_person_pose`)를 Nav2 goal로 중계한다 (`task_manager_node`는 FOLLOWING일 때 Nav2 goal을 보내지 않으므로, 이 역할은 로봇 쪽에서 담당해야 한다).

## 패키지 구조

새 ROS2 패키지 `real_project/src/robot_bridge`. 기존 중앙 패키지(`robot_manager`)와 분리한다 — 로봇 노트북에는 이 패키지와 메시지 패키지(`robot_status`)만 빌드하면 되고, 중앙 전용 노드(`robot_manager`)는 필요 없다.

- 노드 파일: `robot_bridge/robot5_bridge_node.py`
- executable: `robot5_bridge_node`
- 의존성: `rclpy`, `robot_status`(커스텀 msg), `nav2_msgs`, `irobot_create_msgs`, `std_msgs`, `geometry_msgs`, `sensor_msgs`

## 인터페이스

### 구독

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/robot5/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 위치(x, y)·방향(yaw) 캐싱 |
| `/robot5/battery_state` | `sensor_msgs/BatteryState` | 배터리 캐싱 (`percentage`는 0~1이므로 ×100) |
| `/robot5/pause/request` | `std_msgs/Bool` | `True` → 진행 중인 Nav2 goal 취소 |
| `/robot5/dock/request` | `std_msgs/Bool` | `True` → `/robot5/dock` 호출, `False` → `/robot5/undock` 호출 |
| `/robot5/target_person_pose` | `geometry_msgs/PoseStamped` (frame_id=`map`) | FOLLOWING 상태일 때만 Nav2 goal로 전달 |
| `/task/state` | `robot_status/TaskState` | `robot_id == 'robot5'`인 메시지만 캐싱, 현재 state 추적용 |

`/robot5/target_person_pose`는 이미 `map` 프레임 기준 완성된 좌표이므로 TF 변환이나 픽셀/뎁스 처리가 필요 없다 — 수신한 pose를 그대로 재사용해 Nav2 goal을 만든다.

### 발행

| 토픽 | 타입 | 주기/조건 |
|---|---|---|
| `/robot_status` | `robot_status/RobotStatus` | 1Hz 타이머. `robot_id='robot5'`, `battery`, `x`, `y`, `yaw`는 캐시값, `current_task_id=''`(고정). `amcl_pose`·`battery_state`를 아직 한 번도 못 받았으면 이번 tick은 발행하지 않는다 (0,0 같은 가짜 좌표를 내보내지 않기 위함). QoS는 `db_manager_node`/`dummy_status_publisher`와 동일하게 BEST_EFFORT, depth 10 |

`current_task_id`는 `RobotStatus.msg`에서 제거하지 않는다 — 필드는 그대로 두고 로봇 쪽에 채울 데이터 출처가 없으므로 빈 문자열로 발행한다.

### 액션 클라이언트

| 액션 | 타입 | 트리거 |
|---|---|---|
| `/robot5/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | FOLLOWING 상태에서 새 `target_person_pose` 수신 시 (기존 goal 취소 후 재전송). `pause/request=True` 시 취소만 수행 |
| `/robot5/dock` | `irobot_create_msgs/action/Dock` | `dock/request=True`, 빈 goal(`Dock.Goal()`) — `~/.bashrc`의 `robot-dock` alias(`ros2 action send_goal /robot4/dock irobot_create_msgs/action/Dock "{}"`)와 동일한 방식, robot5 기준 |
| `/robot5/undock` | `irobot_create_msgs/action/Undock` | `dock/request=False`, 빈 goal(`Undock.Goal()`) |

## 동작 로직

- **일시정지(pause)**: `pause/request=True` 수신 시 현재 관리 중인 Nav2 goal handle이 있으면 `cancel_goal_async()`만 호출한다. `cmd_vel`을 별도로 발행하지 않는다 — Nav2가 goal 취소 시 자체적으로 정지한다. `False`(재개) 수신 시 이 노드가 별도로 할 일은 없다 — ASSIGNED/TRANSPORTING/RETURNING의 재전송은 중앙(`task_manager_node`)이 담당하고, FOLLOWING은 다음 `target_person_pose` 수신 시 자연스럽게 재개된다.
- **도킹**: 액션 결과(성공/실패)는 로그만 남긴다. `RobotError` 발행 로직은 이번 범위에서 제외한다(트리거 조건이 아직 정해지지 않아 skip하기로 함).
- **FOLLOWING 추종**: `/task/state`로 캐싱한 robot5의 현재 state가 `'FOLLOWING'`이 아니면 `target_person_pose` 메시지를 무시한다. `'FOLLOWING'`이면 메시지가 올 때마다 (사람이 계속 움직이므로) 기존 Nav2 goal을 취소하고 새 goal을 보낸다 — `task_manager_node.send_navigation_goal(replace=True)`와 동일한 패턴.
- 하나의 `/robot5/navigate_to_pose` ActionClient를 pause 취소와 FOLLOWING goal 전송 양쪽에서 공유하며, 현재 goal handle을 노드 상태로 추적한다.

## 에러 처리

이번 범위에서는 `RobotError`를 발행하지 않는다. 액션 실패(도킹 실패, Nav2 서버 미준비 등)는 로그(`get_logger().warn/error`)로만 남긴다. 트리거 조건(배터리 부족, 충돌 등)이 필요해지면 추후 별도 작업으로 추가한다.

## 테스트 방침

- 순수 로직(quaternion→yaw 추출, `RobotStatus`/Nav2 goal 메시지 구성)은 `target_position_pipeline`의 `test_tf_utils.py`/`test_nav2_goal_builder.py`처럼 ROS 스핀 없이 pytest 유닛 테스트로 검증한다.
- 실제 ROS 그래프 동작(amcl/battery_state 실물 연동, Nav2 액션, Dock/Undock 액션)은 하드웨어 없이는 통합 테스트가 불가능하므로 수동 체크리스트로 대체한다:
  - robot5 노트북에서 노드 실행 후 중앙 PC의 `/robot_status`에 실제 위치·배터리가 찍히는지 확인
  - `pause/request=True` 발행 시 진행 중이던 Nav2 goal이 취소되는지 확인
  - `dock/request` True/False 각각에서 `/robot5/dock`, `/robot5/undock` 액션이 호출되는지 확인
  - FOLLOWING이 아닌 상태에서 `target_person_pose`를 보내도 Nav2 goal이 전송되지 않는지 확인
  - FOLLOWING 상태에서 `target_person_pose`가 갱신될 때마다 Nav2 goal이 교체되는지 확인

## 범위 밖 (다음 작업)

- robot11용 브릿지 (지금은 robot5 하드코딩)
- `RobotError` 발행 트리거 로직
- 도킹 스테이션 정렬(접근 좌표) — 사용자가 별도로 하드코딩 예정
