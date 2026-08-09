# webcam PC CLI 파이프라인 테스트 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `robot_manager` 패키지에 웹캠 PC 역할(작업자 호출 좌표 전달, 추적 완료 감지, 배송 확인)을 대신하는 대화형 CLI 노드(`webcam_pc_cli`)를 추가해, 실물 robot11 + 실물 Nav2로 전체 파이프라인(DOCKED→ASSIGNED→FOLLOWING→TRANSPORTING→RETURNING→DOCKED)을 end-to-end 테스트할 수 있게 한다.

**Architecture:** 단일 ROS2 노드가 rclpy를 데몬 스레드에서 spin시키고 메인 스레드에서 stdin 명령 루프를 돈다. 순수 로직(명령 파싱, 목적지/활성 로봇 선택)은 `webcam_pc_cli_utils.py`로 분리해 ROS 스핀 없이 pytest로 검증하고, 노드는 이 로직을 사용해 `/assignment_goal`·`/task/command`·`/robot11/target_person_pose`를 발행하며 `/robot_assignment`·`/task/state`·`/robot_error`를 구독해 터미널에 피드백한다. 상태 전이 유효성은 CLI가 검증하지 않고 `task_manager_node`에 위임한다.

**Tech Stack:** ROS2 Humble, rclpy, 커스텀 `robot_status` 메시지 패키지, `geometry_msgs/PoseStamped`, sqlite3, pytest.

## Global Constraints

- 이동을 유발하는 CLI 명령(`호출`, `추종시작`, `배송모드`, `배송확인`)은 AI가 단독으로 실행하지 않는다 — 사용자가 로봇 옆에서 비상정지를 쥐고 직접 트리거한다. AI가 자율로 실행해도 되는 것은 CLI 노드 실행 자체와 조회 명령(`목적지목록`, `상태`)뿐이다. (Task 7 참조)
- `webcam_pc_cli`는 `central_system.launch.py`에 포함하지 않는다 — 별도 터미널에서 `ros2 run robot_manager webcam_pc_cli`로 직접 실행한다.
- "배송모드"/"배송 모드" 명령어 매칭은 입력에서 공백을 모두 제거한 뒤 접두사 비교로 처리한다.
- robot_id/task_id 자동 채움의 "활성 로봇" 판정은 `task_manager_node.ACTIVE_STATES`와 동일한 `{'ASSIGNED', 'FOLLOWING', 'TRANSPORTING', 'RETURNING'}` 집합을 사용한다. 상태 전이 유효성 자체는 CLI가 검증하지 않고 `task_manager_node`가 `/robot_error`로 돌려주는 응답에 위임한다.
- DB 접근은 `hmi_backend_node.py`와 동일하게 매 호출마다 `sqlite3.connect()`로 열고 쓴다 (별도 커넥션 풀·락·재시도 없음).
- 하드코딩 좌표(사용자 실측값): FOLLOWING mock 10점은 `x=-1.5`·`yaw=-π/2` 고정, `y`는 `0.5`에서 `-4.0`까지 `0.5`씩 감소. 배송 목적지는 `DEST_A(-0.5, -2, π)`, `DEST_B(-4, -3, 0)`. 도킹 복귀 위치는 `(-2.3, -3.6, -π/2)`. 작업자 호출 예시값은 `(-1, 0)`.
- robot5 관련 로직은 추가하지 않는다 (이번 테스트는 robot11 단일 로봇).

---

## File Structure

```
real_project/src/robot_manager/
  robot_manager/webcam_pc_cli_utils.py   # 순수 함수: 명령 파싱, 목적지/활성 로봇 선택, mock 좌표 상수
  robot_manager/webcam_pc_cli.py         # 메인 노드
  robot_manager/task_manager_node.py     # 기존 파일, robot11_dock_pose 기본값만 수정
  setup.py                               # entry_point 추가
  test/__init__.py                       # 신규
  test/test_webcam_pc_cli_utils.py       # 신규
  test/test_webcam_pc_cli_node.py        # 신규
```

---

### Task 1: 순수 로직 (명령 파싱, 목적지/활성 로봇 선택, mock 좌표 상수)

**Files:**
- Create: `real_project/src/robot_manager/robot_manager/webcam_pc_cli_utils.py`
- Create: `real_project/src/robot_manager/test/__init__.py`
- Create: `real_project/src/robot_manager/test/test_webcam_pc_cli_utils.py`

**Interfaces:**
- Produces:
  - `ACTIVE_STATES: Set[str]`
  - `FOLLOWING_MOCK_POSES: List[Tuple[float, float, float]]` (10개, `(x, y, yaw)`)
  - `parse_command(raw_input: str) -> Tuple[str, List[str]]`
  - `parse_call_args(args: List[str]) -> Tuple[Optional[Tuple[float, float]], Optional[str]]`
  - `parse_interval(args: List[str], default: float = 3.0) -> Tuple[Optional[float], Optional[str]]`
  - `select_destination(destinations: List[dict], requested_id: Optional[str]) -> Tuple[Optional[dict], Optional[str]]`
  - `select_active_robot(task_states: Dict[str, str]) -> Tuple[Optional[str], Optional[str]]`

- [ ] **Step 1: 빈 `test/__init__.py` 생성**

`real_project/src/robot_manager/test/__init__.py` — 빈 파일.

- [ ] **Step 2: 실패하는 테스트 작성**

```python
import math

from robot_manager.webcam_pc_cli_utils import (
    FOLLOWING_MOCK_POSES,
    parse_call_args,
    parse_command,
    parse_interval,
    select_active_robot,
    select_destination,
)


def test_parse_command_deliver_without_internal_space():
    assert parse_command('배송모드') == ('배송모드', [])


def test_parse_command_deliver_with_internal_space():
    assert parse_command('배송 모드') == ('배송모드', [])


def test_parse_command_deliver_with_destination_id():
    assert parse_command('배송모드 DEST_A') == ('배송모드', ['DEST_A'])


def test_parse_command_deliver_with_space_and_destination_id():
    assert parse_command('배송 모드 DEST_A') == ('배송모드', ['DEST_A'])


def test_parse_command_simple_word_no_args():
    assert parse_command('상태') == ('상태', [])


def test_parse_command_with_args():
    assert parse_command('호출 -1 0') == ('호출', ['-1', '0'])


def test_parse_command_empty_input_returns_empty_command():
    assert parse_command('   ') == ('', [])


def test_parse_call_args_valid_floats():
    assert parse_call_args(['-1', '0']) == ((-1.0, 0.0), None)


def test_parse_call_args_wrong_arg_count():
    value, error = parse_call_args(['-1'])
    assert value is None
    assert error == '사용법: 호출 <x> <y>'


def test_parse_call_args_non_numeric():
    value, error = parse_call_args(['a', 'b'])
    assert value is None
    assert error == 'x, y는 숫자여야 합니다'


def test_parse_interval_default_when_no_args():
    assert parse_interval([]) == (3.0, None)


def test_parse_interval_valid_value():
    assert parse_interval(['5']) == (5.0, None)


def test_parse_interval_rejects_zero():
    value, error = parse_interval(['0'])
    assert value is None
    assert error == '간격초는 양수여야 합니다'


def test_parse_interval_rejects_negative():
    value, error = parse_interval(['-1'])
    assert value is None
    assert error == '간격초는 양수여야 합니다'


def test_parse_interval_rejects_non_numeric():
    value, error = parse_interval(['abc'])
    assert value is None
    assert error == '간격초는 숫자여야 합니다'


def test_select_destination_found_by_id():
    destinations = [
        {'destination_id': 'DEST_A', 'position_x': -0.5, 'position_y': -2.0, 'orientation_yaw': math.pi},
        {'destination_id': 'DEST_B', 'position_x': -4.0, 'position_y': -3.0, 'orientation_yaw': 0.0},
    ]
    destination, error = select_destination(destinations, 'DEST_B')
    assert error is None
    assert destination['destination_id'] == 'DEST_B'


def test_select_destination_not_found_by_id():
    destinations = [{'destination_id': 'DEST_A', 'position_x': 0.0, 'position_y': 0.0, 'orientation_yaw': 0.0}]
    destination, error = select_destination(destinations, 'DEST_Z')
    assert destination is None
    assert error == '목적지 없음: DEST_Z'


def test_select_destination_auto_selects_when_single():
    destinations = [{'destination_id': 'DEST_A', 'position_x': 0.0, 'position_y': 0.0, 'orientation_yaw': 0.0}]
    destination, error = select_destination(destinations, None)
    assert error is None
    assert destination['destination_id'] == 'DEST_A'


def test_select_destination_requires_id_when_multiple():
    destinations = [
        {'destination_id': 'DEST_A', 'position_x': 0.0, 'position_y': 0.0, 'orientation_yaw': 0.0},
        {'destination_id': 'DEST_B', 'position_x': 0.0, 'position_y': 0.0, 'orientation_yaw': 0.0},
    ]
    destination, error = select_destination(destinations, None)
    assert destination is None
    assert 'DEST_A' in error and 'DEST_B' in error


def test_select_destination_empty_list():
    destination, error = select_destination([], None)
    assert destination is None
    assert error == '등록된 목적지 없음'


def test_select_active_robot_single_active():
    robot_id, error = select_active_robot({'robot11': 'FOLLOWING'})
    assert error is None
    assert robot_id == 'robot11'


def test_select_active_robot_none_active():
    robot_id, error = select_active_robot({})
    assert robot_id is None
    assert error == '활성 작업 없음'


def test_select_active_robot_ignores_docked_and_error():
    robot_id, error = select_active_robot({'robot11': 'DOCKED', 'robot5': 'ERROR'})
    assert robot_id is None
    assert error == '활성 작업 없음'


def test_select_active_robot_multiple_active_requires_selection():
    robot_id, error = select_active_robot({'robot11': 'FOLLOWING', 'robot5': 'ASSIGNED'})
    assert robot_id is None
    assert 'robot11' in error and 'robot5' in error


def test_following_mock_poses_has_ten_points():
    assert len(FOLLOWING_MOCK_POSES) == 10


def test_following_mock_poses_x_and_yaw_are_constant():
    for x, _y, yaw in FOLLOWING_MOCK_POSES:
        assert x == -1.5
        assert math.isclose(yaw, -math.pi / 2)


def test_following_mock_poses_y_decreases_from_0_5_to_minus_4():
    ys = [y for _x, y, _yaw in FOLLOWING_MOCK_POSES]
    assert ys[0] == 0.5
    assert ys[-1] == -4.0
    assert ys == sorted(ys, reverse=True)
```

- [ ] **Step 3: 테스트 실행해서 실패 확인**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
python3 -m pytest src/robot_manager/test/test_webcam_pc_cli_utils.py -v
```
Expected: FAIL (`ModuleNotFoundError: No module named 'robot_manager.webcam_pc_cli_utils'`)

- [ ] **Step 4: 최소 구현 작성**

```python
import math
from typing import Dict, List, Optional, Tuple

ACTIVE_STATES = {'ASSIGNED', 'FOLLOWING', 'TRANSPORTING', 'RETURNING'}

DELIVER_COMMAND = '배송모드'

# 사용자가 확인한 실측 경로: x=-1.5, yaw=-pi/2 고정, y만 0.5에서 -4.0까지 0.5씩 감소
FOLLOWING_MOCK_POSES: List[Tuple[float, float, float]] = [
    (-1.5, 0.5, -math.pi / 2),
    (-1.5, 0.0, -math.pi / 2),
    (-1.5, -0.5, -math.pi / 2),
    (-1.5, -1.0, -math.pi / 2),
    (-1.5, -1.5, -math.pi / 2),
    (-1.5, -2.0, -math.pi / 2),
    (-1.5, -2.5, -math.pi / 2),
    (-1.5, -3.0, -math.pi / 2),
    (-1.5, -3.5, -math.pi / 2),
    (-1.5, -4.0, -math.pi / 2),
]


def parse_command(raw_input: str) -> Tuple[str, List[str]]:
    """stdin 한 줄을 (명령 키워드, 인자 리스트)로 분리한다.

    "배송모드"/"배송 모드"는 공백을 모두 제거한 뒤 접두사로 인식한다.
    """
    text = raw_input.strip()
    compact = text.replace(' ', '')
    if compact.startswith(DELIVER_COMMAND):
        remainder = compact[len(DELIVER_COMMAND):]
        return DELIVER_COMMAND, ([remainder] if remainder else [])
    parts = text.split()
    if not parts:
        return '', []
    return parts[0], parts[1:]


def parse_call_args(args: List[str]) -> Tuple[Optional[Tuple[float, float]], Optional[str]]:
    if len(args) != 2:
        return None, '사용법: 호출 <x> <y>'
    try:
        x, y = float(args[0]), float(args[1])
    except ValueError:
        return None, 'x, y는 숫자여야 합니다'
    return (x, y), None


def parse_interval(args: List[str], default: float = 3.0) -> Tuple[Optional[float], Optional[str]]:
    if not args:
        return default, None
    try:
        value = float(args[0])
    except ValueError:
        return None, '간격초는 숫자여야 합니다'
    if value <= 0:
        return None, '간격초는 양수여야 합니다'
    return value, None


def select_destination(destinations: List[dict], requested_id: Optional[str]) -> Tuple[Optional[dict], Optional[str]]:
    if requested_id:
        for destination in destinations:
            if destination['destination_id'] == requested_id:
                return destination, None
        return None, f'목적지 없음: {requested_id}'
    if not destinations:
        return None, '등록된 목적지 없음'
    if len(destinations) == 1:
        return destinations[0], None
    ids = ', '.join(destination['destination_id'] for destination in destinations)
    return None, f'목적지를 지정하세요: {ids}'


def select_active_robot(task_states: Dict[str, str]) -> Tuple[Optional[str], Optional[str]]:
    active = [robot_id for robot_id, state in task_states.items() if state in ACTIVE_STATES]
    if not active:
        return None, '활성 작업 없음'
    if len(active) == 1:
        return active[0], None
    return None, f'로봇을 지정하세요: {", ".join(sorted(active))}'
```

- [ ] **Step 5: 테스트 실행해서 통과 확인**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
python3 -m pytest src/robot_manager/test/test_webcam_pc_cli_utils.py -v
```
Expected: PASS (27 passed)

- [ ] **Step 6: Commit**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/src/robot_manager/robot_manager/webcam_pc_cli_utils.py real_project/src/robot_manager/test/__init__.py real_project/src/robot_manager/test/test_webcam_pc_cli_utils.py
git commit -m "robot_manager: webcam_pc_cli 순수 로직(명령 파싱/목적지·로봇 선택) 추가"
```

---

### Task 2: 노드 스캐폴딩 (구독 콜백 + destinations 조회)

**Files:**
- Create: `real_project/src/robot_manager/robot_manager/webcam_pc_cli.py`
- Create: `real_project/src/robot_manager/test/test_webcam_pc_cli_node.py`

**Interfaces:**
- Produces:
  - `WebcamPcCliNode` (rclpy `Node` 서브클래스)
  - `WebcamPcCliNode.task_cache: Dict[str, TaskState]`
  - `WebcamPcCliNode.assignment_callback(msg: RobotAssignment) -> None`
  - `WebcamPcCliNode.task_state_callback(msg: TaskState) -> None`
  - `WebcamPcCliNode.error_callback(msg: RobotError) -> None`
  - `WebcamPcCliNode.fetch_destinations() -> List[dict]`
  - `main(args=None)` 진입점 (이후 Task에서 내용이 바뀐다)

- [ ] **Step 1: 실패하는 테스트 작성**

```python
import os
import sqlite3
import tempfile

import rclpy

from robot_status.msg import RobotAssignment, RobotError, TaskState

from robot_manager.webcam_pc_cli import WebcamPcCliNode


def _make_temp_db():
    fd, path = tempfile.mkstemp(suffix='.db')
    os.close(fd)
    conn = sqlite3.connect(path)
    conn.execute('''
        CREATE TABLE destinations (
            destination_id   TEXT PRIMARY KEY,
            destination_name TEXT NOT NULL,
            position_x       REAL NOT NULL,
            position_y       REAL NOT NULL,
            orientation_yaw  REAL NOT NULL
        )
    ''')
    conn.commit()
    conn.close()
    return path


def test_assignment_callback_prints_success(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        msg = RobotAssignment()
        msg.assigned, msg.robot_id, msg.target_x, msg.target_y = True, 'robot11', -1.0, 0.0
        node.assignment_callback(msg)
        assert '배정 성공' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_assignment_callback_prints_failure(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        msg = RobotAssignment()
        msg.assigned = False
        node.assignment_callback(msg)
        assert '배정 실패' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_task_state_callback_caches_per_robot_id():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        msg = TaskState()
        msg.robot_id, msg.state, msg.task_id = 'robot11', 'FOLLOWING', 'TASK_1'
        node.task_state_callback(msg)
        assert node.task_cache['robot11'].state == 'FOLLOWING'
        assert node.task_cache['robot11'].task_id == 'TASK_1'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_error_callback_prints(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        msg = RobotError()
        msg.robot_id, msg.task_id, msg.error_code = 'robot11', 'TASK_1', 'NAV_GOAL_REJECTED'
        node.error_callback(msg)
        assert 'NAV_GOAL_REJECTED' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_fetch_destinations_reads_rows_from_db():
    rclpy.init()
    node = WebcamPcCliNode()
    db_path = _make_temp_db()
    try:
        conn = sqlite3.connect(db_path)
        conn.execute("INSERT INTO destinations VALUES ('DEST_A', '목적지 A', -0.5, -2.0, 3.14159)")
        conn.commit()
        conn.close()
        node.db_path = db_path

        destinations = node.fetch_destinations()

        assert len(destinations) == 1
        assert destinations[0]['destination_id'] == 'DEST_A'
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.remove(db_path)


def test_fetch_destinations_empty_when_table_empty():
    rclpy.init()
    node = WebcamPcCliNode()
    db_path = _make_temp_db()
    try:
        node.db_path = db_path
        assert node.fetch_destinations() == []
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.remove(db_path)
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/robot_manager/test/test_webcam_pc_cli_node.py -v
```
Expected: FAIL (`ModuleNotFoundError: No module named 'robot_manager.webcam_pc_cli'`)

- [ ] **Step 3: 노드 스캐폴딩 작성**

```python
#!/usr/bin/env python3
import os
import sqlite3
from typing import Dict, List

import rclpy
from rclpy.node import Node

from robot_status.msg import AssignmentGoal, RobotAssignment, RobotError, TaskCommand, TaskState


class WebcamPcCliNode(Node):
    def __init__(self):
        super().__init__('webcam_pc_cli_node')
        default_db_path = os.path.abspath('amr.db')
        self.declare_parameter('db_path', os.environ.get('AMR_DB_PATH', default_db_path))
        self.db_path = os.path.abspath(os.path.expanduser(self.get_parameter('db_path').value))

        self.task_cache: Dict[str, TaskState] = {}

        self.assignment_goal_pub = self.create_publisher(AssignmentGoal, '/assignment_goal', 10)
        self.task_command_pub = self.create_publisher(TaskCommand, '/task/command', 10)

        self.assignment_sub = self.create_subscription(
            RobotAssignment, '/robot_assignment', self.assignment_callback, 10)
        self.task_state_sub = self.create_subscription(
            TaskState, '/task/state', self.task_state_callback, 10)
        self.error_sub = self.create_subscription(
            RobotError, '/robot_error', self.error_callback, 10)

        self.get_logger().info('webcam_pc_cli 노드 시작')

    def assignment_callback(self, msg: RobotAssignment) -> None:
        if msg.assigned:
            print(f'[배정 성공] robot_id={msg.robot_id}, 목표=({msg.target_x:.2f}, {msg.target_y:.2f})')
        else:
            print(f'[배정 실패] 목표=({msg.target_x:.2f}, {msg.target_y:.2f})')

    def task_state_callback(self, msg: TaskState) -> None:
        self.task_cache[msg.robot_id] = msg

    def error_callback(self, msg: RobotError) -> None:
        print(f'[오류] robot_id={msg.robot_id}, task_id={msg.task_id}, error_code={msg.error_code}')

    def fetch_destinations(self) -> List[dict]:
        with sqlite3.connect(self.db_path) as conn:
            conn.row_factory = sqlite3.Row
            rows = conn.execute('SELECT * FROM destinations').fetchall()
        return [dict(row) for row in rows]


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPcCliNode()
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
```

- [ ] **Step 4: 테스트 실행해서 통과 확인**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/robot_manager/test/test_webcam_pc_cli_node.py -v
```
Expected: PASS (6 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/src/robot_manager/robot_manager/webcam_pc_cli.py real_project/src/robot_manager/test/test_webcam_pc_cli_node.py
git commit -m "robot_manager: webcam_pc_cli 노드 스캐폴딩 (구독 콜백 + destinations 조회) 추가"
```

---

### Task 3: 명령 핸들러와 대화형 루프

**Files:**
- Modify: `real_project/src/robot_manager/robot_manager/webcam_pc_cli.py`
- Modify: `real_project/src/robot_manager/test/test_webcam_pc_cli_node.py`

**Interfaces:**
- Consumes: Task 1의 `parse_call_args`, `parse_command`, `select_active_robot`, `select_destination`. Task 2의 `WebcamPcCliNode.fetch_destinations`, `task_cache`.
- Produces:
  - `WebcamPcCliNode.cmd_call(args) -> None`, `cmd_list_destinations(args) -> None`, `cmd_worker_detected(args) -> None`, `cmd_deliver(args) -> None`, `cmd_confirm(args) -> None`, `cmd_status(args) -> None`, `cmd_quit(args)` (반환값 `QUIT` 센티널)
  - `WebcamPcCliNode.run_cli() -> None` (Task 4에서 handlers 딕셔너리에 추종 명령이 추가된다)

- [ ] **Step 1: 실패하는 테스트 작성**

`test_webcam_pc_cli_node.py` 상단 import에 추가:
```python
from unittest.mock import Mock
```

파일 하단에 추가:
```python
def test_cmd_call_publishes_assignment_goal():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.assignment_goal_pub = Mock()
        node.cmd_call(['-1', '0'])
        node.assignment_goal_pub.publish.assert_called_once()
        sent = node.assignment_goal_pub.publish.call_args[0][0]
        assert sent.x == -1.0
        assert sent.y == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_call_invalid_args_does_not_publish():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.assignment_goal_pub = Mock()
        node.cmd_call(['only-one'])
        node.assignment_goal_pub.publish.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_list_destinations_prints_rows(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    db_path = _make_temp_db()
    try:
        conn = sqlite3.connect(db_path)
        conn.execute("INSERT INTO destinations VALUES ('DEST_A', '목적지 A', -0.5, -2.0, 3.14159)")
        conn.commit()
        conn.close()
        node.db_path = db_path

        node.cmd_list_destinations([])

        assert 'DEST_A' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.remove(db_path)


def test_cmd_worker_detected_publishes_with_active_robot():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.task_command_pub = Mock()
        state = TaskState()
        state.robot_id, state.state, state.task_id = 'robot11', 'ASSIGNED', 'TASK_1'
        node.task_state_callback(state)

        node.cmd_worker_detected([])

        node.task_command_pub.publish.assert_called_once()
        sent = node.task_command_pub.publish.call_args[0][0]
        assert sent.command == 'WORKER_DETECTED'
        assert sent.robot_id == 'robot11'
        assert sent.task_id == 'TASK_1'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_worker_detected_no_active_robot_does_not_publish():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.task_command_pub = Mock()
        node.cmd_worker_detected([])
        node.task_command_pub.publish.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_deliver_publishes_with_destination_coords():
    rclpy.init()
    node = WebcamPcCliNode()
    db_path = _make_temp_db()
    try:
        conn = sqlite3.connect(db_path)
        conn.execute("INSERT INTO destinations VALUES ('DEST_A', '목적지 A', -0.5, -2.0, 3.14159)")
        conn.commit()
        conn.close()
        node.db_path = db_path
        node.task_command_pub = Mock()
        state = TaskState()
        state.robot_id, state.state, state.task_id = 'robot11', 'FOLLOWING', 'TASK_1'
        node.task_state_callback(state)

        node.cmd_deliver(['DEST_A'])

        sent = node.task_command_pub.publish.call_args[0][0]
        assert sent.command == 'START_TRANSPORT'
        assert sent.target_x == -0.5
        assert sent.target_y == -2.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.remove(db_path)


def test_cmd_deliver_unknown_destination_does_not_publish():
    rclpy.init()
    node = WebcamPcCliNode()
    db_path = _make_temp_db()
    try:
        node.db_path = db_path
        node.task_command_pub = Mock()
        state = TaskState()
        state.robot_id, state.state, state.task_id = 'robot11', 'FOLLOWING', 'TASK_1'
        node.task_state_callback(state)

        node.cmd_deliver(['DEST_Z'])

        node.task_command_pub.publish.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.remove(db_path)


def test_cmd_confirm_publishes():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.task_command_pub = Mock()
        state = TaskState()
        state.robot_id, state.state, state.task_id = 'robot11', 'TRANSPORTING', 'TASK_1'
        node.task_state_callback(state)

        node.cmd_confirm([])

        sent = node.task_command_pub.publish.call_args[0][0]
        assert sent.command == 'DELIVERY_CONFIRMED'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_status_prints_cached_states(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        state = TaskState()
        state.robot_id, state.state, state.task_id = 'robot11', 'FOLLOWING', 'TASK_1'
        node.task_state_callback(state)

        node.cmd_status([])

        assert 'robot11' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_run_cli_dispatches_status_then_quits(monkeypatch, capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        inputs = iter(['상태', '종료'])
        monkeypatch.setattr('builtins.input', lambda _prompt='': next(inputs))

        node.run_cli()

        assert '캐싱된 로봇 상태 없음' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_run_cli_unknown_command_prints_error(monkeypatch, capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        inputs = iter(['이상한명령', '종료'])
        monkeypatch.setattr('builtins.input', lambda _prompt='': next(inputs))

        node.run_cli()

        assert '알 수 없는 명령' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

Run: `python3 -m pytest src/robot_manager/test/test_webcam_pc_cli_node.py -v` (먼저 `source /opt/ros/humble/setup.bash && source install/setup.bash`)
Expected: FAIL (`AttributeError: 'WebcamPcCliNode' object has no attribute 'cmd_call'`)

- [ ] **Step 3: 명령 핸들러와 run_cli 추가**

`webcam_pc_cli.py`의 import 블록을 다음으로 교체:
```python
#!/usr/bin/env python3
import os
import sqlite3
import threading
from typing import Dict, List

import rclpy
from rclpy.node import Node

from robot_status.msg import AssignmentGoal, RobotAssignment, RobotError, TaskCommand, TaskState

from robot_manager.webcam_pc_cli_utils import (
    parse_call_args,
    parse_command,
    select_active_robot,
    select_destination,
)

QUIT = object()
```

클래스 안, `fetch_destinations` 메서드 다음에 추가:
```python
    def cmd_call(self, args: List[str]) -> None:
        parsed, error = parse_call_args(args)
        if error:
            print(error)
            return
        x, y = parsed
        goal = AssignmentGoal()
        goal.x, goal.y = float(x), float(y)
        self.assignment_goal_pub.publish(goal)
        print(f'[호출] AssignmentGoal(x={x}, y={y}) 발행')

    def cmd_list_destinations(self, args: List[str]) -> None:
        destinations = self.fetch_destinations()
        if not destinations:
            print('등록된 목적지 없음')
            return
        for destination in destinations:
            print(f"{destination['destination_id']}: {destination['destination_name']} "
                  f"({destination['position_x']}, {destination['position_y']}, {destination['orientation_yaw']})")

    def _active_robot_task(self):
        task_states = {robot_id: msg.state for robot_id, msg in self.task_cache.items()}
        robot_id, error = select_active_robot(task_states)
        if error:
            print(error)
            return None, None
        return robot_id, self.task_cache[robot_id].task_id

    def _publish_task_command(self, command: str, robot_id: str, task_id: str,
                               target_x: float = 0.0, target_y: float = 0.0, target_yaw: float = 0.0) -> None:
        msg = TaskCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command, msg.robot_id, msg.task_id = command, robot_id, task_id
        msg.target_x, msg.target_y, msg.target_yaw = target_x, target_y, target_yaw
        self.task_command_pub.publish(msg)

    def cmd_worker_detected(self, args: List[str]) -> None:
        robot_id, task_id = self._active_robot_task()
        if robot_id is None:
            return
        self._publish_task_command('WORKER_DETECTED', robot_id, task_id)
        print(f'[작업자감지] robot_id={robot_id} 발행')

    def cmd_deliver(self, args: List[str]) -> None:
        robot_id, task_id = self._active_robot_task()
        if robot_id is None:
            return
        requested_id = args[0] if args else None
        destination, error = select_destination(self.fetch_destinations(), requested_id)
        if error:
            print(error)
            return
        self._publish_task_command(
            'START_TRANSPORT', robot_id, task_id,
            destination['position_x'], destination['position_y'], destination['orientation_yaw'])
        print(f"[배송모드] robot_id={robot_id}, 목적지={destination['destination_id']} 발행")

    def cmd_confirm(self, args: List[str]) -> None:
        robot_id, task_id = self._active_robot_task()
        if robot_id is None:
            return
        self._publish_task_command('DELIVERY_CONFIRMED', robot_id, task_id)
        print(f'[배송확인] robot_id={robot_id} 발행')

    def cmd_status(self, args: List[str]) -> None:
        if not self.task_cache:
            print('캐싱된 로봇 상태 없음')
            return
        for robot_id, msg in self.task_cache.items():
            print(f'{robot_id}: {msg.state} (task_id={msg.task_id})')

    def cmd_quit(self, args: List[str]):
        return QUIT

    def run_cli(self) -> None:
        handlers = {
            '호출': self.cmd_call,
            '목적지목록': self.cmd_list_destinations,
            '작업자감지': self.cmd_worker_detected,
            '배송모드': self.cmd_deliver,
            '배송확인': self.cmd_confirm,
            '상태': self.cmd_status,
            '종료': self.cmd_quit,
        }
        print('webcam_pc_cli 준비 완료. 명령: 호출/목적지목록/작업자감지/배송모드/배송확인/상태/종료')
        while rclpy.ok():
            try:
                line = input('> ')
            except EOFError:
                break
            command, args = parse_command(line)
            if not command:
                continue
            handler = handlers.get(command)
            if handler is None:
                print(f'알 수 없는 명령: {command}')
                continue
            if handler(args) is QUIT:
                break
```

`main()`을 다음으로 교체:
```python
def main(args=None):
    rclpy.init(args=args)
    node = WebcamPcCliNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_cli()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

- [ ] **Step 4: 테스트 실행해서 통과 확인**

Run: `python3 -m pytest src/robot_manager/test/test_webcam_pc_cli_node.py -v`
Expected: PASS (17 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/src/robot_manager/robot_manager/webcam_pc_cli.py real_project/src/robot_manager/test/test_webcam_pc_cli_node.py
git commit -m "robot_manager: webcam_pc_cli 명령 핸들러와 대화형 루프 추가"
```

---

### Task 4: FOLLOWING mock 좌표 순차 발행

**Files:**
- Modify: `real_project/src/robot_manager/robot_manager/webcam_pc_cli.py`
- Modify: `real_project/src/robot_manager/test/test_webcam_pc_cli_node.py`

**Interfaces:**
- Consumes: Task 1의 `FOLLOWING_MOCK_POSES`, `parse_interval`.
- Produces: `WebcamPcCliNode.cmd_follow_start(args) -> None`, `cmd_follow_stop(args) -> None`, `WebcamPcCliNode.following_timer`, `WebcamPcCliNode.following_index`

- [ ] **Step 1: 실패하는 테스트 작성**

파일 하단에 추가:
```python
def test_cmd_follow_start_creates_timer():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.cmd_follow_start([])
        assert node.following_timer is not None
        assert node.following_index == 0
    finally:
        node.cmd_follow_stop([])
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_follow_start_rejects_invalid_interval():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.cmd_follow_start(['-1'])
        assert node.following_timer is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_follow_start_rejects_duplicate_start(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.cmd_follow_start(['100'])
        first_timer = node.following_timer

        node.cmd_follow_start(['100'])

        assert node.following_timer is first_timer
        assert '이미 진행 중' in capsys.readouterr().out
    finally:
        node.cmd_follow_stop([])
        node.destroy_node()
        rclpy.shutdown()


def test_publish_next_following_pose_publishes_correct_pose_and_increments():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.target_pose_pub = Mock()
        node.following_index = 0

        node._publish_next_following_pose()

        node.target_pose_pub.publish.assert_called_once()
        sent = node.target_pose_pub.publish.call_args[0][0]
        assert sent.header.frame_id == 'map'
        assert sent.pose.position.x == -1.5
        assert sent.pose.position.y == 0.5
        assert node.following_index == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_publish_next_following_pose_stops_after_ten_points():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.target_pose_pub = Mock()
        node._stop_following_timer = Mock()
        node.following_index = 10

        node._publish_next_following_pose()

        node.target_pose_pub.publish.assert_not_called()
        node._stop_following_timer.assert_called_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_follow_stop_cancels_timer():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.cmd_follow_start(['100'])
        assert node.following_timer is not None

        node.cmd_follow_stop([])

        assert node.following_timer is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_follow_stop_noop_when_not_running():
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.cmd_follow_stop([])  # 예외 없이 통과해야 함
        assert node.following_timer is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

Run: `python3 -m pytest src/robot_manager/test/test_webcam_pc_cli_node.py -v`
Expected: FAIL (`AttributeError: 'WebcamPcCliNode' object has no attribute 'cmd_follow_start'`)

- [ ] **Step 3: FOLLOWING mock 순차 발행 로직 추가**

import 블록에 추가 (`from robot_manager.webcam_pc_cli_utils import (...)` 안에 `FOLLOWING_MOCK_POSES`, `parse_interval` 추가, 그리고 새 import 2줄 추가):
```python
import math
```
```python
from geometry_msgs.msg import PoseStamped
```
```python
from robot_manager.webcam_pc_cli_utils import (
    FOLLOWING_MOCK_POSES,
    parse_call_args,
    parse_command,
    parse_interval,
    select_active_robot,
    select_destination,
)
```

`__init__` 안, `self.task_command_pub = ...` 다음 줄에 추가:
```python
        self.target_pose_pub = self.create_publisher(PoseStamped, '/robot11/target_person_pose', 10)
```

`__init__` 안, `self.task_cache: Dict[str, TaskState] = {}` 다음 줄에 추가:
```python
        self.following_timer = None
        self.following_index = 0
```

클래스에 새 메서드 추가 (`cmd_status` 다음, `cmd_quit` 이전):
```python
    def _stop_following_timer(self) -> None:
        if self.following_timer is not None:
            self.destroy_timer(self.following_timer)
            self.following_timer = None

    def cmd_follow_start(self, args: List[str]) -> None:
        interval, error = parse_interval(args)
        if error:
            print(error)
            return
        if self.following_timer is not None:
            print('이미 진행 중입니다. 먼저 추종중지를 입력하세요')
            return
        self.following_index = 0
        self.following_timer = self.create_timer(interval, self._publish_next_following_pose)
        print(f'[추종시작] 간격초={interval}')

    def cmd_follow_stop(self, args: List[str]) -> None:
        if self.following_timer is None:
            return
        self._stop_following_timer()
        print('[추종중지]')

    def _publish_next_following_pose(self) -> None:
        if self.following_index >= len(FOLLOWING_MOCK_POSES):
            self._stop_following_timer()
            print('[추종완료] mock 좌표 10개 발행 종료')
            return
        x, y, yaw = FOLLOWING_MOCK_POSES[self.following_index]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self.target_pose_pub.publish(msg)
        print(f'[추종 {self.following_index + 1}/{len(FOLLOWING_MOCK_POSES)}] ({x}, {y}, {yaw:.3f})')
        self.following_index += 1
```

`run_cli`의 `handlers` 딕셔너리에 두 줄 추가 (`'작업자감지': self.cmd_worker_detected,` 다음):
```python
            '추종시작': self.cmd_follow_start,
            '추종중지': self.cmd_follow_stop,
```
같은 메서드의 안내 출력 문자열도 갱신:
```python
        print('webcam_pc_cli 준비 완료. 명령: 호출/목적지목록/작업자감지/추종시작/추종중지/배송모드/배송확인/상태/종료')
```

- [ ] **Step 4: 테스트 실행해서 통과 확인**

Run: `python3 -m pytest src/robot_manager/test/test_webcam_pc_cli_node.py -v`
Expected: PASS (24 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/src/robot_manager/robot_manager/webcam_pc_cli.py real_project/src/robot_manager/test/test_webcam_pc_cli_node.py
git commit -m "robot_manager: webcam_pc_cli FOLLOWING mock 좌표 순차 발행 추가"
```

---

### Task 5: 패키지 등록 + 빌드 검증

**Files:**
- Modify: `real_project/src/robot_manager/setup.py`

**Interfaces:**
- Consumes: `robot_manager` 패키지의 `webcam_pc_cli` 모듈 (Task 1~4)

- [ ] **Step 1: entry_point 추가**

`setup.py`의 `entry_points`를 다음으로 교체:
```python
    entry_points={
        'console_scripts': [
            'db_manager_node = robot_manager.db_manager_node:main',
            'hmi_backend_node = robot_manager.hmi_backend_node:main',
            'robot_assignment_node = robot_manager.robot_assignment_node:main',
            'task_manager_node = robot_manager.task_manager_node:main',
            'deadlock_prevention_node = robot_manager.deadlock_prevention_node:main',
            'dummy_publisher = robot_manager.dummy_status_publisher:main',
            'webcam_pc_cli = robot_manager.webcam_pc_cli:main',
        ],
    },
```

- [ ] **Step 2: 전체 pytest 재실행 (회귀 확인)**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/robot_manager/test/ -v
```
Expected: PASS (51 passed — utils 27 + node 24)

- [ ] **Step 3: colcon build + executable 등록 확인**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
colcon build --packages-select robot_manager
source install/setup.bash
ros2 pkg executables robot_manager | grep webcam_pc_cli
```
Expected: 빌드 성공, `robot_manager webcam_pc_cli` 출력.

- [ ] **Step 4: 실행 후 즉시 종료로 크래시 여부만 확인**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
echo '종료' | timeout 5 ros2 run robot_manager webcam_pc_cli
```
Expected: `webcam_pc_cli 준비 완료...` 안내가 출력되고, `종료` 입력 후 정상 종료(에러 트레이스백 없음). `/assignment_goal` 등 실제 배정을 유발하지 않는 안전한 smoke test다.

- [ ] **Step 5: Commit**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/src/robot_manager/setup.py
git commit -m "robot_manager: webcam_pc_cli entry_point 등록"
```

---

### Task 6: destinations 시드 + robot11 도킹 복귀 위치 반영

**Files:**
- Modify: `real_project/amr.db` (destinations 테이블에 2행 INSERT)
- Modify: `real_project/src/robot_manager/robot_manager/task_manager_node.py`

**Interfaces:** 없음 (데이터/상수 값 변경)

- [ ] **Step 1: destinations 테이블에 실측 목적지 2행 INSERT**

`sqlite3` CLI가 설치되어 있지 않으므로 python3의 `sqlite3` 모듈로 실행한다.

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
python3 -c "
import sqlite3
conn = sqlite3.connect('amr.db')
conn.execute('''
    INSERT INTO destinations (destination_id, destination_name, position_x, position_y, orientation_yaw)
    VALUES (?, ?, ?, ?, ?), (?, ?, ?, ?, ?)
''', ('DEST_A', '목적지 A', -0.5, -2, 3.141592653589793, 'DEST_B', '목적지 B', -4, -3, 0.0))
conn.commit()
conn.close()
"
python3 -c "
import sqlite3
conn = sqlite3.connect('amr.db')
for row in conn.execute('SELECT * FROM destinations'):
    print(row)
"
```
Expected: 두 번째 명령 출력에 `('DEST_A', '목적지 A', -0.5, -2.0, 3.141592653589793)`와 `('DEST_B', '목적지 B', -4.0, -3.0, 0.0)` 두 행이 보인다.

- [ ] **Step 2: task_manager_node.py의 robot11_dock_pose 기본값을 실측 도킹 위치로 변경**

`real_project/src/robot_manager/robot_manager/task_manager_node.py`에서 (`__init__` 안, 8~9번째 줄 부근):

```python
        self.declare_parameter('robot5_dock_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('robot11_dock_pose', [0.0, 0.0, 0.0])
```

`robot11_dock_pose` 줄만 다음으로 교체 (`robot5_dock_pose`는 건드리지 않음):
```python
        self.declare_parameter('robot5_dock_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('robot11_dock_pose', [-2.3, -3.6, -math.pi / 2])
```

`task_manager_node.py` 최상단은 이미 `import math`가 있으므로 추가 import는 필요 없다.

- [ ] **Step 3: 값이 정확히 반영됐는지 파이썬으로 확인**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
python3 -c "
import re
text = open('src/robot_manager/robot_manager/task_manager_node.py', encoding='utf-8').read()
assert \"robot11_dock_pose', [-2.3, -3.6, -math.pi / 2]\" in text, 'robot11_dock_pose 기본값이 반영되지 않음'
assert \"robot5_dock_pose', [0.0, 0.0, 0.0]\" in text, 'robot5_dock_pose가 실수로 바뀜'
print('OK')
"
```
Expected: `OK` 출력.

- [ ] **Step 4: 기존 robot_manager 테스트에 회귀가 없는지 재확인**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/robot_manager/test/ -v
```
Expected: PASS (51 passed) — `task_manager_node.py`를 검증하는 기존 유닛 테스트가 없으므로 이 스위트는 webcam_pc_cli 관련 테스트만 포함하지만, import 에러 등 회귀가 없는지 확인하는 용도다.

- [ ] **Step 5: Commit**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/amr.db real_project/src/robot_manager/robot_manager/task_manager_node.py
git commit -m "destinations 테스트 시드 + robot11 도킹 복귀 위치 반영"
```

---

### Task 7: 실물 로봇 파이프라인 검증 (사용자와 함께 진행 — AI 단독 실행 금지)

**이 태스크는 코드 작성이 없다.** `docs/superpowers/specs/2026-08-08-webcam-pc-cli-pipeline-test-design.md`의 "안전 수칙"·"수동 체크리스트" 섹션을 그대로 수행한다. Global Constraints에 명시된 대로, 이동을 유발하는 모든 명령은 AI가 단독으로 타이핑하지 않는다 — 사용자가 로봇 옆에서 비상정지를 쥐고 각 명령을 직접 트리거·확인하며 진행한다.

- [ ] **Step 1**: 로봇 PC에서 Nav2 bringup 실행 (사용자 직접) + `robot11_bridge_node` 실행 → 중앙 PC의 `/robot_status`에 실제 위치·배터리가 찍히는지 **사용자가 직접 확인**
- [ ] **Step 2**: `ros2 run robot_manager webcam_pc_cli` 실행 (AI가 노드 실행 자체는 진행 가능) → `호출 -1 0`은 **사용자가 직접 입력** → `/robot_assignment` 성공과 로봇이 실제로 주행 시작하는지 확인 (DOCKED → ASSIGNED)
- [ ] **Step 3**: `작업자감지`는 **사용자가 직접 입력** → ASSIGNED → FOLLOWING 전환 확인
- [ ] **Step 4**: `추종시작`은 **사용자가 로봇 옆에서 비상정지를 쥔 채 직접 입력** → `(-1.5, y, -π/2)` 경로를 따라 실제로 주행하는지 관찰, 이상 시 즉시 `추종중지` 입력
- [ ] **Step 5**: `배송모드 DEST_A` (또는 `DEST_B`)는 **사용자가 직접 입력** → FOLLOWING → TRANSPORTING 전환, 실제 목적지 도착 확인
- [ ] **Step 6**: 목적지 도착 후 `배송확인`은 **사용자가 직접 입력** → TRANSPORTING → RETURNING 전환, `(-2.3, -3.6, -π/2)` 도킹 위치로 실제 이동하는지 확인
- [ ] **Step 7**: 도킹 액션 성공 후 RETURNING → DOCKED 전환을 **사용자가 직접 확인**

모든 항목이 확인되면 이 브랜치를 정리한다 (필요 시 `superpowers:finishing-a-development-branch` 스킬 사용).
