# 웹캠 PC - 중앙 PC - AMR PC 통합 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** main 브랜치(웹캠 PC+중앙 PC)와 현재 브랜치(중앙 PC+AMR PC robot11)를 하나로 합쳐, 웹캠 PC → 중앙 PC → AMR PC(robot11 실물)로 이어지는 전체 배송 파이프라인을 완성하고 더 이상 필요 없는 더미를 제거한다.

**Architecture:** ROS2 3-PC 시스템. 웹캠 PC(`person_locator`/`hand_gesture_caller`/`hardhat_detector`, main에서 그대로 반입) → `/person/call_position`(PointStamped) → 중앙 PC(`robot_manager` 5개 노드, 배정/작업관리/HMI/DB/교착방지) → `/{robot_id}/robot_status`, `/{robot_id}/navigate_to_pose` 등 → AMR PC(`robot_bridge`, robot11 실물).

**Tech Stack:** ROS2(rclpy), FastAPI(HMI 백엔드), React(HMI 프론트), SQLite, pytest.

**참고 스펙:** `docs/superpowers/specs/2026-08-08-webcam-central-amr-integration-design.md`

## Global Constraints

- 이 프로젝트는 **실제 물리 로봇(iRobot Create3 기반 robot11)을 움직인다** — 하드웨어 안전을 항상 최우선으로 판단할 것.
- Dock/Undock 액션은 **어떤 경로로든 중복 발행되면 안 된다** (in-flight 가드로 1회만 보장).
- 도킹 상태를 실제로 확인하기 전에는 주행을 시작하지 않는다(`is_docked`/`dock_status_known` tri-state, 기본값으로 인한 오판 금지).
- `irobot_create_msgs/msg/DockStatus`의 정확한 토픽명·필드명은 **가정값**이다. 사용자가 나중에 robot11 PC에서 직접 확인 후 한 곳만 고치면 되도록 구현을 격리한다(Task 2에서 `⚠️` 주석으로 명시).
- `amr.db`에 `is_docked`/`dock_status_known`을 영속화하지 않는다(DB 마이그레이션은 범위 밖, 사용자 결정).
- `CENTRAL_SYSTEM_NODE_FLOW.md` 갱신은 범위 밖(사용자 결정).
- robot5용 `robot_bridge` 제작은 범위 밖. oakd 카메라 기반 실시간 추종좌표 구현도 범위 밖(`webcam_pc_cli`의 FOLLOWING mock으로 계속 대체).
- 모든 신규/변경 로직은 실패 시 `get_logger()`로 원인을 남긴다(운영자가 로그만 보고 원인을 찾을 수 있어야 함).

---

## File Structure

**수정:**
- `real_project/src/robot_status/msg/RobotStatus.msg` — `is_docked`, `dock_status_known` 필드 추가
- `real_project/src/robot_status/msg/AssignmentGoal.msg` — 삭제
- `real_project/src/robot_status/CMakeLists.txt` — `AssignmentGoal.msg` 라인 제거
- `real_project/src/robot_bridge/robot_bridge/robot11_bridge_node.py` — robot_id 파라미터화, per-robot 토픽, DockStatus 구독, in-flight 가드
- `real_project/src/robot_bridge/robot_bridge/pose_utils.py` — `build_robot_status`에 `is_docked`/`dock_status_known` 인자 추가
- `real_project/src/robot_bridge/test/test_robot11_bridge_node.py`, `test_pose_utils.py` — 위 변경에 맞춰 테스트 갱신/추가
- `real_project/src/robot_manager/robot_manager/robot_assignment_node.py` — main 버전으로 교체 + `is_robot_busy` 이식
- `real_project/src/robot_manager/robot_manager/db_manager_node.py` — per-robot 구독으로 변경
- `real_project/src/robot_manager/robot_manager/deadlock_prevention_node.py` — per-robot 구독으로 변경
- `real_project/src/robot_manager/robot_manager/task_manager_node.py` — 자동전환 + 도킹확인 로직 추가
- `real_project/src/robot_manager/test/test_task_manager_node.py` — 신규 테스트 추가
- `real_project/src/robot_manager/robot_manager/hmi_backend_node.py` — main 버전으로 교체 + 배송모드 엔드포인트 추가
- `real_project/amr_delivery_ui/frontend/src/api/robotApi.js`, `App.jsx` — 배송모드 UI 추가
- `real_project/src/robot_manager/setup.py` — `dummy_publisher` entry_point 제거
- `real_project/src/robot_manager/robot_manager/webcam_pc_cli.py`, `webcam_pc_cli_utils.py` — 명령 축소
- `real_project/src/robot_manager/test/test_webcam_pc_cli_node.py`, `test_webcam_pc_cli_utils.py` — 축소에 맞춰 테스트 정리

**삭제:**
- `real_project/src/robot_manager/robot_manager/dummy_status_publisher.py`

**신규 반입(main에서 그대로, 저장소 최상위 `src/`):**
- `src/person_locator/`, `src/hand_gesture_caller/`, `src/hardhat_detector/`

**변경 없음(확인만):** `real_project/src/robot_manager/launch/central_system.launch.py`, `real_project/src/robot_bridge/launch/robot11_bridge.launch.py`, `real_project/src/robot_bridge/package.xml`(이미 `irobot_create_msgs` 의존성 있음)

---

### Task 1: robot_status 메시지 정의 갱신

**Files:**
- Modify: `real_project/src/robot_status/msg/RobotStatus.msg`
- Delete: `real_project/src/robot_status/msg/AssignmentGoal.msg`
- Modify: `real_project/src/robot_status/CMakeLists.txt`

**Interfaces:**
- Produces: `RobotStatus.msg`에 `bool is_docked`, `bool dock_status_known` 필드 (Task 2, 6에서 사용)

- [ ] **Step 1: RobotStatus.msg에 필드 추가**

`real_project/src/robot_status/msg/RobotStatus.msg`를 다음 내용으로 교체:

```
string robot_id
float32 battery
float32 x
float32 y
float32 yaw
string current_task_id
bool is_docked
bool dock_status_known
```

- [ ] **Step 2: AssignmentGoal.msg 삭제**

```bash
rm real_project/src/robot_status/msg/AssignmentGoal.msg
```

- [ ] **Step 3: CMakeLists.txt에서 AssignmentGoal 라인 제거**

`real_project/src/robot_status/CMakeLists.txt`에서 다음 줄을 삭제:
```
  "msg/AssignmentGoal.msg"
```

- [ ] **Step 4: 빌드로 검증**

```bash
cd /home/hwangjeongui/team4_amr_assist/real_project
colcon build --packages-select robot_status
source install/setup.bash
ros2 interface show robot_status/msg/RobotStatus
```

Expected: 빌드 성공, `is_docked`/`dock_status_known` 필드가 출력에 보임. `ros2 interface show robot_status/msg/AssignmentGoal` 실행 시 "Unknown" 에러가 나야 함(삭제 확인).

- [ ] **Step 5: Commit**

```bash
git add real_project/src/robot_status/
git commit -m "robot_status: RobotStatus에 is_docked/dock_status_known 추가, AssignmentGoal 제거"
```

---

### Task 2: robot_bridge — robot_id 파라미터화, per-robot 토픽, 도킹 상태 tri-state, in-flight 가드

**Files:**
- Modify: `real_project/src/robot_bridge/robot_bridge/pose_utils.py`
- Modify: `real_project/src/robot_bridge/robot_bridge/robot11_bridge_node.py`
- Modify: `real_project/src/robot_bridge/test/test_pose_utils.py`
- Modify: `real_project/src/robot_bridge/test/test_robot11_bridge_node.py`

**Interfaces:**
- Consumes: Task 1의 `RobotStatus.is_docked`/`dock_status_known`
- Produces: `Robot11BridgeNode.is_docked: bool`, `Robot11BridgeNode.dock_status_known: bool`, `Robot11BridgeNode.dock_action_in_flight: bool` — Task 6(task_manager_node)이 `/{robot_id}/robot_status`를 통해 간접 소비

- [ ] **Step 1: pose_utils.py의 build_robot_status 테스트를 먼저 갱신**

`real_project/src/robot_bridge/test/test_pose_utils.py`의 `test_build_robot_status_fields`를 다음으로 교체하고, 새 테스트를 추가:

```python
def test_build_robot_status_fields():
    msg = build_robot_status('robot11', 87.5, 1.2, -3.4, 0.5)
    assert msg.robot_id == 'robot11'
    assert msg.battery == 87.5
    assert msg.x == 1.2
    assert msg.y == -3.4
    assert msg.yaw == 0.5
    assert msg.current_task_id == ''
    assert msg.is_docked is False
    assert msg.dock_status_known is False


def test_build_robot_status_with_dock_state():
    msg = build_robot_status('robot11', 87.5, 1.2, -3.4, 0.5, is_docked=True, dock_status_known=True)
    assert msg.is_docked is True
    assert msg.dock_status_known is True
```

- [ ] **Step 2: 실행해서 실패 확인**

```bash
cd /home/hwangjeongui/team4_amr_assist/real_project
colcon build --packages-select robot_status robot_bridge
source install/setup.bash
python3 -m pytest src/robot_bridge/test/test_pose_utils.py -v
```

Expected: `test_build_robot_status_with_dock_state` FAIL (`TypeError: build_robot_status() got an unexpected keyword argument 'is_docked'`), `test_build_robot_status_fields`도 `is_docked` AttributeError로 FAIL.

- [ ] **Step 3: pose_utils.py 구현**

`real_project/src/robot_bridge/robot_bridge/pose_utils.py`의 `build_robot_status` 함수를 다음으로 교체:

```python
def build_robot_status(robot_id: str, battery_percent: float, x: float, y: float, yaw: float,
                        is_docked: bool = False, dock_status_known: bool = False) -> RobotStatus:
    msg = RobotStatus()
    msg.robot_id = robot_id
    msg.battery = float(battery_percent)
    msg.x = float(x)
    msg.y = float(y)
    msg.yaw = float(yaw)
    msg.current_task_id = ''
    msg.is_docked = bool(is_docked)
    msg.dock_status_known = bool(dock_status_known)
    return msg
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
python3 -m pytest src/robot_bridge/test/test_pose_utils.py -v
```

Expected: 전부 PASS.

- [ ] **Step 5: robot11_bridge_node.py 테스트를 먼저 갱신 (도킹 관련 부분)**

`real_project/src/robot_bridge/test/test_robot11_bridge_node.py`에서 `test_build_status_message_after_pose_and_battery`에 다음 두 줄을 `assert status.current_task_id == ''` 뒤에 추가:

```python
        assert status.is_docked is False
        assert status.dock_status_known is False
```

기존 도킹 테스트 3개(`test_dock_request_true_sends_dock_goal`, `test_dock_request_false_sends_undock_goal`, `test_dock_request_skips_when_action_server_not_ready`)를 다음으로 교체:

```python
def test_dock_request_true_sends_dock_goal():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.dock_client = Mock()
        node.dock_client.wait_for_server.return_value = True
        node.undock_client = Mock()

        node.dock_callback(Bool(data=True))

        node.dock_client.send_goal_async.assert_called_once()
        node.undock_client.send_goal_async.assert_not_called()
        assert node.dock_action_in_flight is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dock_request_false_sends_undock_goal():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.dock_client = Mock()
        node.undock_client = Mock()
        node.undock_client.wait_for_server.return_value = True

        node.dock_callback(Bool(data=False))

        node.undock_client.send_goal_async.assert_called_once()
        node.dock_client.send_goal_async.assert_not_called()
        assert node.dock_action_in_flight is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dock_request_skips_when_action_server_not_ready():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.dock_client = Mock()
        node.dock_client.wait_for_server.return_value = False

        node.dock_callback(Bool(data=True))

        node.dock_client.send_goal_async.assert_not_called()
        assert node.dock_action_in_flight is False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dock_request_ignored_while_action_in_flight():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.dock_client = Mock()
        node.dock_client.wait_for_server.return_value = True
        node.undock_client = Mock()
        node.dock_action_in_flight = True

        node.dock_callback(Bool(data=True))

        node.dock_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dock_response_rejected_clears_in_flight_guard():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.dock_action_in_flight = True
        future = Mock()
        future.result.return_value = Mock(accepted=False)

        node._dock_response_callback(future)

        assert node.dock_action_in_flight is False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dock_result_clears_in_flight_guard():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.dock_action_in_flight = True
        result_future = Mock()
        result_future.result.return_value.result.is_docked = True

        node._dock_result_callback(result_future)

        assert node.dock_action_in_flight is False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dock_status_callback_updates_is_docked_and_known():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        assert node.dock_status_known is False

        status_msg = DockStatus()
        status_msg.is_docked = True
        node.dock_status_callback(status_msg)

        assert node.is_docked is True
        assert node.dock_status_known is True
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

파일 상단 import에 `from irobot_create_msgs.msg import DockStatus`를 추가.

- [ ] **Step 6: 실행해서 실패 확인**

```bash
python3 -m pytest src/robot_bridge/test/test_robot11_bridge_node.py -v
```

Expected: 새로 추가/교체한 테스트들이 `AttributeError`(`dock_action_in_flight`, `dock_status_callback`, `is_docked` 없음) 또는 `ImportError`(`DockStatus`)로 FAIL.

- [ ] **Step 7: robot11_bridge_node.py 구현**

`real_project/src/robot_bridge/robot_bridge/robot11_bridge_node.py` 전체를 다음으로 교체:

```python
#!/usr/bin/env python3
from copy import deepcopy
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from robot_status.msg import RobotStatus, TaskState

from robot_bridge.pose_utils import build_robot_status, is_followable_pose, quaternion_to_yaw


class Robot11BridgeNode(Node):
    def __init__(self):
        super().__init__('robot11_bridge_node')

        # robot_id는 파라미터로 뺐다 — 코드는 이 파일 그대로 두고 파라미터만 바꾸면
        # robot5 등 다른 로봇에도 재사용할 수 있다(robot5용 브릿지 제작 자체는 이번 범위 밖).
        self.declare_parameter('robot_id', 'robot11')
        self.robot_id = str(self.get_parameter('robot_id').value)

        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.latest_x: Optional[float] = None
        self.latest_y: Optional[float] = None
        self.latest_yaw: Optional[float] = None
        self.latest_battery_percent: Optional[float] = None

        self.is_docked = False
        self.dock_status_known = False
        self.dock_action_in_flight = False

        self.current_task_state: str = ''
        self.nav_generation = 0

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, f'/{self.robot_id}/amcl_pose', self.amcl_pose_callback, 10)
        # Create3/TurtleBot4 센서 토픽은 BEST_EFFORT로 발행되는 경우가 많다.
        # BEST_EFFORT 구독자는 BEST_EFFORT/RELIABLE 발행자 모두와 호환된다.
        self.battery_sub = self.create_subscription(
            BatteryState, f'/{self.robot_id}/battery_state', self.battery_callback,
            qos_profile_sensor_data)

        # ⚠️ 토픽명/메시지 타입/필드명 가정값 — robot11 PC에서
        # `ros2 topic list | grep dock`, `ros2 interface show irobot_create_msgs/msg/DockStatus`로
        # 실제 값을 확인한 뒤 이 두 줄(토픽명, is_docked 필드명)만 고치면 된다.
        self.dock_status_sub = self.create_subscription(
            DockStatus, f'/{self.robot_id}/dock_status', self.dock_status_callback,
            qos_profile_sensor_data)

        self.status_pub = self.create_publisher(RobotStatus, f'/{self.robot_id}/robot_status', status_qos)
        self.status_timer = self.create_timer(1.0, self.publish_robot_status)

        self.nav_goal_handle = None

        self.pause_sub = self.create_subscription(
            Bool, f'/{self.robot_id}/pause/request', self.pause_callback, 10)

        self.nav_client = ActionClient(self, NavigateToPose, f'/{self.robot_id}/navigate_to_pose')

        self.dock_sub = self.create_subscription(
            Bool, f'/{self.robot_id}/dock/request', self.dock_callback, 10)

        self.dock_client = ActionClient(self, Dock, f'/{self.robot_id}/dock')
        self.undock_client = ActionClient(self, Undock, f'/{self.robot_id}/undock')

        self.target_person_pose_sub = self.create_subscription(
            PoseStamped, f'/{self.robot_id}/target_person_pose', self.target_person_pose_callback, 10)
        self.task_state_sub = self.create_subscription(
            TaskState, '/task/state', self.task_state_callback, 10)

        self.get_logger().info(f'{self.robot_id} 브릿지 노드 시작')

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.latest_x, self.latest_y = position.x, position.y
        self.latest_yaw = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)

    def battery_callback(self, msg: BatteryState) -> None:
        self.latest_battery_percent = msg.percentage * 100.0

    def dock_status_callback(self, msg: DockStatus) -> None:
        self.is_docked = bool(msg.is_docked)
        self.dock_status_known = True

    def build_status_message(self) -> Optional[RobotStatus]:
        if self.latest_x is None or self.latest_battery_percent is None:
            return None
        return build_robot_status(
            self.robot_id, self.latest_battery_percent, self.latest_x, self.latest_y, self.latest_yaw,
            is_docked=self.is_docked, dock_status_known=self.dock_status_known)

    def publish_robot_status(self) -> None:
        msg = self.build_status_message()
        if msg is not None:
            self.status_pub.publish(msg)

    def pause_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        # 아직 응답이 오지 않은 in-flight goal도 무효화한다.
        # (generation을 올려두면 뒤늦게 수락된 goal이 응답 콜백에서 취소된다)
        self.nav_generation += 1
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None

    def dock_callback(self, msg: Bool) -> None:
        if self.dock_action_in_flight:
            self.get_logger().warn(f'{self.robot_id} Dock/Undock 진행 중 — 새 요청 무시')
            return
        if msg.data:
            self._send_dock_goal()
        else:
            self._send_undock_goal()

    def _send_dock_goal(self) -> None:
        if not self.dock_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('dock 액션 서버 대기 중')
            return
        self.dock_action_in_flight = True
        future = self.dock_client.send_goal_async(Dock.Goal())
        future.add_done_callback(self._dock_response_callback)

    def _send_undock_goal(self) -> None:
        if not self.undock_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('undock 액션 서버 대기 중')
            return
        self.dock_action_in_flight = True
        future = self.undock_client.send_goal_async(Undock.Goal())
        future.add_done_callback(self._undock_response_callback)

    def _dock_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.dock_action_in_flight = False
            self.get_logger().warn('dock goal 거부됨')
            return
        goal_handle.get_result_async().add_done_callback(self._dock_result_callback)

    def _dock_result_callback(self, future) -> None:
        self.dock_action_in_flight = False
        self.get_logger().info(f'dock 결과: is_docked={future.result().result.is_docked}')

    def _undock_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.dock_action_in_flight = False
            self.get_logger().warn('undock goal 거부됨')
            return
        goal_handle.get_result_async().add_done_callback(self._undock_result_callback)

    def _undock_result_callback(self, future) -> None:
        self.dock_action_in_flight = False
        self.get_logger().info(f'undock 결과: is_docked={future.result().result.is_docked}')

    def task_state_callback(self, msg: TaskState) -> None:
        if msg.robot_id == self.robot_id:
            self.current_task_state = msg.state

    def target_person_pose_callback(self, msg: PoseStamped) -> None:
        if self.current_task_state != 'FOLLOWING':
            return
        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f"target_person_pose의 frame_id가 'map'이 아님 ({msg.header.frame_id}) — 무시")
            return
        position = msg.pose.position
        orientation = msg.pose.orientation
        if not is_followable_pose(position.x, position.y, position.z,
                                  orientation.x, orientation.y, orientation.z, orientation.w):
            self.get_logger().warn('무효한 target_person_pose 무시 (추적 대상 없음)')
            return
        self._send_follow_goal(msg)

    def _send_follow_goal(self, pose: PoseStamped) -> None:
        # 이 콜백은 카메라 프레임레이트(10~30Hz)로 불리므로 블로킹 대기를 쓰면 안 된다.
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('navigate_to_pose 액션 서버 대기 중')
            return
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
        goal = NavigateToPose.Goal()
        goal.pose = deepcopy(pose)  # 수신 메시지를 직접 변형하지 않는다
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.nav_generation += 1
        generation = self.nav_generation
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda result: self._follow_goal_response_callback(result, generation))

    def _follow_goal_response_callback(self, future, generation: int) -> None:
        goal_handle = future.result()
        if generation != self.nav_generation:
            # pause 또는 더 새로운 goal이 이 goal을 무효화했다. 수락된 상태로 두면
            # 로봇이 계속 주행하므로 반드시 취소한다.
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().warn('follow goal 거부됨')
            return
        self.nav_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda _result: self._follow_result_callback(goal_handle))

    def _follow_result_callback(self, goal_handle) -> None:
        # goal이 정상 종료되면 stale handle이 남지 않도록 비운다.
        if self.nav_goal_handle is goal_handle:
            self.nav_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = Robot11BridgeNode()
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

주의: `_dock_response_callback`/`_undock_response_callback`을 테스트에서 직접 호출할 때 `future.result()`가 `Mock(accepted=False)`를 반환하도록 설정했으므로, 실제 `goal_handle.get_result_async()...`로 이어지는 성공 경로는 Step 5의 `test_dock_result_clears_in_flight_guard`처럼 `_dock_result_callback`을 직접 호출해 검증한다(액션 결과 future를 직접 만드는 것보다 안정적).

- [ ] **Step 8: 테스트 통과 확인**

```bash
python3 -m pytest src/robot_bridge/test/ -v
```

Expected: 전부 PASS.

- [ ] **Step 9: flake8 확인 (프로젝트 test_depend에 포함됨)**

```bash
python3 -m flake8 src/robot_bridge/robot_bridge/robot11_bridge_node.py src/robot_bridge/robot_bridge/pose_utils.py --max-line-length=200
```

Expected: 에러 없음.

- [ ] **Step 10: Commit**

```bash
git add real_project/src/robot_bridge/
git commit -m "robot_bridge: robot_id 파라미터화, per-robot 토픽, 도킹 상태 tri-state, dock in-flight 가드"
```

---

### Task 3: robot_assignment_node — main 버전 이식 + is_robot_busy

**Files:**
- Modify: `real_project/src/robot_manager/robot_manager/robot_assignment_node.py`

**Interfaces:**
- Consumes: `geometry_msgs/PointStamped`(`/person/call_position`), `RobotStatus`(`/{robot_id}/robot_status`, Task 1의 `current_task_id` 필드는 기존 유지 필드라 이미 존재)
- Produces: `RobotAssignment`(`/robot_assignment`, 기존과 동일 스키마)

이 노드는 main에서 이미 실물 웹캠 PC(`person_locator`)와 짝을 이뤄 테스트된 로직이라 처음부터 새로 짜지 않고 main 버전을 통째로 가져온 뒤, HEAD가 추가한 `is_robot_busy` 판정만 얹는다. 이 파일에 대한 기존 pytest 테스트 파일은 없다(두 브랜치 모두 없음) — flake8과 수동 임포트 검증으로 확인한다.

- [ ] **Step 1: main 버전을 그대로 가져오기**

```bash
cd /home/hwangjeongui/team4_amr_assist
git show main:real_project/src/robot_manager/robot_manager/robot_assignment_node.py > real_project/src/robot_manager/robot_manager/robot_assignment_node.py
```

- [ ] **Step 2: is_robot_busy 이식 — import 및 판정 메서드 추가**

`real_project/src/robot_manager/robot_manager/robot_assignment_node.py`에서 `is_status_fresh` 메서드 바로 다음에 추가:

```python
    @staticmethod
    def is_robot_busy(status: RobotStatus) -> bool:     # 로봇 상태가 작업 중인지 판단
        current_task_id = str(getattr(status, 'current_task_id', '')).strip()

        return bool(current_task_id)
```

- [ ] **Step 3: select_robot의 작업중 검사에 is_robot_busy 추가**

`select_robot` 메서드에서 다음 줄을 찾아:

```python
            # 3. 현재 작업 여부 검사
            if self.is_managed_robot_busy(robot_id):
                continue
```

다음으로 교체:

```python
            # 3. 현재 작업 여부 검사
            if self.is_robot_busy(status) or self.is_managed_robot_busy(robot_id):
                continue
```

- [ ] **Step 4: get_failure_reason의 idle_statuses 계산에도 is_robot_busy 추가**

`get_failure_reason` 메서드에서 다음 줄을 찾아:

```python
        idle_statuses = [status for robot_id, (status, received_at) in self.robot_statuses.items() if self.is_status_fresh(received_at, now) and not self.is_managed_robot_busy(robot_id)]
```

다음으로 교체:

```python
        idle_statuses = [status for robot_id, (status, received_at) in self.robot_statuses.items() if self.is_status_fresh(received_at, now) and not self.is_robot_busy(status) and not self.is_managed_robot_busy(robot_id)]
```

- [ ] **Step 5: 문법·임포트 검증**

```bash
cd /home/hwangjeongui/team4_amr_assist/real_project
colcon build --packages-select robot_manager
source install/setup.bash
python3 -c "from robot_manager.robot_assignment_node import RobotAssignmentNode; print('OK')"
python3 -m flake8 src/robot_manager/robot_manager/robot_assignment_node.py --max-line-length=200
```

Expected: `OK` 출력, flake8 에러 없음.

- [ ] **Step 6: Commit**

```bash
git add real_project/src/robot_manager/robot_manager/robot_assignment_node.py
git commit -m "robot_assignment_node: main 버전(PointStamped/per-robot 구독) 이식 + is_robot_busy 판정 추가"
```

---

### Task 4: db_manager_node — per-robot 구독으로 변경

**Files:**
- Modify: `real_project/src/robot_manager/robot_manager/db_manager_node.py`

**Interfaces:**
- Consumes: `RobotStatus`(`/{robot_id}/robot_status`, robot5·robot11 개별 토픽)

기존 HEAD 로직(`started_at` 컬럼, `current_task_id` fallback)은 그대로 두고, 구독 방식만 main 패턴으로 바꾼다.

- [ ] **Step 1: VALID_ROBOTS 클래스 상수 추가**

`real_project/src/robot_manager/robot_manager/db_manager_node.py`에서:

```python
class DbManagerNode(Node):
    def __init__(self):
```

를

```python
class DbManagerNode(Node):
    VALID_ROBOTS = ('robot5', 'robot11')

    def __init__(self):
```

로 교체.

- [ ] **Step 2: 구독 설정을 per-robot으로 교체**

다음 줄을 찾아:

```python
        self.status_subscription = self.create_subscription(RobotStatus, '/robot_status', self.status_callback, qos_profile)
```

다음으로 교체:

```python
        self.status_subscriptions = [self.create_subscription(RobotStatus, f'/{robot_id}/robot_status', lambda msg, expected=robot_id: self.status_callback(expected, msg), qos_profile) for robot_id in self.VALID_ROBOTS]
```

- [ ] **Step 3: status_callback 시그니처와 검증 로직 변경**

다음을 찾아:

```python
    def status_callback(self, msg):
        """로봇 상태 메시지 수신 시 타임스탬프와 최신 데이터 갱신"""

        current_time = self.get_clock().now().nanoseconds / 1e9  # 초 단위 변환
        
        # msg.robot_id는 string ('robot5' 또는 'robot11')
        robot_id = msg.robot_id
        self.last_msg_time[robot_id] = current_time
        self.latest_status[robot_id] = msg
```

다음으로 교체:

```python
    def status_callback(self, expected_robot_id, msg):
        """로봇 상태 메시지 수신 시 타임스탬프와 최신 데이터 갱신"""

        current_time = self.get_clock().now().nanoseconds / 1e9  # 초 단위 변환

        # msg.robot_id는 string ('robot5' 또는 'robot11')
        robot_id = str(msg.robot_id)
        if robot_id != expected_robot_id:
            self.get_logger().warning(f'토픽과 robot_id 불일치: expected={expected_robot_id}, received={robot_id}')
            return
        self.last_msg_time[robot_id] = current_time
        self.latest_status[robot_id] = msg
```

- [ ] **Step 4: 검증**

```bash
cd /home/hwangjeongui/team4_amr_assist/real_project
colcon build --packages-select robot_manager
source install/setup.bash
python3 -c "from robot_manager.db_manager_node import DbManagerNode; print('OK')"
python3 -m flake8 src/robot_manager/robot_manager/db_manager_node.py --max-line-length=200
```

Expected: `OK` 출력, flake8 에러 없음.

- [ ] **Step 5: Commit**

```bash
git add real_project/src/robot_manager/robot_manager/db_manager_node.py
git commit -m "db_manager_node: robot_status를 per-robot 토픽으로 구독하도록 변경"
```

---

### Task 5: deadlock_prevention_node — per-robot 구독으로 변경

**Files:**
- Modify: `real_project/src/robot_manager/robot_manager/deadlock_prevention_node.py`

**Interfaces:**
- Consumes: `RobotStatus`(`/{robot_id}/robot_status`)

- [ ] **Step 1: 구독 설정을 per-robot으로 교체**

`real_project/src/robot_manager/robot_manager/deadlock_prevention_node.py`에서:

```python
        self.status_sub = self.create_subscription(RobotStatus, '/robot_status', self.status_callback, qos)
```

를

```python
        self.status_subscriptions = [self.create_subscription(RobotStatus, f'/{robot_id}/robot_status', lambda msg, expected=robot_id: self.status_callback(expected, msg), qos) for robot_id in (self.ROBOT5, self.ROBOT11)]
```

로 교체.

- [ ] **Step 2: status_callback 시그니처와 검증 로직 변경**

다음을 찾아:

```python
    def status_callback(self, msg: RobotStatus) -> None:
        robot_id = self.normalize_robot_id(msg.robot_id)
        if robot_id not in (self.ROBOT5, self.ROBOT11):
            return
        previous = self.snapshots.get(robot_id)
```

다음으로 교체:

```python
    def status_callback(self, expected_robot_id: str, msg: RobotStatus) -> None:
        robot_id = self.normalize_robot_id(msg.robot_id)
        if robot_id != expected_robot_id:
            self.get_logger().warning(f'토픽과 robot_id 불일치: expected={expected_robot_id}, received={robot_id}')
            return
        previous = self.snapshots.get(robot_id)
```

- [ ] **Step 3: 검증**

```bash
cd /home/hwangjeongui/team4_amr_assist/real_project
colcon build --packages-select robot_manager
source install/setup.bash
python3 -c "from robot_manager.deadlock_prevention_node import DeadlockPreventionNode; print('OK')"
python3 -m flake8 src/robot_manager/robot_manager/deadlock_prevention_node.py --max-line-length=200
```

Expected: `OK` 출력, flake8 에러 없음.

- [ ] **Step 4: Commit**

```bash
git add real_project/src/robot_manager/robot_manager/deadlock_prevention_node.py
git commit -m "deadlock_prevention_node: robot_status를 per-robot 토픽으로 구독하도록 변경"
```

---

### Task 6: task_manager_node — 작업자감지/배송확인 자동화 + 도킹 상태 확인 후 진행

**Files:**
- Modify: `real_project/src/robot_manager/robot_manager/task_manager_node.py`
- Modify: `real_project/src/robot_manager/test/test_task_manager_node.py`

**Interfaces:**
- Consumes: `RobotStatus`(`/{robot_id}/robot_status`, Task 1의 `is_docked`/`dock_status_known`)
- Produces: 기존 `/task/state`, `/robot_error`, `/{robot_id}/dock/request` 스키마 동일(변경 없음)

**Step 1: 실패하는 테스트를 먼저 작성**

`real_project/src/robot_manager/test/test_task_manager_node.py` 맨 위 import에 `RobotStatus`와 `Duration`을 추가:

```python
from rclpy.duration import Duration
from robot_status.msg import RobotAssignment, RobotError, RobotStatus
```

파일 끝에 다음 테스트들을 추가:

```python
def _robot_status(robot_id='robot11', is_docked=False, dock_status_known=True):
    msg = RobotStatus()
    msg.robot_id = robot_id
    msg.is_docked, msg.dock_status_known = is_docked, dock_status_known
    return msg


def test_assignment_does_not_navigate_when_dock_status_unknown():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True

        _assign(node)

        node.nav_clients['robot11'].send_goal_async.assert_not_called()
        assert node.tasks['robot11'].awaiting_dock_check is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_assignment_navigates_immediately_when_known_undocked():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.robot_status_callback(_robot_status(is_docked=False, dock_status_known=True))
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True

        _assign(node)

        node.nav_clients['robot11'].send_goal_async.assert_called_once()
        assert node.tasks['robot11'].awaiting_dock_check is False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_assignment_requests_undock_once_when_docked():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.robot_status_callback(_robot_status(is_docked=True, dock_status_known=True))
        node.dock_publishers['robot11'] = Mock()
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True

        _assign(node)

        node.dock_publishers['robot11'].publish.assert_called_once()
        assert node.dock_publishers['robot11'].publish.call_args[0][0].data is False
        assert node.tasks['robot11'].undock_requested is True
        node.nav_clients['robot11'].send_goal_async.assert_not_called()

        # 같은 로봇 상태(is_docked=True)가 다시 와도 두 번째 언도킹 요청은 없어야 한다.
        node.robot_status_callback(_robot_status(is_docked=True, dock_status_known=True))
        node.dock_publishers['robot11'].publish.assert_called_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_undock_confirmed_then_navigation_starts():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.robot_status_callback(_robot_status(is_docked=True, dock_status_known=True))
        node.dock_publishers['robot11'] = Mock()
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        _assign(node)

        node.robot_status_callback(_robot_status(is_docked=False, dock_status_known=True))

        node.nav_clients['robot11'].send_goal_async.assert_called_once()
        assert node.tasks['robot11'].undock_requested is False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dock_wait_timeout_publishes_error_once():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.error_pub = Mock()
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        _assign(node)  # dock_status_known 없음 -> awaiting_dock_check=True

        task = node.tasks['robot11']
        task.dock_check_started_at = node.get_clock().now() - Duration(seconds=11.0)

        node.retry_navigation_goals()

        node.error_pub.publish.assert_called_once()
        assert node.error_pub.publish.call_args[0][0].error_code == 'DOCK_STATUS_UNKNOWN_TIMEOUT'

        # 다음 호출에서 같은 에러를 또 발행하지 않아야 한다(dock_check_started_at이 None으로 초기화됨).
        node.retry_navigation_goals()
        node.error_pub.publish.assert_called_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_worker_arrival_auto_transitions_to_following():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.robot_status_callback(_robot_status(is_docked=False, dock_status_known=True))
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        _assign(node)

        node.handle_navigation_result('robot11', 'TO_WORKER', True, '')

        assert node.tasks['robot11'].state == 'FOLLOWING'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_destination_arrival_auto_returns_and_sends_dock_goal():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.robot_status_callback(_robot_status(is_docked=False, dock_status_known=True))
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        _assign(node)
        task = node.tasks['robot11']
        task.state, task.goal_type = 'TRANSPORTING', 'TO_DESTINATION'

        node.handle_navigation_result('robot11', 'TO_DESTINATION', True, '')

        assert node.tasks['robot11'].state == 'RETURNING'
        assert node.tasks['robot11'].goal_type == 'TO_DOCK'
        assert node.nav_clients['robot11'].send_goal_async.call_count == 2  # 배정 이동 + 복귀 이동
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_worker_detected_command_no_longer_handled():
    rclpy.init()
    node = TaskManagerNode()
    try:
        node.robot_status_callback(_robot_status(is_docked=False, dock_status_known=True))
        node.error_pub = Mock()
        node.nav_clients['robot11'] = Mock()
        node.nav_clients['robot11'].wait_for_server.return_value = True
        _assign(node)

        from robot_status.msg import TaskCommand
        cmd = TaskCommand()
        cmd.command, cmd.robot_id, cmd.task_id = 'WORKER_DETECTED', 'robot11', node.tasks['robot11'].task_id
        node.command_callback(cmd)

        assert node.tasks['robot11'].state == 'ASSIGNED'  # 더 이상 이 커맨드로 전환되지 않음
        node.error_pub.publish.assert_called_once()
        assert 'INVALID_TRANSITION' in node.error_pub.publish.call_args[0][0].error_code
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: 실행해서 실패 확인**

```bash
cd /home/hwangjeongui/team4_amr_assist/real_project
colcon build --packages-select robot_status robot_manager
source install/setup.bash
python3 -m pytest src/robot_manager/test/test_task_manager_node.py -v
```

Expected: 신규 테스트 전부 FAIL (`AttributeError: 'TaskManagerNode' object has no attribute 'robot_status_callback'` 등), 기존 5개 테스트는 PASS.

- [ ] **Step 3: task_manager_node.py 구현 — import와 상수**

`real_project/src/robot_manager/robot_manager/task_manager_node.py` 상단 import를:

```python
from robot_status.msg import DeadlockPermission, NavigationResult, RobotAssignment, RobotError, TaskCommand, TaskState
```

에서

```python
from robot_status.msg import DeadlockPermission, NavigationResult, RobotAssignment, RobotError, RobotStatus, TaskCommand, TaskState
```

로 교체. `ManagedTask` dataclass에 필드 3개 추가(`goal_completed: bool = False` 다음 줄):

```python
    goal_completed: bool = False
    awaiting_dock_check: bool = False
    dock_check_started_at: object = None
    undock_requested: bool = False
```

`TaskManagerNode` 클래스 상수에 타임아웃 추가:

```python
    COMMAND_REJECTION_CODES = (
        'UNKNOWN_ROBOT_ID', 'TASK_NOT_FOUND', 'STALE_TASK_COMMAND',
        'INVALID_DESTINATION', 'ROBOT_ALREADY_HAS_TASK', 'INVALID_TRANSITION_',
    )
    DOCK_WAIT_TIMEOUT_SEC = 10.0
```

- [ ] **Step 4: __init__에 로봇 상태 구독 추가**

`self.error_sub = self.create_subscription(RobotError, '/robot_error', self.error_callback, 10)` 다음 줄에 추가:

```python
        self.status_subscriptions = [self.create_subscription(RobotStatus, f'/{robot_id}/robot_status', self.robot_status_callback, 10) for robot_id in self.VALID_ROBOTS]
        self.robot_dock_states: Dict[str, Tuple[bool, bool]] = {}
```

- [ ] **Step 5: assignment_callback을 도킹 확인 경유로 변경**

다음을 찾아:

```python
        task = ManagedTask(task_id=task_id, robot_id=robot_id, state='ASSIGNED', goal_type='TO_WORKER', target=(float(msg.target_x), float(msg.target_y), 0.0))
        self.tasks[robot_id] = task
        self.publish_state(task, 'AMR 배정 완료')
        self.send_navigation_goal(task)
```

다음으로 교체:

```python
        task = ManagedTask(task_id=task_id, robot_id=robot_id, state='ASSIGNED', goal_type='TO_WORKER', target=(float(msg.target_x), float(msg.target_y), 0.0))
        self.tasks[robot_id] = task
        self.publish_state(task, 'AMR 배정 완료')
        self.start_task_navigation(task)

    def start_task_navigation(self, task: ManagedTask) -> None:
        """배정 직후 주행 시작 전 도킹 상태를 확인한다. dock_status_known이 False면
        (즉 아직 실제 도킹 여부를 모르면) 절대 먼저 주행시키지 않는다."""
        robot_id = task.robot_id
        is_docked, dock_status_known = self.robot_dock_states.get(robot_id, (False, False))
        if not dock_status_known:
            task.awaiting_dock_check = True
            task.dock_check_started_at = self.get_clock().now()
            self.get_logger().warn(f'{robot_id} 도킹 상태 미확인 — DockStatus 대기 중')
            return
        if not is_docked:
            self.send_navigation_goal(task)
            return
        self.request_undock_once(task)

    def request_undock_once(self, task: ManagedTask) -> None:
        if task.undock_requested:
            return
        task.undock_requested = True
        task.dock_check_started_at = self.get_clock().now()
        self.dock_publishers[task.robot_id].publish(Bool(data=False))
        self.get_logger().info(f'{task.robot_id} 언도킹 요청 발행, is_docked=False 대기 시작')

    def robot_status_callback(self, msg: RobotStatus) -> None:
        robot_id = self.normalize_robot_id(msg.robot_id)
        if robot_id not in self.VALID_ROBOTS:
            return
        is_docked, dock_status_known = bool(msg.is_docked), bool(msg.dock_status_known)
        self.robot_dock_states[robot_id] = (is_docked, dock_status_known)
        if not dock_status_known:
            return
        task = self.tasks.get(robot_id)
        if task is None:
            return
        if task.awaiting_dock_check:
            task.awaiting_dock_check = False
            if is_docked:
                self.request_undock_once(task)
            else:
                self.send_navigation_goal(task)
            return
        if task.undock_requested and not is_docked:
            task.undock_requested = False
            self.send_navigation_goal(task)
```

- [ ] **Step 6: command_callback에서 WORKER_DETECTED/DELIVERY_CONFIRMED 제거**

다음을 찾아:

```python
        elif command == 'WORKER_DETECTED' and task.state == 'ASSIGNED':
            if not task.goal_completed:
                self.invalidate_navigation_goal(task)
            self.transition(task, 'FOLLOWING', '작업자 추종 시작')
        elif command == 'START_TRANSPORT' and task.state == 'FOLLOWING':
            if not math.isfinite(msg.target_x) or not math.isfinite(msg.target_y):
                self.publish_error(robot_id, task.task_id, 'INVALID_DESTINATION')
                return
            task.goal_type, task.target = 'TO_DESTINATION', (float(msg.target_x), float(msg.target_y), float(msg.target_yaw))
            task.goal_completed = False
            self.transition(task, 'TRANSPORTING', '작업자가 배송 모드로 전환')
            self.send_navigation_goal(task, replace=True)
        elif command == 'DELIVERY_CONFIRMED' and task.state == 'TRANSPORTING':
            task.goal_type, task.target = 'TO_DOCK', tuple(float(value) for value in self.get_parameter(f'{robot_id}_dock_pose').value)
            task.goal_completed = False
            self.transition(task, 'RETURNING', '배송 확인 완료')
            self.send_navigation_goal(task, replace=True)
        elif command == 'CANCEL' and task.state in self.ACTIVE_STATES:
```

다음으로 교체(`WORKER_DETECTED`/`DELIVERY_CONFIRMED` 분기 삭제, 나머지는 그대로):

```python
        elif command == 'START_TRANSPORT' and task.state == 'FOLLOWING':
            if not math.isfinite(msg.target_x) or not math.isfinite(msg.target_y):
                self.publish_error(robot_id, task.task_id, 'INVALID_DESTINATION')
                return
            task.goal_type, task.target = 'TO_DESTINATION', (float(msg.target_x), float(msg.target_y), float(msg.target_yaw))
            task.goal_completed = False
            self.transition(task, 'TRANSPORTING', '작업자가 배송 모드로 전환')
            self.send_navigation_goal(task, replace=True)
        elif command == 'CANCEL' and task.state in self.ACTIVE_STATES:
```

- [ ] **Step 7: handle_navigation_result를 자동전환으로 변경**

다음을 찾아:

```python
    def handle_navigation_result(self, robot_id: str, goal_type: str, success: bool, error_code: str) -> None:
        task = self.tasks.get(robot_id)
        if task is None:
            return
        task.goal_handle, task.goal_pending = None, False
        if not success:
            self.transition(task, 'ERROR', error_code or 'NAVIGATION_FAILED')
            self.publish_error(robot_id, task.task_id, error_code or 'NAVIGATION_FAILED')
        else:
            task.goal_completed = True
        if success and goal_type == 'TO_WORKER':
            self.publish_state(task, '작업자 위치 도착, 작업자 감지 대기')
        elif success and goal_type == 'TO_DESTINATION':
            self.publish_state(task, '배송 위치 도착, 작업자 배송 확인 대기')
        elif success and goal_type == 'TO_DOCK':
            self.dock_publishers[robot_id].publish(Bool(data=True))
            self.transition(task, 'DOCKED', '도킹 위치 복귀 완료')
```

다음으로 교체:

```python
    def handle_navigation_result(self, robot_id: str, goal_type: str, success: bool, error_code: str) -> None:
        task = self.tasks.get(robot_id)
        if task is None:
            return
        task.goal_handle, task.goal_pending = None, False
        if not success:
            self.transition(task, 'ERROR', error_code or 'NAVIGATION_FAILED')
            self.publish_error(robot_id, task.task_id, error_code or 'NAVIGATION_FAILED')
            return
        task.goal_completed = True
        if goal_type == 'TO_WORKER':
            # 웹캠 PC의 실제 작업자 감지 알고리즘은 아직 없다 — 배정 위치에 물리적으로
            # 도착한 것 자체를 감지 완료로 간주하고 대기 없이 바로 FOLLOWING으로 넘어간다.
            self.transition(task, 'FOLLOWING', '작업자 위치 도착, 작업자 감지 완료')
        elif goal_type == 'TO_DESTINATION':
            # 배송확인은 로컬라이제이션(Nav2 액션 성공) 기반으로 자동 처리한다.
            task.goal_type, task.target = 'TO_DOCK', tuple(float(value) for value in self.get_parameter(f'{robot_id}_dock_pose').value)
            task.goal_completed = False
            self.transition(task, 'RETURNING', '배송 위치 도착, 배송 확인 완료')
            self.send_navigation_goal(task, replace=True)
        elif goal_type == 'TO_DOCK':
            self.dock_publishers[robot_id].publish(Bool(data=True))
            self.transition(task, 'DOCKED', '도킹 위치 복귀 완료')
```

- [ ] **Step 8: retry_navigation_goals에 도킹 대기 타임아웃 점검 추가**

다음을 찾아:

```python
    def retry_navigation_goals(self) -> None:
        for task in self.tasks.values():
            if task.state in ('ASSIGNED', 'TRANSPORTING', 'RETURNING') and task.target and not task.goal_completed and task.goal_handle is None and not task.goal_pending:
                self.send_navigation_goal(task)
```

다음으로 교체:

```python
    def retry_navigation_goals(self) -> None:
        now = self.get_clock().now()
        for task in self.tasks.values():
            if (task.state in ('ASSIGNED', 'TRANSPORTING', 'RETURNING') and task.target
                    and not task.goal_completed and task.goal_handle is None and not task.goal_pending
                    and not task.awaiting_dock_check and not task.undock_requested):
                self.send_navigation_goal(task)
            self.check_dock_wait_timeout(task, now)

    def check_dock_wait_timeout(self, task: ManagedTask, now) -> None:
        if not (task.awaiting_dock_check or task.undock_requested):
            return
        if task.dock_check_started_at is None:
            return
        elapsed_sec = (now - task.dock_check_started_at).nanoseconds / 1e9
        if elapsed_sec < self.DOCK_WAIT_TIMEOUT_SEC:
            return
        error_code = 'DOCK_STATUS_UNKNOWN_TIMEOUT' if task.awaiting_dock_check else 'UNDOCK_CONFIRM_TIMEOUT'
        self.publish_error(task.robot_id, task.task_id, error_code)
        self.get_logger().error(f'{task.robot_id} {error_code}: {elapsed_sec:.1f}초 경과, 사람 개입 필요')
        # 타임아웃 에러는 1회만 발행한다 — 플래그(awaiting_dock_check/undock_requested)는
        # 그대로 두어 주행을 계속 막되, dock_check_started_at만 비워 중복 발행을 막는다.
        task.dock_check_started_at = None
```

- [ ] **Step 9: 테스트 통과 확인**

```bash
python3 -m pytest src/robot_manager/test/test_task_manager_node.py -v
```

Expected: 전부 PASS.

- [ ] **Step 10: flake8 확인**

```bash
python3 -m flake8 src/robot_manager/robot_manager/task_manager_node.py --max-line-length=200
```

Expected: 에러 없음.

- [ ] **Step 11: Commit**

```bash
git add real_project/src/robot_manager/robot_manager/task_manager_node.py real_project/src/robot_manager/test/test_task_manager_node.py
git commit -m "task_manager_node: 작업자감지/배송확인 자동화 + 도킹 상태 확인 후 진행(fail-safe, 1회 언도킹, 타임아웃 에러)"
```

---

### Task 7: hmi_backend_node — main 버전 채택

**Files:**
- Modify: `real_project/src/robot_manager/robot_manager/hmi_backend_node.py`

HEAD는 이 파일을 전혀 건드리지 않았으므로 병합 충돌이 없다. main 버전을 그대로 가져온다.

- [ ] **Step 1: main 버전으로 교체**

```bash
cd /home/hwangjeongui/team4_amr_assist
git show main:real_project/src/robot_manager/robot_manager/hmi_backend_node.py > real_project/src/robot_manager/robot_manager/hmi_backend_node.py
```

- [ ] **Step 2: 검증**

```bash
cd /home/hwangjeongui/team4_amr_assist/real_project
colcon build --packages-select robot_manager
source install/setup.bash
python3 -c "from robot_manager.hmi_backend_node import HmiBackendNode; print('OK')"
python3 -m flake8 src/robot_manager/robot_manager/hmi_backend_node.py --max-line-length=200
```

Expected: `OK` 출력, flake8 에러 없음.

- [ ] **Step 3: Commit**

```bash
git add real_project/src/robot_manager/robot_manager/hmi_backend_node.py
git commit -m "hmi_backend_node: main 버전(teleop/cancel/필드 확장) 채택"
```

---

### Task 8: hmi_backend_node — 배송모드 엔드포인트 추가 (격리 블록, 제거 쉽게)

**Files:**
- Modify: `real_project/src/robot_manager/robot_manager/hmi_backend_node.py`

**Interfaces:**
- Produces: `POST /api/robot/{robot_id}/transport {destination_id: str}` — Task 9(프론트엔드)가 소비

- [ ] **Step 1: HmiBackendNode에 배송모드 메서드 추가**

`hmi_backend_node.py`에서 `cancel_task` 메서드 바로 다음에, 명확히 구분되는 블록으로 추가:

```python
    # ===== 배송모드 (임시 — 로봇 부착 UI가 생기면 이 블록 전체를 지우면 됨) =====
    def start_transport(self, robot_id: str, destination_id: str):
        """FOLLOWING 상태의 로봇에게 목적지를 지정해 배송모드(START_TRANSPORT)를 시작시킨다."""
        if robot_id not in self.control_states:
            return False, f'지원하지 않는 AMR ID: {robot_id}'
        with sqlite3.connect(self.db_path) as conn:
            conn.row_factory = sqlite3.Row
            row = conn.execute(
                'SELECT position_x, position_y, orientation_yaw FROM destinations WHERE destination_id = ?',
                (destination_id,),
            ).fetchone()
        if row is None:
            return False, f'등록되지 않은 목적지: {destination_id}'
        command = TaskCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.robot_id, command.command = robot_id, 'START_TRANSPORT'
        command.target_x, command.target_y, command.target_yaw = float(row['position_x']), float(row['position_y']), float(row['orientation_yaw'])
        self.task_command_publisher.publish(command)
        self.get_logger().info(f'AMR {robot_id} 배송모드 요청: 목적지={destination_id}')
        return True, ''
    # ===== 배송모드 끝 =====
```

- [ ] **Step 2: FastAPI 요청 모델과 엔드포인트 추가**

파일 상단 `class DockRequest(BaseModel):` 블록 다음에 추가:

```python
# ===== 배송모드 (임시 — 로봇 부착 UI가 생기면 이 블록 전체를 지우면 됨) =====
class TransportRequest(BaseModel):
    destination_id: str
# ===== 배송모드 끝 =====
```

`@app.post("/api/robot/{robot_id}/cancel")` 엔드포인트 다음에 추가:

```python
# ===== 배송모드 (임시 — 로봇 부착 UI가 생기면 이 블록 전체를 지우면 됨) =====
@app.post("/api/robot/{robot_id}/transport")
def start_robot_transport(robot_id: str, data: TransportRequest):
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 노드가 준비되지 않았습니다.")
    ok, message = ros_node.start_transport(robot_id, data.destination_id)
    if not ok:
        raise HTTPException(status_code=404, detail=message)
    return {"accepted": True, "robot_id": robot_id, "destination_id": data.destination_id}
# ===== 배송모드 끝 =====
```

- [ ] **Step 3: 수동 검증 (이 파일에는 기존 pytest 테스트가 없음, curl로 확인)**

```bash
cd /home/hwangjeongui/team4_amr_assist/real_project
colcon build --packages-select robot_manager
source install/setup.bash
python3 -c "from robot_manager.hmi_backend_node import HmiBackendNode, TransportRequest, start_robot_transport; print('OK')"
python3 -m flake8 src/robot_manager/robot_manager/hmi_backend_node.py --max-line-length=200
```

Expected: `OK` 출력, flake8 에러 없음. (서버를 실제로 띄워 `curl -X POST localhost:8000/api/robot/robot11/transport -d '{"destination_id":"DEST_A"}'`로 확인하는 것은 amr.db에 destinations 데이터가 있는 실제 실행 환경에서 수행 — 이 스텝은 Task 12 완료 후 전체 통합 확인 때 함께 한다.)

- [ ] **Step 4: Commit**

```bash
git add real_project/src/robot_manager/robot_manager/hmi_backend_node.py
git commit -m "hmi_backend_node: 배송모드(START_TRANSPORT) HMI 엔드포인트 추가 (격리 블록)"
```

---

### Task 9: 프론트엔드 — 배송모드 UI 추가

**Files:**
- Modify: `real_project/amr_delivery_ui/frontend/src/api/robotApi.js`
- Modify: `real_project/amr_delivery_ui/frontend/src/App.jsx`

**Interfaces:**
- Consumes: Task 8의 `POST /api/robot/{robot_id}/transport`, 기존 `GET /api/database/table/destinations`(이미 화이트리스트에 있음, 새 엔드포인트 불필요)

- [ ] **Step 1: robotApi.js에 함수 추가**

`real_project/amr_delivery_ui/frontend/src/api/robotApi.js`의 `export const robotApi = { ... }` 안, `setDock` 줄 다음에 추가:

```javascript
  setDock: (dock, id) => req(`/api/robot/${id}/dock`, { method: 'POST', body: JSON.stringify({ dock }) }),
  // ===== 배송모드 (임시 — 로봇 부착 UI가 생기면 이 두 줄을 지우면 됨) =====
  getDestinations: () => req('/api/database/table/destinations'),
  startTransport: (id, destinationId) => req(`/api/robot/${id}/transport`, { method: 'POST', body: JSON.stringify({ destination_id: destinationId }) }),
```

- [ ] **Step 2: App.jsx에 TransportBox 컴포넌트 추가**

`real_project/amr_delivery_ui/frontend/src/App.jsx`에서 `function FleetMap({ robots, selectedId, onSelect }) {` 바로 위에 새 컴포넌트를 추가(완전히 독립된 함수라 통째로 지우기 쉬움):

```javascript
// ===== 배송모드 (임시 — 로봇 부착 UI가 생기면 이 컴포넌트 전체와 아래 호출부 한 줄을 지우면 됨) =====
function TransportBox({ selected, busy, execute }) {
  const [destinations, setDestinations] = useState([]);
  const [destinationId, setDestinationId] = useState('');
  useEffect(() => {
    let alive = true;
    robotApi.getDestinations().then((data) => { if (alive) setDestinations(data.rows || []); }).catch(() => {});
    return () => { alive = false; };
  }, []);
  useEffect(() => { if (!destinationId && destinations[0]) setDestinationId(destinations[0].destination_id); }, [destinations, destinationId]);
  return <section className="transport-box"><header><strong>배송모드</strong><small>작업자 추종 중에만 사용</small></header><div><select value={destinationId} onChange={(event) => setDestinationId(event.target.value)} disabled={busy || destinations.length === 0}>{destinations.map((dest) => <option key={dest.destination_id} value={dest.destination_id}>{dest.destination_name}</option>)}</select><button disabled={busy || !destinationId} onClick={() => execute(() => robotApi.startTransport(selected.robot_id, destinationId), '배송 시작 요청 완료')}>배송 시작</button></div></section>;
}
// ===== 배송모드 끝 =====
```

`<section className="dock-box">...</section>` 블록 바로 다음(teleop-box 이전)에 렌더 호출을 추가:

```javascript
          {selectedTask.state === 'FOLLOWING' && <TransportBox selected={selected} busy={busy} execute={execute} />}
```

- [ ] **Step 3: 수동 확인 (프론트엔드는 pytest 대상이 아님 — 개발 서버로 확인)**

```bash
cd /home/hwangjeongui/team4_amr_assist/real_project/amr_delivery_ui/frontend
npm run build
```

Expected: 빌드 에러 없이 성공. (실제 동작 확인은 Task 12까지 끝난 뒤 전체 통합 실행에서 `npm run dev`로 FOLLOWING 상태 로봇을 선택해 배송모드 박스가 뜨는지 육안 확인 — 이건 실제 로봇/DB 데이터가 필요해 사용자 확인 필요.)

- [ ] **Step 4: Commit**

```bash
git add real_project/amr_delivery_ui/frontend/src/api/robotApi.js real_project/amr_delivery_ui/frontend/src/App.jsx
git commit -m "frontend: 배송모드(목적지 선택 + 배송 시작) UI 추가"
```

---

### Task 10: dummy_status_publisher 제거

**Files:**
- Delete: `real_project/src/robot_manager/robot_manager/dummy_status_publisher.py`
- Modify: `real_project/src/robot_manager/setup.py`

robot5/robot11 모두 실물 브릿지로 대체될 예정이라 더미는 더 이상 필요 없다(스펙 5번, 사용자 결정).

- [ ] **Step 1: 파일 삭제**

```bash
rm /home/hwangjeongui/team4_amr_assist/real_project/src/robot_manager/robot_manager/dummy_status_publisher.py
```

- [ ] **Step 2: entry_point 제거**

`real_project/src/robot_manager/setup.py`에서 다음 줄을 삭제:

```python
            'dummy_publisher = robot_manager.dummy_status_publisher:main',
```

- [ ] **Step 3: 검증**

```bash
cd /home/hwangjeongui/team4_amr_assist/real_project
colcon build --packages-select robot_manager
source install/setup.bash
ros2 pkg executables robot_manager
```

Expected: `dummy_publisher`가 목록에 없음, 나머지 5개(db_manager_node, hmi_backend_node, robot_assignment_node, task_manager_node, deadlock_prevention_node, webcam_pc_cli)는 그대로 있음.

- [ ] **Step 4: Commit**

```bash
git add -A real_project/src/robot_manager/
git commit -m "robot_manager: dummy_status_publisher 제거 (robot5/robot11 모두 실물 브릿지로 대체 예정)"
```

---

### Task 11: webcam_pc_cli 축소 — 대체된 더미 명령 제거

**Files:**
- Modify: `real_project/src/robot_manager/robot_manager/webcam_pc_cli.py`
- Modify: `real_project/src/robot_manager/robot_manager/webcam_pc_cli_utils.py`
- Modify: `real_project/src/robot_manager/test/test_webcam_pc_cli_node.py`
- Modify: `real_project/src/robot_manager/test/test_webcam_pc_cli_utils.py`

`호출`(person_locator가 대체), `작업자감지`/`배송확인`(자동화됨), `배송모드`(HMI로 이동), `목적지목록`(더 이상 쓸모없음)을 제거하고 `추종시작`/`추종중지`/`상태`/`종료`만 남긴다.

- [ ] **Step 1: webcam_pc_cli_utils.py에서 제거 대상 함수/상수 삭제**

`real_project/src/robot_manager/robot_manager/webcam_pc_cli_utils.py`를 다음으로 교체:

```python
import math
from typing import Dict, List, Optional, Set, Tuple

ACTIVE_STATES: Set[str] = {'ASSIGNED', 'FOLLOWING', 'TRANSPORTING', 'RETURNING'}

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
    """stdin 한 줄을 (명령 키워드, 인자 리스트)로 분리한다."""
    text = raw_input.strip()
    parts = text.split()
    if not parts:
        return '', []
    return parts[0], parts[1:]


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
```

(`DELIVER_COMMAND` 관련 분기, `parse_call_args`, `select_destination`, `select_active_robot`을 삭제했다 — 전부 제거된 CLI 명령 전용이었다.)

- [ ] **Step 2: webcam_pc_cli.py를 축소된 버전으로 교체**

`real_project/src/robot_manager/robot_manager/webcam_pc_cli.py`를 다음으로 교체:

```python
#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from robot_status.msg import RobotError, TaskState

from robot_manager.webcam_pc_cli_utils import (
    FOLLOWING_MOCK_POSES,
    parse_command,
    parse_interval,
)

QUIT = object()


class WebcamPcCliNode(Node):
    def __init__(self):
        super().__init__('webcam_pc_cli_node')

        self.task_cache = {}

        self.target_pose_pub = self.create_publisher(PoseStamped, '/robot11/target_person_pose', 10)

        self.following_timer = None
        self.following_index = 0

        self.task_state_sub = self.create_subscription(
            TaskState, '/task/state', self.task_state_callback, 10)
        self.error_sub = self.create_subscription(
            RobotError, '/robot_error', self.error_callback, 10)

        self.get_logger().info('webcam_pc_cli 노드 시작 (FOLLOWING mock 전용 — 호출/작업자감지/배송은 실물 웹캠 PC와 HMI가 담당)')

    def task_state_callback(self, msg: TaskState) -> None:
        self.task_cache[msg.robot_id] = msg

    def error_callback(self, msg: RobotError) -> None:
        print(f'[오류] robot_id={msg.robot_id}, task_id={msg.task_id}, error_code={msg.error_code}')

    def cmd_status(self, args) -> None:
        cached = dict(self.task_cache)
        if not cached:
            print('캐싱된 로봇 상태 없음')
            return
        for robot_id, msg in cached.items():
            print(f'{robot_id}: {msg.state} (task_id={msg.task_id})')

    def _stop_following_timer(self) -> None:
        if self.following_timer is not None:
            self.destroy_timer(self.following_timer)
            self.following_timer = None

    def cmd_follow_start(self, args) -> None:
        interval, error = parse_interval(args)
        if error:
            print(error)
            return
        if self.following_timer is not None:
            print('이미 진행 중입니다. 먼저 추종중지를 입력하세요')
            return
        robot_state = self.task_cache.get('robot11')
        if robot_state is None or robot_state.state != 'FOLLOWING':
            print('[경고] robot11이 FOLLOWING 상태가 아닙니다 — robot_bridge가 target_person_pose를 무시할 수 있습니다')
        self.following_index = 0
        self.following_timer = self.create_timer(interval, self._publish_next_following_pose)
        print(f'[추종시작] 간격초={interval}')

    def cmd_follow_stop(self, args) -> None:
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

    def cmd_quit(self, args):
        return QUIT

    def run_cli(self) -> None:
        handlers = {
            '추종시작': self.cmd_follow_start,
            '추종중지': self.cmd_follow_stop,
            '상태': self.cmd_status,
            '종료': self.cmd_quit,
        }
        print('webcam_pc_cli 준비 완료 (FOLLOWING mock 전용). 명령: 추종시작/추종중지/상태/종료')
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
            try:
                if handler(args) is QUIT:
                    break
            except Exception as exc:
                print(f'[오류] 명령 처리 중 예외 발생: {exc}')


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
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: test_webcam_pc_cli_utils.py를 남는 함수만 검증하도록 교체**

`real_project/src/robot_manager/test/test_webcam_pc_cli_utils.py`를 다음으로 교체:

```python
from robot_manager.webcam_pc_cli_utils import FOLLOWING_MOCK_POSES, parse_command, parse_interval


def test_parse_command_simple_word_no_args():
    assert parse_command('상태') == ('상태', [])


def test_parse_command_with_args():
    assert parse_command('추종시작 5') == ('추종시작', ['5'])


def test_parse_command_empty_input_returns_empty_command():
    assert parse_command('   ') == ('', [])


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


def test_following_mock_poses_has_ten_points():
    assert len(FOLLOWING_MOCK_POSES) == 10
```

- [ ] **Step 4: test_webcam_pc_cli_node.py를 남는 명령만 검증하도록 교체**

`real_project/src/robot_manager/test/test_webcam_pc_cli_node.py`를 다음으로 교체:

```python
import rclpy
from unittest.mock import Mock

from robot_status.msg import RobotError, TaskState

from robot_manager.webcam_pc_cli import WebcamPcCliNode


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


def test_cmd_follow_start_warns_when_robot11_not_following(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        node.cmd_follow_start([])
        assert '[경고]' in capsys.readouterr().out
        assert node.following_timer is not None
    finally:
        node.cmd_follow_stop([])
        node.destroy_node()
        rclpy.shutdown()


def test_cmd_follow_start_no_warning_when_robot11_following(capsys):
    rclpy.init()
    node = WebcamPcCliNode()
    try:
        state = TaskState()
        state.robot_id, state.state, state.task_id = 'robot11', 'FOLLOWING', 'TASK_1'
        node.task_state_callback(state)

        node.cmd_follow_start([])

        assert '[경고]' not in capsys.readouterr().out
        assert node.following_timer is not None
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

- [ ] **Step 5: 테스트 통과 확인**

```bash
cd /home/hwangjeongui/team4_amr_assist/real_project
colcon build --packages-select robot_manager
source install/setup.bash
python3 -m pytest src/robot_manager/test/test_webcam_pc_cli_node.py src/robot_manager/test/test_webcam_pc_cli_utils.py -v
```

Expected: 전부 PASS.

- [ ] **Step 6: flake8 확인**

```bash
python3 -m flake8 src/robot_manager/robot_manager/webcam_pc_cli.py src/robot_manager/robot_manager/webcam_pc_cli_utils.py --max-line-length=200
```

Expected: 에러 없음.

- [ ] **Step 7: Commit**

```bash
git add real_project/src/robot_manager/robot_manager/webcam_pc_cli.py real_project/src/robot_manager/robot_manager/webcam_pc_cli_utils.py real_project/src/robot_manager/test/test_webcam_pc_cli_node.py real_project/src/robot_manager/test/test_webcam_pc_cli_utils.py
git commit -m "webcam_pc_cli: 호출/작업자감지/배송모드/배송확인/목적지목록 제거, FOLLOWING mock 전용으로 축소"
```

---

### Task 12: 웹캠 PC 패키지 반입 (person_locator, hand_gesture_caller, hardhat_detector)

**Files:**
- Create: `src/person_locator/`, `src/hand_gesture_caller/`, `src/hardhat_detector/` (main과 동일 내용)

main과 동일하게 저장소 최상위 `src/`에 그대로 가져온다(웹캠 PC가 물리적으로 다른 머신이라 `real_project/src/`와 분리된 구조를 유지).

- [ ] **Step 1: git status로 현재 작업 트리 확인 (덮어쓸 기존 파일 없는지)**

```bash
cd /home/hwangjeongui/team4_amr_assist
git status
ls src/ 2>/dev/null || echo "src/ 없음 — 새로 생성됨"
```

Expected: `src/` 디렉터리가 없거나 비어있음(현재 브랜치엔 이 패키지들이 없어야 함).

- [ ] **Step 2: main에서 세 패키지를 체크아웃**

```bash
git checkout main -- src/person_locator src/hand_gesture_caller src/hardhat_detector
```

- [ ] **Step 3: 반입된 내용 확인**

```bash
git status
find src/person_locator src/hand_gesture_caller src/hardhat_detector -type f | sort
```

Expected: 세 패키지의 모든 파일이 신규(staged) 상태로 나타남. `robot_status` 패키지를 참조하는 파일이 없어야 함(웹캠 PC는 중앙 PC의 커스텀 메시지에 의존하지 않고 표준 메시지만 씀 — 스펙 조사에서 이미 확인됨):

```bash
grep -rl "robot_status" src/person_locator src/hand_gesture_caller src/hardhat_detector || echo "참조 없음 (예상대로)"
```

- [ ] **Step 4: Commit**

```bash
git add src/person_locator src/hand_gesture_caller src/hardhat_detector
git commit -m "웹캠 PC 패키지 반입: person_locator, hand_gesture_caller, hardhat_detector (main에서 그대로)"
```

---

## 최종 통합 확인 (사용자 필요 — 실제 로봇/네트워크 환경)

이 계획의 각 태스크는 코드 레벨(유닛 테스트, flake8, colcon build)로 검증되지만, 아래는 이 저장소 밖에서 실제 하드웨어/네트워크가 있어야 확인 가능하다. 구현이 끝나면 스펙 문서의 "실제 로봇에서만 검증 가능한 항목" 8개를 사용자가 직접 확인해야 한다:

1. `irobot_create_msgs/msg/DockStatus`의 실제 토픽명·필드명 (Task 2의 `⚠️` 주석 위치 한 곳만 수정하면 됨).
2. 도킹/언도킹 물리 동작의 타이밍·안전성.
3. `robot11_dock_pose` 실측값이 새 로직과도 맞는지.
4. 웹캠 PC + 중앙 PC + AMR PC 3대 동시 ROS2 네트워크 설정.
5. 자동전환(작업자 도착 즉시 FOLLOWING, 배송지 도착 즉시 RETURNING)의 실제 동선 안전성.
6. 센서 QoS(BEST_EFFORT) 실측 호환성.
7. oakd 카메라 부재로 FOLLOWING은 여전히 mock 좌표로만 검증됨.
8. `DOCK_STATUS_UNKNOWN_TIMEOUT`/`UNDOCK_CONFIRM_TIMEOUT`의 10초 임계값 적정성.
