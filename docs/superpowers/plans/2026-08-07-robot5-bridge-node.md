# robot5 브릿지 노드 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** robot5 노트북(중앙 PC와 별도 컴퓨터)에서 실행되는 `robot_bridge` ROS2 패키지를 만들어, 중앙 시스템(`task_manager_node`)이 기대하는 `/robot_status` 발행, `pause`/`dock` 명령 처리, FOLLOWING 상태의 사람 추종 목표를 Nav2 goal로 중계하는 역할을 실제로 채운다.

**Architecture:** 단일 ROS2 노드(`robot5_bridge_node`)가 AMCL 위치·배터리 상태를 구독해 `/robot_status`를 1Hz로 발행하고, 중앙의 `pause`/`dock` Bool 토픽을 실제 Nav2 goal 취소 및 Create3 Dock/Undock 액션 호출로 연결하며, `/robot5/target_person_pose`를 FOLLOWING 상태에서만 검증 후 Nav2 goal로 전달한다. 순수 변환 로직(쿼터니언→yaw, 유효성 검증, 메시지 빌더)은 별도 모듈(`pose_utils.py`)로 분리해 ROS 스핀 없이 pytest로 테스트한다.

**Tech Stack:** ROS2 Humble, rclpy, `nav2_msgs/action/NavigateToPose`, `irobot_create_msgs/action/Dock,Undock`, `sensor_msgs/msg/BatteryState`, `geometry_msgs`, 커스텀 `robot_status` 메시지 패키지, pytest.

## Global Constraints

- robot_id는 `'robot5'`로 하드코딩한다 (파라미터화하지 않음).
- `/robot5/target_person_pose`는 이미 `map` 프레임이므로 TF 변환을 하지 않는다.
- `orientation`이 `(0,0,0,0)`인 `target_person_pose`는 무효 메시지로 취급해 무시한다 (추적 대상 없음 표시).
- `/robot_status`는 QoS `BEST_EFFORT`, depth 10으로 발행한다 (`db_manager_node`/`dummy_status_publisher`와 동일).
- `RobotStatus.current_task_id`는 항상 빈 문자열로 발행한다. `RobotStatus.msg` 파일 자체는 수정하지 않는다.
- 배터리는 `BatteryState.percentage`(0~1)를 ×100 해서 0~100 스케일로 발행한다.
- pause는 Nav2 goal 취소만 한다 — `cmd_vel`을 별도로 발행하지 않는다.
- `RobotError`는 이번 범위에서 발행하지 않는다.
- 도킹/언도킹은 `irobot_create_msgs/action/Dock`, `Undock`을 빈 goal로 실제 호출한다 (스텁 아님).
- 이 패키지는 `robot_manager`에 의존하지 않는다 — 로봇 노트북에는 `robot_bridge` + `robot_status`만 빌드하면 되어야 한다.
- **실물 로봇이 움직이는 검증 단계(Task 9)는 AI가 원격 단독으로 실행하지 않는다.** 사용자가 로봇 옆에서 비상정지를 쥐고 함께 진행한다.

---

## File Structure

```
real_project/src/robot_bridge/
  package.xml
  setup.py
  setup.cfg
  resource/robot_bridge
  robot_bridge/__init__.py
  robot_bridge/pose_utils.py         # 순수 함수: quaternion_to_yaw, is_valid_quaternion, build_robot_status
  robot_bridge/robot5_bridge_node.py # 메인 노드
  launch/robot5_bridge.launch.py
  test/test_pose_utils.py
  test/test_robot5_bridge_node.py
```

---

### Task 1: 패키지 스캐폴딩 + 빈 노드 실행 확인

**Files:**
- Create: `real_project/src/robot_bridge/package.xml`
- Create: `real_project/src/robot_bridge/setup.py`
- Create: `real_project/src/robot_bridge/setup.cfg`
- Create: `real_project/src/robot_bridge/resource/robot_bridge`
- Create: `real_project/src/robot_bridge/robot_bridge/__init__.py`
- Create: `real_project/src/robot_bridge/robot_bridge/robot5_bridge_node.py`

**Interfaces:**
- Produces: `robot_bridge.robot5_bridge_node.Robot5BridgeNode` (rclpy `Node` 서브클래스, 이후 Task에서 이 클래스에 구독/발행/액션을 추가한다), `main(args=None)` 진입점.

- [ ] **Step 1: package.xml 작성**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_bridge</name>
  <version>0.0.0</version>
  <description>robot5 노트북에서 중앙 시스템과 실제 로봇(AMCL/Nav2/Create3)을 연결하는 브릿지 노드</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>robot_status</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>irobot_create_msgs</depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: setup.py 작성**

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='robot5 노트북용 중앙 시스템 연동 브릿지 노드',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot5_bridge_node = robot_bridge.robot5_bridge_node:main',
        ],
    },
)
```

- [ ] **Step 3: setup.cfg 작성**

```ini
[develop]
script_dir=$base/lib/robot_bridge
[install]
install_scripts=$base/lib/robot_bridge
```

- [ ] **Step 4: resource marker와 `__init__.py` 생성**

`real_project/src/robot_bridge/resource/robot_bridge` — 빈 파일.
`real_project/src/robot_bridge/robot_bridge/__init__.py` — 빈 파일.

- [ ] **Step 5: 빈 노드 작성**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

ROBOT_ID = 'robot5'


class Robot5BridgeNode(Node):
    def __init__(self):
        super().__init__('robot5_bridge_node')
        self.get_logger().info(f'{ROBOT_ID} 브릿지 노드 시작')


def main(args=None):
    rclpy.init(args=args)
    node = Robot5BridgeNode()
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

- [ ] **Step 6: colcon build로 검증**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
colcon build --packages-select robot_bridge
source install/setup.bash
timeout 2 ros2 run robot_bridge robot5_bridge_node; echo "exit code: $?"
```
Expected: 빌드 성공(`Summary: 1 package finished`), 노드가 `robot5 브릿지 노드 시작` 로그를 찍고 timeout으로 종료(exit code 124) — 크래시 없이 뜬다는 뜻.

- [ ] **Step 7: Commit**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/src/robot_bridge
git commit -m "robot_bridge 패키지 스캐폴딩과 빈 노드 추가"
```

---

### Task 2: pose_utils 순수 함수 (quaternion→yaw, 유효성 검증, RobotStatus 빌더)

**Files:**
- Create: `real_project/src/robot_bridge/robot_bridge/pose_utils.py`
- Test: `real_project/src/robot_bridge/test/test_pose_utils.py`

**Interfaces:**
- Produces:
  - `quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float`
  - `is_valid_quaternion(x: float, y: float, z: float, w: float) -> bool`
  - `build_robot_status(robot_id: str, battery_percent: float, x: float, y: float, yaw: float) -> robot_status.msg.RobotStatus`

- [ ] **Step 1: 실패하는 테스트 작성**

```python
import math

from robot_bridge.pose_utils import build_robot_status, is_valid_quaternion, quaternion_to_yaw


def test_quaternion_to_yaw_identity_is_zero():
    assert quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == 0.0


def test_quaternion_to_yaw_90_degrees():
    # z축 기준 90도(pi/2) 회전 쿼터니언
    yaw = quaternion_to_yaw(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-6)


def test_is_valid_quaternion_rejects_all_zero():
    assert is_valid_quaternion(0.0, 0.0, 0.0, 0.0) is False


def test_is_valid_quaternion_accepts_identity():
    assert is_valid_quaternion(0.0, 0.0, 0.0, 1.0) is True


def test_build_robot_status_fields():
    msg = build_robot_status('robot5', 87.5, 1.2, -3.4, 0.5)
    assert msg.robot_id == 'robot5'
    assert msg.battery == 87.5
    assert msg.x == 1.2
    assert msg.y == -3.4
    assert msg.yaw == 0.5
    assert msg.current_task_id == ''
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
colcon build --packages-select robot_bridge robot_status
source install/setup.bash
python3 -m pytest src/robot_bridge/test/test_pose_utils.py -v
```
Expected: FAIL (`ModuleNotFoundError: No module named 'robot_bridge.pose_utils'` 또는 `ImportError`)

- [ ] **Step 3: 최소 구현 작성**

```python
import math

from robot_status.msg import RobotStatus


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """z축 회전만 있다고 가정하고 쿼터니언에서 yaw(라디안)를 추출한다."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def is_valid_quaternion(x: float, y: float, z: float, w: float) -> bool:
    """모든 성분이 0인 쿼터니언(추적 대상 없음을 뜻하는 무효 메시지)을 걸러낸다."""
    return not (x == 0.0 and y == 0.0 and z == 0.0 and w == 0.0)


def build_robot_status(robot_id: str, battery_percent: float, x: float, y: float, yaw: float) -> RobotStatus:
    msg = RobotStatus()
    msg.robot_id = robot_id
    msg.battery = float(battery_percent)
    msg.x = float(x)
    msg.y = float(y)
    msg.yaw = float(yaw)
    msg.current_task_id = ''
    return msg
```

- [ ] **Step 4: 테스트 실행해서 통과 확인**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
colcon build --packages-select robot_bridge
source install/setup.bash
python3 -m pytest src/robot_bridge/test/test_pose_utils.py -v
```
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/src/robot_bridge/robot_bridge/pose_utils.py real_project/src/robot_bridge/test/test_pose_utils.py
git commit -m "robot_bridge: pose_utils 순수 함수(yaw 추출/유효성 검증/RobotStatus 빌더) 추가"
```

---

### Task 3: RobotStatus 발행 (AMCL 위치·배터리 캐싱 + 1Hz 타이머)

**Files:**
- Modify: `real_project/src/robot_bridge/robot_bridge/robot5_bridge_node.py` (Task 1의 스켈레톤에 구독/발행 추가)
- Test: `real_project/src/robot_bridge/test/test_robot5_bridge_node.py` (신규)

**Interfaces:**
- Consumes: `pose_utils.quaternion_to_yaw`, `pose_utils.build_robot_status` (Task 2)
- Produces:
  - `Robot5BridgeNode.amcl_pose_callback(msg: PoseWithCovarianceStamped) -> None`
  - `Robot5BridgeNode.battery_callback(msg: BatteryState) -> None`
  - `Robot5BridgeNode.build_status_message() -> Optional[RobotStatus]` (이후 Task들의 테스트에서도 이 메서드로 발행 내용을 검증한다)
  - `Robot5BridgeNode.publish_robot_status() -> None`

- [ ] **Step 1: 실패하는 테스트 작성**

```python
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState

from robot_bridge.robot5_bridge_node import Robot5BridgeNode


def _amcl_msg(x, y, yaw_w=1.0, yaw_z=0.0):
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = yaw_z
    msg.pose.pose.orientation.w = yaw_w
    return msg


def test_build_status_message_none_before_data_received():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        assert node.build_status_message() is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_build_status_message_after_pose_and_battery():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.amcl_pose_callback(_amcl_msg(1.5, -2.0))
        battery_msg = BatteryState()
        battery_msg.percentage = 0.75
        node.battery_callback(battery_msg)

        status = node.build_status_message()
        assert status is not None
        assert status.robot_id == 'robot5'
        assert status.x == 1.5
        assert status.y == -2.0
        assert status.battery == 75.0
        assert status.current_task_id == ''
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
colcon build --packages-select robot_bridge
source install/setup.bash
python3 -m pytest src/robot_bridge/test/test_robot5_bridge_node.py -v
```
Expected: FAIL (`AttributeError: 'Robot5BridgeNode' object has no attribute 'amcl_pose_callback'`)

- [ ] **Step 3: 노드에 구독/발행 로직 추가**

`robot5_bridge_node.py`를 다음으로 전체 교체:

```python
#!/usr/bin/env python3
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState

from robot_status.msg import RobotStatus

from robot_bridge.pose_utils import build_robot_status, quaternion_to_yaw

ROBOT_ID = 'robot5'


class Robot5BridgeNode(Node):
    def __init__(self):
        super().__init__('robot5_bridge_node')

        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.latest_x: Optional[float] = None
        self.latest_y: Optional[float] = None
        self.latest_yaw: Optional[float] = None
        self.latest_battery_percent: Optional[float] = None

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, f'/{ROBOT_ID}/amcl_pose', self.amcl_pose_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, f'/{ROBOT_ID}/battery_state', self.battery_callback, 10)

        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', status_qos)
        self.status_timer = self.create_timer(1.0, self.publish_robot_status)

        self.get_logger().info(f'{ROBOT_ID} 브릿지 노드 시작')

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.latest_x, self.latest_y = position.x, position.y
        self.latest_yaw = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)

    def battery_callback(self, msg: BatteryState) -> None:
        self.latest_battery_percent = msg.percentage * 100.0

    def build_status_message(self) -> Optional[RobotStatus]:
        if self.latest_x is None or self.latest_battery_percent is None:
            return None
        return build_robot_status(
            ROBOT_ID, self.latest_battery_percent, self.latest_x, self.latest_y, self.latest_yaw)

    def publish_robot_status(self) -> None:
        msg = self.build_status_message()
        if msg is not None:
            self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Robot5BridgeNode()
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
colcon build --packages-select robot_bridge
source install/setup.bash
python3 -m pytest src/robot_bridge/test/test_robot5_bridge_node.py -v
```
Expected: PASS (2 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/src/robot_bridge/robot_bridge/robot5_bridge_node.py real_project/src/robot_bridge/test/test_robot5_bridge_node.py
git commit -m "robot_bridge: AMCL/배터리 캐싱 후 /robot_status 1Hz 발행 추가"
```

---

### Task 4: pause 처리 (Nav2 goal 취소)

**Files:**
- Modify: `real_project/src/robot_bridge/robot_bridge/robot5_bridge_node.py`
- Test: `real_project/src/robot_bridge/test/test_robot5_bridge_node.py`

**Interfaces:**
- Produces: `Robot5BridgeNode.pause_callback(msg: Bool) -> None`, `Robot5BridgeNode.nav_client`(`ActionClient`), `Robot5BridgeNode.nav_goal_handle` (다음 Task에서도 이 속성을 공유해서 사용한다)

- [ ] **Step 1: 실패하는 테스트 작성**

`test_robot5_bridge_node.py`에 추가:

```python
from unittest.mock import Mock

from std_msgs.msg import Bool


def test_pause_true_cancels_active_goal():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        fake_goal_handle = Mock()
        node.nav_goal_handle = fake_goal_handle

        node.pause_callback(Bool(data=True))

        fake_goal_handle.cancel_goal_async.assert_called_once()
        assert node.nav_goal_handle is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pause_true_without_active_goal_does_nothing():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.pause_callback(Bool(data=True))  # 예외 없이 통과해야 함
        assert node.nav_goal_handle is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pause_false_does_not_touch_goal():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        fake_goal_handle = Mock()
        node.nav_goal_handle = fake_goal_handle

        node.pause_callback(Bool(data=False))

        fake_goal_handle.cancel_goal_async.assert_not_called()
        assert node.nav_goal_handle is fake_goal_handle
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

Run: `python3 -m pytest src/robot_bridge/test/test_robot5_bridge_node.py -v` (source install/setup.bash 먼저)
Expected: FAIL (`AttributeError: 'Robot5BridgeNode' object has no attribute 'nav_goal_handle'`)

- [ ] **Step 3: pause 로직과 nav_client 추가**

`robot5_bridge_node.py`의 import에 추가:
```python
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool
```

`__init__` 안, `self.status_timer = ...` 다음 줄에 추가:
```python
        self.nav_goal_handle = None

        self.pause_sub = self.create_subscription(
            Bool, f'/{ROBOT_ID}/pause/request', self.pause_callback, 10)

        self.nav_client = ActionClient(self, NavigateToPose, f'/{ROBOT_ID}/navigate_to_pose')
```

새 메서드 추가 (`publish_robot_status` 다음):
```python
    def pause_callback(self, msg: Bool) -> None:
        if msg.data and self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
```

- [ ] **Step 4: 테스트 실행해서 통과 확인**

Run: `python3 -m pytest src/robot_bridge/test/test_robot5_bridge_node.py -v`
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/src/robot_bridge/robot_bridge/robot5_bridge_node.py real_project/src/robot_bridge/test/test_robot5_bridge_node.py
git commit -m "robot_bridge: pause/request 수신 시 Nav2 goal 취소 추가"
```

---

### Task 5: dock/undock 처리 (Create3 Dock/Undock 액션 호출)

**Files:**
- Modify: `real_project/src/robot_bridge/robot_bridge/robot5_bridge_node.py`
- Test: `real_project/src/robot_bridge/test/test_robot5_bridge_node.py`

**Interfaces:**
- Produces: `Robot5BridgeNode.dock_callback(msg: Bool) -> None`, `Robot5BridgeNode.dock_client`, `Robot5BridgeNode.undock_client`

- [ ] **Step 1: 실패하는 테스트 작성**

`test_robot5_bridge_node.py` 파일 하단에 추가:

```python
def test_dock_request_true_sends_dock_goal():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.dock_client = Mock()
        node.dock_client.wait_for_server.return_value = True
        node.undock_client = Mock()

        node.dock_callback(Bool(data=True))

        node.dock_client.send_goal_async.assert_called_once()
        node.undock_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dock_request_false_sends_undock_goal():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.dock_client = Mock()
        node.undock_client = Mock()
        node.undock_client.wait_for_server.return_value = True

        node.dock_callback(Bool(data=False))

        node.undock_client.send_goal_async.assert_called_once()
        node.dock_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dock_request_skips_when_action_server_not_ready():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.dock_client = Mock()
        node.dock_client.wait_for_server.return_value = False

        node.dock_callback(Bool(data=True))

        node.dock_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

Run: `python3 -m pytest src/robot_bridge/test/test_robot5_bridge_node.py -v`
Expected: FAIL (`AttributeError: 'Robot5BridgeNode' object has no attribute 'dock_client'`)

- [ ] **Step 3: dock/undock 로직 추가**

import 추가:
```python
from irobot_create_msgs.action import Dock, Undock
```

`__init__` 안, `self.nav_client = ...` 다음 줄에 추가:
```python
        self.dock_sub = self.create_subscription(
            Bool, f'/{ROBOT_ID}/dock/request', self.dock_callback, 10)

        self.dock_client = ActionClient(self, Dock, f'/{ROBOT_ID}/dock')
        self.undock_client = ActionClient(self, Undock, f'/{ROBOT_ID}/undock')
```

새 메서드 추가 (`pause_callback` 다음):
```python
    def dock_callback(self, msg: Bool) -> None:
        if msg.data:
            self._send_dock_goal()
        else:
            self._send_undock_goal()

    def _send_dock_goal(self) -> None:
        if not self.dock_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('dock 액션 서버 대기 중')
            return
        future = self.dock_client.send_goal_async(Dock.Goal())
        future.add_done_callback(self._dock_response_callback)

    def _send_undock_goal(self) -> None:
        if not self.undock_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('undock 액션 서버 대기 중')
            return
        future = self.undock_client.send_goal_async(Undock.Goal())
        future.add_done_callback(self._undock_response_callback)

    def _dock_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('dock goal 거부됨')
            return
        goal_handle.get_result_async().add_done_callback(
            lambda result: self.get_logger().info(f'dock 결과: is_docked={result.result().result.is_docked}'))

    def _undock_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('undock goal 거부됨')
            return
        goal_handle.get_result_async().add_done_callback(
            lambda result: self.get_logger().info(f'undock 결과: is_docked={result.result().result.is_docked}'))
```

- [ ] **Step 4: 테스트 실행해서 통과 확인**

Run: `python3 -m pytest src/robot_bridge/test/test_robot5_bridge_node.py -v`
Expected: PASS (8 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/src/robot_bridge/robot_bridge/robot5_bridge_node.py real_project/src/robot_bridge/test/test_robot5_bridge_node.py
git commit -m "robot_bridge: dock/request 수신 시 실제 Dock/Undock 액션 호출 추가"
```

---

### Task 6: FOLLOWING 상태에서 target_person_pose를 Nav2 goal로 전달

**Files:**
- Modify: `real_project/src/robot_bridge/robot_bridge/robot5_bridge_node.py`
- Test: `real_project/src/robot_bridge/test/test_robot5_bridge_node.py`

**Interfaces:**
- Consumes: `pose_utils.is_valid_quaternion` (Task 2)
- Produces: `Robot5BridgeNode.task_state_callback(msg: TaskState) -> None`, `Robot5BridgeNode.target_person_pose_callback(msg: PoseStamped) -> None`, `Robot5BridgeNode.current_task_state: str`

- [ ] **Step 1: 실패하는 테스트 작성**

`test_robot5_bridge_node.py` 파일 하단에 추가:

```python
from geometry_msgs.msg import PoseStamped

from robot_status.msg import TaskState


def _valid_person_pose(x=1.0, y=2.0):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    return msg


def _invalid_person_pose():
    msg = PoseStamped()
    msg.pose.orientation.w = 0.0  # geometry_msgs/Quaternion 기본값은 w=1.0(유효)이므로 명시적으로 0으로 만들어야 무효 케이스가 됨
    return msg


def test_task_state_callback_filters_by_robot_id():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        other_robot = TaskState()
        other_robot.robot_id = 'robot11'
        other_robot.state = 'FOLLOWING'
        node.task_state_callback(other_robot)
        assert node.current_task_state == ''

        this_robot = TaskState()
        this_robot.robot_id = 'robot5'
        this_robot.state = 'FOLLOWING'
        node.task_state_callback(this_robot)
        assert node.current_task_state == 'FOLLOWING'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_person_pose_ignored_when_not_following():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.nav_client = Mock()
        node.current_task_state = 'TRANSPORTING'

        node.target_person_pose_callback(_valid_person_pose())

        node.nav_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_person_pose_ignored_when_invalid_quaternion():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.nav_client = Mock()
        node.current_task_state = 'FOLLOWING'

        node.target_person_pose_callback(_invalid_person_pose())

        node.nav_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_person_pose_sends_goal_when_following_and_valid():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.nav_client = Mock()
        node.nav_client.wait_for_server.return_value = True
        node.current_task_state = 'FOLLOWING'

        node.target_person_pose_callback(_valid_person_pose(x=3.0, y=4.0))

        node.nav_client.send_goal_async.assert_called_once()
        sent_goal = node.nav_client.send_goal_async.call_args[0][0]
        assert sent_goal.pose.pose.position.x == 3.0
        assert sent_goal.pose.pose.position.y == 4.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_person_pose_cancels_previous_goal_before_resend():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.nav_client = Mock()
        node.nav_client.wait_for_server.return_value = True
        node.current_task_state = 'FOLLOWING'
        previous_handle = Mock()
        node.nav_goal_handle = previous_handle

        node.target_person_pose_callback(_valid_person_pose())

        previous_handle.cancel_goal_async.assert_called_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

Run: `python3 -m pytest src/robot_bridge/test/test_robot5_bridge_node.py -v`
Expected: FAIL (`AttributeError: 'Robot5BridgeNode' object has no attribute 'current_task_state'`)

- [ ] **Step 3: FOLLOWING 추종 로직 추가**

import 추가:
```python
from geometry_msgs.msg import PoseStamped
from robot_status.msg import TaskState
from robot_bridge.pose_utils import build_robot_status, is_valid_quaternion, quaternion_to_yaw
```
(기존 `from robot_bridge.pose_utils import build_robot_status, quaternion_to_yaw` 줄을 위 줄로 교체)

`__init__` 안, `self.latest_battery_percent = None` 다음 줄에 추가:
```python
        self.current_task_state: str = ''
        self.nav_generation = 0
```

`self.undock_client = ...` 다음 줄에 추가:
```python
        self.target_person_pose_sub = self.create_subscription(
            PoseStamped, f'/{ROBOT_ID}/target_person_pose', self.target_person_pose_callback, 10)
        self.task_state_sub = self.create_subscription(
            TaskState, '/task/state', self.task_state_callback, 10)
```

새 메서드 추가 (`_undock_response_callback` 다음):
```python
    def task_state_callback(self, msg: TaskState) -> None:
        if msg.robot_id == ROBOT_ID:
            self.current_task_state = msg.state

    def target_person_pose_callback(self, msg: PoseStamped) -> None:
        if self.current_task_state != 'FOLLOWING':
            return
        orientation = msg.pose.orientation
        if not is_valid_quaternion(orientation.x, orientation.y, orientation.z, orientation.w):
            self.get_logger().warn('무효한 target_person_pose 무시 (추적 대상 없음)')
            return
        self._send_follow_goal(msg)

    def _send_follow_goal(self, pose: PoseStamped) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('navigate_to_pose 액션 서버 대기 중')
            return
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.nav_generation += 1
        generation = self.nav_generation
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda result: self._follow_goal_response_callback(result, generation))

    def _follow_goal_response_callback(self, future, generation: int) -> None:
        if generation != self.nav_generation:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('follow goal 거부됨')
            return
        self.nav_goal_handle = goal_handle
```

- [ ] **Step 4: 테스트 실행해서 통과 확인**

Run: `python3 -m pytest src/robot_bridge/test/test_robot5_bridge_node.py -v`
Expected: PASS (13 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/src/robot_bridge/robot_bridge/robot5_bridge_node.py real_project/src/robot_bridge/test/test_robot5_bridge_node.py
git commit -m "robot_bridge: FOLLOWING 상태에서 target_person_pose를 Nav2 goal로 전달"
```

---

### Task 7: launch 파일

**Files:**
- Create: `real_project/src/robot_bridge/launch/robot5_bridge.launch.py`

**Interfaces:**
- Consumes: `robot_bridge` 패키지의 `robot5_bridge_node` executable (Task 1~6)

- [ ] **Step 1: launch 파일 작성**

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_bridge', executable='robot5_bridge_node', name='robot5_bridge_node', output='screen'),
    ])
```

- [ ] **Step 2: 빌드 후 launch 파일이 인식되는지 확인**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
colcon build --packages-select robot_bridge
source install/setup.bash
ros2 launch robot_bridge robot5_bridge.launch.py &
sleep 2
ros2 node list | grep robot5_bridge_node
kill %1
```
Expected: `ros2 node list` 출력에 `/robot5_bridge_node`가 보인다.

- [ ] **Step 3: Commit**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/src/robot_bridge/launch/robot5_bridge.launch.py
git commit -m "robot_bridge: robot5_bridge_node launch 파일 추가"
```

---

### Task 8: 전체 빌드/린트 최종 확인

**Files:** 없음 (검증만)

- [ ] **Step 1: 전체 워크스페이스 빌드**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
colcon build
```
Expected: 모든 패키지(`robot_status`, `robot_manager`, `robot_bridge`) 빌드 성공, 에러 없음.

- [ ] **Step 2: 전체 pytest 실행**

Run:
```bash
source install/setup.bash
python3 -m pytest src/robot_bridge/test/ -v
```
Expected: Task 2~6에서 작성한 모든 테스트 PASS (총 18개: pose_utils 5개 + node 13개).

- [ ] **Step 3: flake8/pep257 린트 (선택, 실패해도 진행 가능하나 확인은 할 것)**

Run:
```bash
cd /home/rokey/team4_amr_assist/real_project
colcon test --packages-select robot_bridge
colcon test-result --verbose
```
경고가 있으면 내용을 확인하고, 명백한 실수(미사용 import 등)는 고친다. 이 단계에서 새 기능을 추가하지 않는다.

- [ ] **Step 4: Commit (린트 수정이 있었던 경우만)**

```bash
cd /home/rokey/team4_amr_assist
git add real_project/src/robot_bridge
git commit -m "robot_bridge: 린트 경고 정리"
```

---

### Task 9: 실물 로봇 검증 (사용자와 함께 진행 — AI 단독 실행 금지)

**이 태스크는 코드 작성이 없다.** `docs/superpowers/specs/2026-08-07-robot5-bridge-node-design.md`의 "테스트 방침" 섹션에 정의된 수동 체크리스트를 그대로 수행한다. Global Constraints에 명시된 대로, 이 단계는 로봇이 실제로 움직이므로 사용자가 로봇 옆에서 비상정지를 쥐고 각 단계를 직접 트리거·확인하며 함께 진행해야 한다. AI가 로봇 노트북에 원격으로 접속해 이 단계를 혼자 실행하고 판단하지 않는다.

- [ ] **Step 1**: `amr_person_tracking` 브랜치를 병합했는지 확인 (안 됐으면 FOLLOWING 관련 항목은 스킵하고 나머지만 진행)
- [ ] **Step 2**: robot5 노트북에서 `ros2 launch robot_bridge robot5_bridge.launch.py` 실행, AMCL·Nav2·Create3 드라이버가 이미 떠 있는 상태에서 중앙 PC의 `/robot_status`에 실제 위치·배터리가 찍히는지 **사용자가 직접 확인**
- [ ] **Step 3**: `/robot5/pause/request`에 `True` 발행 시 진행 중이던 Nav2 goal이 취소되는지 **사용자가 로봇 옆에서 직접 확인**
- [ ] **Step 4**: `/robot5/dock/request` True/False 각각 발행 — **도킹 스테이션 근처에서, 사용자가 함께, 최초엔 저속/근접 상태로** `/robot5/dock`, `/robot5/undock` 액션이 정상 동작하는지 확인
- [ ] **Step 5**: FOLLOWING이 아닌 상태에서 `/robot5/target_person_pose`를 보내도 Nav2 goal이 전송되지 않는지 확인
- [ ] **Step 6**: FOLLOWING 상태에서 `target_person_pose`가 갱신될 때마다 Nav2 goal이 교체되는지 **사용자가 로봇 옆에서 추종 동작을 감독하며** 확인

모든 항목이 확인되면 이 브랜치를 `superpowers:finishing-a-development-branch` 스킬로 정리한다.
