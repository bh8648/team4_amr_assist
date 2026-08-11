# amr_person_tracking → main 병합 Implementation Plan

> **통합 전 작업 기록:** 이 계획의 `worker_tracking_bridge_node` 및
> `target_person_pose_raw` 방식은 최종 통합 과정에서 최신 `main`의 직접 처리 방식으로
> 대체됐다. 실행 기준은 프로젝트 루트의 `README_INTEGRATION.md`와 현재 소스 코드이다.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `feature/amr_person_tracking`의 사람 검출·추적 파이프라인을 `src/robot_PC/amr_person_tracking/`으로 무수정 병합하고, `robot_bridge` 패키지에 `worker_tracking/enable → worker_detected/worker_tracking_lost/target_person_pose` 계약을 구현하는 `worker_tracking_bridge_node`를 신규 작성해 FOLLOWING 파이프라인을 완성한다.

**Architecture:** `reid_tracking_node`(무수정, amr_person_tracking)가 발행하는 `target_person_pose`를 launch 시점에 `SetRemap`으로 `target_person_pose_raw`로 리다이렉트하고, 신규 `worker_tracking_bridge_node`가 그 raw 토픽을 구독해 `robot_bridge_node`가 기대하는 enable 게이팅·감지·유실 신호를 얹어 최종 `target_person_pose`로 재발행한다. `robot_bridge_node.py`(기존 Nav2 goal 전송 로직)는 완전히 무수정.

**Tech Stack:** ROS2 Humble, rclpy, ament_python, pytest.

## Global Constraints

- 참조 설계: `docs/superpowers/specs/2026-08-10-amr-person-tracking-merge-design.md` (모든 태스크의 세부 사항은 이 문서와 일치해야 함)
- 작업 브랜치: `feature/amr_person_tracking_merge` (이미 생성·체크아웃됨)
- `src/amr_person_tracking`(feature 브랜치 원본)의 코드는 **무수정**으로 이식한다 — 병합 후 어떤 태스크에서도 `src/robot_PC/amr_person_tracking/` 내부 파일을 편집하지 않는다.
- 기존 `robot_bridge_node.py`와 `test_robot_bridge_node.py`는 **무수정**.
- `evidence/`(mp4 증거영상)는 병합 대상에서 제외한다. `docs/amr_person_tracking_pipeline_flow.md`, `docs/session_backup_amr_person_tracking.md`는 포함한다.
- YOLO/ReID 모델 가중치(`*.pt`, `*.onnx`)는 저장소 정책상(`.gitignore`) 원래도 git에 안 들어간다 — 이번 병합으로 새로 생기는 문제가 아니므로 플레이스홀더나 우회 코드를 추가하지 않는다.
- 이 개발 환경에는 카메라/라이다/실물 로봇이 없다. 하드웨어가 필요한 검증은 각 태스크에 "사용자 확인 필요"로 명시하고, 에이전트는 코드 작성·정적 검증·유닛테스트까지만 수행한다.

---

## 파일 구조

| 파일 | 역할 |
|---|---|
| `src/robot_PC/amr_person_tracking/**` | feature 브랜치에서 무수정 이식된 검출·추적·회피 파이프라인 (4노드) |
| `src/robot_PC/robot_bridge/robot_bridge/worker_tracking_bridge_node.py` | 신규. enable 게이팅 + `target_person_pose` 중계 + `worker_detected`/`worker_tracking/lost` 발행 |
| `src/robot_PC/robot_bridge/test/test_worker_tracking_bridge_node.py` | 신규. 위 노드의 유닛테스트 |
| `src/robot_PC/robot_bridge/setup.py` | 수정. 신규 노드 entry_point 추가 |
| `src/robot_PC/robot_bridge/package.xml` | 수정. `amr_person_tracking` exec_depend 추가 |
| `src/robot_PC/robot_bridge/launch/robot.launch.py` | 수정. 신규 노드 실행 + `GroupAction`/`SetRemap`으로 감싼 `amr_person_tracking.launch.py` include |

---

### Task 1: amr_person_tracking 패키지 이식

**Files:**
- Create: `src/robot_PC/amr_person_tracking/**` (feature/amr_person_tracking 브랜치 전체, `evidence/` 제외)
- Create: `docs/amr_person_tracking_pipeline_flow.md`
- Create: `docs/session_backup_amr_person_tracking.md`

**Interfaces:**
- Produces: ROS2 패키지 `amr_person_tracking` (노드: `oakd_detector_node`, `leg_detector_bridge_node`, `reid_tracking_node`, `predictive_avoidance_node`, `debug_viewer_node`, `depth_view_republisher_node`, `mock_webcam_publisher_node`, `webcam_person_bridge_node`), launch 파일 `launch/amr_person_tracking.launch.py` (인자: `namespace`, `publish_debug_image` 등).이후 태스크가 이 launch 파일 경로(`get_package_share_directory('amr_person_tracking')`)와 `namespace` 인자를 그대로 사용한다.

- [ ] **Step 1: feature 브랜치에서 패키지와 관련 문서를 현재 브랜치로 가져온다**

```bash
git checkout feature/amr_person_tracking -- src/amr_person_tracking docs/amr_person_tracking_pipeline_flow.md docs/session_backup_amr_person_tracking.md
```

- [ ] **Step 2: evidence가 딸려오지 않았는지, 예상 파일 구조인지 확인**

```bash
git status --short | grep -c '^A'
find src/amr_person_tracking -maxdepth 2 | sort
test -d evidence && echo "FAIL: evidence 디렉토리가 생성됨" || echo "OK: evidence 없음"
```

Expected: `find` 출력에 `amr_person_tracking/amr_person_tracking`, `config`, `launch`, `resource`, `rviz`, `setup.py`, `test`, `tools` 등이 보이고, `evidence` 디렉토리는 생성되지 않아야 한다 (git checkout -- 로 지정 경로만 가져왔으므로).

- [ ] **Step 3: robot_PC 하위로 이동**

```bash
git mv src/amr_person_tracking src/robot_PC/amr_person_tracking
find src/robot_PC/amr_person_tracking -maxdepth 1
```

Expected: `src/robot_PC/amr_person_tracking/amr_person_tracking`, `config`, `launch`, `package.xml`, `resource`, `rviz`, `setup.cfg`, `setup.py`, `test`, `tools` 존재.

- [ ] **Step 4: 커밋**

```bash
git add -A src/robot_PC/amr_person_tracking docs/amr_person_tracking_pipeline_flow.md docs/session_backup_amr_person_tracking.md
git commit -m "$(cat <<'EOF'
amr_person_tracking 패키지를 src/robot_PC로 무수정 이식

feature/amr_person_tracking 브랜치의 검출·추적·회피 4노드 패키지를
robot_PC 하위로 옮긴다 (evidence/ 대용량 영상은 제외). 코드는 무수정이며,
worker_tracking_bridge_node(다음 태스크)가 이 패키지의 target_person_pose
출력을 소비한다.
EOF
)"
```

---

### Task 2: worker_tracking_bridge_node 구현 (TDD)

**Files:**
- Create: `src/robot_PC/robot_bridge/robot_bridge/worker_tracking_bridge_node.py`
- Test: `src/robot_PC/robot_bridge/test/test_worker_tracking_bridge_node.py`
- Modify: `src/robot_PC/robot_bridge/setup.py`
- Modify: `src/robot_PC/robot_bridge/package.xml`

**Interfaces:**
- Consumes: 없음 (신규 독립 노드, 토픽으로만 `robot_bridge_node`/`amr_person_tracking`과 연결됨).
- Produces: 클래스 `WorkerTrackingBridgeNode(Node)` (모듈 `robot_bridge.worker_tracking_bridge_node`), 생성자 `WorkerTrackingBridgeNode(robot_id: str = '')`. 콜백 `enable_callback(msg: std_msgs.msg.Bool)`, `raw_pose_callback(msg: geometry_msgs.msg.PoseStamped)`, `check_worker_lost()`. 퍼블리셔 속성명 `target_pose_pub`, `worker_detected_pub`, `worker_lost_pub` (테스트에서 Mock으로 치환해 검증). 파라미터 `robot_id`(robot5|robot11), `worker_lost_timeout`(float, 기본 60.0). 구독 토픽: `{robot_id}/worker_tracking/enable`, `{robot_id}/target_person_pose_raw`. 발행 토픽: `{robot_id}/target_person_pose`, `{robot_id}/worker_detected`, `{robot_id}/worker_tracking/lost`. 콘솔 스크립트 `worker_tracking_bridge_node = robot_bridge.worker_tracking_bridge_node:main` (Task 3의 launch 파일이 이 실행 파일명을 그대로 사용).

- [ ] **Step 1: 실패하는 테스트 작성**

`src/robot_PC/robot_bridge/test/test_worker_tracking_bridge_node.py`:

```python
from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from robot_bridge.worker_tracking_bridge_node import WorkerTrackingBridgeNode


def _pose(x=1.0, y=2.0):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.w = 1.0
    return msg


@pytest.fixture
def node():
    rclpy.init()
    n = WorkerTrackingBridgeNode(robot_id='robot5')
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_disabled_ignores_raw_pose(node):
    node.target_pose_pub = Mock()
    node.worker_detected_pub = Mock()

    node.raw_pose_callback(_pose())

    node.target_pose_pub.publish.assert_not_called()
    node.worker_detected_pub.publish.assert_not_called()


def test_enabled_forwards_pose_and_sends_detected_once(node):
    node.target_pose_pub = Mock()
    node.worker_detected_pub = Mock()
    node.enable_callback(Bool(data=True))

    node.raw_pose_callback(_pose())
    node.target_pose_pub.publish.assert_called_once()
    node.worker_detected_pub.publish.assert_called_once()
    assert node.worker_detected_pub.publish.call_args.args[0].data is True

    node.raw_pose_callback(_pose(x=1.5))
    assert node.target_pose_pub.publish.call_count == 2
    node.worker_detected_pub.publish.assert_called_once()


def test_lost_timeout_fires_once(node):
    node.worker_lost_timeout = 1.0
    node.worker_lost_pub = Mock()
    node.enable_callback(Bool(data=True))
    node.last_pose_time_ns = node.get_clock().now().nanoseconds - int(2e9)

    node.check_worker_lost()
    node.worker_lost_pub.publish.assert_called_once()
    assert node.worker_lost_pub.publish.call_args.args[0].data is True

    node.check_worker_lost()
    node.worker_lost_pub.publish.assert_called_once()


def test_reenable_resets_detected_and_lost_latches(node):
    node.target_pose_pub = Mock()
    node.worker_detected_pub = Mock()
    node.worker_lost_pub = Mock()
    node.worker_lost_timeout = 1.0

    node.enable_callback(Bool(data=True))
    node.raw_pose_callback(_pose())
    node.last_pose_time_ns = node.get_clock().now().nanoseconds - int(2e9)
    node.check_worker_lost()
    node.worker_lost_pub.publish.assert_called_once()

    node.enable_callback(Bool(data=False))
    node.enable_callback(Bool(data=True))
    node.raw_pose_callback(_pose())

    assert node.worker_detected_pub.publish.call_count == 2


def test_invalid_robot_id_is_rejected():
    rclpy.init()
    try:
        with pytest.raises(ValueError, match='robot_id'):
            WorkerTrackingBridgeNode(robot_id='robot7')
    finally:
        rclpy.shutdown()
```

- [ ] **Step 2: 테스트가 실패하는지 확인 (모듈이 아직 없음)**

```bash
cd src/robot_PC/robot_bridge && python3 -m pytest test/test_worker_tracking_bridge_node.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'robot_bridge.worker_tracking_bridge_node'`.

- [ ] **Step 3: 노드 구현**

`src/robot_PC/robot_bridge/robot_bridge/worker_tracking_bridge_node.py`:

```python
#!/usr/bin/env python3
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class WorkerTrackingBridgeNode(Node):
    """robot_bridge_node가 기대하는 worker_tracking/enable-detected-lost 계약을 채운다."""

    def __init__(self, robot_id: str = ''):
        super().__init__('worker_tracking_bridge_node')
        self.declare_parameter('robot_id', robot_id)
        self.robot_id = str(self.get_parameter('robot_id').value).strip()
        if self.robot_id not in ('robot5', 'robot11'):
            raise ValueError('robot_id는 robot5 또는 robot11이어야 합니다.')
        self.declare_parameter('worker_lost_timeout', 60.0)
        self.worker_lost_timeout = float(self.get_parameter('worker_lost_timeout').value)
        if self.worker_lost_timeout <= 0.0:
            raise ValueError('worker_lost_timeout은 0보다 커야 합니다.')
        topic_prefix = f'/{self.robot_id}'

        self.enabled = False
        self.last_pose_time_ns: Optional[int] = None
        self.detected_sent = False
        self.lost_sent = False

        # robot_bridge_node가 늦게 뜬 이 노드에도 현재 enable 상태를 즉시 전달하도록
        # 발행 측과 동일한 latched QoS로 구독한다.
        enable_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable_sub = self.create_subscription(
            Bool, f'{topic_prefix}/worker_tracking/enable', self.enable_callback, enable_qos)
        self.raw_pose_sub = self.create_subscription(
            PoseStamped, f'{topic_prefix}/target_person_pose_raw', self.raw_pose_callback, 10)

        self.target_pose_pub = self.create_publisher(PoseStamped, f'{topic_prefix}/target_person_pose', 10)
        self.worker_detected_pub = self.create_publisher(Bool, f'{topic_prefix}/worker_detected', 10)
        self.worker_lost_pub = self.create_publisher(Bool, f'{topic_prefix}/worker_tracking/lost', 10)

        self.lost_check_timer = self.create_timer(1.0, self.check_worker_lost)
        self.get_logger().info(f'{self.robot_id} 작업자 추적 브릿지 노드 시작')

    def enable_callback(self, msg: Bool) -> None:
        """enable이 새로 켜질 때마다 이번 추종 구간의 감지/유실 상태를 초기화한다."""
        if msg.data and not self.enabled:
            self.last_pose_time_ns = None
            self.detected_sent = False
            self.lost_sent = False
        self.enabled = msg.data

    def raw_pose_callback(self, msg: PoseStamped) -> None:
        """활성화 상태일 때만 추종 좌표를 중계하고 최초 수신을 감지 신호로 알린다."""
        if not self.enabled:
            return
        self.target_pose_pub.publish(msg)
        self.last_pose_time_ns = self.get_clock().now().nanoseconds
        self.lost_sent = False
        if not self.detected_sent:
            self.worker_detected_pub.publish(Bool(data=True))
            self.detected_sent = True

    def check_worker_lost(self) -> None:
        """활성화 상태에서 유실 제한시간을 넘기면 한 번만 유실 신호를 보낸다."""
        if not self.enabled or self.last_pose_time_ns is None or self.lost_sent:
            return
        elapsed_s = (self.get_clock().now().nanoseconds - self.last_pose_time_ns) / 1e9
        if elapsed_s >= self.worker_lost_timeout:
            self.worker_lost_pub.publish(Bool(data=True))
            self.lost_sent = True


def main(args=None):
    """robot_id 파라미터로 선택된 작업자 추적 브릿지 실행 진입점."""
    rclpy.init(args=args)
    node = WorkerTrackingBridgeNode()
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

- [ ] **Step 4: 테스트 통과 확인**

```bash
cd src/robot_PC/robot_bridge && python3 -m pytest test/test_worker_tracking_bridge_node.py -v
```

Expected: 5개 테스트 전부 PASS.

- [ ] **Step 5: setup.py에 entry_point 추가**

`src/robot_PC/robot_bridge/setup.py`의 `entry_points`를 다음과 같이 수정:

```python
    entry_points={'console_scripts': [
        'robot_bridge_node = robot_bridge.robot_bridge_node:main',
        'worker_tracking_bridge_node = robot_bridge.worker_tracking_bridge_node:main',
    ]},
```

- [ ] **Step 6: package.xml에 amr_person_tracking exec_depend 추가**

`src/robot_PC/robot_bridge/package.xml`의 `<exec_depend>robot_hmi_backend</exec_depend>` 다음 줄에 추가:

```xml
  <exec_depend>amr_person_tracking</exec_depend>
```

- [ ] **Step 7: robot_bridge 패키지 전체 회귀 테스트 (기존 테스트 무수정 확인)**

```bash
cd src/robot_PC/robot_bridge && python3 -m pytest test/ -v
```

Expected: 기존 `test_robot_bridge_node.py`, `test_pose_utils.py` 포함 전부 PASS (기존 파일 무수정이므로 실패하면 안 됨).

- [ ] **Step 8: 커밋**

```bash
git add src/robot_PC/robot_bridge/robot_bridge/worker_tracking_bridge_node.py \
        src/robot_PC/robot_bridge/test/test_worker_tracking_bridge_node.py \
        src/robot_PC/robot_bridge/setup.py \
        src/robot_PC/robot_bridge/package.xml
git commit -m "$(cat <<'EOF'
worker_tracking_bridge_node 추가 - enable/detected/lost 계약 구현

robot_bridge_node가 기대하지만 어디에도 구현되어 있지 않던
worker_tracking/enable 구독, worker_detected/worker_tracking_lost 발행
계약을 채운다. target_person_pose_raw를 받아 enable 상태일 때만
target_person_pose로 중계한다.
EOF
)"
```

---

### Task 3: launch 배선

**Files:**
- Modify: `src/robot_PC/robot_bridge/launch/robot.launch.py`

**Interfaces:**
- Consumes: Task 1의 `amr_person_tracking` 패키지 이름과 `launch/amr_person_tracking.launch.py`(인자 `namespace`, `publish_debug_image`), Task 2의 콘솔 스크립트 이름 `worker_tracking_bridge_node`.
- Produces: 없음 (최종 배선. 이후 태스크 없음).

- [ ] **Step 1: robot.launch.py 전체를 다음 내용으로 교체**

`src/robot_PC/robot_bridge/launch/robot.launch.py`:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


HMI_PORTS = {'robot5': 8005, 'robot11': 8011}


def _launch_nodes(context):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    if robot_id not in HMI_PORTS:
        raise ValueError('robot_id는 robot5 또는 robot11이어야 합니다.')
    configured_port = LaunchConfiguration('web_port').perform(context).strip()
    web_port = int(configured_port) if configured_port else HMI_PORTS[robot_id]

    amr_person_tracking_launch = os.path.join(
        get_package_share_directory('amr_person_tracking'), 'launch', 'amr_person_tracking.launch.py')
    target_person_pose_topic = f'/{robot_id}/target_person_pose'
    target_person_pose_raw_topic = f'/{robot_id}/target_person_pose_raw'

    return [
        Node(
            package='robot_bridge', executable='robot_bridge_node',
            namespace=robot_id, name='robot_bridge_node', output='screen',
            parameters=[{'robot_id': robot_id}],
        ),
        Node(
            package='robot_bridge', executable='worker_tracking_bridge_node',
            namespace=robot_id, name='worker_tracking_bridge_node', output='screen',
            parameters=[{'robot_id': robot_id}],
        ),
        Node(
            package='robot_hmi_backend', executable='hmi_backend_node',
            namespace=robot_id, name='robot_hmi_backend_node', output='screen',
            parameters=[{'robot_id': robot_id, 'web_port': web_port}],
        ),
        # reid_tracking_node(amr_person_tracking, 무수정)가 원래 발행하는
        # target_person_pose를 raw 토픽으로 리다이렉트해, worker_tracking_bridge_node가
        # 최종 target_person_pose의 유일한 발행자가 되게 한다. GroupAction으로 감싸지
        # 않으면 이 리맵이 위 두 Node에도 전역 적용되어 버린다.
        GroupAction(actions=[
            SetRemap(src=target_person_pose_topic, dst=target_person_pose_raw_topic),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(amr_person_tracking_launch),
                launch_arguments={
                    'namespace': robot_id,
                    'publish_debug_image': 'false',
                }.items(),
            ),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            description='이 PC가 제어할 로봇 ID: robot5 또는 robot11',
        ),
        DeclareLaunchArgument(
            'web_port', default_value='',
            description='비어 있으면 robot5=8005, robot11=8011을 사용',
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
```

- [ ] **Step 2: 문법·구성 정적 검증 (하드웨어 불필요)**

```bash
python3 -m py_compile src/robot_PC/robot_bridge/launch/robot.launch.py
python3 -c "
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch import LaunchDescription
ld = LaunchDescription([
    GroupAction(actions=[
        SetRemap(src='/robot5/target_person_pose', dst='/robot5/target_person_pose_raw'),
    ]),
])
print('launch 구성 OK')
"
```

Expected: 둘 다 에러 없이 통과, `launch 구성 OK` 출력.

**참고 (사용자 확인 필요, 이 태스크에서 실행하지 않음):** `get_package_share_directory('amr_person_tracking')`이 실제로 경로를 찾으려면 `colcon build`로 워크스페이스가 설치되어 있어야 하고, `ros2 launch robot_bridge robot.launch.py robot_id:=robot5`로 실제 구동해 `ros2 topic list`에 `/robot5/target_person_pose_raw`와 `/robot5/target_person_pose`가 둘 다 잡히는지, `ros2 topic info /robot5/target_person_pose`의 Publisher가 `worker_tracking_bridge_node` 하나뿐인지는 실물 로봇(또는 최소 depthai/라이다 장치가 연결된 환경)에서 사용자가 직접 확인해야 한다.

- [ ] **Step 3: 커밋**

```bash
git add src/robot_PC/robot_bridge/launch/robot.launch.py
git commit -m "$(cat <<'EOF'
robot.launch.py에 amr_person_tracking + worker_tracking_bridge_node 배선

GroupAction으로 감싼 SetRemap을 통해 reid_tracking_node의 target_person_pose
발행을 target_person_pose_raw로 리다이렉트하고, worker_tracking_bridge_node가
그 raw 토픽을 소비해 최종 target_person_pose를 발행하도록 배선한다.
EOF
)"
```

---

### Task 4: 전체 빌드/테스트 최종 확인

**Files:** 없음 (검증만).

**Interfaces:** 없음.

- [ ] **Step 1: colcon build (이 환경에 ultralytics/opencv/torch 설치되어 있음 — Task 2 사전 점검 결과)**

```bash
cd /home/rokey/team4_amr_assist && colcon build --packages-select amr_person_tracking robot_bridge --symlink-install
```

Expected: 두 패키지 모두 `Finished`. 만약 `depthai`/`depthai_ros_driver` 관련 import 에러가 나면(카메라 드라이버 하드웨어 패키지), 이는 실물 로봇 PC에만 있는 의존성일 수 있으니 에러 메시지를 그대로 사용자에게 보고하고 다음 단계로 넘어가지 않는다 — **여기서 막히면 사용자에게 depthai_ros_driver 설치 여부를 확인 요청**.

- [ ] **Step 2: robot_bridge 전체 pytest (colcon test 경유)**

```bash
cd /home/rokey/team4_amr_assist && colcon test --packages-select robot_bridge --event-handlers console_direct+
colcon test-result --verbose
```

Expected: `test_robot_bridge_node.py`, `test_pose_utils.py`, `test_worker_tracking_bridge_node.py` 전부 PASS, 실패 0건.

- [ ] **Step 3: git status로 의도치 않은 변경 없는지 최종 확인**

```bash
git status --short
git log --oneline main..HEAD
```

Expected: `git status --short`는 비어 있어야 하고(모두 커밋됨), `git log`에는 Task 1~4의 커밋들과 설계 문서 커밋 2개(총 6개 내외)만 보여야 한다.

---

## 이 계획으로 다루지 않는 것 (설계 문서 "알려진 한계" 참고)

- `worker_tracking/enable=False`일 때 `amr_person_tracking`의 검출 노드 자체를 멈추는 것 (발행만 게이팅됨, 자원 절약은 후속 작업).
- YOLO/ReID 모델 파일 확보, `depthai_ros_driver` 설치, 라이다 파라미터 실측 재검증, 엔드투엔드 추종 동작 확인 — 전부 실물 로봇에서 사용자가 직접 확인해야 하는 항목 (설계 문서의 "검증 범위" 표 참고).
