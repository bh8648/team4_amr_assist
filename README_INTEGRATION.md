# Team4 AMR Assist 통합본 안내

이 폴더는 2026-08-11 기준으로 아래 개발 흐름을 하나로 합친 로컬 통합본이다.
GitHub 원격 저장소에는 이 통합 과정에서 어떤 브랜치나 커밋도 push하지 않았다.

| 통합 기준 | 커밋 | 반영한 역할 |
|---|---|---|
| `origin/main` | `1567af2` | 최신 통합본과 Create3 실제 `DockStatus` 기반 HMI 도킹 상태 판정 |
| `feature/amr_person_tracking_merge` | `330c7a8` | AMR OAK-D·LiDAR 사람 인식/추종 파이프라인과 관련 문서 |
| 이전 `main` 기준 | `7ac3829` | 중앙 PC, DB/HMI, 목적지 4곳, 최신 공통 Robot bridge |
| `feature/integ/person_locator` | `b1f7bc8` | 새 웹캠 호출 방식, 호모그래피 cam1, 안전모 판정 연동 |

## 1. 장비 배치

| 구분 | 장비/역할 | IP |
|---|---|---|
| 중앙 PC | AMR 배정, Task 상태, DB, 중앙 HMI | `192.168.109.83` |
| 웹캠 PC | 카메라, YOLO-Pose, 손동작, 호모그래피, 안전모 판정 | `192.168.109.52` |
| AMR 5 PC | Robot 5 Nav2·추종·Robot bridge·부착 HMI | `192.168.109.51` |
| AMR 11 PC | Robot 11 Nav2·추종·Robot bridge·부착 HMI | `192.168.109.82` |
| TurtleBot 5 | 실제 TurtleBot 4 기반 AMR | `192.168.109.105` |
| TurtleBot 11 | 실제 TurtleBot 4 기반 AMR | `192.168.109.111` |

IP는 코드에 하드코딩하지 않았다. 모든 PC에서 같은 ROS Domain을 사용하고
`ROS_LOCALHOST_ONLY=0`으로 설정해야 한다. 메시지의 `header.stamp`와 센서 동기화를 위해
PC와 TurtleBot의 시스템 시간도 NTP/Chrony 등으로 맞춰야 한다.

## 2. 최종 동작 흐름

1. 웹캠 PC의 `camera_publisher`가 `/camera/image_raw/compressed`를 발행한다.
2. `pose_locator_node`가 작업자를 추적하고, 발 위치 픽셀을 cam1 호모그래피로
   Nav2 `map` 좌표로 바꾼다.
3. `wrist_gesture_node`가 `OPEN → CLOSED → OPEN`을 확인하면 웹캠 PC 내부 토픽인
   `/person/call_trigger`를 한 번 발행한다.
4. `pose_locator_node`가 캐시한 최신 작업자 `map` 좌표를
   `/person/call_position` (`geometry_msgs/PointStamped`)으로 한 번 발행한다.
5. 중앙 PC의 `robot_assignment_node`가 Robot 5/11 상태를 비교해 한 대를 배정하고,
   `task_manager_node`가 해당 AMR을 작업자 위치로 보낸다.
6. 도착한 AMR의 OAK-D·LiDAR 추종 파이프라인이
   `/<robot_id>/target_person_pose`를 발행한다. `robot_bridge_node`가 첫 유효 pose를
   `WORKER_DETECTED`로 변환해 `ASSIGNED → FOLLOWING` 전환을 요청한다.
7. FOLLOWING에서는 사람 위치 자체가 아니라 사람 앞 `0.9 m` 지점을 Nav2 goal로 사용한다.
   사람을 잃으면 Nav2 `Spin`으로 재탐색하고, 제한 시간까지 찾지 못하면
   `WORKER_LOST`를 중앙에 한 번만 보고한다.
8. 안전모 미착용이 감지되면 중앙 `task_manager_node`가 현재 FOLLOWING 중인 로봇에만
   경고음을 보낸다.

`/person/position`과 `/person/call_position`은 이미 호모그래피로 계산된 `map` 좌표이므로
별도의 TF 변환 노드가 필요하지 않다. `/person/call_trigger`는 웹캠 PC 내부 신호이고,
중앙 PC가 실제로 받는 호출 인터페이스는 `/person/call_position`이다.

## 3. 통합 과정에서 선택·수정한 사항

- AMR 추종 실행 구조는 최신 `main`의 직접 처리 방식을 선택했다.
  - `robot_bridge_node`가 첫 target pose, 안전거리 goal, 유실 watchdog과 Spin을 직접 처리한다.
  - 역할이 중복되는 구형 `worker_tracking_bridge_node`, 테스트, console entry point를 제거했다.
  - `/target_person_pose_raw` 리맵도 사용하지 않는다.
- AMR 인식 노드는 최신 `main` 버전을 사용하되,
  `leg_detector_bridge_node`의 TF listener는 별도 spin thread에서 돌도록 병합했다.
- 공식 BoT-SORT 설정인 `config/botsort_reid.yaml`을 기본 추적 설정으로 사용한다.
- `robot.launch.py`에 `pose_model_path`와 `tracker_config_path` 인자를 추가해
  Robot 5/11 공통 launch에서 하위 추종 launch까지 경로가 전달되게 했다.
- 웹캠 코드는 `feature/integ/person_locator` 버전을 사용했다.
  - `/person/call_trigger`는 내부 신호로 한정했다.
  - 중앙 호출은 좌표를 담은 `/person/call_position` 하나로 통일했다.
  - `person_tf_broadcaster_node`를 제거했다.
  - cam1 호모그래피와 JSON 파라미터, 안전모 검출을 반영했다.
- 중앙 `task_manager_node`에는 안전모 경고 기능만 병합했다.
  최신 `main`에서 제거된 `deadlock_prevention_node`와 `DeadlockPermission`은 되살리지 않았다.
- 최신 `main`의 DB 종료 처리, 지도 경로, HMI 테스트와 목적지 4곳을 반영했다.
- `origin/main` `1567af2`의 Create3 실제 `DockStatus` 기반 도킹 상태 판정을 반영했다.
- 작업자가 한 명인 현재 운영 조건에 맞춰 BoT-SORT의 confidence/association 기준과
  `reid_tracking_node`의 트랙 유실 유예·위치 게이트·단일 후보 재채택 기준을 완화했다.
  관련 값은 `amr_person_tracking.launch.py`의 launch 인자로 다시 조정할 수 있다.
- OAK-D 입력은 단일 사용자 전용으로 제한했다. 같은 쪽 무릎과 발목이 함께 보이지 않는
  얼굴·상체 위주의 원거리 검출은 버리고, 조건을 만족하는 박스가 여러 개면 하체 가시성이
  가장 좋은 한 명만 추적 입력으로 발행한다.
- 사용하지 않는 전용 외형 임베딩 기능은 기본 비활성화했다. 비활성 상태에서는 인코더와
  임베딩 토픽 구독·발행, `[reid]` 통계 타이머를 생성하지 않아 관련 로그도 나오지 않는다.
- AMR PC의 추종 launch는 기본으로 5초마다 `BANDWIDTH`, `NETWORK`, `QOS` 진단 로그를
  출력하고 `/<robot_id>/diagnostics`에도 같은 상태를 발행한다. 로컬 NIC의 RX/TX·drop·error와
  Raspberry Pi 카메라 및 TurtleBot4 센서 토픽의 수신률·최근 수신 시각·publisher·QoS
  incompatibility를 함께 확인한다. 원격 장비의 NIC 카운터를 직접 읽는 방식은 아니며,
  원격 두 구간의 대역폭은 실제 수신한 ROS 메시지 payload의 근삿값이다.
- 과거 worker bridge 설계 문서는 이력 보존용으로 남겼고, 문서 맨 위에
  최종 실행 기준이 아니라는 표시를 추가했다.

## 4. `yolo11n-pose.pt` 배치 위치

모델 가중치는 용량 때문에 이 ZIP에 포함하지 않았다. 또한 프로젝트 `.gitignore`가
`*.pt`를 제외하므로 GitHub에 올리는 파일이 아니라 각 실행 PC에 별도로 복사해야 한다.

같은 `yolo11n-pose.pt` 파일을 용도별 PC의 아래 위치에 둔다.

### AMR 5 PC와 AMR 11 PC

각 AMR PC의 저장소에서 다음 위치에 한 부씩 둔다.

```text
src/robot_PC/amr_person_tracking/config/yolo11n-pose.pt
```

`colcon build` 시 다음 설치 경로로 복사되며, `robot.launch.py`의 기본값이 이 위치를 본다.

```text
install/amr_person_tracking/share/amr_person_tracking/config/yolo11n-pose.pt
```

### 웹캠 PC

웹캠 PC의 저장소에서는 다음 위치에 둔다.

```text
src/webcam_PC/person_locator/models/yolo11n-pose.pt
```

빌드 후 기본 경로는 다음과 같다.

```text
install/person_locator/share/person_locator/models/yolo11n-pose.pt
```

안전모 기능을 사용할 경우 학습된 안전모 모델도 별도로 필요하다.

```text
src/webcam_PC/hardhat_detector/models/detect_warn_yolo11n_best.pt
```

모델을 저장소 밖에 둘 수도 있다. 이때 launch 인자로 절대 경로를 전달한다.

```bash
# AMR 5 예시
ros2 launch robot_bridge robot.launch.py \
  robot_id:=robot5 \
  pose_model_path:=/home/rokey/models/yolo11n-pose.pt

# 웹캠 PC 예시
ros2 launch person_locator person_locator.launch.py \
  model_path:=/home/rokey/models/yolo11n-pose.pt

# 안전모 모델 예시
ros2 launch hardhat_detector hardhat_detector.launch.py \
  model_path:=/home/rokey/models/detect_warn_yolo11n_best.pt
```

## 5. PC별 빌드

이 저장소에는 중앙 PC용과 AMR PC용으로 이름이 같은 `robot_status` 패키지가 각각 있다.
따라서 저장소 루트 전체를 한 번에 빌드하지 말고, 각 물리 PC에서 자기 폴더만 빌드한다.

### 중앙 PC

```bash
cd ~/team4_amr_assist
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/main_PC
source install/setup.bash
ros2 launch robot_manager central_system.launch.py
```

### 웹캠 PC

먼저 위 모델 파일을 배치한 뒤 빌드한다.

```bash
cd ~/team4_amr_assist
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/webcam_PC
source install/setup.bash
```

현재 통합본에는 네 노드를 한 번에 띄우는 launch가 없으므로 각각 실행한다.

```bash
ros2 run person_locator camera_publisher
ros2 launch person_locator person_locator.launch.py
ros2 run hand_gesture_caller wrist_gesture_node
ros2 launch hardhat_detector hardhat_detector.launch.py
```

### AMR 5 PC

Nav2, OAK-D, LiDAR와 Robot 5 TF가 먼저 정상 발행되는 상태에서 실행한다.

```bash
cd ~/team4_amr_assist
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/robot_PC
source install/setup.bash
ros2 launch robot_bridge robot.launch.py robot_id:=robot5
```

진단은 기본 활성화되어 있으며 자동 선택된 NIC가 실제 통신 인터페이스와 다르면 명시한다.

```bash
ros2 launch robot_bridge robot.launch.py \
  robot_id:=robot5 \
  network_interface:=wlan0 \
  bandwidth_warn_mbps:=80.0
```

첫 10초는 DDS discovery를 기다려 연결 경고를 유예한다. 이후 RGB/depth, scan, odom, tf의
publisher 부재·수신 정지·기준 미달 Hz 또는 incompatible QoS가 있으면 WARN으로 출력한다.
진단이 필요 없는 운영에서는 `enable_transport_diagnostics:=false`로 끌 수 있다.

### AMR 11 PC

```bash
cd ~/team4_amr_assist
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/robot_PC
source install/setup.bash
ros2 launch robot_bridge robot.launch.py robot_id:=robot11
```

`robot.launch.py`는 추종 파이프라인, 공통 Robot bridge와 로봇 부착 HMI 백엔드를 함께
실행한다. 추종을 빼고 bridge/HMI만 점검하려면 `enable_person_tracking:=false`를 붙인다.

중앙 React 화면을 사용할 때는 중앙 PC에서 프런트엔드 의존성을 설치해 실행한다.

```bash
cd ~/team4_amr_assist/amr_delivery_ui/frontend
npm ci
npm run dev -- --host 0.0.0.0
```

## 6. 배포 전 확인

```bash
# 공통 ROS 설정 확인
echo "$ROS_DOMAIN_ID"
echo "$ROS_LOCALHOST_ONLY"

# 웹캠 호출 인터페이스
ros2 topic echo /person/call_position

# 중앙에서 두 AMR 상태 수신 확인
ros2 topic echo /robot_status

# 선택된 로봇의 로컬 추종 출력 확인
ros2 topic echo /robot5/target_person_pose
# 또는
ros2 topic echo /robot11/target_person_pose
```

모델 파일 존재 여부도 빌드 전에 확인한다.

```bash
test -f src/robot_PC/amr_person_tracking/config/yolo11n-pose.pt
test -f src/webcam_PC/person_locator/models/yolo11n-pose.pt
```

## 7. 아직 실기 확인이 필요한 사항

- 이 통합본에는 모델 가중치가 없으므로 모델을 넣기 전에는 실제 YOLO 추론이 시작되지 않는다.
- 실제 TurtleBot 5/11의 Nav2·OAK-D·LiDAR·TF 연결과 전체 ROS 2 빌드는 각 장비에서 확인해야 한다.
- 안전모 상태 메시지는 작업자 ID를 담지 않는 전역 `Bool`이다. 현재처럼 한 작업자를
  한 로봇이 추종하는 시나리오에는 맞지만, 두 작업자를 동시에 추종하려면 메시지 확장이 필요하다.
- `/person/position`은 현재 디버깅/HMI용 연속 좌표이고 중앙 배정에는 사용하지 않는다.
- 호모그래피 기준 지도와 두 AMR의 Nav2 지도가 동일해야 하며, 웹캠 위치나 각도가 바뀌면
  `person_homography_cam1.yaml`을 다시 보정해야 한다.
