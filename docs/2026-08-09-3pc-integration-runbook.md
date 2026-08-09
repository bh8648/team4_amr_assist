# 웹캠 PC - 중앙 PC - AMR PC 통합 실행/테스트 런북

이 문서는 `feature/hju/robot11_bridge_node` 브랜치(웹캠 PC-중앙 PC-AMR PC 통합)를 실제로 돌리고 검증하기 위한 PC별 명령어 모음이다. 각 PC 섹션은 그 PC의 터미널에서 위에서 아래로 순서대로 실행하면 된다.

**PC 구성**: 웹캠 PC(사람 인식/호출), 중앙 PC(배정/작업관리/HMI/DB, `real_project/src/robot_manager`), AMR PC(robot11 실물, `real_project/src/robot_bridge`). 셋 다 같은 ROS2 네트워크(같은 `ROS_DOMAIN_ID`, 서로 통신 가능한 네트워크)에 있어야 한다.

**공통 사전 준비 (3개 PC 전부)**:
```bash
git clone <repo-url> team4_amr_assist   # 또는 이미 있으면 git pull
cd team4_amr_assist
git checkout feature/hju/robot11_bridge_node
source /opt/ros/humble/setup.bash
```

---

## 0. 먼저 할 것 — DockStatus 실제 값 확인 (AMR PC, robot_bridge 빌드 전)

이 브랜치의 도킹 안전 로직은 `irobot_create_msgs/msg/DockStatus`의 토픽명·필드명을 **가정값**으로 구현해뒀다. robot11 노트북에서 실제 로봇을 켜고 다음을 실행해서 확인해라:

```bash
ros2 topic list | grep -i dock
ros2 interface show irobot_create_msgs/msg/DockStatus
```

`real_project/src/robot_bridge/robot_bridge/robot11_bridge_node.py` 52~57번 줄(`⚠️` 주석 부분)의 토픽명(`/{robot_id}/dock_status`)과 필드명(`msg.is_docked`)이 실제와 다르면 그 두 줄만 고치면 된다. **이걸 확인/수정하기 전에는 로봇에 배정을 보내지 마라** — 가정이 틀리면 도킹 상태를 영원히 "모름"으로 판단해서(안전한 방향이지만) 로봇이 안 움직이거나, 값 자체가 다르면 최악의 경우 실제 안전장치가 무력화될 수 있다.

---

## 1. 중앙 PC — 빌드 + 유닛테스트 + 노드 실행

### 1-1. 빌드

```bash
cd team4_amr_assist/real_project
colcon build --packages-select robot_status robot_manager
source install/setup.bash
```

### 1-2. 유닛테스트 (하드웨어 없이 지금 바로 가능)

```bash
cd team4_amr_assist/real_project
pytest src/robot_manager/test/ -v
```
기대 결과: **42 passed**. (`conftest.py`/`pytest.ini`가 이미 커밋돼 있어서 추가 플래그 없이 plain `pytest`로 돈다.)

### 1-3. DB 준비 (최초 1회, `amr.db`가 이미 있으면 생략)

`real_project/amr.db`에는 이미 `destinations` 테이블에 `DEST_A`(목적지 A, -0.5,-2.0), `DEST_B`(목적지 B, -4.0,-3.0)가 들어있다. 배송모드 테스트에 이 두 destination_id를 쓰면 된다.

### 1-4. 5개 중앙 노드 실행

```bash
cd team4_amr_assist/real_project
export AMR_DB_PATH=$(pwd)/amr.db
export AMR_MAP_YAML=$(pwd)/amr_delivery_ui/frontend/maps/map2.yaml
ros2 launch robot_manager central_system.launch.py db_path:=$AMR_DB_PATH map_yaml_path:=$AMR_MAP_YAML
```
`amr_allocation_node`, `task_manager_node`, `hmi_manager_node`(HTTP :8000), `db_manager_node`, `deadlock_prevention_node` 5개가 한 번에 뜬다.

### 1-5. HMI 프론트엔드 실행 (별도 터미널, 같은 중앙 PC 또는 브라우저로 접근할 아무 PC)

```bash
cd team4_amr_assist/real_project/amr_delivery_ui/frontend
npm install   # 최초 1회
npm run dev
```
`http://<중앙PC-IP>:5173` 접속, 로그인 `rokey` / `1234`(환경변수 `HMI_USERNAME`/`HMI_PASSWORD`로 바꿀 수 있음, 기본값).

---

## 2. AMR PC (robot11) — 빌드 + 유닛테스트 + 브릿지 실행

### 2-1. 빌드

```bash
cd team4_amr_assist/real_project
colcon build --packages-select robot_status robot_bridge
source install/setup.bash
```

### 2-2. 유닛테스트 (로봇 없이도 가능 — `irobot_create_msgs`만 설치돼 있으면 됨, robot11 노트북에는 이미 있을 것)

```bash
cd team4_amr_assist/real_project
pytest src/robot_bridge/test/ -v
```
기대 결과: **36 passed**.

### 2-3. robot11 실물 스택 (Nav2/AMCL/Create3 드라이버가 이미 떠 있다는 전제)

```bash
cd team4_amr_assist/real_project
ros2 launch robot_bridge robot11_bridge.launch.py
```

---

## 3. 웹캠 PC — 빌드 + 노드 실행

person_locator/hand_gesture_caller/hardhat_detector는 저장소 최상위 `src/`에 있다(중앙/AMR PC와 별도 워크스페이스).

### 3-1. 빌드

```bash
cd team4_amr_assist
colcon build --packages-select person_locator hand_gesture_caller hardhat_detector
source install/setup.bash
```
(person_locator/hardhat_detector는 YOLO 모델(.pt)이 필요하다 — `models/` 아래 가중치 파일이 없으면 `colcon build`는 되지만 노드 실행 시 모델 로드에서 실패한다. 원래 main 브랜치에서 쓰던 모델 파일을 그대로 가져와야 한다.)

### 3-2. 카메라 발행 (실제 웹캠이 붙은 이 PC에서)

```bash
ros2 run person_locator camera_publisher
```

### 3-3. 사람 위치추정 + TF 브로드캐스트 (호모그래피 캘리브레이션이 이미 끝나있다는 전제 — `config/person_homography.yaml`)

```bash
ros2 launch person_locator person_locator.launch.py
```

### 3-4. 손짓 호출 감지

```bash
ros2 run hand_gesture_caller wrist_gesture_node
```

### 3-5. 안전모 감지 (선택 — 현재 중앙 PC 어디서도 이 신호를 구독하지 않으니, 이번 통합 테스트의 필수 경로는 아니다)

```bash
ros2 launch hardhat_detector hardhat_detector.launch.py
```

---

## 4. 통합 시나리오 테스트 체크리스트

전 PC가 다 뜬 상태에서 아래 순서로 확인한다. 왼쪽은 무엇을 트리거하는지, 오른쪽은 어디서 확인하는지다.

| # | 시나리오 | 트리거 | 확인 위치 |
|---|---|---|---|
| 1 | 도킹 상태 확인 | robot11 전원 켜고 대기 | AMR PC 로그에 `robot_status` 발행, 중앙 PC `ros2 topic echo /robot11/robot_status`에서 `is_docked`/`dock_status_known` 값 확인 |
| 2 | 손짓 호출 → 배정 | 웹캠 앞에서 손 폈다-쥐었다-폈다 | 중앙 PC 로그에 `AMR robot11 배정 성공`, HMI 화면에 robot11이 "작업자에게 이동"으로 바뀜 |
| 3 | 도킹 상태였던 경우 자동 언도킹 | (시나리오 2 유발 전 robot11이 도킹 중이었다면) | AMR PC에서 Undock 액션이 실행되는지, 중앙 PC 로그에 "언도킹 요청 발행" → 이후 "is_docked=False 확인" 로그 |
| 4 | 작업자 위치 도착 → 자동 FOLLOWING | robot11이 목적지에 실제 도착 | HMI에 "작업자 추종"으로 자동 전환(별도 명령 없이) |
| 5 | 배송모드 시작 | HMI에서 FOLLOWING 상태 로봇 선택 → 목적지 드롭다운(`DEST_A`/`DEST_B`) → "배송 시작" 버튼 | 로봇이 해당 좌표로 주행 시작, HMI가 "배송 중"으로 전환 |
| 6 | 배송지 도착 → 자동 복귀 | robot11이 배송지에 실제 도착 | HMI가 "복귀 중"으로 자동 전환, robot11이 도킹 위치로 주행 |
| 7 | 도킹 완료 | robot11이 도킹 위치 도착 + Dock 액션 성공 | HMI가 "도킹 완료"로 전환 |
| 8 | 취소(도킹확인 대기 중) | 시나리오 2~3 사이(도킹확인/언도킹 대기 중)에 HMI에서 "작업 취소" 클릭 | robot11이 움직이지 않아야 함, HMI가 "도킹 완료"로 바로 전환(ERROR로 안 빠져야 함 — 이번에 고친 부분) |
| 9 | 일시정지/재개 | HMI "일시정지" → "운행 재개" | 로봇이 즉시 멈추고, 재개 시 원래 목적지로 이어서 주행 |
| 10 | 교착 방지 | robot5(더미 없음이라 현재는 재현 어려움 — robot5 실물 브릿지 생기면 테스트) | 스킵 가능, 이번 범위 아님 |
| 11 | 텔레옵/도킹·언도킹 수동 조작 | HMI 하단 방향키 버튼, 도킹/언도킹 버튼 | robot11이 반응하는지 |

**하드웨어에서만 확인 가능한 나머지 항목**은 `docs/superpowers/specs/2026-08-08-webcam-central-amr-integration-design.md`의 "실제 로봇에서만 검증 가능한 항목" 절을 참고(도킹 물리 동작 타이밍, `robot11_dock_pose` 실측값, 3PC 네트워크, 센서 QoS 등).

---

## 5. 알려진 이슈 / 트러블슈팅

- **`irobot_create_msgs` 미설치**: 이 개발 환경(중앙 PC 역할로 쓴 머신)에는 없었다. robot11 AMR PC에는 Create3 드라이버와 함께 이미 설치돼 있어야 한다. 없으면 `sudo apt install ros-humble-irobot-create-msgs`.
- **`uvicorn`/`fastapi` 미설치**: 중앙 PC에서 `hmi_backend_node` 실행에 필요. `pip3 install uvicorn fastapi`. (이 브랜치의 `setup.py`에는 명시돼 있지 않으니 수동 설치 필요 — main도 동일한 상태.)
- **pytest가 `PluginValidationError`로 죽는 경우**: `real_project/conftest.py`, `real_project/pytest.ini`가 이미 이 문제(ROS Humble의 `launch_testing_ros` pytest 플러그인 충돌)를 해결해뒀다. 혹시 다른 디렉터리에서 pytest를 돌리면(즉 `real_project/pytest.ini`가 적용 안 되는 위치) `pytest -p no:launch_testing -p no:launch_ros ...`로 직접 플래그를 줘도 된다.
- **`hardhat_detector`/`person_locator` 모델 파일(.pt)**: 이 브랜치에는 코드만 반입됐고 학습된 가중치 파일 자체는 git에 없을 수 있다(용량 문제로 원래 main에서도 별도 관리했을 가능성). 웹캠 PC에 기존 모델 파일이 있는 경로를 그대로 쓰거나, `model_path` launch argument로 지정해라.
