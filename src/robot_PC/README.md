# Robot5/Robot11 공통 실행 패키지

이 폴더의 동일한 ROS 2 패키지를 Robot5 PC와 Robot11 PC에 배포한다. 실행 시
`robot_id`만 다르게 지정하며, 브릿지와 HMI 백엔드는 해당 ID의 상태와 토픽만
처리한다.

## 빌드

저장소 루트에서 다음을 실행한다.

```bash
colcon build --base-paths src/robot_PC --symlink-install
source install/setup.bash
```

## 사람 추적(amr_person_tracking) 의존성

`robot.launch.py`는 기본적으로 `amr_person_tracking`(사람 검출·추적) 파이프라인도
함께 띄운다. 이 패키지는 rosdep으로 안 잡히는 pip 의존성을 쓴다:

```bash
pip install ultralytics opencv-python scipy numpy
```

그리고 rosdep 대상 의존성(`depthai_ros_driver`, `irobot_create_msgs`)이 설치되어
있어야 한다. YOLO pose 모델(`yolo11n-pose.pt`)과 ReID onnx 모델은 저장소에
커밋되어 있지 않다(`.gitignore` 정책) — 인터넷이 되는 PC라면 ultralytics가 처음
실행 시 자동 다운로드하고, 오프라인이라면 미리 받아 배치해야 한다.

카메라/라이다 등 하드웨어나 위 의존성이 아직 준비되지 않은 PC에서 브릿지와 HMI만
먼저 확인하려면 추적 파이프라인을 꺼서 띄울 수 있다:

```bash
ros2 launch robot_bridge robot.launch.py robot_id:=robot5 enable_person_tracking:=false
```

## Robot5 PC

```bash
ros2 launch robot_bridge robot.launch.py robot_id:=robot5
```

## Robot11 PC

```bash
ros2 launch robot_bridge robot.launch.py robot_id:=robot11
```

관리자 백엔드의 `8000` 포트와 충돌하지 않도록 Robot5 HMI는 `8005`, Robot11
HMI는 `8011`을 자동 사용한다. 필요할 때만 `web_port:=...`로 재정의한다.

브릿지는 `/<robot_id>/...` 로컬 토픽과 액션만 사용하며, `/task/state` 같은 공통
토픽에서는 메시지의 `robot_id`가 자신의 설정과 일치할 때만 처리한다.
