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
