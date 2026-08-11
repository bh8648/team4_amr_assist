# 공용 작업자 HMI

robot5와 robot11이 같은 React 소스를 사용하고 Vite mode 환경 파일로 로봇 ID만 구분한다. 관리자 백엔드는 `8000`, Robot5 HMI 백엔드는 `8005`, Robot11 HMI 백엔드는 `8011`을 사용한다.

## robot5

```bash
colcon build --base-paths src/robot_PC --symlink-install
source install/setup.bash
ros2 launch robot_bridge robot.launch.py robot_id:=robot5
cd robot_hmi_ui
npm install
npm run dev:robot5 -- --port 5180
```

브라우저 또는 태블릿에서 `http://<robot5-PC-IP>:5180`을 연다.

## robot11

```bash
colcon build --base-paths src/robot_PC --symlink-install
source install/setup.bash
ros2 launch robot_bridge robot.launch.py robot_id:=robot11
cd robot_hmi_ui
npm install
npm run dev:robot11 -- --port 5180
```

브라우저 또는 태블릿에서 `http://<robot11-PC-IP>:5180`을 연다.

백엔드가 프론트엔드와 다른 장비에 있으면 `.env.robot5` 또는 `.env.robot11`의 `VITE_HMI_API_URL`을 활성화하고 실제 주소를 입력한다.

## 검증

```bash
npm test
npm run build:robot5
npm run build:robot11
npm run test:e2e:robot5
npm run test:e2e:robot11
```
