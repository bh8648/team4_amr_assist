# amr_person_tracking 파이프라인 - 구현 기능 정리 & 시각화 확인 가이드

패키지: `src/amr_person_tracking`
전체 흐름: `oakd_detector_node` + `leg_detector_bridge_node` → `reid_tracking_node` → `predictive_avoidance_node`
(디버그용 `debug_viewer_node`가 oakd_detector_node의 오버레이를 자동으로 띄워줌)

한 번에 전부 기동:
```bash
ros2 launch amr_person_tracking amr_person_tracking.launch.py namespace:=robot5
```

---

## 1. oakd_detector_node - YOLO-pose 근접 3D 위치추정

**하는 일**: OAK-D RGB/Depth를 시간동기 구독 → YOLO11-pose로 사람 bbox+발끝(ankle) keypoint 추출 →
발끝 픽셀의 depth를 CameraInfo로 3D 역투영 → tf2로 base_link(근접판정)/map(발행) 변환 →
`Detection3DArray` 발행. 발끝 좌표 산출 방식을 3등급(`toe_direct`/`knee_corrected`/`angle_only`)으로
나눠 신뢰도(covariance)에 반영하고, 근접(<0.8m) 시 IR 센서와 교차확인해 안전모드 신호도 낸다.

| 항목 | 값 |
|---|---|
| 입력 | `oakd/rgb/image_raw/compressed`, `oakd/stereo/image_raw/compressedDepth`, `oakd/rgb/camera_info` |
| 출력 | `vision/detections_3d` (Detection3DArray), `vision/proximity_alert` (Bool, latched), `diagnostics` |
| 디버그 출력 | `vision/oakd_detector/debug/compressed` (CompressedImage, `publish_debug_image:=true`일 때만) |

### 시각화로 확인하는 방법
YOLO-pose 오버레이 영상이 여기서 나온다 - launch 시 `publish_debug_image` 기본값이 `true`라
`debug_viewer_node`가 자동으로 `cv2.imshow` 창을 띄운다 (X11/Wayland 세션 필요, SSH면 `-X`).

단독 실행 시에는 **`oakd_detector_node`와 `debug_viewer_node` 둘 다** 떠 있어야 창에 이미지가
보인다 - `debug_viewer_node`만 실행하면 구독할 이미지가 없어 창이 비어 있다:
```bash
ros2 run amr_person_tracking oakd_detector_node --ros-args -p publish_debug_image:=true &
ros2 run amr_person_tracking debug_viewer_node --ros-args -p image_topic:=/robot5/vision/oakd_detector/debug/compressed
# 또는 rqt로 대체
ros2 run rqt_image_view rqt_image_view /robot5/vision/oakd_detector/debug/compressed
```
오버레이에는 YOLO bbox/스켈레톤(`result.plot()`), 발끝 픽셀 원(노란색), 등급+거리 텍스트,
RGB/Depth 동기 시간차(`dt=`)가 같이 찍힌다. 등급이 계속 `angle_only`로만 나오면 keypoint 신뢰도
임계값(`kp_conf_threshold`)이나 조명/거리 문제를 의심할 것.

이미지가 안 뜰 때 확인 순서:
```bash
ros2 topic info /robot5/vision/oakd_detector/debug/compressed --verbose   # Publisher count가 0이면 oakd_detector_node 미실행
ros2 topic hz /robot5/vision/detections_3d                               # 실제 프레임이 처리되는지
ros2 topic echo /robot5/diagnostics --once                               # fps, tf_failures, depth_invalid 등
```

수치로 확인:
```bash
ros2 topic echo /robot5/vision/detections_3d --once   # Detection3D 배열, results[0]가 person 가설
ros2 topic echo /robot5/diagnostics                    # fps, nearest_distance_m, grade_* 카운트 등
```

---

## 2. leg_detector_bridge_node - 라이다 다리검출 (★ 외부 패키지 의존 제거 후 자체 구현)

### 왜 바뀌었나
원래 계획은 외부 패키지 `mowito/ros2_leg_detector`(ROS2 Foxy 기준, rosdep/apt 배포 없음, OpenCV
3.4.12 명시 의존)를 워크스페이스에 클론/빌드해 그 출력(`leg_detector_msgs/PersonArray`)을
편입시키는 것이었다. 그런데 이 워크스페이스가 Humble이라 그 패키지가 빌드되지 않는 것을 확인했다.

그래서 **외부 ROS2 패키지 의존을 완전히 제거**하고, `LaserScan`에서 다리를 직접 검출하는 로직을
`leg_detection_utils.py`에 순수 Python(rclpy도 불필요)으로 새로 구현했다. 즉 사용자 환경에
아무것도 새로 설치/빌드하지 않고 동일한 기능(라이다 기반 근접 다리검출 → 사람 후보 위치 →
재식별 노드로 편입)을 그대로 낸다. **바깥 인터페이스(출력 토픽/스키마/디버그 마커)는 이전과
동일하게 유지**했으므로 `reid_tracking_node`는 이 교체로 전혀 수정할 필요가 없었다. 바뀐 건
입력뿐: `people_tracked`(외부 패키지가 트래킹까지 끝낸 결과) 대신 가공 안 된 `LaserScan`을
직접 받아 검출+트래킹을 이 노드가 전부 수행한다. `package.xml`의 `leg_detector_msgs` 의존도
제거됐다 (더 이상 클론/빌드가 필요 없음).

**하는 일**: LaserScan → 극좌표를 직교좌표로 변환(무효 리턴은 클러스터 경계로 취급) →
인접 포인트 간 거리 기반 순차(jump-distance) 클러스터링 → 다리 크기(포인트 수, 폭
`leg_diameter_min`~`leg_diameter_max`)에 맞는 클러스터만 채택 → 가까운 다리 클러스터 두 개를
그리디로 사람 한 명(양다리)으로 짝짓기, `allow_single_leg_detection`이 참이면 짝 없는 다리도
단독 후보로 채택 → tf2로 map 변환 → `reid_tracking_node`의 트래커(`tracking_utils.Track`)와
동일한 방식(`match_track`)으로 프레임 간 지속 id(`leg_<n>`) 부여 → `Detection3DArray` 발행.

> 알고리즘은 mowito/ros2_leg_detector 같은 학습된 분류기(랜덤포레스트/SVM)가 아니라 거리/폭
> 임계값만 쓰는 기하 휴리스틱이다. "다리 두 개가 나란히 있으면 사람"이라는 핵심 신호는 잡아내지만,
> 의자/테이블 다리 같은 정적 물체를 사람으로 오검출할 여지는 원본보다 크다 - 실측 후
> `cluster_distance_threshold` / `leg_diameter_min` / `leg_diameter_max` / `leg_pair_max_distance`를
> 현장 라이다 특성(각해상도, 노이즈, 사람 다리 두께)에 맞춰 튜닝하는 것을 권장.

| 항목 | 값 |
|---|---|
| 입력 | `scan` (sensor_msgs/LaserScan) - 파라미터 `scan_topic`, 기본 `/robot5/scan` |
| 출력 | `vision/leg_detections_3d` (Detection3DArray, frame_id=map) |
| 디버그 출력 | `vision/leg_detections/markers` (MarkerArray, `publish_markers:=true` 기본) |

### 시각화로 확인하는 방법
라이다 스캔에는 카메라 이미지가 없어 오버레이를 그릴 수 없다 - RViz2의 MarkerArray로 확인한다.

```bash
rviz2
```
RViz Displays 패널에서:
1. `Fixed Frame`을 `map`으로 설정
2. `Add` → `By topic` → `/robot5/vision/leg_detections/markers` → `MarkerArray` 추가
   (주황 구체 = 검출된 다리쌍/다리 위치, 흰 텍스트 = `leg_<지속id>`)
3. 원본 스캔과 겹쳐 보려면 `Add` → `/robot5/scan` → `LaserScan` 추가 (점들 중 어떤 게
   다리 클러스터로 잡혔는지 눈으로 대조 가능)
4. TF 디스플레이도 추가해두면 로봇 base_link/laser 프레임 위치를 같이 확인할 수 있다

수치로 확인:
```bash
ros2 topic echo /robot5/vision/leg_detections_3d --once
ros2 topic hz /robot5/scan                         # 라이다 발행 주기 확인 (검출 안 되면 우선 확인)
```
검출이 하나도 안 뜨면: `ros2 topic echo /robot5/scan --once`로 `range_min/max`, `angle_increment`가
기대값인지, 그리고 `leg_detector_bridge_node` 파라미터의 `scan_range_limit`가 그 범위와 맞는지부터
확인할 것.

---

## 3. reid_tracking_node - 재식별/트래킹

**하는 일**: 웹캠(외부)/OAK-D/라이다 다리검출 세 출처의 `Detection3DArray`를 통합해 가려짐/프레임
이탈에도 같은 사람을 같은 지속 트랙 id로 유지한다.
- 웹캠↔OAK-D: 위치+속도 게이팅으로 매칭(`match_track`), 출처 전환 첫 프레임만 위치 블렌딩
- 웹캠/OAK-D ↔ 라이다 다리검출: 예측 위치로 시간정렬 → 게이팅 → 속도벡터 유사도로 모호성 해소
  → `lidar_lock_confirm_frames` 연속 프레임 최고 후보 시 "락온" → 락온 중엔 재매칭 없이 그
  라이다 트랙만 추종 → 락온 중에도 웹캠/OAK-D 원본과 계속 대조해(`swap_check_streak`) 어긋나면
  "ID 스왑 의심"으로 락온 해제, 재매칭

| 항목 | 값 |
|---|---|
| 입력 | `vision/detections_3d`(oakd), `<webcam_ns>/vision/detections_3d`(외부), `vision/leg_detections_3d` |
| 출력 | `vision/tracked_detections_3d` (Detection3DArray, id=지속 트랙ID), `target_person_pose` (PoseStamped, 현재 추종 대상) |

### 시각화로 확인하는 방법
`target_person_pose`는 표준 `PoseStamped`라 RViz가 바로 지원한다:
1. RViz `Add` → `/robot5/target_person_pose` → `Pose` 추가 (화살표로 추종 대상 위치/방향 표시)
2. 사람이 OAK-D↔라이다 구간을 오갈 때 화살표가 끊기지 않고 부드럽게 이어지는지가
   재식별이 제대로 동작하는지의 핵심 확인 포인트

`tracked_detections_3d`(Detection3DArray)는 RViz 기본 디스플레이 플러그인이 없어 CLI로 확인:
```bash
ros2 topic echo /robot5/vision/tracked_detections_3d
```
락온 상태 전환(락온 확정/유실/ID 스왑 의심)은 노드 로그로 바로 보인다:
```bash
ros2 launch amr_person_tracking amr_person_tracking.launch.py 2>&1 | grep "라이다 락온"
```
`ros2 topic hz /robot5/target_person_pose`로 추종 대상이 끊기는지(사람이 완전히 유실됐는지)도
같이 확인하면 좋다.

---

## 4. predictive_avoidance_node - 예측적 회피

**하는 일**: `tracked_detections_3d`의 트랙별 위치 시계열에 등속도 모델 2D 칼만필터
(`predictive_utils.ConstantVelocityKalman2D`)를 적용해 속도를 추정하고, `predict_horizon`초 뒤
예측 위치 주변에 속도 비례 반경의 가상 포인트를 만들거나(방식 A), 로봇 쪽 접근 속도 성분에 비례해
local_costmap의 inflation 반경/scaling factor를 동적으로 조정한다(방식 B). `avoidance_mode`
파라미터로 `pointcloud`/`costmap_params`/`both` 선택.

| 항목 | 값 |
|---|---|
| 입력 | `vision/tracked_detections_3d` |
| 출력 (방식 A) | `vision/predicted_obstacle_points` (PointCloud2) |
| 출력 (방식 B) | `<ns>/local_costmap/local_costmap/set_parameters` 서비스 호출 (inflation_radius, cost_scaling_factor) |

> 방식 A가 실제로 costmap에 장애물로 반영되려면 Nav2 voxel_layer의 `observation_sources`에
> `predicted_points_topic`이 등록돼 있어야 한다 (costmap 설정 소유자와 별도 협의 필요 - 코드
> 주석에 명시돼 있음).

### 시각화로 확인하는 방법
방식 A (가상 포인트):
1. RViz `Add` → `/robot5/vision/predicted_obstacle_points` → `PointCloud2` 추가
2. 추종 대상이 움직이면 그 진행 방향 앞쪽에 원형으로 점들이 나타나는지 확인 (속도가 빠를수록
   반경이 커짐 - `virtual_point_radius_base + virtual_point_radius_gain * speed`)
3. voxel_layer에 등록돼 있다면 Nav2 `Costmap2D` 디스플레이(local costmap)에서도 그 자리에
   장애물 코스트가 얹히는 것으로 이어서 확인 가능

방식 B (costmap inflation 동적 조정):
```bash
ros2 param get /robot5/local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /robot5/local_costmap/local_costmap inflation_layer.cost_scaling_factor
```
RViz에서 local_costmap `Costmap2D` 디스플레이를 띄워두면 사람이 로봇 쪽으로 빠르게 다가올 때
회피 영역(색이 번지는 반경)이 실시간으로 넓어지는 것을 눈으로 볼 수 있다.

속도 추정 자체가 맞는지 확인하려면 칼만필터 상태를 직접 볼 수는 없으니, 대신 예측 포인트의
움직임(위 PointCloud2)이 실제 사람 이동 방향/속도와 맞는지로 간접 검증한다.

---

## 전체 파이프라인을 한 번에 눈으로 검증하는 순서 (권장)

1. `rviz2` 실행 → Fixed Frame=`map`, TF, `/robot5/scan`(LaserScan), 로봇 모델/맵 추가
2. `ros2 launch amr_person_tracking amr_person_tracking.launch.py` 실행
   → `oakd_detector_node`의 cv2 디버그 창이 자동으로 뜬다 (사람이 카메라에 보이는지 확인)
3. RViz에 `/robot5/vision/leg_detections/markers`(MarkerArray) 추가 → 근접 시 다리검출 확인
4. RViz에 `/robot5/target_person_pose`(Pose) 추가 → 추종 대상이 OAK-D↔라이다 구간을 오갈 때
   화살표가 안 끊기는지 확인 (재식별/트래킹 검증)
5. RViz에 `/robot5/vision/predicted_obstacle_points`(PointCloud2) 추가 → 사람이 빠르게
   움직일 때 진행 방향에 가상 포인트가 나타나는지 확인 (예측 회피 검증)
6. 문제 생기면 `ros2 topic echo /robot5/diagnostics`로 oakd_detector_node 상태(fps, tf 실패,
   depth 무효 등)부터 확인
