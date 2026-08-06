#!/usr/bin/env python3

import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from ultralytics import YOLO


class Pc1WebcamWorkerNode(Node):
    def __init__(self) -> None:
        super().__init__('pc1_webcam_worker_node')

        # ------------------------------------------------------------------
        # PC1 역할:
        # 1) 외부 웹캠 RGB 프레임을 받는다.
        # 2) YOLO pose 모델로 작업자를 검출한다.
        # 3) 검출 결과를 화면에 시각화한다.
        # 4) 호모그래피 행렬이 준비되면 camera -> map 좌표로 변환한다.
        # 5) 아직 보정값이 없으면 mock map pose로 전체 시나리오를 먼저 검증한다.
        #
        # 현재 구현은 "간단 시나리오를 빨리 붙이는 최소 버전"이다.
        # 즉 사람 검출과 화면 시각화, worker pose 발행 구조는 바로 쓸 수 있다.
        #
        # 추후 채워야 할 부분:
        # - 웹캠 팀이 준 실제 homography matrix 반영
        # - 여러 사람 중 진짜 작업자 선택 규칙
        # - 수신호 클래스별 분기
        # - timestamp 동기화 정책
        # - Pose yaw 계산 정교화
        # ------------------------------------------------------------------
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('model_path', '/home/rokey/Downloads/yolov8n-pose.pt')
        self.declare_parameter('worker_pose_topic', '/pc1/worker_pose')
        self.declare_parameter('worker_status_topic', '/pc1/worker_status')
        self.declare_parameter('loop_hz', 10.0)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('show_window', True)
        self.declare_parameter('window_name', 'PC1 YOLO Worker View')
        self.declare_parameter('use_mock_pose_without_homography', True)
        self.declare_parameter('mock_worker_x', 1.0)
        self.declare_parameter('mock_worker_y', 0.0)
        self.declare_parameter('mock_worker_yaw', 0.0)
        # 빈 리스트를 기본값으로 선언하면 ROS2가 BYTE_ARRAY로 추론할 수 있다.
        # 실제로는 실수 9개를 받을 파라미터이므로 타입을 명시적으로 DOUBLE_ARRAY로 선언한다.
        self.declare_parameter('homography_matrix', Parameter.Type.DOUBLE_ARRAY)

        self.camera_index = int(self.get_parameter('camera_index').value)
        self.model_path = self.get_parameter('model_path').value
        self.worker_pose_topic = self.get_parameter('worker_pose_topic').value
        self.worker_status_topic = self.get_parameter('worker_status_topic').value
        self.loop_hz = float(self.get_parameter('loop_hz').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.window_name = self.get_parameter('window_name').value
        self.use_mock_pose_without_homography = bool(self.get_parameter('use_mock_pose_without_homography').value)
        self.mock_worker_x = float(self.get_parameter('mock_worker_x').value)
        self.mock_worker_y = float(self.get_parameter('mock_worker_y').value)
        self.mock_worker_yaw = float(self.get_parameter('mock_worker_yaw').value)
        self.homography_values = list(self.get_parameter('homography_matrix').value)

        self.pose_pub = self.create_publisher(PoseStamped, self.worker_pose_topic, 10)
        self.status_pub = self.create_publisher(String, self.worker_status_topic, 10)

        self.model = YOLO(self.model_path)
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f'Failed to open webcam index {self.camera_index}')

        self.homography_matrix = self.load_homography_matrix()
        self.last_detection_state = 'NO_DETECTION'

        timer_period = 1.0 / max(self.loop_hz, 0.5)
        self.create_timer(timer_period, self.main_loop)
        self.publish_status('READY', 'pc1 webcam worker node started')

    def load_homography_matrix(self) -> Optional[np.ndarray]:
        # 실제 보정값이 아직 없으면 빈 배열일 수 있다.
        # 그 경우 전체 흐름을 먼저 보려고 mock pose fallback을 쓴다.
        # 여기서 9개 값만 정확히 들어오면 아래 main_loop에서는 바로
        # image point -> map point 변환 경로를 타게 된다.
        if len(self.homography_values) != 9:
            return None
        try:
            return np.asarray(self.homography_values, dtype=float).reshape(3, 3)
        except ValueError:
            return None

    def main_loop(self) -> None:
        # 이 노드의 핵심 반복문이다.
        # 순서는 항상:
        # 1) 프레임 읽기
        # 2) YOLO 추론
        # 3) pose skeleton 시각화
        # 4) 대표 image point 계산
        # 5) homography 또는 mock pose로 map pose 생성
        # 6) pose publish
        #
        # 나중에 웹캠 팀 코드가 붙으면 이 함수 안에 timestamp, frame_id,
        # detection class 분기, confidence filtering 강화가 추가될 가능성이 크다.
        ret, frame = self.cap.read()
        if not ret:
            self.publish_status('ERROR', 'webcam frame read failed')
            return

        results = self.model.predict(source=frame, conf=self.confidence_threshold, verbose=False)
        if not results:
            self.render_frame(frame, None)
            self.publish_status('NO_DETECTION', 'no yolo results')
            return

        result = results[0]
        keypoints = self.extract_primary_keypoints(result)
        annotated = self.draw_pose_overlay(frame.copy(), keypoints)
        self.render_frame(annotated, keypoints)

        if keypoints is None:
            self.publish_status('NO_DETECTION', 'worker not detected')
            return

        image_point = self.compute_worker_image_point(keypoints)
        worker_pose = self.compute_worker_pose_from_image_point(image_point)
        if worker_pose is None:
            self.publish_status('NO_POSE', 'worker detected but map pose unavailable')
            return

        self.pose_pub.publish(worker_pose)
        self.publish_status('WORKER_POSE_READY', f'worker pose published x={worker_pose.pose.position.x:.3f}, y={worker_pose.pose.position.y:.3f}')

    def extract_primary_keypoints(self, result) -> Optional[np.ndarray]:
        # 첫 번째 사람을 단순히 primary target으로 쓴다.
        # 추후 작업자 선택 규칙이 생기면 여기서 ID 추적, 수신호 조건, 거리 우선순위를 넣으면 된다.
        try:
            if result.keypoints is None or result.keypoints.xy is None:
                return None
            xy = result.keypoints.xy
            if len(xy) == 0:
                return None
            first = xy[0]
            if hasattr(first, 'cpu'):
                return first.cpu().numpy()
            return np.asarray(first)
        except Exception:
            return None

    def compute_worker_image_point(self, keypoints: np.ndarray) -> tuple[float, float]:
        # 바닥 위에서 이동하는 AMR 접근 목표는 사람 "중심"보다 "바닥 접점에 가까운 점"이 더 유리하다.
        # 그래서 현재 우선순위는:
        # 1) 좌/우 발목(15, 16) 평균
        # 2) 발목 하나만 있으면 그 점 사용
        # 3) 발목이 없으면 좌/우 무릎(13, 14) 평균
        # 4) 그것도 없으면 전체 유효 keypoint 평균 fallback
        #
        # 추후 더 정교하게 가려면:
        # - 발끝/바닥선 투영
        # - bbox 하단 중심과 keypoint 혼합
        # - 발 keypoint confidence 비교
        # 같은 보정이 들어갈 수 있다.
        ankle_indices = [15, 16]
        knee_indices = [13, 14]

        ankle_points = self.collect_valid_points(keypoints, ankle_indices)
        if ankle_points:
            center = np.mean(np.asarray(ankle_points), axis=0)
            return float(center[0]), float(center[1])

        knee_points = self.collect_valid_points(keypoints, knee_indices)
        if knee_points:
            center = np.mean(np.asarray(knee_points), axis=0)
            return float(center[0]), float(center[1])

        valid = keypoints[~np.isnan(keypoints).any(axis=1)]
        if len(valid) == 0:
            h, w = 0.0, 0.0
            return w, h
        center = np.mean(valid, axis=0)
        return float(center[0]), float(center[1])

    def collect_valid_points(self, keypoints: np.ndarray, indices: list[int]) -> list[np.ndarray]:
        # 특정 keypoint 후보들 중 실제로 검출된 점만 모은다.
        # 현재는 NaN 여부만 보지만, 추후 confidence가 필요하면 여기서 함께 필터링하면 된다.
        valid_points: list[np.ndarray] = []
        for index in indices:
            if index >= len(keypoints):
                continue
            point = keypoints[index]
            if np.isnan(point[0]) or np.isnan(point[1]):
                continue
            valid_points.append(point)
        return valid_points

    def compute_worker_pose_from_image_point(self, image_point: tuple[float, float]) -> Optional[PoseStamped]:
        # 현재는 "외부 웹캠 대표점 1개"를 바로 map pose 1개로 바꾸는 구조다.
        # 실제 보정값이 들어오면 homography 경로를 타고,
        # 아직 보정이 없으면 mock pose로 중앙/AMR 파트를 먼저 검증한다.
        #
        # 추후 확장 포인트:
        # - detection confidence 기반 pose 무효화
        # - 화면 특정 영역 밖 검출 무시
        # - 작업자 yaw를 수신호 방향 또는 이동 방향으로 계산
        u, v = image_point

        if self.homography_matrix is not None:
            pixel_vec = np.array([u, v, 1.0], dtype=float)
            map_vec = self.homography_matrix @ pixel_vec
            if abs(map_vec[2]) < 1e-9:
                self.publish_status('ERROR', 'homography divide by zero')
                return None
            x = float(map_vec[0] / map_vec[2])
            y = float(map_vec[1] / map_vec[2])
            yaw = 0.0
            return self.make_pose(x, y, yaw)

        if self.use_mock_pose_without_homography:
            # 실제 보정값을 받기 전에는 mock pose를 써서 PC2/PC3 흐름을 먼저 테스트한다.
            return self.make_pose(self.mock_worker_x, self.mock_worker_y, self.mock_worker_yaw)

        return None

    def draw_pose_overlay(self, frame: np.ndarray, keypoints: Optional[np.ndarray]) -> np.ndarray:
        # 요청한 시각화는 bbox보다 skeleton 중심으로 맞춘다.
        if keypoints is None:
            cv2.putText(frame, 'NO WORKER DETECTED', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return frame

        skeleton_pairs = [
            (5, 7), (7, 9),
            (6, 8), (8, 10),
            (5, 6), (5, 11), (6, 12),
            (11, 12), (11, 13), (13, 15),
            (12, 14), (14, 16),
        ]

        for x, y in keypoints:
            if np.isnan(x) or np.isnan(y):
                continue
            cv2.circle(frame, (int(x), int(y)), 4, (0, 255, 255), -1)

        for a, b in skeleton_pairs:
            if a >= len(keypoints) or b >= len(keypoints):
                continue
            x1, y1 = keypoints[a]
            x2, y2 = keypoints[b]
            if np.isnan(x1) or np.isnan(y1) or np.isnan(x2) or np.isnan(y2):
                continue
            cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 200, 0), 2)

        u, v = self.compute_worker_image_point(keypoints)
        cv2.circle(frame, (int(u), int(v)), 7, (0, 0, 255), -1)
        cv2.putText(frame, f'worker center=({int(u)}, {int(v)})', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 255, 50), 2)
        return frame

    def render_frame(self, frame: np.ndarray, _keypoints: Optional[np.ndarray]) -> None:
        # 운영 시 headless 환경이면 show_window=false로 꺼둘 수 있다.
        # 지금은 개발 단계라서 시각적으로 바로 검출 여부를 보려고 OpenCV 창을 띄운다.
        if self.show_window:
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        # PC1의 최종 산출물은 결국 map 기준 PoseStamped 하나다.
        # PC2는 이 pose만 보고 goal을 만들기 때문에,
        # 이후 포맷을 바꾸더라도 최소한 frame_id='map' 계약은 유지하는 게 좋다.
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.yaw_to_quaternion(yaw)
        return pose

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        )

    def publish_status(self, state: str, detail: str) -> None:
        # NO_DETECTION이 너무 자주 반복되면 로그/상태 토픽이 불필요하게 시끄러워진다.
        # 그래서 같은 상태가 연속되면 일부는 억제한다.
        # 추후에는 JSON 필드에 timestamp, confidence, detection_count를 추가할 수 있다.
        if state == self.last_detection_state and state == 'NO_DETECTION':
            return
        self.last_detection_state = state
        self.status_pub.publish(String(data=f'{{"state":"{state}","detail":"{detail}"}}'))

    def destroy_node(self):
        # 웹캠 장치와 OpenCV 창은 명시적으로 닫아주는 편이 이후 재실행 시 안전하다.
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Pc1WebcamWorkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
