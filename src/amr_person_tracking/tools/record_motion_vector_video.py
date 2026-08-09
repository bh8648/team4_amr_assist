#!/usr/bin/env python3
"""
record_motion_vector_video.py

이동 벡터(속도 추정)와 예측 회피 출력을 **위에서 내려다본 map 평면**으로 그려 mp4로 녹화한다.

카메라 영상 오버레이로는 "지금 이 사람이 어느 방향으로 얼마나 빠르게 가고 있다고 파이프라인이
믿는지"가 보이지 않는다. 그건 map 좌표계의 벡터라서 위에서 내려다봐야 한다.

[무엇을 그리나]
  - 로봇 위치(원점, tf base_link)와 진행 방향
  - `tracked_detections_3d`의 각 트랙 위치와 지속 ID
  - `predicted_obstacle_points`(predictive_avoidance_node가 내는 가상 포인트 링)
  - 트랙별 **이동 벡터 화살표**(predict_horizon 뒤 예측 위치까지)

[속도를 왜 여기서 다시 계산하나]
`predictive_avoidance_node`는 속도를 토픽으로 내보내지 않고 예측 결과(가상 포인트 링)만
발행한다. 처음에는 "트랙에서 가장 가까운 링 중심"으로 화살표를 그렸는데, 사람이 둘 이상이면
링과 트랙이 잘못 짝지어져 4m/s 같은 허구의 속도가 표시됐다. 그래서 이 도구는 노드와 **같은
클래스·같은 기본 파라미터**(ConstantVelocityKalman2D, process_noise=1.0,
measurement_noise=0.1)로 트랙별 속도를 직접 추정해 그린다 - 노드 내부 추정을 충실히 재현한
값이며, 노드가 실제로 낸 가상 포인트는 배경에 함께 겹쳐 그려 서로 대조할 수 있게 했다.

[사용법]
  export ROS_DISCOVERY_SERVER="" ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77
  python3 record_motion_vector_video.py out.mp4 --fps 10
  # 다른 터미널에서 ros2 bag play, 끝나면 Ctrl-C
"""

import argparse
import math
import shutil
import subprocess
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection3DArray

from amr_person_tracking.predictive_utils import ConstantVelocityKalman2D


class MotionVectorRenderer(Node):

    def __init__(self, args):
        super().__init__('record_motion_vector_video')
        self.args = args
        self.frames = []
        self.latest_tracks = []      # [(id, x, y, (vx, vy)), ...]
        self.latest_pred = []        # [(x, y), ...] 가상 포인트
        self.n_track_msgs = 0
        self.n_pred_msgs = 0
        self.speeds = []          # 화살표로 그린 속도 표본(진단용)
        self.kalman_states = {}   # track id -> ConstantVelocityKalman2D (노드와 동일 재현)
        self.robot_xy = None      # map_frame에서의 로봇 위치 (TF)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Detection3DArray, args.tracked_topic, self.on_tracks, 10)
        self.create_subscription(
            PointCloud2, args.predicted_topic, self.on_predicted, qos_profile_sensor_data)
        self.create_timer(1.0 / args.fps, self.render)

    def _lookup_robot(self):
        """map_frame에서의 로봇 위치. 캔버스를 여기에 맞춰야 거리 링이 실제 이격거리가 된다.

        트랙 좌표는 map_frame(이 bag에서는 odom) 기준이라 로봇을 원점에 고정해 그리면
        거리 링이 'odom 원점에서의 거리'가 돼 사람과 로봇 사이 거리를 잘못 읽게 된다.
        """
        try:
            tr = self.tf_buffer.lookup_transform(
                self.args.map_frame, self.args.base_frame, rclpy.time.Time())
        except Exception:
            return
        self.robot_xy = (tr.transform.translation.x, tr.transform.translation.y)

    def on_tracks(self, msg):
        self.n_track_msgs += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tracks = []
        for det in msg.detections:
            for r in det.results:
                if r.hypothesis.class_id != 'person':
                    continue
                x, y = r.pose.pose.position.x, r.pose.pose.position.y
                kf = self.kalman_states.get(det.id)
                if kf is None:
                    kf = ConstantVelocityKalman2D(
                        x, y, stamp, process_noise=self.args.kf_process_noise,
                        measurement_noise=self.args.kf_measurement_noise)
                    self.kalman_states[det.id] = kf
                else:
                    kf.update(x, y, stamp)
                tracks.append((det.id, x, y, kf.velocity()))
        self.latest_tracks = tracks
        # 오래된 트랙 상태 정리
        for tid in [t for t, k in self.kalman_states.items()
                    if stamp - k.last_stamp > 3.0]:
            del self.kalman_states[tid]

    def on_predicted(self, msg):
        self.n_pred_msgs += 1
        self.latest_pred = [(p[0], p[1]) for p in
                            point_cloud2.read_points(msg, ('x', 'y'), skip_nans=True)]

    # --------------------------------------------------------------- 렌더링

    def _to_px(self, x, y):
        """map(m) -> 캔버스 픽셀. 로봇을 캔버스 중앙에 두고 +x가 위쪽.

        로봇을 아래쪽에 두면 로봇 뒤(-x)에 있는 사람이 화면 밖으로 잘린다 - 이 현장 bag이
        실제로 그랬다. 전방향을 동등하게 보여주는 편이 안전하다.
        """
        rx, ry = self.robot_xy or (0.0, 0.0)
        s = self.args.px_per_m
        c = self.args.size // 2
        return int(c - (y - ry) * s), int(c - (x - rx) * s)

    def render(self):
        self._lookup_robot()
        n = self.args.size
        img = np.full((n, n, 3), 28, dtype=np.uint8)

        # 거리 링(1m 간격)과 로봇
        for r in range(1, int(self.args.range_m) + 1):
            rx, ry = self.robot_xy or (0.0, 0.0)
            cv2.circle(img, self._to_px(rx, ry), int(r * self.args.px_per_m), (55, 55, 55), 1)
            tx, ty = self._to_px(rx + r, ry)
            cv2.putText(img, f'{r}m', (tx + 4, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (90, 90, 90), 1, cv2.LINE_AA)
        rob = self._to_px(*(self.robot_xy or (0.0, 0.0)))
        cv2.circle(img, rob, 7, (0, 200, 255), -1)
        cv2.putText(img, 'ROBOT', (rob[0] - 24, rob[1] + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 200, 255), 1, cv2.LINE_AA)

        # 예측 가상 포인트 링
        for px, py in self.latest_pred:
            cv2.circle(img, self._to_px(px, py), 2, (120, 120, 255), -1)

        # 트랙과 이동 벡터 (트랙별 KF에서 직접 - 링 매칭 artifact 없음)
        for tid, x, y, (vx, vy) in self.latest_tracks:
            p = self._to_px(x, y)
            cv2.circle(img, p, 9, (255, 0, 255), 2)
            cv2.putText(img, f'id:{tid}', (p[0] + 12, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 0, 255), 1, cv2.LINE_AA)
            speed = math.hypot(vx, vy)
            self.speeds.append(speed)
            if speed > 0.05:
                h = self.args.predict_horizon
                cv2.arrowedLine(img, p, self._to_px(x + vx * h, y + vy * h),
                                (80, 255, 80), 2, tipLength=0.25)
                mid = self._to_px(x + vx * h / 2, y + vy * h / 2)
                cv2.putText(img, f'{speed:.2f} m/s', (mid[0] + 8, mid[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 255, 80), 1, cv2.LINE_AA)

        legend = [('ROBOT (TF %s<-%s, 링=1m 간격 실거리)' % (self.args.map_frame,
                                                          self.args.base_frame),
                   (0, 200, 255)),
                  ('추종/트랙 위치', (255, 0, 255)),
                  ('이동 벡터 (predict_horizon 뒤 예측)', (80, 255, 80)),
                  ('예측 가상 포인트(costmap 마킹용)', (120, 120, 255))]
        for i, (text, color) in enumerate(legend):
            cv2.putText(img, text, (10, 22 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        color, 1, cv2.LINE_AA)

        self.frames.append(img)


def _write_video(frames, out_path, fps):
    h, w = frames[0].shape[:2]
    if shutil.which('ffmpeg'):
        cmd = ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
               '-s', f'{w}x{h}', '-r', str(fps), '-i', 'pipe:0',
               '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-preset', 'veryfast', out_path]
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        for f in frames:
            proc.stdin.write(f.tobytes())
        proc.stdin.close()
        proc.wait()
        return
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    for f in frames:
        writer.write(f)
    writer.release()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out')
    ap.add_argument('--tracked-topic', default='/robot5/vision/tracked_detections_3d')
    ap.add_argument('--predicted-topic', default='/robot5/vision/predicted_obstacle_points')
    ap.add_argument('--fps', type=float, default=10.0)
    ap.add_argument('--size', type=int, default=700, help='정사각 캔버스 한 변(px)')
    ap.add_argument('--range-m', type=float, default=5.0, help='표시할 최대 거리(m)')
    ap.add_argument('--predict-horizon', type=float, default=1.0,
                    help='화살표 길이 = 속도 x 이 시간. 노드의 predict_horizon과 맞추면 좋다')
    ap.add_argument('--map-frame', default='map', help='트랙 좌표계. 이 bag 재생 시엔 odom')
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--kf-process-noise', type=float, default=1.0)
    ap.add_argument('--kf-measurement-noise', type=float, default=0.1)
    # --ros-args 이후는 전부 ROS로 넘긴다(TF 토픽 remap용). argparse에 흘리면 위치인자로
    # 잘못 먹힌다.
    argv = sys.argv[1:]
    split = argv.index('--ros-args') if '--ros-args' in argv else len(argv)
    args = ap.parse_args(argv[:split])
    args.px_per_m = (args.size * 0.8) / (2 * args.range_m)

    rclpy.init(args=sys.argv[:1] + argv[split:])
    node = MotionVectorRenderer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    except Exception as exc:
        print(f'수집 중단({type(exc).__name__}: {exc})', file=sys.stderr)

    print(f'tracked_detections {node.n_track_msgs}건 / predicted_points {node.n_pred_msgs}건',
          file=sys.stderr)
    if node.speeds:
        a = np.array(node.speeds)
        print(f'이동벡터 크기(m/s) n={a.size}: 중앙값 {np.median(a):.2f} | '
              f'90% {np.percentile(a, 90):.2f} | 최대 {a.max():.2f} | '
              f'사람 보행 상한(2.0m/s) 초과 {(a > 2.0).mean() * 100:.0f}%', file=sys.stderr)
    if node.frames:
        _write_video(node.frames, args.out, args.fps)
        print(f'{len(node.frames)}프레임 -> {args.out}', file=sys.stderr)
    else:
        print('렌더한 프레임이 없습니다', file=sys.stderr)
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
