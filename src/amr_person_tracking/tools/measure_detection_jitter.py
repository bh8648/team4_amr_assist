#!/usr/bin/env python3
"""
measure_detection_jitter.py

"추종 좌표가 튀는 게 ID를 잘못 붙여서인가, 아니면 측정값 자체가 튀어서인가"를 가르는 도구.

reid_tracking_node를 거치기 **전**의 oakd 원시 검출 3D 좌표를, 같은 YOLO track_id의 연속
프레임 사이에서 직접 비교한다. 여기서 나온 점프가 최종 target_person_pose의 점프와 비슷하면
ID 매칭이 아니라 측정(발끝 픽셀/depth) 문제다.

[이 도구로 밝힌 것 - 2026-08-09, my_new_bag5]
  원시 측정 프레임간 점프: 중앙값 0.194m / 90% 0.658m / 최대 1.486m / 0.5m 초과 12회
  같은 실행의 target_person_pose 점프도 0.5m 초과 정확히 12회 -> 측정 노이즈가 그대로
  출력으로 통과하고 있었다(추적기가 위치를 측정값에 스냅하고 필터를 안 걸기 때문).
  변화를 성분 분해하면 depth 방향 0.4m 초과 15프레임 vs 가로 5프레임으로 **거리(depth)가
  주범**이었고, 큰 점프에서 배경 거리(2.51m)가 반복 등장했다 = 발끝 픽셀이 사람이 아니라
  뒤쪽 벽/바닥을 찍는 것. 이 진단이 vision_utils.estimate_person_depth 도입 근거다.

[주의] 로봇이 정지한 bag에서 써야 카메라계 점프 = 세계계 점프로 볼 수 있다(--check-odom이
자동 확인). 주행 중 bag이면 tf를 태워야 하므로 이 도구의 전제가 깨진다.

[사용법]
  python3 measure_detection_jitter.py --bag rosbag/my_new_bag5
"""

import argparse
import math
import sqlite3
import sys
from pathlib import Path

import cv2
import numpy as np


def _rows(cur, topic):
    cur.execute('SELECT id FROM topics WHERE name=?', (topic,))
    row = cur.fetchone()
    if row is None:
        raise KeyError(f'토픽 없음: {topic}')
    cur.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (row[0],))
    return cur.fetchall()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True, help='rosbag 디렉터리')
    ap.add_argument('--ns', default='/robot5')
    ap.add_argument('--pose-model', default='yolo11n-pose.pt')
    ap.add_argument('--conf', type=float, default=0.3)
    ap.add_argument('--max-dt', type=float, default=0.5, help='이 이상 벌어진 프레임쌍은 제외')
    ap.add_argument('--jump-threshold', type=float, default=0.5)
    args = ap.parse_args()

    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage, CameraInfo
    from nav_msgs.msg import Odometry
    from ultralytics import YOLO
    from amr_person_tracking.vision_utils import (
        backproject_pixel, decode_compressed_depth, estimate_foot_pixel,
        estimate_person_depth, sample_depth_patch)

    db = sorted(Path(args.bag).glob('*.db3'))
    if not db:
        print(f'{args.bag}에 .db3가 없습니다', file=sys.stderr)
        return 1
    conn = sqlite3.connect(str(db[0]))
    cur = conn.cursor()

    od = _rows(cur, f'{args.ns}/odom')
    p0 = deserialize_message(od[0][1], Odometry).pose.pose.position
    p1 = deserialize_message(od[-1][1], Odometry).pose.pose.position
    moved = math.hypot(p1.x - p0.x, p1.y - p0.y)
    print(f'로봇 이동량 {moved:.3f}m ' +
          ('(정지 - 카메라계 점프를 세계계 점프로 봐도 됨)' if moved < 0.3
           else '(주행 중! 이 도구의 전제가 깨집니다)'))

    ci = deserialize_message(_rows(cur, f'{args.ns}/oakd/rgb/camera_info')[0][1], CameraInfo)
    fx, fy, cx, cy = ci.k[0], ci.k[4], ci.k[2], ci.k[5]
    rgb = _rows(cur, f'{args.ns}/oakd/rgb/image_raw/compressed')
    dep = _rows(cur, f'{args.ns}/oakd/stereo/image_raw/compressedDepth')
    conn.close()
    dts = np.array([d[0] for d in dep])

    model = YOLO(args.pose_model)
    model.predictor = None
    variants = {'현재 파이프라인(발끝 depth 검증 포함)': {}, '검증 없이 발끝 depth 그대로': {}}
    jumps = {k: [] for k in variants}
    grades = {}
    substituted = 0
    total = 0

    for ts, data in rgb:
        frame = cv2.imdecode(
            np.frombuffer(deserialize_message(data, CompressedImage).data, np.uint8),
            cv2.IMREAD_COLOR)
        if frame is None:
            continue
        j = int(np.argmin(np.abs(dts - ts)))
        if abs(dts[j] - ts) > 60e6:
            continue
        depth_img = decode_compressed_depth(
            deserialize_message(dep[j][1], CompressedImage).data)
        if depth_img is None:
            continue
        res = model.track(frame, persist=True, classes=[0], conf=args.conf, verbose=False)[0]
        boxes = res.boxes
        if boxes is None or boxes.id is None:
            continue
        kx = res.keypoints.xy.cpu().numpy() if res.keypoints is not None else None
        kc = (res.keypoints.conf.cpu().numpy()
              if res.keypoints is not None and res.keypoints.conf is not None else None)
        t = ts / 1e9
        for i in range(len(boxes)):
            bbox = boxes.xyxy[i].cpu().numpy()
            tid = int(boxes.id[i])
            u, v, grade = estimate_foot_pixel(
                kx[i] if kx is not None else None, kc[i] if kc is not None else None, bbox, 0.5)
            grades[grade] = grades.get(grade, 0) + 1
            z_raw = sample_depth_patch(depth_img, u, v, 5, 0.001)
            if z_raw is None or z_raw <= 0:
                continue
            total += 1
            person_z = estimate_person_depth(depth_img, bbox, 0.001)
            z_fixed = z_raw
            if person_z is not None and abs(z_raw - person_z) > 0.4:
                z_fixed = person_z
                substituted += 1
            for key, z in (('현재 파이프라인(발끝 depth 검증 포함)', z_fixed),
                           ('검증 없이 발끝 depth 그대로', z_raw)):
                pt = backproject_pixel(u, v, z, fx, fy, cx, cy)
                prev = variants[key].get(tid)
                if prev is not None:
                    pt0, x0, y0, z0 = prev
                    dt = t - pt0
                    if 0 < dt < args.max_dt:
                        jumps[key].append(
                            math.dist((pt[0], pt[1], pt[2]), (x0, y0, z0)))
                variants[key][tid] = (t, pt[0], pt[1], pt[2])

    print(f'\n발끝 픽셀 등급 분포: {grades}')
    if total:
        print(f'발끝 depth를 사람 깊이로 대체: {substituted}/{total} ({substituted / total * 100:.1f}%)')
    for key in variants:
        arr = np.array(jumps[key])
        if arr.size == 0:
            print(f'\n{key}: 표본 없음')
            continue
        print(f'\n{key}  (n={arr.size})')
        print(f'  중앙값 {np.median(arr):.3f}m | 90% {np.percentile(arr, 90):.3f}m | '
              f'최대 {arr.max():.3f}m | {args.jump_threshold}m 초과 {(arr > args.jump_threshold).sum()}회')
    return 0


if __name__ == '__main__':
    sys.exit(main())
