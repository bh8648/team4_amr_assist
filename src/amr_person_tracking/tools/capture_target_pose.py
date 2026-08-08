#!/usr/bin/env python3
"""
capture_target_pose.py

reid_tracking_node의 최종 융합 출력을 캡처해 추종 안정성을 정량화하는 검증 도구.

`target_person_pose`(로봇이 실제로 따라갈 좌표)와 `tracked_detections_3d`(그 순간 융합
출력에 잡힌 트랙들)를 함께 기록한다. 사람이 낼 수 없는 속도의 순간이동이 있으면 매칭이
엉뚱한 검출에 붙었다는 뜻이므로, 프레임 간 이동거리가 핵심 지표다.

[사용법]
  # 반드시 격리된 도메인에서 (실기 접속 셸 설정이면 실시간 로봇 데이터가 섞여 들어온다)
  export ROS_DISCOVERY_SERVER="" ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77
  python3 capture_target_pose.py out.txt        # 다른 터미널에서 ros2 bag play
  python3 capture_target_pose.py --analyze out.txt --bag-start <epoch> [--from 55 --to 68]

[출력 형식] 한 줄에 하나: "<stamp> <x> <y> :: <track_id>,<x>,<y> | ..."
"""

import argparse
import math
import sys


def _record(out_path, target_topic, tracked_topic):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from vision_msgs.msg import Detection3DArray

    records = []

    class Capture(Node):
        def __init__(self):
            super().__init__('capture_target_pose')
            self.latest_tracks = []
            self.create_subscription(Detection3DArray, tracked_topic, self.on_tracks, 10)
            self.create_subscription(PoseStamped, target_topic, self.on_pose, 10)

        def on_tracks(self, msg):
            tracks = []
            for det in msg.detections:
                for r in det.results:
                    if r.hypothesis.class_id == 'person':
                        tracks.append((det.id, r.pose.pose.position.x, r.pose.pose.position.y))
            self.latest_tracks = tracks

        def on_pose(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            records.append(
                (stamp, msg.pose.position.x, msg.pose.position.y, list(self.latest_tracks)))

    rclpy.init()
    node = Capture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    with open(out_path, 'w') as f:
        for stamp, x, y, tracks in records:
            tstr = ' | '.join(f'{tid},{tx},{ty}' for tid, tx, ty in tracks)
            f.write(f'{stamp:.3f} {x} {y} :: {tstr}\n')
    print(f'{len(records)}개 target_person_pose 기록 -> {out_path}', file=sys.stderr)
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


def _load(path):
    recs = []
    for line in open(path).read().strip().split('\n'):
        if not line.strip():
            continue
        head, tracks_str = line.split(' :: ')
        stamp, x, y = head.split()
        tracks = []
        if tracks_str.strip():
            for chunk in tracks_str.split(' | '):
                tid, tx, ty = chunk.split(',')
                tracks.append((tid, float(tx), float(ty)))
        recs.append((float(stamp), float(x), float(y), tracks))
    return recs


def _analyze(path, bag_start, t_from, t_to, jump_threshold):
    recs = _load(path)
    print(f'총 {len(recs)}개 기록')
    if bag_start is None and recs:
        bag_start = recs[0][0]
        print(f'(--bag-start 미지정 - 첫 기록 시각을 0으로 둠)')

    prev = None
    max_jump = 0.0
    jumps = []
    max_tracks = 0
    for stamp, x, y, tracks in recs:
        t = stamp - bag_start
        if not (t_from <= t <= t_to):
            continue
        max_tracks = max(max_tracks, len(tracks))
        if prev is not None:
            d = math.hypot(x - prev[1], y - prev[2])
            dt = stamp - prev[0]
            speed = d / dt if dt > 0 else float('inf')
            if d > jump_threshold:
                jumps.append((round(t, 2), round(d, 2), round(speed, 2)))
            max_jump = max(max_jump, d)
        prev = (stamp, x, y)

    print(f'구간 [{t_from}, {t_to}]s')
    print(f'  최대 순간 점프: {max_jump:.2f}m')
    print(f'  {jump_threshold}m 초과 점프: {len(jumps)}건')
    for t, d, v in jumps:
        print(f'    t={t}s  {d}m  {v}m/s')
    print(f'  동시 트랙 수 최대: {max_tracks}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('path', help='기록 파일 경로 (기록 모드=출력, --analyze=입력)')
    ap.add_argument('--analyze', action='store_true', help='기록 대신 기존 파일을 분석')
    ap.add_argument('--bag-start', type=float, default=None,
                    help='bag metadata.yaml의 starting_time epoch (상대 시각 계산용)')
    ap.add_argument('--from', dest='t_from', type=float, default=float('-inf'))
    ap.add_argument('--to', dest='t_to', type=float, default=float('inf'))
    ap.add_argument('--jump-threshold', type=float, default=0.5, help='점프로 셀 최소 이동(m)')
    ap.add_argument('--target-topic', default='/robot5/target_person_pose')
    ap.add_argument('--tracked-topic', default='/robot5/vision/tracked_detections_3d')
    args = ap.parse_args()

    if args.analyze:
        _analyze(args.path, args.bag_start, args.t_from, args.t_to, args.jump_threshold)
    else:
        _record(args.path, args.target_topic, args.tracked_topic)


if __name__ == '__main__':
    main()
