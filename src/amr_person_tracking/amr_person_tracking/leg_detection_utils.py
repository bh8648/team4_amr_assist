#!/usr/bin/env python3
"""
leg_detection_utils.py

leg_detector_bridge_node가 쓰는 순수 함수 모음. rclpy에 의존하지 않아 노드 없이 단위 검증이
가능하다 (tracking_utils.py/vision_utils.py와 동일한 패턴).

mowito/ros2_leg_detector(Foxy 기준, leg_detector_msgs 커스텀 인터페이스 필요)를 이 워크스페이스
(Humble)에 빌드하려 했으나 호환이 안 돼, 외부 패키지 없이 LiDAR 드라이버가 표준으로 내보내는
sensor_msgs/LaserScan만으로 다리쌍을 직접 검출하는 경량 휴리스틱으로 대체했다.

[알고리즘]
  1. scan_to_points   - LaserScan 극좌표 -> 라이다 프레임 (x, y) 목록. 무효 리턴/범위초과는 None
  2. cluster_points    - 인접 포인트 간 거리가 cluster_distance_threshold를 넘으면 클러스터 분리
                         (무효 리턴도 클러스터 경계로 취급)
  3. filter_leg_clusters - 점 개수와 클러스터 폭(첫-끝 포인트 거리)이 사람 다리 굵기 범위
                         (leg_diameter_min ~ max)에 드는 클러스터만 다리 후보로 남김
  4. pair_legs         - 가까운 다리 후보 두 개를 그리디로 묶어 사람 중심점으로. 짝을 못 찾은
                         다리는 allow_single_leg가 참이면 단독으로도 사람으로 인정
                         (한쪽 다리가 다른 쪽에 가려 한 다리만 보이는 경우가 흔해서)

정식 트래커(가중치 학습, 칼만필터 등) 없이 거리/폭 임계값만 쓰는 단순 검출기라 실제 환경(라이다
각해상도, 노이즈, 사람 다리 두께)에 맞춰 파라미터 튜닝이 필요하다.
"""

import math


def distance(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def scan_to_points(msg, range_limit=None):
    """LaserScan -> 라이다 프레임 (x, y) 목록. 무효/범위초과 리턴은 None으로 자리만 채워
    cluster_points가 그 지점에서 클러스터를 끊을 수 있게 한다."""
    max_r = msg.range_max if range_limit is None else min(msg.range_max, range_limit)
    points = []
    angle = msg.angle_min
    for r in msg.ranges:
        if math.isfinite(r) and msg.range_min <= r <= max_r:
            points.append((r * math.cos(angle), r * math.sin(angle)))
        else:
            points.append(None)
        angle += msg.angle_increment
    return points


def cluster_points(points, cluster_distance_threshold):
    """연속된 유효 포인트를 거리 기준으로 묶는다 (jump-distance 클러스터링)."""
    clusters = []
    current = []
    prev = None
    for p in points:
        if p is None:
            if current:
                clusters.append(current)
                current = []
            prev = None
            continue
        if prev is not None and distance(prev, p) > cluster_distance_threshold:
            clusters.append(current)
            current = []
        current.append(p)
        prev = p
    if current:
        clusters.append(current)
    return clusters


def filter_leg_clusters(clusters, min_points, diameter_min, diameter_max):
    """점 개수 + 클러스터 폭(첫-끝 포인트 거리, 다리 단면 굵기의 근사치)으로 다리 후보만 남긴다."""
    legs = []
    for c in clusters:
        if len(c) < min_points:
            continue
        width = distance(c[0], c[-1])
        if diameter_min <= width <= diameter_max:
            cx = sum(p[0] for p in c) / len(c)
            cy = sum(p[1] for p in c) / len(c)
            legs.append((cx, cy))
    return legs


def pair_legs(leg_points, pair_max_distance, allow_single_leg=True):
    """가까운 다리 후보 쌍을 그리디(거리 오름차순)로 묶어 사람 중심점 목록을 만든다."""
    candidates = []
    for i in range(len(leg_points)):
        for j in range(i + 1, len(leg_points)):
            d = distance(leg_points[i], leg_points[j])
            if d <= pair_max_distance:
                candidates.append((d, i, j))
    candidates.sort(key=lambda t: t[0])

    used = [False] * len(leg_points)
    persons = []
    for _d, i, j in candidates:
        if used[i] or used[j]:
            continue
        used[i] = used[j] = True
        persons.append(((leg_points[i][0] + leg_points[j][0]) / 2.0,
                         (leg_points[i][1] + leg_points[j][1]) / 2.0))

    if allow_single_leg:
        persons.extend(leg_points[idx] for idx in range(len(leg_points)) if not used[idx])

    return persons


def detect_persons(msg, range_limit, cluster_distance_threshold, min_cluster_points,
                    leg_diameter_min, leg_diameter_max, pair_max_distance, allow_single_leg=True):
    """LaserScan 하나에서 사람 중심점(라이다 프레임 (x, y)) 목록을 뽑는 진입점 함수."""
    points = scan_to_points(msg, range_limit)
    clusters = cluster_points(points, cluster_distance_threshold)
    legs = filter_leg_clusters(clusters, min_cluster_points, leg_diameter_min, leg_diameter_max)
    return pair_legs(legs, pair_max_distance, allow_single_leg)
