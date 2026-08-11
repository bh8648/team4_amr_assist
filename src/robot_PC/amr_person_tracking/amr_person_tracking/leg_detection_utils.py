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


def fit_circle_kasa(points):
    """점들에 원을 대수적 최소자승(Kåsa)으로 적합한다.

    x^2+y^2 = D*x + E*y + F 형태의 선형계로 바꿔 (D, E, F)를 Cramer's rule로 직접 푼다 -
    numpy 없이 math만으로 계산 가능하고, 이 모듈의 다른 함수들처럼 rclpy 의존 없이 단위
    검증할 수 있게 유지하기 위함이다.

    반환: (cx, cy, r). 점들이 거의 일직선(특이 행렬)이면 None - 벽/모서리처럼 곡률이 없는
    클러스터의 전형적인 신호다.
    """
    n = len(points)
    Sx = sum(p[0] for p in points)
    Sy = sum(p[1] for p in points)
    Sxx = sum(p[0] ** 2 for p in points)
    Syy = sum(p[1] ** 2 for p in points)
    Sxy = sum(p[0] * p[1] for p in points)
    Sxz = sum(p[0] * (p[0] ** 2 + p[1] ** 2) for p in points)
    Syz = sum(p[1] * (p[0] ** 2 + p[1] ** 2) for p in points)
    Sz = sum(p[0] ** 2 + p[1] ** 2 for p in points)

    # [Sxx Sxy Sx; Sxy Syy Sy; Sx Sy n] @ [D E F]^T = [Sxz Syz Sz]^T
    det = (Sxx * (Syy * n - Sy * Sy)
           - Sxy * (Sxy * n - Sy * Sx)
           + Sx * (Sxy * Sy - Syy * Sx))
    if abs(det) < 1e-9:
        return None

    det_d = (Sxz * (Syy * n - Sy * Sy)
              - Sxy * (Syz * n - Sy * Sz)
              + Sx * (Syz * Sy - Syy * Sz))
    det_e = (Sxx * (Syz * n - Sy * Sz)
              - Sxz * (Sxy * n - Sy * Sx)
              + Sx * (Sxy * Sz - Syz * Sx))
    det_f = (Sxx * (Syy * Sz - Syz * Sy)
              - Sxy * (Sxy * Sz - Syz * Sx)
              + Sxz * (Sxy * Sy - Syy * Sx))

    d = det_d / det
    e = det_e / det
    f = det_f / det

    cx, cy = d / 2.0, e / 2.0
    r_sq = f + cx ** 2 + cy ** 2
    if r_sq < 0:
        return None
    return cx, cy, math.sqrt(r_sq)


def circle_fit_rms_residual(points, cx, cy, r):
    """점들이 적합된 원 경계에서 얼마나 벗어나는지(RMS, m). 작을수록 실제로 둥근 단면에
    가깝다 - 평평한 벽이나 꺾인 모서리는 어떤 작은 원에도 잘 안 붙어 값이 커진다."""
    errs = [distance(p, (cx, cy)) - r for p in points]
    return math.sqrt(sum(e * e for e in errs) / len(errs))


def filter_leg_clusters(clusters, min_points, diameter_min, diameter_max,
                         circularity_filter_enabled=True,
                         leg_circle_fit_radius_max=0.20,
                         leg_circle_fit_rms_max=0.02):
    """점 개수 + 클러스터 폭(첫-끝 포인트 거리, 다리 단면 굵기의 근사치)으로 다리 후보만 남긴다.

    폭(chord) 검사만으로는 볼록한 다리 원호와 오목하게 꺾인 벽 모서리를 구분 못 한다(실측
    로스백에서 방 모서리가 다리로 오검출되는 걸 확인함). circularity_filter_enabled가 True면
    추가로 원형적합을 검사해 벽/모서리처럼 곡률이 없거나 큰 반지름으로만 맞는 클러스터를
    걸러낸다. 이 게이팅은 오탐을 줄이기 위한 것으로, 다리 위치 자체는 여전히 클러스터
    점들의 평균(centroid)을 쓴다 - 다운스트림 트래킹 좌표 계산 방식은 바꾸지 않는다.
    """
    legs = []
    for c in clusters:
        if len(c) < min_points:
            continue
        width = distance(c[0], c[-1])
        if not (diameter_min <= width <= diameter_max):
            continue

        if circularity_filter_enabled:
            fit = fit_circle_kasa(c)
            if fit is None:
                continue  # 거의 일직선 - 벽/모서리
            cx_fit, cy_fit, r_fit = fit
            if r_fit > leg_circle_fit_radius_max:
                continue  # 반지름이 비정상적으로 큼 - 거의 직선에 억지로 맞춘 원
            if circle_fit_rms_residual(c, cx_fit, cy_fit, r_fit) > leg_circle_fit_rms_max:
                continue  # 작은 원에 잘 안 붙음 - 평평한 벽이나 꺾인 모서리

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
                    leg_diameter_min, leg_diameter_max, pair_max_distance, allow_single_leg=True,
                    circularity_filter_enabled=True,
                    leg_circle_fit_radius_max=0.20, leg_circle_fit_rms_max=0.02):
    """LaserScan 하나에서 사람 중심점(라이다 프레임 (x, y)) 목록을 뽑는 진입점 함수."""
    points = scan_to_points(msg, range_limit)
    clusters = cluster_points(points, cluster_distance_threshold)
    legs = filter_leg_clusters(
        clusters, min_cluster_points, leg_diameter_min, leg_diameter_max,
        circularity_filter_enabled=circularity_filter_enabled,
        leg_circle_fit_radius_max=leg_circle_fit_radius_max,
        leg_circle_fit_rms_max=leg_circle_fit_rms_max)
    return pair_legs(legs, pair_max_distance, allow_single_leg)


class StaticBackgroundFilter:
    """책상/의자 다리처럼 실제로 원통형이라 형상(원형적합)만으로는 사람 다리와 구별이 안 되는
    정적 물체를, "계속 같은 자리에 있다"는 시간적 근거로 걸러낸다.

    실측(my_new_bag2, 168개 스캔)에서 확인됨: 원형적합 필터를 통과하는 후보 중 다수가 로봇
    근거리(0.4~1.2m)의 몇몇 위치에 전체 스캔의 56~97%라는 압도적 빈도로 반복 등장했다 - 실제
    사람이라면 불가능한 지속성이고, 실제로 그 위치엔 책상/의자 다리가 있었다.

    [설계 변경 이력]
    1) "노드 시작 후 N초만 학습하고 그 뒤로는 고정" - 학습 구간 동안 사람이 물체를 가리면
       영영 배경으로 못 배우는 문제 확인 (실측 11~14초/22~25초 구간).
    2) "사람/다리쌍(person) 트랙이 안 움직이면 확정" - pair_legs()의 그리디 짝짓기가 스캔마다
       다른 후보와 묶여 완전히 정지한 물체의 "짝지어진 중심점"도 5~10cm씩 튀는 게 확인돼,
       사람의 실제 정지 드리프트(초당 ~1.5cm)와 크기가 겹쳐 구분 불가.
    3) "짝짓기 이전 개별 다리 위치를 그리드 셀 단위 상시 관측, 끊김(gap) 있으면 리셋" - 반복
       재검증(my_new_bag2 끝부분)에서, 사람이 그 물체 앞을 자주 오가며 매번 1초 넘는 공백을
       만들면 짧은 세션 안에 끊김 없는 3초를 못 채워 계속 미확정으로 남는 걸 확인.
    4) (현재) 이 클래스는 이제 "확정된 위치 집합 저장 + 근접 판정"만 담당하는 얇은 저장소다.
       "언제 확정할지" 판단은 predictive_utils.LegKalmanTracker가 개별 다리 후보별 등속도
       칼만필터로 맡는다 - 칼만필터는 관측 사이 공백(occlusion)이 있어도 그 구간만큼 공정
       잡음이 커질 뿐 상태가 리셋되지 않으므로, 위 3)의 gap-reset 문제를 구조적으로 피한다.
       leg_detector_bridge_node가 LegKalmanTracker.update()가 "정지 확정"이라고 돌려주는
       위치에 대해 confirm_static()을 호출한다.
    """

    def __init__(self, cell_size=0.05, exclusion_radius=0.10):
        self.cell_size = cell_size
        self.exclusion_radius = exclusion_radius
        self._static_cells = set()

    def _cell(self, x, y):
        return (round(x / self.cell_size), round(y / self.cell_size))

    def confirm_static(self, x, y):
        """이 위치를 정적 배경으로 (그 세션 동안) 영구 등록한다. 여러 번 불러도 안전하다."""
        self._static_cells.add(self._cell(x, y))

    def release(self, x, y):
        """이 위치 주변의 정적 배경 등록을 해제한다. 확정된 적이 없으면 아무 일도 안 한다.

        "정적 = 영구"였던 이전 동작은 오래 서 있던 **사람**이 배경으로 확정되면 그 자리에서
        영영 검출되지 않는다는 뜻이었다(세션 내 해제 경로가 없었다). 사람은 결국 다시 움직이므로,
        움직임이 관측되면 되돌릴 수 있어야 한다. 반환값은 실제로 지운 셀 수.
        """
        removed = 0
        for cell in self._cells_within(x, y):
            self._static_cells.discard(cell)
            removed += 1
        return removed

    def _cells_within(self, x, y):
        """(x, y)의 exclusion_radius 안에 있는 등록 셀들."""
        r2 = self.exclusion_radius ** 2
        hit = []
        for cx, cy in self._static_cells:
            dx = cx * self.cell_size - x
            dy = cy * self.cell_size - y
            if dx * dx + dy * dy <= r2:
                hit.append((cx, cy))
        return hit

    @property
    def static_cell_count(self):
        return len(self._static_cells)

    def is_background(self, x, y):
        """(x, y)가 confirm_static()으로 확정된 정적 배경 위치의 exclusion_radius 이내면 True."""
        if not self._static_cells:
            return False
        span = int(math.ceil(self.exclusion_radius / self.cell_size)) + 1
        cx, cy = self._cell(x, y)
        for dx in range(-span, span + 1):
            for dy in range(-span, span + 1):
                cell = (cx + dx, cy + dy)
                if cell not in self._static_cells:
                    continue
                cell_center = (cell[0] * self.cell_size, cell[1] * self.cell_size)
                if distance((x, y), cell_center) <= self.exclusion_radius:
                    return True
        return False
