#!/usr/bin/env python3
"""
leg_detection_utils.py의 원형적합(곡률) 필터를 검증한다. rclpy에 의존하지 않는 순수 함수라
노드 기동 없이 바로 돌아간다.

실제 로스백(my_new_bag2)의 전체 168개 스캔을 이 필터로 직접 재생해본 결과, 평균 검출 후보 수가
6.76개(옛 동작)에서 5.86개(필터 적용)로 줄었다 - 방 전체의 정적 클러터(둥근 다리/모서리 등)를
전부 없애진 못하지만, 아래 3케이스처럼 명백히 곡률이 없는 클러스터(벽/모서리)는 확실히 걸러낸다.
"""

import math

import pytest

from amr_person_tracking.leg_detection_utils import (
    StaticBackgroundFilter,
    circle_fit_rms_residual,
    filter_leg_clusters,
    fit_circle_kasa,
)

RMS_MAX = 0.01
RADIUS_MAX = 0.20


def _leg_arc_points():
    """반지름 0.05m 원호 위 점 6개 - 실제 다리 단면의 근사."""
    cx0, cy0, r0 = 0.3, 0.3, 0.05
    thetas = [-0.6, -0.36, -0.12, 0.12, 0.36, 0.6]
    return [(cx0 + r0 * math.cos(t), cy0 + r0 * math.sin(t)) for t in thetas]


def _flat_wall_points():
    """일직선 위 점 6개 - 평평한 벽."""
    return [(0.1 * i, 2.0) for i in range(6)]


def _corner_points():
    """두 벽이 직각으로 만나는 지점의 점들 - 실제 로스백에서 다리로 오검출됐던 형태."""
    return [(0.0, 2.0), (0.05, 2.0), (0.10, 2.0),
            (0.10, 1.95), (0.10, 1.90), (0.10, 1.85)]


def test_fit_circle_kasa_recovers_real_circle():
    points = _leg_arc_points()
    fit = fit_circle_kasa(points)
    assert fit is not None
    cx, cy, r = fit
    assert cx == pytest.approx(0.3, abs=1e-6)
    assert cy == pytest.approx(0.3, abs=1e-6)
    assert r == pytest.approx(0.05, abs=1e-6)
    assert circle_fit_rms_residual(points, cx, cy, r) < 1e-6


def test_fit_circle_kasa_returns_none_for_straight_line():
    fit = fit_circle_kasa(_flat_wall_points())
    assert fit is None


def test_fit_circle_kasa_corner_fails_rms_check():
    points = _corner_points()
    fit = fit_circle_kasa(points)
    assert fit is not None  # 특이하진 않지만
    cx, cy, r = fit
    rms = circle_fit_rms_residual(points, cx, cy, r)
    assert rms > RMS_MAX, '모서리가 작은 원에 그럴듯하게 붙어버리면 필터가 못 거른다'


def test_filter_leg_clusters_accepts_real_leg_with_circularity_filter():
    legs = filter_leg_clusters(
        [_leg_arc_points()], min_points=3, diameter_min=0.05, diameter_max=0.25,
        circularity_filter_enabled=True,
        leg_circle_fit_radius_max=RADIUS_MAX, leg_circle_fit_rms_max=RMS_MAX)
    assert len(legs) == 1


def test_filter_leg_clusters_rejects_corner_with_circularity_filter():
    legs = filter_leg_clusters(
        [_corner_points()], min_points=3, diameter_min=0.05, diameter_max=0.25,
        circularity_filter_enabled=True,
        leg_circle_fit_radius_max=RADIUS_MAX, leg_circle_fit_rms_max=RMS_MAX)
    assert legs == []


def test_filter_leg_clusters_old_behavior_reproduces_corner_false_positive():
    """circularity_filter_enabled=False면 옛 동작(폭만 검사) 그대로라 모서리도 다리로 잡힌다 -
    이게 로스백 재생 영상에서 실제로 관찰된 오검출과 정확히 같은 상황이다."""
    legs = filter_leg_clusters(
        [_corner_points()], min_points=3, diameter_min=0.05, diameter_max=0.25,
        circularity_filter_enabled=False)
    assert len(legs) == 1


def test_filter_leg_clusters_flat_wall_rejected_either_way():
    for enabled in (True, False):
        legs = filter_leg_clusters(
            [_flat_wall_points()], min_points=3, diameter_min=0.05, diameter_max=0.25,
            circularity_filter_enabled=enabled)
        # 일직선은 폭(첫-끝 거리)이 0.5m로 diameter_max(0.25m)를 넘어 애초에 폭 검사에서부터
        # 걸러진다 - 곡률 필터 on/off와 무관하게 항상 탈락해야 한다.
        assert legs == []


# ---------------------------------------------------------------------------
# StaticBackgroundFilter: 원형적합만으론 못 거르는 책상/의자 다리(진짜 원통형) 대응.
# 실측(my_new_bag2, 168개 스캔)에서, 원형적합 통과 후보 중 다수가 로봇 근거리(0.4~1.2m)의
# 몇몇 위치에 전체 스캔의 56~97% 빈도로 반복 등장함을 확인했다 - 실제 책상/의자 다리였다.
#
# [설계 변경 이력] 처음엔 "노드 시작 후 N초 학습 후 고정" -> "사람/다리쌍 트랙 단위로 상시
# confirm_static()" 순으로 바뀌었는데, 후자도 실측 재검증(같은 bag 11~14초/22~25초 구간)에서
# 문제가 드러났다: 짝짓기(pair_legs)된 중심점은 스캔마다 어떤 후보끼리 묶이느냐에 따라
# 5~10cm씩 튀어서(정지한 물체인데도) 사람의 실제 미세 이동과 구분이 안 됐다. 그래서 지금은
# 짝짓기 이전의 개별 다리 위치를 그리드 셀 단위로 상시 관측하는 observe(x, y, stamp) 방식
# 이다 - 같은 셀이 max_gap_sec 이내 간격으로 confirm_duration_sec 이상 끊김 없이 관측되면
# 그 셀을 정적 배경으로 확정한다.
# ---------------------------------------------------------------------------

def test_static_background_filter_confirms_after_continuous_observation():
    f = StaticBackgroundFilter(cell_size=0.05, exclusion_radius=0.10,
                                confirm_duration_sec=3.0, max_gap_sec=1.0)
    for t in (0.0, 1.0, 2.0, 2.9):
        f.observe(0.43, 0.40, t)
        assert f.is_background(0.43, 0.40) is False  # 아직 3초 안 됨

    f.observe(0.43, 0.40, 3.0)  # 첫 관측(t=0)부터 3.0초 경과 - 확정
    assert f.is_background(0.43, 0.40) is True
    assert f.is_background(0.46, 0.42) is True   # exclusion_radius 이내는 근처도 배경 취급
    assert f.is_background(1.0, 1.0) is False    # 관측된 적 없는 위치


def test_static_background_filter_gap_resets_continuous_run():
    f = StaticBackgroundFilter(cell_size=0.05, exclusion_radius=0.10,
                                confirm_duration_sec=3.0, max_gap_sec=1.0)
    f.observe(0.0, 0.0, 0.0)
    f.observe(0.0, 0.0, 2.9)   # 간격 2.9s > max_gap_sec(1.0s) - 연속 구간이 끊긴다
    assert f.is_background(0.0, 0.0) is False
    f.observe(0.0, 0.0, 5.8)   # 2.9에서 다시 시작(run_start=5.8) - 아직 3.0초 안 지남
    assert f.is_background(0.0, 0.0) is False
    for t in (6.5, 7.5, 8.5):  # 매 간격 <= max_gap_sec(1.0s) - 연속 구간 유지
        f.observe(0.0, 0.0, t)
    assert f.is_background(0.0, 0.0) is False  # run_start(5.8)부터 아직 3.0초 안 됨
    f.observe(0.0, 0.0, 8.9)  # run_start(5.8)부터 3.1초 경과 - 확정
    assert f.is_background(0.0, 0.0) is True


def test_static_background_filter_slow_drift_never_confirms():
    """사람이 서 있어도 실측상 초당 ~1.5cm씩 계속 드리프트한다 - 그리드 셀(5cm) 경계를
    수 초 안에 넘나들며 한 셀에 3초 연속 머물지 못해야 배경으로 오인되지 않는다."""
    f = StaticBackgroundFilter(cell_size=0.05, exclusion_radius=0.10,
                                confirm_duration_sec=3.0, max_gap_sec=1.0)
    drift_per_sec = 0.02  # 5cm 셀을 2.5초마다 넘는 속도 - 실측 1.5cm/s보다도 빠르게 잡아 여유를 둠
    for i in range(100):
        t = i * 0.13
        f.observe(drift_per_sec * t, 0.0, t)
    assert f.static_cell_count == 0


def test_static_background_filter_position_outside_radius_not_excluded():
    f = StaticBackgroundFilter(cell_size=0.05, exclusion_radius=0.10, confirm_duration_sec=1.0)
    f.observe(0.0, 0.0, 0.0)
    f.observe(0.0, 0.0, 1.0)
    assert f.is_background(0.0, 0.0) is True
    assert f.is_background(0.5, 0.5) is False  # exclusion_radius(0.10m) 밖


def test_static_background_filter_never_observed_never_excludes():
    f = StaticBackgroundFilter()
    assert f.static_cell_count == 0
    assert f.is_background(0.0, 0.0) is False
