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
# [설계 변경 이력] "언제 확정할지"를 판단하는 로직은 이 클래스 안에 있다가(고정 학습창 ->
# person 트랙 단위 -> 그리드 셀 연속관측 순으로 변경) predictive_utils.LegKalmanTracker로
# 옮겨갔다 - 자세한 이유는 클래스 docstring 참고. 이 클래스는 이제 "확정된 위치 집합 저장
# + 근접 판정"만 담당하는 얇은 저장소라 테스트도 단순하다 (LegKalmanTracker 자체 테스트는
# test_predictive_utils.py).
# ---------------------------------------------------------------------------

def test_static_background_filter_confirmed_position_excludes_nearby():
    f = StaticBackgroundFilter(cell_size=0.05, exclusion_radius=0.10)
    f.confirm_static(0.43, 0.40)

    assert f.is_background(0.43, 0.40) is True
    assert f.is_background(0.46, 0.42) is True  # exclusion_radius 이내는 근처도 배경 취급
    assert f.is_background(1.0, 1.0) is False   # 확정된 적 없는 위치


def test_static_background_filter_position_outside_radius_not_excluded():
    f = StaticBackgroundFilter(cell_size=0.05, exclusion_radius=0.10)
    f.confirm_static(0.0, 0.0)
    assert f.is_background(0.0, 0.0) is True
    assert f.is_background(0.5, 0.5) is False  # exclusion_radius(0.10m) 밖


def test_static_background_filter_nothing_confirmed_never_excludes():
    f = StaticBackgroundFilter()
    assert f.static_cell_count == 0
    assert f.is_background(0.0, 0.0) is False


def test_static_background_filter_confirm_is_idempotent():
    f = StaticBackgroundFilter(cell_size=0.05, exclusion_radius=0.10)
    f.confirm_static(0.2, 0.2)
    f.confirm_static(0.2, 0.2)  # 같은 위치를 여러 번 확정해도 안전해야 한다
    assert f.static_cell_count == 1
    assert f.is_background(0.2, 0.2) is True
