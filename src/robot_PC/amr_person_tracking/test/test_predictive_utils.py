#!/usr/bin/env python3
"""predictive_utils.py의 LegKalmanTracker를 검증한다. leg_detector_bridge_node가 개별 다리
후보(짝짓기 이전)의 정지 여부를 판단하는 데 쓰는 클래스로, StaticBackgroundFilter의 이전 두
설계(person 트랙 단위 정지 판정, 그리드 셀 연속관측)가 각각 짝짓기 튐과 occlusion으로 인한
관측 공백에 실패한 것을 등속도 칼만필터로 보완한다 (클래스 docstring 참고).
"""

from amr_person_tracking.predictive_utils import LegKalmanTracker

CONFIRM_SEC = 3.0
SPEED_TH = 0.01


def make_tracker(**kwargs):
    defaults = dict(match_gate=0.10, confirm_duration_sec=CONFIRM_SEC,
                     stationary_speed_threshold=SPEED_TH, kf_timeout_sec=5.0,
                     process_noise=0.01, measurement_noise=0.0025)
    defaults.update(kwargs)
    return LegKalmanTracker(**defaults)


def test_stationary_leg_confirmed_after_duration():
    """완전히 정지한 다리 하나를 confirm_duration_sec 이상 계속 관측하면 정지로 확정돼야 한다."""
    tracker = make_tracker()
    stationary_positions = []
    t = 0.0
    while t <= CONFIRM_SEC + 0.5:
        stationary_positions = tracker.update([(0.5, 0.3)], t)
        t += 0.13
    assert len(stationary_positions) == 1
    x, y = stationary_positions[0]
    assert abs(x - 0.5) < 0.01
    assert abs(y - 0.3) < 0.01


def test_not_confirmed_before_duration_elapsed():
    """confirm_duration_sec이 지나기 전에는 정지한 물체여도 확정되면 안 된다."""
    tracker = make_tracker()
    t = 0.0
    while t < CONFIRM_SEC - 0.2:
        stationary = tracker.update([(0.5, 0.3)], t)
        assert stationary == []
        t += 0.13


def test_occlusion_gap_does_not_reset_confirmation():
    """그리드 셀 연속관측 방식(max_gap_sec 초과 시 리셋)이 실패했던 바로 그 상황 - 사람이
    물체 앞을 반복해서 오가며 매번 1초 넘는 관측 공백을 만들어도, 칼만필터는 상태가 리셋되지
    않으므로 "첫 관측부터의 누적 나이"가 계속 쌓여 결국 확정돼야 한다."""
    tracker = make_tracker()
    # 짧은 관측 구간(0.5초) 사이사이에 1.5초짜리 공백(occlusion)을 반복해서 끼워 넣는다 -
    # 각 공백은 이전 설계(max_gap_sec=1.0s)라면 리셋을 유발했을 길이다.
    schedule = []
    t = 0.0
    for _ in range(4):
        burst_end = t + 0.5
        while t <= burst_end:
            schedule.append(t)
            t += 0.13
        t += 1.5  # occlusion 공백

    stationary = []
    for t in schedule:
        stationary = tracker.update([(0.5, 0.3)], t)
    assert len(stationary) == 1  # 반복된 공백에도 불구하고 결국 정지로 확정됨


def test_moving_leg_never_confirmed():
    """실측 "서 있는 사람" 드리프트(~1.5cm/s)보다 빠르게 계속 움직이는 다리는 confirm_duration이
    한참 지나도 확정되면 안 된다."""
    tracker = make_tracker()
    speed = 0.05  # 5cm/s - 실측 드리프트보다 훨씬 빠르게 잡아 여유를 둠
    t = 0.0
    stationary = []
    while t <= CONFIRM_SEC + 3.0:
        stationary = tracker.update([(speed * t, 0.0)], t)
        t += 0.13
    assert stationary == []


def test_two_close_legs_matched_exclusively():
    """한 스캔에 다리 후보 두 개가 있으면 서로 다른 칼만필터에 배타적으로 배정돼야 한다
    (하나가 두 후보를 동시에 삼키면 안 됨)."""
    tracker = make_tracker(match_gate=0.10)
    for t in [i * 0.13 for i in range(5)]:
        tracker.update([(0.0, 0.0), (0.3, 0.0)], t)
    assert len(tracker._kalman) == 2


def test_stale_leg_pruned_after_timeout():
    """한동안(kf_timeout_sec 이상) 안 보인 다리는 내부 상태에서 제거돼야 한다 - 장시간 실행
    시 메모리가 무한정 늘어나지 않도록."""
    tracker = make_tracker(kf_timeout_sec=2.0)
    tracker.update([(0.0, 0.0)], 0.0)
    assert len(tracker._kalman) == 1
    tracker.update([(5.0, 5.0)], 3.0)  # 다른 위치 갱신이 와도 오래된 다리는 정리돼야 함
    assert len(tracker._kalman) == 1  # (0,0)은 정리되고 (5,5)만 남음


def test_moving_positions_reports_only_moving_legs():
    """정적 배경 확정을 되돌릴 근거로 쓰는 목록이라, 정지한 다리는 빠져야 한다."""
    t = make_tracker()
    # 한 다리는 제자리, 다른 다리는 초당 0.5m로 이동
    for i in range(20):
        stamp = i * 0.1
        t.update([(0.0, 0.0), (2.0 + 0.05 * i, 0.0)], stamp)

    moving = t.moving_positions(release_speed_threshold=0.05)
    assert len(moving) == 1
    assert moving[0][0] > 2.0          # 움직인 쪽만 보고된다

    # 임계를 실제 속도보다 높이면 아무것도 안 나온다
    assert t.moving_positions(release_speed_threshold=5.0) == []
