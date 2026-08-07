#!/usr/bin/env python3
"""
tracking_utils.py의 배타적 트랙 배정(match_track/assign_tracks)과 시간 역순 갱신 방어
(Track.update)를 검증한다. rclpy에 의존하지 않는 순수 함수라 노드 기동 없이 바로 돌아간다.
"""

import pytest

from amr_person_tracking.tracking_utils import Track, assign_tracks, match_track


def test_match_track_exclude_ids_prevents_double_assignment():
    tracks = {1: Track(1, x=0.0, y=0.0, stamp=10.0, source='oakd')}

    first = match_track(tracks, x=0.05, y=0.0, stamp=10.1, gating_max_speed=2.0, min_gate=0.3)
    assert first == 1

    # 이미 이번 콜백에서 트랙 1을 차지한 뒤라면, 여전히 게이트 안에 있어도 두 번째 검출은
    # 같은 트랙을 받지 않는다.
    second = match_track(
        tracks, x=0.05, y=0.0, stamp=10.1, gating_max_speed=2.0, min_gate=0.3,
        exclude_ids={first})
    assert second is None


def test_match_track_no_exclude_backward_compat():
    tracks = {1: Track(1, x=0.0, y=0.0, stamp=10.0, source='oakd')}
    track_id = match_track(tracks, x=0.05, y=0.0, stamp=10.1, gating_max_speed=2.0, min_gate=0.3)
    assert track_id == 1


def test_assign_tracks_no_duplicate_ids_for_two_close_detections():
    """실제 리뷰에서 재현된 상황: 트랙 하나에 서로 다른 위치의 검출 두 개가 둘 다 게이트 이내에
    들어와 예전 코드에서는 둘 다 leg_6으로 발행됐다."""
    tracks = {6: Track(6, x=1.0, y=1.0, stamp=10.0, source='lidar_leg')}
    detections = [(1.05, 1.0), (0.95, 1.0)]

    result = assign_tracks(
        tracks, detections, stamp=10.1, gating_max_speed=2.0, min_gate=0.3)

    assigned = [tid for tid in result if tid is not None]
    assert len(assigned) == len(set(assigned)), '같은 트랙 id가 한 콜백 내 두 검출에 중복 배정됨'


def test_assign_tracks_new_track_for_unmatched_detection():
    tracks = {1: Track(1, x=0.0, y=0.0, stamp=10.0, source='oakd')}
    # 두 번째 검출은 기존 트랙에서 아주 멀리 떨어져 있어 게이트 밖.
    detections = [(0.05, 0.0), (10.0, 10.0)]

    result = assign_tracks(
        tracks, detections, stamp=10.1, gating_max_speed=2.0, min_gate=0.3)

    assert result[0] == 1
    assert result[1] is None


def test_update_rejects_stale_timestamp():
    track = Track(1, x=0.0, y=0.0, stamp=10.0, source='oakd')

    accepted = track.update(x=5.0, y=5.0, stamp=9.5, source='webcam')

    assert accepted is False
    assert track.x == 0.0
    assert track.y == 0.0
    assert track.last_stamp == 10.0


def test_update_tolerates_tiny_negative_jitter():
    track = Track(1, x=0.0, y=0.0, stamp=10.0, source='oakd')

    accepted = track.update(x=1.0, y=0.0, stamp=9.995, source='webcam', position_alpha=1.0)

    assert accepted is True
    assert track.x == pytest.approx(1.0)
    # last_stamp는 절대 뒤로 가지 않는다 (max(10.0, 9.995) == 10.0).
    assert track.last_stamp == 10.0


def test_update_normal_forward_progress_unaffected():
    track = Track(1, x=0.0, y=0.0, stamp=10.0, source='oakd')

    accepted = track.update(
        x=1.0, y=0.0, stamp=11.0, source='oakd', velocity_alpha=1.0, position_alpha=1.0)

    assert accepted is True
    assert track.x == pytest.approx(1.0)
    assert track.last_stamp == 11.0
    assert track.vx == pytest.approx(1.0)
