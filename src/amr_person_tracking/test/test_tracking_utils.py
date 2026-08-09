#!/usr/bin/env python3
"""
tracking_utils.py의 배타적 트랙 배정(match_track/assign_tracks)과 시간 역순 갱신 방어
(Track.update)를 검증한다. rclpy에 의존하지 않는 순수 함수라 노드 기동 없이 바로 돌아간다.
"""

import math

import pytest

from amr_person_tracking.tracking_utils import (
    Track,
    assign_tracks,
    cosine_similarity,
    match_track,
)


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


def test_cosine_similarity_returns_none_when_unavailable():
    assert cosine_similarity(None, [1.0, 0.0]) is None
    assert cosine_similarity([1.0, 0.0], None) is None
    assert cosine_similarity([1.0, 0.0], [1.0, 0.0, 0.0]) is None, '차원이 다르면 비교 불가'
    assert cosine_similarity([0.0, 0.0], [1.0, 0.0]) is None, '영벡터는 방향이 없어 비교 불가'
    assert cosine_similarity([1.0, 0.0], [1.0, 0.0]) == pytest.approx(1.0)
    assert cosine_similarity([1.0, 0.0], [0.0, 1.0]) == pytest.approx(0.0)


def test_appearance_cost_picks_matching_identity_among_gated_candidates():
    """두 트랙이 모두 게이트 안에 있어 위치만으로는 뒤바뀔 수 있는 상황에서, 외형이
    맞는 쪽으로 배정돼야 한다. 3인 교차에서 관측된 신원 뒤바뀜을 겨냥한 검증이다."""
    a = Track('A', x=0.0, y=0.0, stamp=10.0, source='oakd')
    a.embedding = [1.0, 0.0]
    b = Track('B', x=0.2, y=0.0, stamp=10.0, source='oakd')
    b.embedding = [0.0, 1.0]
    tracks = {'A': a, 'B': b}

    # 검출0은 B 쪽 외형, 검출1은 A 쪽 외형인데 위치는 서로 반대편에 가깝게 둔다.
    detections = [(0.05, 0.0), (0.15, 0.0)]
    embeddings = [[0.0, 1.0], [1.0, 0.0]]

    without = assign_tracks(tracks, detections, stamp=10.05,
                            gating_max_speed=2.0, min_gate=0.3)
    with_app = assign_tracks(tracks, detections, stamp=10.05,
                             gating_max_speed=2.0, min_gate=0.3,
                             embeddings=embeddings, appearance_weight=1.0)

    assert without == ['A', 'B'], '위치만 보면 가까운 순서대로 붙는다(대조군)'
    assert with_app == ['B', 'A'], '외형을 반영하면 신원이 맞는 쪽으로 뒤집혀야 한다'


def test_appearance_weight_ignored_when_embeddings_missing():
    """임베딩을 주지 않는 출처(웹캠 등)에서는 기존 위치 기반 동작 그대로여야 한다."""
    a = Track('A', x=0.0, y=0.0, stamp=10.0, source='oakd')
    tracks = {'A': a}
    detections = [(0.05, 0.0)]

    assert assign_tracks(tracks, detections, stamp=10.05, gating_max_speed=2.0,
                         min_gate=0.3, embeddings=[None],
                         appearance_weight=10.0) == ['A']


def test_appearance_cost_never_beats_position_gate():
    """외형이 완벽히 일치해도 게이트 밖이면 붙지 않아야 한다(멀리 있는 닮은 사람 방지)."""
    a = Track('A', x=0.0, y=0.0, stamp=10.0, source='oakd')
    a.embedding = [1.0, 0.0]
    tracks = {'A': a}

    result = assign_tracks(tracks, [(50.0, 50.0)], stamp=10.05, gating_max_speed=2.0,
                           min_gate=0.3, embeddings=[[1.0, 0.0]], appearance_weight=1.0)
    assert result == [None]


def test_update_embedding_smooths_and_renormalizes():
    track = Track(1, x=0.0, y=0.0, stamp=10.0, source='oakd')
    track.update_embedding([1.0, 0.0])
    assert track.embedding == [1.0, 0.0], '첫 임베딩은 그대로 채택'

    track.update_embedding([0.0, 1.0], alpha=0.5)
    norm = math.sqrt(sum(v * v for v in track.embedding))
    assert norm == pytest.approx(1.0), 'EMA 후 다시 정규화돼야 코사인 비교가 일관된다'
    # 한 프레임이 통째로 덮어쓰지 않고 두 방향이 섞여야 한다.
    assert track.embedding[0] == pytest.approx(track.embedding[1])

    track.update_embedding(None)
    assert track.embedding is not None, '임베딩 없음은 기존 값을 지우지 않는다'


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


def _kalman_track(track_id, x, y, stamp, measurement_noise=0.0025):
    from amr_person_tracking.predictive_utils import ConstantVelocityKalman2D
    kf = ConstantVelocityKalman2D(x, y, stamp, process_noise=1.0,
                                  measurement_noise=measurement_noise)
    return Track(track_id, x, y, stamp, 'oakd', kalman=kf)


def test_kalman_track_does_not_snap_to_noisy_measurement():
    """핵심: 예전엔 position_alpha=1.0이라 관측값에 그대로 스냅해 검출 스파이크가 곧 출력이었다.
    칼만필터를 붙이면 한 프레임짜리 이상치가 그대로 반영되지 않아야 한다."""
    track = _kalman_track(1, 0.0, 0.0, 10.0)
    # 정상 관측 몇 번으로 상태를 안정시킨다.
    for k in range(1, 5):
        track.update(0.0, 0.0, 10.0 + k * 0.1, 'oakd')
    # 1.5m 떨어진 이상치 한 방
    track.update(1.5, 0.0, 10.5, 'oakd')

    assert track.x < 1.0, f'이상치에 그대로 끌려가면 안 된다 (x={track.x})'

    plain = Track(2, 0.0, 0.0, 10.0, 'oakd')  # 칼만 없음 = 기존 스냅 동작
    for k in range(1, 5):
        plain.update(0.0, 0.0, 10.0 + k * 0.1, 'oakd')
    plain.update(1.5, 0.0, 10.5, 'oakd')
    assert plain.x == pytest.approx(1.5), '대조군: 필터가 없으면 관측값으로 그대로 스냅한다'


def test_mahalanobis_is_none_without_kalman():
    plain = Track(1, 0.0, 0.0, 10.0, 'oakd')
    assert plain.mahalanobis(0.1, 0.0, 10.1) is None


def test_mahalanobis_grows_with_distance():
    track = _kalman_track(1, 0.0, 0.0, 10.0)
    near = track.mahalanobis(0.05, 0.0, 10.1)
    far = track.mahalanobis(3.0, 0.0, 10.1)
    assert near is not None and far is not None
    assert far > near, '멀수록 마할라노비스 거리가 커야 한다'


def test_mahalanobis_gate_rejects_far_detection_in_assign():
    track = _kalman_track(1, 0.0, 0.0, 10.0)
    tracks = {1: track}
    # 게이트를 아주 좁게(0.5) 주면 멀리 있는 관측은 배정되지 않아야 한다.
    assert assign_tracks(tracks, [(5.0, 5.0)], 10.1, 2.0, 0.3,
                         mahalanobis_gate=0.5) == [None]
    # 같은 자리 관측은 통과한다.
    assert assign_tracks(tracks, [(0.0, 0.0)], 10.1, 2.0, 0.3,
                         mahalanobis_gate=3.035) == [1]


def test_mahalanobis_gate_falls_back_to_euclidean_without_kalman():
    """칼만필터가 없는 트랙은 마할라노비스를 못 구하므로 기존 유클리드 게이트로 폴백한다."""
    plain = Track(1, 0.0, 0.0, 10.0, 'oakd')
    tracks = {1: plain}
    assert assign_tracks(tracks, [(0.05, 0.0)], 10.1, 2.0, 0.3, mahalanobis_gate=3.035) == [1]
    assert assign_tracks(tracks, [(50.0, 0.0)], 10.1, 2.0, 0.3, mahalanobis_gate=3.035) == [None]
