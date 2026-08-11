#!/usr/bin/env python3
"""webcam_bridge_utils.py 검증.

외부 웹캠 PC와 AMR의 클럭이 어긋났을 때 파이프라인이 조용히 망가지는 두 경로를
막는 로직이라, 나중에 누가 "단순화"하지 않도록 실행 가능한 형태로 고정해 둔다.
"""

from amr_person_tracking.webcam_bridge_utils import (
    SkewTracker,
    is_message_fresh,
    is_valid_point,
    resolve_stamp,
)


def test_skew_tracker_uses_running_min_as_offset():
    """편도 지연은 항상 0 이상이라, (로컬-원격)의 최솟값이 순수 클럭 오프셋에 가장 가깝다."""
    t = SkewTracker()
    t.update(remote_sec=100.0, local_sec=100.5)   # 지연 0.5 포함
    t.update(remote_sec=101.0, local_sec=101.2)   # 지연 0.2 포함  <- 가장 깨끗
    offset = t.update(remote_sec=102.0, local_sec=102.9)
    assert abs(offset - 0.2) < 1e-9
    # 마지막 표본이 추정치보다 얼마나 늦었는지(지터)
    assert abs(t.jitter - 0.7) < 1e-9


def test_receive_time_policy_ignores_remote_clock():
    """기본 정책은 원격 클럭을 아예 쓰지 않는다 - 5초 어긋나 있어도 결과가 같아야 한다."""
    assert resolve_stamp('receive_time', remote_sec=95.0, local_sec=100.0, offset=5.0) == 100.0
    assert resolve_stamp('receive_time', remote_sec=105.0, local_sec=100.0, offset=-5.0) == 100.0


def test_passthrough_and_offset_corrected():
    assert resolve_stamp('passthrough', 95.0, 100.0, 5.0) == 95.0
    # 원격이 5초 뒤처져 있으면 보정 후 로컬 시간축으로 올라와야 한다
    assert resolve_stamp('offset_corrected', 95.0, 100.0, 5.0) == 100.0
    # 오프셋을 아직 모르면 원본 그대로 (0으로 취급)
    assert resolve_stamp('offset_corrected', 95.0, 100.0, None) == 95.0


def test_stale_message_dropped_when_older_than_max_age():
    """Wi-Fi가 멈췄다 재전송이 몰리면 수 초 전 위치가 '지금'으로 둔갑한다."""
    # 오프셋 5.0이 이미 반영돼 있으므로 실제 지연은 0.3초 -> 통과
    assert is_message_fresh(remote_sec=95.0, local_sec=100.3, offset=5.0, max_age=1.0)
    # 실제 지연 3.0초 -> 폐기
    assert not is_message_fresh(remote_sec=95.0, local_sec=103.0, offset=5.0, max_age=1.0)
    # max_age <= 0 이면 검사 안 함 (이 패키지의 다른 파라미터와 같은 규약)
    assert is_message_fresh(95.0, 103.0, 5.0, max_age=0.0)


def test_is_valid_point_rejects_nan_and_inf():
    """호모그래피가 발산하면 실제로 NaN이 나온다."""
    assert is_valid_point(1.0, -2.0)
    assert not is_valid_point(float('nan'), 0.0)
    assert not is_valid_point(0.0, float('inf'))
