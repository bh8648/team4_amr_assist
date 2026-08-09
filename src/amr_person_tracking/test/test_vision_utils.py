#!/usr/bin/env python3
"""
vision_utils.py의 발끝 depth 검증(estimate_person_depth) 검증.

이 함수는 "발끝 픽셀 주변이 통째로 뒤쪽 벽/바닥을 찍어 거리가 배경값으로 튀는" 실측 실패
(my_new_bag5에서 발끝 depth로 배경 거리 2.51m가 반복 등장)를 막기 위한 것이라, 테스트도
그 상황을 그대로 재현한다.
"""

import numpy as np

from amr_person_tracking.vision_utils import (
    KP_LEFT_ANKLE,
    KP_LEFT_KNEE,
    KP_RIGHT_ANKLE,
    KP_RIGHT_KNEE,
    estimate_foot_pixel,
    estimate_person_depth,
    foot_pixel_candidates,
)

BBOX = (100.0, 50.0, 200.0, 400.0)


def _kp(entries):
    """(17,2) 좌표 / (17,) 신뢰도를 만든다. entries: {인덱스: (u, v, conf)}"""
    xy = np.zeros((17, 2), dtype=np.float32)
    conf = np.zeros(17, dtype=np.float32)
    for idx, (u, v, c) in entries.items():
        xy[idx] = (u, v)
        conf[idx] = c
    return xy, conf


def test_two_ankles_give_two_separate_candidates_not_an_average():
    """핵심 회귀 방지: 두 발목을 평균내면 그 중점이 다리 사이 허공이라 배경 depth가 찍힌다
    (실측 22.1%). 반드시 각 발목을 개별 후보로 내보내야 한다."""
    xy, conf = _kp({KP_LEFT_ANKLE: (120.0, 380.0, 0.9),
                    KP_RIGHT_ANKLE: (180.0, 390.0, 0.9)})
    cands = foot_pixel_candidates(xy, conf, BBOX, 0.5)

    assert len(cands) == 2, '두 발목이 보이면 후보도 두 개여야 한다'
    assert {(c[0], c[1]) for c in cands} == {(120.0, 380.0), (180.0, 390.0)}
    assert all(c[2] == 'toe_direct' for c in cands)
    # 평균(150, 385)이 후보로 들어가면 안 된다 - 그게 예전 버그다.
    assert (150.0, 385.0) not in {(c[0], c[1]) for c in cands}


def test_single_visible_ankle_gives_one_candidate():
    xy, conf = _kp({KP_LEFT_ANKLE: (120.0, 380.0, 0.9),
                    KP_RIGHT_ANKLE: (180.0, 390.0, 0.1)})  # 오른발목 신뢰도 미달
    cands = foot_pixel_candidates(xy, conf, BBOX, 0.5)
    assert cands == [(120.0, 380.0, 'toe_direct')]


def test_falls_back_to_knees_then_bbox_bottom():
    xy, conf = _kp({KP_LEFT_KNEE: (130.0, 300.0, 0.9), KP_RIGHT_KNEE: (170.0, 305.0, 0.9)})
    cands = foot_pixel_candidates(xy, conf, BBOX, 0.5)
    assert [c[2] for c in cands] == ['knee_corrected', 'knee_corrected']
    # 무릎은 좌우 위치만 믿고 접지 높이는 bbox 하단(y2)을 쓴다.
    assert all(c[1] == BBOX[3] for c in cands)

    cands = foot_pixel_candidates(None, None, BBOX, 0.5)
    assert cands == [(150.0, 400.0, 'angle_only')], 'keypoint가 없으면 bbox 하단 중앙'


def test_estimate_foot_pixel_returns_first_candidate():
    """기존 호출부 호환 - 하나만 필요하면 첫 후보를 준다."""
    xy, conf = _kp({KP_LEFT_ANKLE: (120.0, 380.0, 0.9),
                    KP_RIGHT_ANKLE: (180.0, 390.0, 0.9)})
    assert estimate_foot_pixel(xy, conf, BBOX, 0.5) == \
        foot_pixel_candidates(xy, conf, BBOX, 0.5)[0]

SCALE = 0.001  # mm -> m


def _scene(person_mm, background_mm, width=100, height=200, person_frac=0.6):
    """bbox 안에 사람(가까움)과 배경(멂)이 섞인 depth 이미지를 만든다."""
    img = np.full((height, width), background_mm, dtype=np.uint16)
    x0 = int(width * (1 - person_frac) / 2)
    x1 = width - x0
    img[:, x0:x1] = person_mm
    return img


def test_picks_person_not_background():
    """사람이 bbox 면적의 절반 정도만 차지해도, 더 가까운 사람 쪽 깊이가 나와야 한다."""
    img = _scene(person_mm=1200, background_mm=2510)
    z = estimate_person_depth(img, (0, 0, 100, 200), SCALE)
    assert z is not None
    assert abs(z - 1.2) < 0.05, f'배경(2.51m)이 아니라 사람(1.2m)이 나와야 하는데 {z}'


def test_background_only_returns_background():
    """사람이 없으면(전부 배경) 그 값을 그대로 돌려준다 - 이 함수는 판정이 아니라 추정이고,
    배경/사람 판정은 호출부가 발끝 depth와 비교해서 한다."""
    img = np.full((200, 100), 2510, dtype=np.uint16)
    z = estimate_person_depth(img, (0, 0, 100, 200), SCALE)
    assert z is not None and abs(z - 2.51) < 0.05


def test_ignores_zero_and_out_of_range():
    """유효하지 않은 depth(0=측정실패)와 범위 밖 값은 통계에서 빠져야 한다."""
    img = _scene(person_mm=1500, background_mm=2000)
    img[:50, :] = 0          # 측정 실패 영역
    img[50:60, :] = 20000    # 20m - 범위 밖 이상치
    z = estimate_person_depth(img, (0, 0, 100, 200), SCALE)
    assert z is not None
    assert 1.4 < z < 1.6, f'유효 depth만 써야 하는데 {z}'


def test_returns_none_when_too_few_valid_pixels():
    img = np.zeros((200, 100), dtype=np.uint16)
    assert estimate_person_depth(img, (0, 0, 100, 200), SCALE) is None


def test_returns_none_for_zero_area_bbox():
    img = _scene(person_mm=1200, background_mm=2510)
    assert estimate_person_depth(img, (50, 50, 50, 50), SCALE) is None


def test_narrow_bbox_keeps_full_width():
    """아주 좁은 bbox는 좌우 20% 잘라내기가 0px이 되어 폭을 그대로 쓴다(값이 나와야 정상).

    잘라내기는 실루엣 경계 혼입을 줄이려는 보정이지 필수 조건이 아니므로, 좁다고 해서
    None을 주면 멀리 있는 사람의 검출을 통째로 버리게 된다."""
    img = _scene(person_mm=1200, background_mm=2510)
    z = estimate_person_depth(img, (10, 10, 12, 200), SCALE)
    assert z is not None


def test_handles_bbox_outside_image_bounds():
    """bbox가 이미지 밖으로 나가도(초근접이라 흔하다) 크래시 없이 잘라서 처리한다."""
    img = _scene(person_mm=1200, background_mm=2510)
    z = estimate_person_depth(img, (-30, -50, 130, 260), SCALE)
    assert z is not None
