#!/usr/bin/env python3
"""
vision_utils.py의 발끝 depth 검증(estimate_person_depth) 검증.

이 함수는 "발끝 픽셀 주변이 통째로 뒤쪽 벽/바닥을 찍어 거리가 배경값으로 튀는" 실측 실패
(my_new_bag5에서 발끝 depth로 배경 거리 2.51m가 반복 등장)를 막기 위한 것이라, 테스트도
그 상황을 그대로 재현한다.
"""

import numpy as np

from amr_person_tracking.vision_utils import estimate_person_depth

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
