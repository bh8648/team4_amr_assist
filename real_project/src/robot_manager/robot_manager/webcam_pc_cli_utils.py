import math
from typing import List, Optional, Set, Tuple

ACTIVE_STATES: Set[str] = {'ASSIGNED', 'FOLLOWING', 'TRANSPORTING', 'RETURNING'}

# 사용자가 확인한 실측 경로: x=-1.5, yaw=-pi/2 고정, y만 0.5에서 -4.0까지 0.5씩 감소
FOLLOWING_MOCK_POSES: List[Tuple[float, float, float]] = [
    (-1.5, 0.5, -math.pi / 2),
    (-1.5, 0.0, -math.pi / 2),
    (-1.5, -0.5, -math.pi / 2),
    (-1.5, -1.0, -math.pi / 2),
    (-1.5, -1.5, -math.pi / 2),
    (-1.5, -2.0, -math.pi / 2),
    (-1.5, -2.5, -math.pi / 2),
    (-1.5, -3.0, -math.pi / 2),
    (-1.5, -3.5, -math.pi / 2),
    (-1.5, -4.0, -math.pi / 2),
]


def parse_command(raw_input: str) -> Tuple[str, List[str]]:
    """stdin 한 줄을 (명령 키워드, 인자 리스트)로 분리한다."""
    text = raw_input.strip()
    parts = text.split()
    if not parts:
        return '', []
    return parts[0], parts[1:]


def parse_interval(args: List[str], default: float = 3.0) -> Tuple[Optional[float], Optional[str]]:
    if not args:
        return default, None
    try:
        value = float(args[0])
    except ValueError:
        return None, '간격초는 숫자여야 합니다'
    if value <= 0:
        return None, '간격초는 양수여야 합니다'
    return value, None
