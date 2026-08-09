import math
from typing import Dict, List, Optional, Set, Tuple

ACTIVE_STATES: Set[str] = {'ASSIGNED', 'FOLLOWING', 'TRANSPORTING', 'RETURNING'}

DELIVER_COMMAND = '배송모드'

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
    """stdin 한 줄을 (명령 키워드, 인자 리스트)로 분리한다.

    "배송모드"/"배송 모드"는 공백을 모두 제거한 뒤 접두사로 인식한다.
    """
    text = raw_input.strip()
    compact = text.replace(' ', '')
    if compact.startswith(DELIVER_COMMAND):
        remainder = compact[len(DELIVER_COMMAND):]
        return DELIVER_COMMAND, ([remainder] if remainder else [])
    parts = text.split()
    if not parts:
        return '', []
    return parts[0], parts[1:]


def parse_call_args(args: List[str]) -> Tuple[Optional[Tuple[float, float]], Optional[str]]:
    if len(args) != 2:
        return None, '사용법: 호출 <x> <y>'
    try:
        x, y = float(args[0]), float(args[1])
    except ValueError:
        return None, 'x, y는 숫자여야 합니다'
    return (x, y), None


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


def select_destination(destinations: List[dict], requested_id: Optional[str]) -> Tuple[Optional[dict], Optional[str]]:
    if requested_id:
        for destination in destinations:
            if destination['destination_id'] == requested_id:
                return destination, None
        return None, f'목적지 없음: {requested_id}'
    if not destinations:
        return None, '등록된 목적지 없음'
    if len(destinations) == 1:
        return destinations[0], None
    ids = ', '.join(destination['destination_id'] for destination in destinations)
    return None, f'목적지를 지정하세요: {ids}'


def select_active_robot(
        task_states: Dict[str, str],
        requested_id: Optional[str] = None) -> Tuple[Optional[str], Optional[str]]:
    active = [robot_id for robot_id, state in task_states.items() if state in ACTIVE_STATES]
    if requested_id:
        if requested_id in active:
            return requested_id, None
        return None, f'{requested_id}는 활성 상태가 아닙니다'
    if not active:
        return None, '활성 작업 없음'
    if len(active) == 1:
        return active[0], None
    return None, f'로봇을 지정하세요: {", ".join(sorted(active))}'
