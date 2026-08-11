"""Dock station에서 캡처한 AMCL 초기 pose를 저장·로드하는 함수."""

import math
import os
from pathlib import Path

import yaml


DEFAULT_POSE_FILE = '~/.config/team4_amr_assist/initial_poses.yaml'
DEFAULT_COVARIANCE_XY = 0.25
DEFAULT_COVARIANCE_YAW = 0.06853891945200942


def validate_initial_pose(values):
    """pose 필드를 float로 변환하고 안전한 범위인지 확인한다."""
    required = ('x', 'y', 'yaw')
    try:
        result = {name: float(values[name]) for name in required}
        result['covariance_xy'] = float(
            values.get('covariance_xy', DEFAULT_COVARIANCE_XY))
        result['covariance_yaw'] = float(
            values.get('covariance_yaw', DEFAULT_COVARIANCE_YAW))
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('initial pose에 x, y, yaw 숫자가 필요합니다.') from error
    if not all(math.isfinite(value) for value in result.values()):
        raise ValueError('initial pose에 NaN/inf를 사용할 수 없습니다.')
    if result['covariance_xy'] <= 0.0 or result['covariance_yaw'] <= 0.0:
        raise ValueError('initial pose covariance는 0보다 커야 합니다.')
    return result


def load_initial_pose(path, robot_id):
    """YAML에서 robot_id의 dock pose를 로드한다."""
    pose_path = Path(os.path.expanduser(str(path)))
    if not pose_path.is_file():
        raise FileNotFoundError(str(pose_path))
    with pose_path.open(encoding='utf-8') as pose_file:
        document = yaml.safe_load(pose_file) or {}
    entries = document.get('initial_poses', document)
    if not isinstance(entries, dict) or robot_id not in entries:
        raise KeyError(f'{robot_id} initial pose가 {pose_path}에 없습니다.')
    return validate_initial_pose(entries[robot_id])


def save_initial_pose(path, robot_id, values):
    """robot_id pose를 기존 로봇 값과 함께 YAML에 원자적으로 저장한다."""
    pose = validate_initial_pose(values)
    pose_path = Path(os.path.expanduser(str(path)))
    document = {}
    if pose_path.exists():
        with pose_path.open(encoding='utf-8') as pose_file:
            document = yaml.safe_load(pose_file) or {}
        if not isinstance(document, dict):
            raise ValueError(f'{pose_path}의 YAML 최상위가 mapping이 아닙니다.')
    entries = document.setdefault('initial_poses', {})
    if not isinstance(entries, dict):
        raise ValueError(f'{pose_path}의 initial_poses가 mapping이 아닙니다.')
    entries[str(robot_id)] = pose

    pose_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = pose_path.with_name(f'.{pose_path.name}.tmp')
    with temporary_path.open('w', encoding='utf-8') as pose_file:
        yaml.safe_dump(document, pose_file, allow_unicode=True, sort_keys=True)
    os.replace(temporary_path, pose_path)
    return pose_path
