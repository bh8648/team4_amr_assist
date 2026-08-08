import math

from robot_manager.webcam_pc_cli_utils import (
    FOLLOWING_MOCK_POSES,
    parse_call_args,
    parse_command,
    parse_interval,
    select_active_robot,
    select_destination,
)


def test_parse_command_deliver_without_internal_space():
    assert parse_command('배송모드') == ('배송모드', [])


def test_parse_command_deliver_with_internal_space():
    assert parse_command('배송 모드') == ('배송모드', [])


def test_parse_command_deliver_with_destination_id():
    assert parse_command('배송모드 DEST_A') == ('배송모드', ['DEST_A'])


def test_parse_command_deliver_with_space_and_destination_id():
    assert parse_command('배송 모드 DEST_A') == ('배송모드', ['DEST_A'])


def test_parse_command_simple_word_no_args():
    assert parse_command('상태') == ('상태', [])


def test_parse_command_with_args():
    assert parse_command('호출 -1 0') == ('호출', ['-1', '0'])


def test_parse_command_empty_input_returns_empty_command():
    assert parse_command('   ') == ('', [])


def test_parse_call_args_valid_floats():
    assert parse_call_args(['-1', '0']) == ((-1.0, 0.0), None)


def test_parse_call_args_wrong_arg_count():
    value, error = parse_call_args(['-1'])
    assert value is None
    assert error == '사용법: 호출 <x> <y>'


def test_parse_call_args_non_numeric():
    value, error = parse_call_args(['a', 'b'])
    assert value is None
    assert error == 'x, y는 숫자여야 합니다'


def test_parse_interval_default_when_no_args():
    assert parse_interval([]) == (3.0, None)


def test_parse_interval_valid_value():
    assert parse_interval(['5']) == (5.0, None)


def test_parse_interval_rejects_zero():
    value, error = parse_interval(['0'])
    assert value is None
    assert error == '간격초는 양수여야 합니다'


def test_parse_interval_rejects_negative():
    value, error = parse_interval(['-1'])
    assert value is None
    assert error == '간격초는 양수여야 합니다'


def test_parse_interval_rejects_non_numeric():
    value, error = parse_interval(['abc'])
    assert value is None
    assert error == '간격초는 숫자여야 합니다'


def test_select_destination_found_by_id():
    destinations = [
        {'destination_id': 'DEST_A', 'position_x': -0.5, 'position_y': -2.0, 'orientation_yaw': math.pi},
        {'destination_id': 'DEST_B', 'position_x': -4.0, 'position_y': -3.0, 'orientation_yaw': 0.0},
    ]
    destination, error = select_destination(destinations, 'DEST_B')
    assert error is None
    assert destination['destination_id'] == 'DEST_B'


def test_select_destination_not_found_by_id():
    destinations = [{'destination_id': 'DEST_A', 'position_x': 0.0, 'position_y': 0.0, 'orientation_yaw': 0.0}]
    destination, error = select_destination(destinations, 'DEST_Z')
    assert destination is None
    assert error == '목적지 없음: DEST_Z'


def test_select_destination_auto_selects_when_single():
    destinations = [{'destination_id': 'DEST_A', 'position_x': 0.0, 'position_y': 0.0, 'orientation_yaw': 0.0}]
    destination, error = select_destination(destinations, None)
    assert error is None
    assert destination['destination_id'] == 'DEST_A'


def test_select_destination_requires_id_when_multiple():
    destinations = [
        {'destination_id': 'DEST_A', 'position_x': 0.0, 'position_y': 0.0, 'orientation_yaw': 0.0},
        {'destination_id': 'DEST_B', 'position_x': 0.0, 'position_y': 0.0, 'orientation_yaw': 0.0},
    ]
    destination, error = select_destination(destinations, None)
    assert destination is None
    assert 'DEST_A' in error and 'DEST_B' in error


def test_select_destination_empty_list():
    destination, error = select_destination([], None)
    assert destination is None
    assert error == '등록된 목적지 없음'


def test_select_active_robot_single_active():
    robot_id, error = select_active_robot({'robot11': 'FOLLOWING'})
    assert error is None
    assert robot_id == 'robot11'


def test_select_active_robot_none_active():
    robot_id, error = select_active_robot({})
    assert robot_id is None
    assert error == '활성 작업 없음'


def test_select_active_robot_ignores_docked_and_error():
    robot_id, error = select_active_robot({'robot11': 'DOCKED', 'robot5': 'ERROR'})
    assert robot_id is None
    assert error == '활성 작업 없음'


def test_select_active_robot_multiple_active_requires_selection():
    robot_id, error = select_active_robot({'robot11': 'FOLLOWING', 'robot5': 'ASSIGNED'})
    assert robot_id is None
    assert 'robot11' in error and 'robot5' in error


def test_select_active_robot_requested_id_present_and_active():
    robot_id, error = select_active_robot(
        {'robot11': 'FOLLOWING', 'robot5': 'ASSIGNED'}, requested_id='robot5')
    assert error is None
    assert robot_id == 'robot5'


def test_select_active_robot_requested_id_not_active():
    robot_id, error = select_active_robot(
        {'robot11': 'FOLLOWING', 'robot5': 'DOCKED'}, requested_id='robot5')
    assert robot_id is None
    assert error == 'robot5는 활성 상태가 아닙니다'


def test_select_active_robot_requested_id_absent_entirely():
    robot_id, error = select_active_robot({'robot11': 'FOLLOWING'}, requested_id='robot5')
    assert robot_id is None
    assert error == 'robot5는 활성 상태가 아닙니다'


def test_following_mock_poses_has_ten_points():
    assert len(FOLLOWING_MOCK_POSES) == 10


def test_following_mock_poses_x_and_yaw_are_constant():
    for x, _y, yaw in FOLLOWING_MOCK_POSES:
        assert x == -1.5
        assert math.isclose(yaw, -math.pi / 2)


def test_following_mock_poses_y_decreases_from_0_5_to_minus_4():
    ys = [y for _x, y, _yaw in FOLLOWING_MOCK_POSES]
    assert ys[0] == 0.5
    assert ys[-1] == -4.0
    assert ys == sorted(ys, reverse=True)
