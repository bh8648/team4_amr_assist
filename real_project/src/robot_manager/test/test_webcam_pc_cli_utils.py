from robot_manager.webcam_pc_cli_utils import FOLLOWING_MOCK_POSES, parse_command, parse_interval


def test_parse_command_simple_word_no_args():
    assert parse_command('상태') == ('상태', [])


def test_parse_command_with_args():
    assert parse_command('추종시작 5') == ('추종시작', ['5'])


def test_parse_command_empty_input_returns_empty_command():
    assert parse_command('   ') == ('', [])


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


def test_following_mock_poses_has_ten_points():
    assert len(FOLLOWING_MOCK_POSES) == 10
