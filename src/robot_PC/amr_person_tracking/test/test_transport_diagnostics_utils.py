"""전송 진단 계산 로직 단위 테스트."""

import pytest

from amr_person_tracking.transport_diagnostics_utils import (
    bits_per_second,
    classify_topic,
    counter_delta,
    parse_default_interface,
)


def test_parse_default_interface_uses_up_default_route():
    route = (
        'Iface Destination Gateway Flags RefCnt Use Metric Mask MTU Window IRTT\n'
        'wlan0 00000000 0101A8C0 0003 0 0 600 00000000 0 0 0\n'
        'eth0 0001A8C0 00000000 0001 0 0 100 00FFFFFF 0 0 0\n'
    )
    assert parse_default_interface(route) == 'wlan0'


def test_counter_delta_handles_interface_counter_reset():
    assert counter_delta(120, 100) == 20
    assert counter_delta(5, 100) == 5


def test_bits_per_second():
    assert bits_per_second(1_000_000, 2.0) == pytest.approx(4_000_000.0)
    assert bits_per_second(10, 0.0) == 0.0


@pytest.mark.parametrize(
    'age,publishers,hz,minimum,grace,expected',
    [
        (None, 0, 0.0, 2.0, True, 'WAIT'),
        (None, 0, 0.0, 2.0, False, 'NO_PUBLISHER'),
        (None, 1, 0.0, 2.0, False, 'NO_DATA'),
        (3.0, 1, 10.0, 2.0, False, 'STALE'),
        (30.0, 1, 0.0, 0.0, False, 'OK'),
        (0.1, 1, 1.0, 2.0, False, 'SLOW'),
        (0.1, 1, 5.0, 2.0, False, 'OK'),
    ],
)
def test_classify_topic(age, publishers, hz, minimum, grace, expected):
    assert classify_topic(age, publishers, hz, minimum, grace, 2.5) == expected
