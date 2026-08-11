"""transport_diagnostics_node의 ROS 비의존 계산 함수."""


def parse_default_interface(route_text):
    """/proc/net/route 내용에서 활성 기본 경로 인터페이스를 찾는다."""
    for line in route_text.splitlines()[1:]:
        fields = line.split()
        if len(fields) < 4 or fields[1] != '00000000':
            continue
        try:
            flags = int(fields[3], 16)
        except ValueError:
            continue
        if flags & 0x1:  # RTF_UP
            return fields[0]
    return None


def counter_delta(current, previous):
    """누적 카운터 차이. 인터페이스 재시작으로 값이 줄면 현재값부터 다시 센다."""
    current = int(current)
    previous = int(previous)
    return current - previous if current >= previous else current


def bits_per_second(byte_count, elapsed_sec):
    """구간 바이트 수를 bit/s로 변환한다."""
    if elapsed_sec <= 0.0:
        return 0.0
    return max(0.0, float(byte_count)) * 8.0 / elapsed_sec


def classify_topic(age_sec, publisher_count, actual_hz, minimum_hz,
                   in_startup_grace=False, stale_after_sec=2.5):
    """토픽 상태를 WAIT/NO_PUBLISHER/NO_DATA/STALE/SLOW/OK 중 하나로 분류한다."""
    if in_startup_grace and (publisher_count <= 0 or age_sec is None):
        return 'WAIT'
    if publisher_count <= 0:
        return 'NO_PUBLISHER'
    if age_sec is None:
        return 'NO_DATA'
    # CameraInfo처럼 시작 시 한 번만 오거나 매우 저빈도로 오는 토픽은 minimum_hz=0으로
    # 설정한다. 한 번 수신된 뒤에는 마지막 수신 시각만으로 연결 장애로 오판하지 않는다.
    if minimum_hz <= 0.0:
        return 'OK'
    if age_sec > stale_after_sec:
        return 'STALE'
    if actual_hz < minimum_hz:
        return 'SLOW'
    return 'OK'
