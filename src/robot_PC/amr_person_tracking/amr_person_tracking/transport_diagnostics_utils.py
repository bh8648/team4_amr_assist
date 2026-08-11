"""transport_diagnostics_node의 ROS 비의존 계산 함수."""


def parse_discovery_servers(value):
    """ROS_DISCOVERY_SERVER를 ``(server_id, host, port)`` 목록으로 파싱한다.

    세미콜론 사이의 빈 항목은 Fast DDS server id를 예약하므로
    버리지 않고 원래 index를 server id로 유지한다.
    """
    servers = []
    issues = []
    for server_id, raw_entry in enumerate(str(value or '').split(';')):
        entry = raw_entry.strip()
        if not entry:
            continue
        host, separator, port_text = entry.rpartition(':')
        host = host.strip().strip('[]')
        if not separator or not host:
            issues.append(f'server{server_id}:invalid_address')
            continue
        try:
            port = int(port_text)
        except ValueError:
            issues.append(f'server{server_id}:invalid_port')
            continue
        if not 1 <= port <= 65535:
            issues.append(f'server{server_id}:invalid_port')
            continue
        servers.append((server_id, host, port))
    return servers, issues


def discovery_config_issues(rmw_implementation, localhost_only, servers,
                            parse_issues=(), require_server=True):
    """현장 Fast DDS discovery 환경설정의 명확한 충돌을 반환한다."""
    issues = list(parse_issues)
    rmw = str(rmw_implementation or '').strip()
    localhost = str(localhost_only or '').strip().lower()
    if require_server and not servers:
        issues.append('discovery_server_unset')
    if servers and rmw not in ('', 'rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp'):
        issues.append(f'rmw_not_fastrtps:{rmw}')
    if servers and localhost in ('1', 'true', 'yes'):
        issues.append('localhost_only_blocks_remote_server')
    return issues


def discovery_runtime_issues(config_issues, servers, reachability,
                             remote_endpoint_count, in_startup_grace=False,
                             require_server=True):
    """server host 도달성과 실제 DDS endpoint 발견 결과를 합쳐 판정한다."""
    issues = list(config_issues)
    for server_id, host, port in servers:
        if reachability.get(host) is False:
            issues.append(f'server{server_id}_host_unreachable:{host}:{port}')
    if (require_server and servers and not in_startup_grace
            and int(remote_endpoint_count) <= 0):
        issues.append('remote_graph_empty')
    return issues


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
