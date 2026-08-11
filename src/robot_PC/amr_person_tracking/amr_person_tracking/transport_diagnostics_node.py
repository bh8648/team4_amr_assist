#!/usr/bin/env python3
"""Raspberry Pi - Robot PC - TurtleBot4 전송 구간을 5초마다 진단한다."""

from dataclasses import dataclass
import os
import shutil
import subprocess
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos_event import SubscriptionEventCallbacks
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from tf2_msgs.msg import TFMessage

from amr_person_tracking.transport_diagnostics_utils import (
    bits_per_second,
    classify_topic,
    counter_delta,
    discovery_config_issues,
    discovery_runtime_issues,
    parse_default_interface,
    parse_discovery_servers,
)


@dataclass
class TopicProbe:
    """한 토픽의 최근 진단 구간 상태."""

    label: str
    source: str
    topic: str
    minimum_hz: float
    count: int = 0
    byte_count: int = 0
    last_seen: float = None
    incompatible_qos: int = 0
    subscription: object = None


def _message_size(msg):
    """고빈도 메시지를 재직렬화하지 않고 wire payload 크기를 가볍게 근사한다."""
    if isinstance(msg, CompressedImage):
        return len(msg.data) + len(msg.format) + len(msg.header.frame_id) + 32
    if isinstance(msg, LaserScan):
        return 4 * (len(msg.ranges) + len(msg.intensities)) + len(msg.header.frame_id) + 80
    if isinstance(msg, TFMessage):
        return sum(160 + len(tf.header.frame_id) + len(tf.child_frame_id)
                   for tf in msg.transforms)
    if isinstance(msg, CameraInfo):
        return 8 * (len(msg.d) + len(msg.k) + len(msg.r) + len(msg.p)) + 128
    if isinstance(msg, Odometry):
        return 760 + len(msg.header.frame_id) + len(msg.child_frame_id)
    return 0


class TransportDiagnosticsNode(Node):
    """로컬 NIC와 원격 장비 토픽의 대역폭/네트워크/QoS 상태를 집계한다."""

    NIC_COUNTERS = ('rx_bytes', 'tx_bytes', 'rx_dropped', 'tx_dropped',
                    'rx_errors', 'tx_errors')

    def __init__(self):
        super().__init__('transport_diagnostics_node')
        self.declare_parameter('namespace', 'robot5')
        self.declare_parameter('period_sec', 5.0)
        self.declare_parameter('startup_grace_sec', 10.0)
        self.declare_parameter('network_interface', '')
        self.declare_parameter('bandwidth_warn_mbps', 80.0)
        self.declare_parameter('stale_after_sec', 2.5)
        self.declare_parameter('require_discovery_server', True)
        self.declare_parameter('discovery_probe_period_sec', 30.0)
        self.declare_parameter('diagnostics_topic', '/robot5/diagnostics')
        self.declare_parameter('rgb_topic', '/robot5/oakd/rgb/image_raw/compressed')
        self.declare_parameter('depth_topic', '/robot5/oakd/stereo/image_raw/compressedDepth')
        self.declare_parameter('camera_info_topic', '/robot5/oakd/rgb/camera_info')
        self.declare_parameter('scan_topic', '/robot5/scan')
        self.declare_parameter('odom_topic', '/robot5/odom')
        self.declare_parameter('tf_topic', '/robot5/tf')
        self.declare_parameter('camera_min_hz', 2.0)
        self.declare_parameter('scan_min_hz', 3.0)
        self.declare_parameter('odom_min_hz', 5.0)
        self.declare_parameter('tf_min_hz', 5.0)

        self.namespace = str(self.get_parameter('namespace').value)
        self.period_sec = max(1.0, float(self.get_parameter('period_sec').value))
        self.startup_grace_sec = max(0.0, float(self.get_parameter('startup_grace_sec').value))
        self.bandwidth_warn_mbps = float(self.get_parameter('bandwidth_warn_mbps').value)
        self.stale_after_sec = max(0.1, float(self.get_parameter('stale_after_sec').value))
        self.require_discovery_server = bool(
            self.get_parameter('require_discovery_server').value)
        self.discovery_probe_period_sec = max(
            5.0, float(self.get_parameter('discovery_probe_period_sec').value))
        requested_interface = str(self.get_parameter('network_interface').value).strip()
        self.network_interface = requested_interface or self._default_interface()

        discovery_value = os.environ.get('ROS_DISCOVERY_SERVER', '')
        self.discovery_servers, parse_issues = parse_discovery_servers(discovery_value)
        self.discovery_config_issues = discovery_config_issues(
            os.environ.get('RMW_IMPLEMENTATION', ''),
            os.environ.get('ROS_LOCALHOST_ONLY', ''),
            self.discovery_servers,
            parse_issues,
            self.require_discovery_server,
        )
        self.discovery_reachability = {}
        self.discovery_last_probe_at = None

        self.started_at = time.monotonic()
        self.last_report_at = self.started_at
        self.nic_previous = self._read_nic_counters()
        self.probes = []

        camera_hz = float(self.get_parameter('camera_min_hz').value)
        self._add_probe('rgb', 'raspberry_pi', CompressedImage, 'rgb_topic', camera_hz)
        self._add_probe('depth', 'raspberry_pi', CompressedImage, 'depth_topic', camera_hz)
        self._add_probe('camera_info', 'raspberry_pi', CameraInfo, 'camera_info_topic', 0.0)
        self._add_probe('scan', 'turtlebot4', LaserScan, 'scan_topic',
                        float(self.get_parameter('scan_min_hz').value))
        self._add_probe('odom', 'turtlebot4', Odometry, 'odom_topic',
                        float(self.get_parameter('odom_min_hz').value))
        self._add_probe('tf', 'turtlebot4', TFMessage, 'tf_topic',
                        float(self.get_parameter('tf_min_hz').value))

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, self.get_parameter('diagnostics_topic').value, 10)
        self.report_timer = self.create_timer(self.period_sec, self._report)

    def _add_probe(self, label, source, msg_type, topic_parameter, minimum_hz):
        topic = str(self.get_parameter(topic_parameter).value)
        probe = TopicProbe(label, source, topic, max(0.0, minimum_hz))

        def on_message(msg):
            probe.count += 1
            probe.byte_count += _message_size(msg)
            probe.last_seen = time.monotonic()

        def on_incompatible_qos(event):
            change = int(getattr(event, 'total_count_change', 1))
            probe.incompatible_qos += max(1, change)

        probe.subscription = self.create_subscription(
            msg_type,
            topic,
            on_message,
            qos_profile_sensor_data,
            event_callbacks=SubscriptionEventCallbacks(incompatible_qos=on_incompatible_qos),
        )
        self.probes.append(probe)

    def _default_interface(self):
        try:
            with open('/proc/net/route', encoding='utf-8') as route_file:
                return parse_default_interface(route_file.read())
        except OSError:
            return None

    def _read_nic_counters(self):
        if not self.network_interface:
            return None
        base = f'/sys/class/net/{self.network_interface}'
        values = {}
        try:
            for name in self.NIC_COUNTERS:
                with open(os.path.join(base, 'statistics', name), encoding='utf-8') as value_file:
                    values[name] = int(value_file.read().strip())
            with open(os.path.join(base, 'operstate'), encoding='utf-8') as state_file:
                values['operstate'] = state_file.read().strip()
        except (OSError, ValueError):
            return None
        return values

    def _refresh_discovery_reachability(self, now):
        """Discovery Server host ICMP 도달성을 저빈도로 갱신한다.

        UDP 11811 listener 자체는 ICMP로 확정할 수 없다. 대신 host ping과
        타입을 명시한 ROS 구독에서 발견한 실제 endpoint를 결합해 판정한다.
        """
        if (self.discovery_last_probe_at is not None
                and now - self.discovery_last_probe_at < self.discovery_probe_period_sec):
            return
        self.discovery_last_probe_at = now
        ping = shutil.which('ping')
        for _, host, _ in self.discovery_servers:
            if host in self.discovery_reachability and ping is None:
                continue
            if ping is None:
                self.discovery_reachability[host] = None
                continue
            try:
                result = subprocess.run(
                    [ping, '-n', '-c', '1', '-W', '1', host],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=1.5,
                    check=False,
                )
                self.discovery_reachability[host] = result.returncode == 0
            except (OSError, subprocess.TimeoutExpired):
                self.discovery_reachability[host] = False

    def _nic_report(self, current, elapsed):
        if current is None:
            return None, ['nic_unavailable']
        previous = self.nic_previous or current
        delta = {name: counter_delta(current[name], previous[name]) for name in self.NIC_COUNTERS}
        rx_mbps = bits_per_second(delta['rx_bytes'], elapsed) / 1e6
        tx_mbps = bits_per_second(delta['tx_bytes'], elapsed) / 1e6
        issues = []
        if current.get('operstate') != 'up':
            issues.append(f'link_{current.get("operstate", "unknown")}')
        dropped = delta['rx_dropped'] + delta['tx_dropped']
        errors = delta['rx_errors'] + delta['tx_errors']
        if dropped:
            issues.append(f'drop+{dropped}')
        if errors:
            issues.append(f'error+{errors}')
        if self.bandwidth_warn_mbps > 0.0 and max(rx_mbps, tx_mbps) > self.bandwidth_warn_mbps:
            issues.append('bandwidth_high')
        return {
            'rx_mbps': rx_mbps,
            'tx_mbps': tx_mbps,
            'drops': dropped,
            'errors': errors,
            'state': current.get('operstate', 'unknown'),
        }, issues

    @staticmethod
    def _status(name, level, message, values):
        status = DiagnosticStatus()
        status.name = name
        status.hardware_id = 'transport'
        status.level = level
        status.message = message
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in values]
        return status

    def _report(self):
        now = time.monotonic()
        elapsed = max(now - self.last_report_at, 1e-6)
        in_grace = now - self.started_at < self.startup_grace_sec

        nic_current = self._read_nic_counters()
        nic, nic_issues = self._nic_report(nic_current, elapsed)
        source_bytes = {'raspberry_pi': 0, 'turtlebot4': 0}
        network_issues = list(nic_issues)
        qos_issues = []
        topic_parts = []
        diagnostic_values = []
        remote_endpoint_count = 0

        for probe in self.probes:
            hz = probe.count / elapsed
            age = None if probe.last_seen is None else max(0.0, now - probe.last_seen)
            # Humble Subscription에는 get_publisher_count()가 없으므로 Node graph API를 쓴다.
            publishers = self.count_publishers(probe.topic)
            remote_endpoint_count += publishers
            state = classify_topic(
                age, publishers, hz, probe.minimum_hz,
                in_startup_grace=in_grace, stale_after_sec=self.stale_after_sec)
            source_bytes[probe.source] += probe.byte_count
            age_text = 'n/a' if age is None else f'{age:.1f}s'
            topic_parts.append(f'{probe.label}={hz:.1f}Hz/{age_text}/{state}')
            diagnostic_values.extend([
                (f'{probe.label}.hz', f'{hz:.2f}'),
                (f'{probe.label}.age_sec', age_text),
                (f'{probe.label}.publishers', publishers),
                (f'{probe.label}.qos_incompatible', probe.incompatible_qos),
            ])
            if state in ('NO_DATA', 'STALE', 'SLOW'):
                network_issues.append(f'{probe.label}:{state}')
            if state == 'NO_PUBLISHER':
                qos_issues.append(f'{probe.label}:publisher=0')
            if probe.incompatible_qos:
                qos_issues.append(f'{probe.label}:incompatible+{probe.incompatible_qos}')
            probe.count = 0
            probe.byte_count = 0
            probe.incompatible_qos = 0

        raspberry_mbps = bits_per_second(source_bytes['raspberry_pi'], elapsed) / 1e6
        turtlebot_mbps = bits_per_second(source_bytes['turtlebot4'], elapsed) / 1e6
        if nic is None:
            nic_text = f'{self.network_interface or "unknown"}:n/a'
        else:
            nic_text = (f'{self.network_interface}:rx={nic["rx_mbps"]:.1f}/'
                        f'tx={nic["tx_mbps"]:.1f}Mbps drop={nic["drops"]} err={nic["errors"]}')

        bandwidth_issues = [issue for issue in nic_issues if issue == 'bandwidth_high']
        network_issues = [issue for issue in network_issues if issue != 'bandwidth_high']
        bandwidth_level = DiagnosticStatus.WARN if bandwidth_issues else DiagnosticStatus.OK
        network_level = (
            DiagnosticStatus.WARN
            if network_issues and not in_grace else DiagnosticStatus.OK)
        qos_level = DiagnosticStatus.WARN if qos_issues and not in_grace else DiagnosticStatus.OK
        self._refresh_discovery_reachability(now)
        discovery_issues = discovery_runtime_issues(
            self.discovery_config_issues,
            self.discovery_servers,
            self.discovery_reachability,
            remote_endpoint_count,
            in_startup_grace=in_grace,
            require_server=self.require_discovery_server,
        )
        discovery_level = (
            DiagnosticStatus.WARN if discovery_issues else DiagnosticStatus.OK)
        server_parts = []
        for server_id, host, port in self.discovery_servers:
            reachable = self.discovery_reachability.get(host)
            state = 'unknown' if reachable is None else ('up' if reachable else 'down')
            server_parts.append(f'id{server_id}={host}:{port}/{state}')
        server_text = ','.join(server_parts) if server_parts else 'multicast/unset'

        # Humble의 메시지 상수는 b'\x00'/b'\x01'이라 OK도 bool()이 True다.
        # 상태 상수의 truthiness가 아니라 실제 issue 목록으로 사람이 보는 로그를 만든다.
        bandwidth_log = (
            'WARN:' + ','.join(bandwidth_issues) if bandwidth_issues else 'OK')
        network_log = (
            'WARN:' + ','.join(network_issues)
            if network_issues and not in_grace else 'OK')
        qos_log = (
            'WARN:' + ','.join(qos_issues) if qos_issues and not in_grace else 'OK')
        discovery_log = (
            'WARN:' + ','.join(discovery_issues) if discovery_issues else 'OK')
        log_text = (
            f'[transport {elapsed:.1f}s] BANDWIDTH={bandwidth_log} '
            f'{nic_text} pi_rx={raspberry_mbps:.2f}Mbps tb4_rx={turtlebot_mbps:.2f}Mbps | '
            f'NETWORK={network_log} '
            f'{" ".join(topic_parts)} | '
            f'QOS={qos_log} | DISCOVERY={discovery_log} '
            f'{server_text} endpoints={remote_endpoint_count}'
        )
        has_warnings = bool(
            bandwidth_issues or discovery_issues
            or (not in_grace and (network_issues or qos_issues)))
        # Humble logger는 같은 호출 위치의 severity 변경을 허용하지 않는다.
        if has_warnings:
            self.get_logger().warn(log_text)
        else:
            self.get_logger().info(log_text)

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        common_values = [
            ('interface', self.network_interface or 'unknown'),
            ('raspberry_pi_rx_mbps', f'{raspberry_mbps:.3f}'),
            ('turtlebot4_rx_mbps', f'{turtlebot_mbps:.3f}'),
        ]
        if nic is not None:
            common_values.extend([
                ('nic_rx_mbps', f'{nic["rx_mbps"]:.3f}'),
                ('nic_tx_mbps', f'{nic["tx_mbps"]:.3f}'),
                ('nic_drops', nic['drops']),
                ('nic_errors', nic['errors']),
                ('nic_state', nic['state']),
            ])
        discovery_values = [
            ('rmw_implementation', os.environ.get('RMW_IMPLEMENTATION', 'default')),
            ('ros_domain_id', os.environ.get('ROS_DOMAIN_ID', 'default')),
            ('ros_localhost_only', os.environ.get('ROS_LOCALHOST_ONLY', 'default')),
            ('ros_super_client', os.environ.get('ROS_SUPER_CLIENT', 'unset')),
            ('ros_discovery_server', os.environ.get('ROS_DISCOVERY_SERVER', '')),
            ('servers', server_text),
            ('remote_endpoint_count', remote_endpoint_count),
            ('udp_listener_note',
             'host ping + ROS endpoint evidence; UDP port not directly probed'),
        ]
        msg.status = [
            self._status(
                f'{self.namespace}: transport/bandwidth', bandwidth_level,
                '정상' if not bandwidth_issues else ', '.join(bandwidth_issues), common_values),
            self._status(
                f'{self.namespace}: transport/network', network_level,
                ('연결 대기' if in_grace else
                 ('정상' if not network_issues else ', '.join(network_issues))),
                diagnostic_values),
            self._status(
                f'{self.namespace}: transport/qos', qos_level,
                '연결 대기' if in_grace else ('정상' if not qos_issues else ', '.join(qos_issues)),
                diagnostic_values),
            self._status(
                f'{self.namespace}: transport/discovery', discovery_level,
                ('연결 대기' if in_grace and not discovery_issues else
                 ('정상' if not discovery_issues else ', '.join(discovery_issues))),
                discovery_values),
        ]
        self.diagnostics_pub.publish(msg)
        self.nic_previous = nic_current
        self.last_report_at = now


def main(args=None):
    rclpy.init(args=args)
    node = TransportDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
