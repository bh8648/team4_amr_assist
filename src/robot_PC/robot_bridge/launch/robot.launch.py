from launch import LaunchDescription
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


HMI_PORTS = {'robot5': 8005, 'robot11': 8011}
DEFAULT_INITIAL_POSE_FILE = os.path.expanduser(
    '~/.config/team4_amr_assist/initial_poses.yaml')


def _launch_nodes(context):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    if robot_id not in HMI_PORTS:
        raise ValueError('robot_id는 robot5 또는 robot11이어야 합니다.')
    configured_port = LaunchConfiguration('web_port').perform(context).strip()
    web_port = int(configured_port) if configured_port else HMI_PORTS[robot_id]
    return [
        Node(
            package='robot_bridge', executable='initial_pose_publisher_node',
            namespace=robot_id, name='initial_pose_publisher_node', output='screen',
            condition=IfCondition(LaunchConfiguration('auto_initial_pose')),
            parameters=[{
                'robot_id': robot_id,
                'pose_file': LaunchConfiguration('initial_pose_file'),
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
                'initial_pose_delay_sec': LaunchConfiguration(
                    'initial_pose_delay_sec'),
                'initial_pose_retry_sec': LaunchConfiguration(
                    'initial_pose_retry_sec'),
                'initial_pose_max_attempts': LaunchConfiguration(
                    'initial_pose_max_attempts'),
                'initial_pose_reload_sec': LaunchConfiguration(
                    'initial_pose_reload_sec'),
                'initial_pose_confirm_position_tolerance': LaunchConfiguration(
                    'initial_pose_confirm_position_tolerance'),
                'initial_pose_confirm_yaw_tolerance': LaunchConfiguration(
                    'initial_pose_confirm_yaw_tolerance'),
            }],
        ),
        # 브릿지만 띄우고 추종 노드를 빠뜨리는 현장 실수를 막기 위해 같은 명령에서 함께 기동한다.
        # 카메라/추종을 별도로 진단할 때는 enable_person_tracking:=false로 끌 수 있다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('amr_person_tracking'),
                'launch', 'amr_person_tracking.launch.py')),
            condition=IfCondition(LaunchConfiguration('enable_person_tracking')),
            launch_arguments={
                'namespace': robot_id,
                'pose_model_path': LaunchConfiguration('pose_model_path'),
                'tracker_config_path': LaunchConfiguration('tracker_config_path'),
                'camera_priority_timeout': LaunchConfiguration(
                    'camera_priority_timeout'),
                'target_coast_timeout': LaunchConfiguration('target_coast_timeout'),
                'target_coast_max_extrapolation': LaunchConfiguration(
                    'target_coast_max_extrapolation'),
                'enable_transport_diagnostics': LaunchConfiguration(
                    'enable_transport_diagnostics'),
                'transport_diagnostics_period': LaunchConfiguration(
                    'transport_diagnostics_period'),
                'network_interface': LaunchConfiguration('network_interface'),
                'bandwidth_warn_mbps': LaunchConfiguration('bandwidth_warn_mbps'),
                'require_discovery_server': LaunchConfiguration(
                    'require_discovery_server'),
                'publish_debug_image': LaunchConfiguration('publish_debug_image'),
                'publish_markers': LaunchConfiguration('publish_markers'),
            }.items(),
        ),
        Node(
            package='robot_bridge', executable='robot_bridge_node',
            namespace=robot_id, name='robot_bridge_node', output='screen',
            parameters=[{'robot_id': robot_id}],
        ),
        Node(
            package='robot_hmi_backend', executable='hmi_backend_node',
            namespace=robot_id, name='robot_hmi_backend_node', output='screen',
            condition=IfCondition(LaunchConfiguration('enable_hmi_backend')),
            parameters=[{'robot_id': robot_id, 'web_port': web_port}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            description='이 PC가 제어할 로봇 ID: robot5 또는 robot11',
        ),
        DeclareLaunchArgument(
            'web_port', default_value='',
            description='비어 있으면 robot5=8005, robot11=8011을 사용',
        ),
        DeclareLaunchArgument(
            'enable_hmi_backend', default_value='true',
            description='Robot PC HMI 백엔드를 함께 실행할지 여부',
        ),
        DeclareLaunchArgument(
            'enable_person_tracking', default_value='true',
            description='OAK-D/LiDAR 사람 추종 파이프라인을 함께 실행할지 여부',
        ),
        DeclareLaunchArgument(
            'publish_debug_image', default_value='false',
            description='OAK-D 검출 오버레이 이미지 발행 및 디버그 창 표시',
        ),
        DeclareLaunchArgument(
            'publish_markers', default_value='false',
            description='LiDAR 다리 검출 RViz Marker 발행',
        ),
        DeclareLaunchArgument(
            'camera_priority_timeout', default_value='0.5',
            description='마지막 카메라 관측을 LiDAR보다 우선할 시간(초)',
        ),
        DeclareLaunchArgument(
            'target_coast_timeout', default_value='1.0',
            description='두 센서가 함께 끊겼을 때 목표를 짧게 예측할 시간(초)',
        ),
        DeclareLaunchArgument(
            'target_coast_max_extrapolation', default_value='0.5',
            description='센서 공백 중 최대 등속도 외삽 시간(초)',
        ),
        DeclareLaunchArgument(
            'auto_initial_pose', default_value='true',
            description='저장한 dock pose를 AMCL initial pose로 자동 설정',
        ),
        DeclareLaunchArgument(
            'initial_pose_file', default_value=DEFAULT_INITIAL_POSE_FILE,
            description='로봇별 dock pose를 저장한 YAML 경로',
        ),
        DeclareLaunchArgument(
            'initial_pose_x', default_value='',
            description='YAML 대신 사용할 map x(비우면 YAML 사용)',
        ),
        DeclareLaunchArgument(
            'initial_pose_y', default_value='',
            description='YAML 대신 사용할 map y(비우면 YAML 사용)',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw', default_value='',
            description='YAML 대신 사용할 map yaw rad(비우면 YAML 사용)',
        ),
        DeclareLaunchArgument(
            'initial_pose_delay_sec', default_value='2.0',
            description='AMCL discovery를 기다린 후 첫 initial pose를 보낼 시간',
        ),
        DeclareLaunchArgument(
            'initial_pose_retry_sec', default_value='1.0',
            description='AMCL pose 확인 전 initial pose 재전송 주기',
        ),
        DeclareLaunchArgument(
            'initial_pose_max_attempts', default_value='10',
            description='Dock initial pose 최대 전송 횟수',
        ),
        DeclareLaunchArgument(
            'initial_pose_reload_sec', default_value='2.0',
            description='누락된 dock pose 파일을 재측정 후 다시 읽는 주기',
        ),
        DeclareLaunchArgument(
            'initial_pose_confirm_position_tolerance', default_value='0.75',
            description='AMCL initial pose 확인 위치 허용오차(m)',
        ),
        DeclareLaunchArgument(
            'initial_pose_confirm_yaw_tolerance', default_value='0.8',
            description='AMCL initial pose 확인 방향 허용오차(rad)',
        ),
        DeclareLaunchArgument(
            'pose_model_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('amr_person_tracking'),
                'config',
                'yolo11n-pose.pt',
            ]),
            description='AMR 사람 검출 YOLO-pose 가중치의 절대 경로',
        ),
        DeclareLaunchArgument(
            'tracker_config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('amr_person_tracking'),
                'config',
                'botsort_reid.yaml',
            ]),
            description='AMR 사람 추적기 설정 파일의 절대 경로',
        ),
        DeclareLaunchArgument(
            'enable_transport_diagnostics', default_value='true',
            description='5초 주기 전송 대역폭/네트워크/QoS 진단을 실행할지 여부',
        ),
        DeclareLaunchArgument(
            'transport_diagnostics_period', default_value='5.0',
            description='전송 진단 로그 주기(초)',
        ),
        DeclareLaunchArgument(
            'network_interface', default_value='',
            description='진단할 Robot PC NIC. 비어 있으면 기본 경로를 자동 선택',
        ),
        DeclareLaunchArgument(
            'bandwidth_warn_mbps', default_value='80.0',
            description='Robot PC NIC RX/TX 대역폭 WARN 기준(Mbit/s)',
        ),
        DeclareLaunchArgument(
            'require_discovery_server', default_value='true',
            description='현장 Fast DDS Discovery Server 설정을 필수로 진단',
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
