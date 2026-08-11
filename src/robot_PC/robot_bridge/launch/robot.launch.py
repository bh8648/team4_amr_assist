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


def _launch_nodes(context):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    if robot_id not in HMI_PORTS:
        raise ValueError('robot_id는 robot5 또는 robot11이어야 합니다.')
    configured_port = LaunchConfiguration('web_port').perform(context).strip()
    web_port = int(configured_port) if configured_port else HMI_PORTS[robot_id]
    return [
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
                # 운영 PC는 디스플레이가 없을 수 있으므로 OpenCV 디버그 창은 기본 비활성화한다.
                'publish_debug_image': 'false',
                'publish_markers': 'false',
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
            'enable_person_tracking', default_value='true',
            description='OAK-D/LiDAR 사람 추종 파이프라인을 함께 실행할지 여부',
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
        OpaqueFunction(function=_launch_nodes),
    ])
