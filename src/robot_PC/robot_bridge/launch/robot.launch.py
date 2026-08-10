from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


HMI_PORTS = {'robot5': 8005, 'robot11': 8011}


def _launch_nodes(context):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    if robot_id not in HMI_PORTS:
        raise ValueError('robot_id는 robot5 또는 robot11이어야 합니다.')
    configured_port = LaunchConfiguration('web_port').perform(context).strip()
    web_port = int(configured_port) if configured_port else HMI_PORTS[robot_id]
    return [
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
        OpaqueFunction(function=_launch_nodes),
    ])
