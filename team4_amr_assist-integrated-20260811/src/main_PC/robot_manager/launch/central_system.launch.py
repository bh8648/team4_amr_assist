from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_map = os.path.join(get_package_share_directory('robot_manager'), 'maps', 'map2.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('db_path', default_value='amr.db'),
        DeclareLaunchArgument('map_yaml_path', default_value=default_map),
        Node(package='robot_manager', executable='robot_assignment_node', name='robot_assignment_node', output='screen', parameters=[{'map_yaml_path': LaunchConfiguration('map_yaml_path')}]),
        Node(package='robot_manager', executable='task_manager_node', name='task_manager_node', output='screen'),
        # 로봇 PC가 DB 파일을 직접 공유하지 않도록 중앙에서 목적지 목록을 배포한다.
        Node(package='robot_manager', executable='destination_manager_node', name='destination_manager_node', output='screen', parameters=[{'db_path': LaunchConfiguration('db_path')}]),
        Node(package='robot_manager', executable='hmi_backend_node', name='hmi_manager_node', output='screen', parameters=[{'db_path': LaunchConfiguration('db_path'), 'map_yaml_path': LaunchConfiguration('map_yaml_path')}]),
        Node(package='robot_manager', executable='db_manager_node', name='db_manager_node', output='screen', parameters=[{'db_path': LaunchConfiguration('db_path')}]),
        # 현재 운영 범위에서는 교착 방지 기능을 사용하지 않는다.
        # 노드를 실행하지 않아도 task_manager의 일반 작업 흐름은 독립적으로 동작한다.
    ])
