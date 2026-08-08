from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('db_path', default_value='amr.db'),
        DeclareLaunchArgument('map_yaml_path', default_value='amr_delivery_ui/frontend/maps/map2.yaml'),
        Node(package='robot_manager', executable='robot_assignment_node', name='amr_allocation_node', output='screen', parameters=[{'map_yaml_path': LaunchConfiguration('map_yaml_path')}]),
        Node(package='robot_manager', executable='task_manager_node', name='task_manager_node', output='screen'),
        Node(package='robot_manager', executable='hmi_backend_node', name='hmi_manager_node', output='screen', parameters=[{'db_path': LaunchConfiguration('db_path'), 'map_yaml_path': LaunchConfiguration('map_yaml_path')}]),
        Node(package='robot_manager', executable='db_manager_node', name='db_manager_node', output='screen', parameters=[{'db_path': LaunchConfiguration('db_path')}]),
        Node(package='robot_manager', executable='deadlock_prevention_node', name='deadlock_prevention_node', output='screen'),
    ])
