from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_bridge', executable='robot5_bridge_node', name='robot5_bridge_node', output='screen'),
    ])
