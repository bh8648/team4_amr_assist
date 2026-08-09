from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # robot5 PC에서는 robot5 전용 브릿지만 실행한다.
        Node(package='robot5_bridge', executable='robot5_bridge_node',
             name='robot5_bridge_node', output='screen'),
    ])
