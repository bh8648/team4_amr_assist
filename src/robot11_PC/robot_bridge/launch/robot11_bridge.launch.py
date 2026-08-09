from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # robot11 PC에서는 robot11 전용 브릿지 노드 하나만 실행한다.
        Node(package='robot_bridge', executable='robot11_bridge_node', name='robot11_bridge_node', output='screen'),
    ])
