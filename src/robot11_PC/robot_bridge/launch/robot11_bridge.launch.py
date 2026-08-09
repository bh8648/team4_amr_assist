from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 공통 브릿지 구현에 robot11 ID를 명시해 기존 실행 동작을 유지한다.
        Node(package='robot_bridge', executable='robot11_bridge_node', name='robot11_bridge_node',
             parameters=[{'robot_id': 'robot11'}], output='screen'),
    ])
