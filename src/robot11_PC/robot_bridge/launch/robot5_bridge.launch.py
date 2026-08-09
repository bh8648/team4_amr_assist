from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # robot5 PC에서는 이 launch만 실행해 robot5 namespace의 센서·액션과 연결한다.
        Node(package='robot_bridge', executable='robot5_bridge_node', name='robot5_bridge_node',
             parameters=[{'robot_id': 'robot5'}], output='screen'),
    ])
