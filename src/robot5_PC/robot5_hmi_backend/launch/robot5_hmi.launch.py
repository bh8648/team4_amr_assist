from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 이 PC에 연결된 로봇 ID를 고정해 다른 로봇의 HMI 명령을 만들지 못하게 한다.
        Node(
            package='robot5_hmi_backend', executable='hmi_backend_node',
            name='robot5_hmi_backend_node', output='screen',
            parameters=[{'robot_id': 'robot5', 'web_port': 8000}],
        ),
    ])
