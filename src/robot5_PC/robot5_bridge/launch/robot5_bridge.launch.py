from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # robot5 PC에서 브릿지와 탑재 HMI 백엔드를 함께 실행한다.
        Node(package='robot5_bridge', executable='robot5_bridge_node',
             name='robot5_bridge_node', output='screen'),
        Node(package='robot5_hmi_backend', executable='hmi_backend_node',
             name='robot5_hmi_backend_node', output='screen',
             parameters=[{'robot_id': 'robot5', 'web_port': 8000}]),
    ])
