import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('amr_flow')

    pc1_params = os.path.join(package_share, 'config', 'pc1_webcam_worker.yaml')
    pc2_params = os.path.join(package_share, 'config', 'pc2_goal_coordinator.yaml')
    pc3_executor_params = os.path.join(package_share, 'config', 'pc3_amr_executor.yaml')
    pc3_status_params = os.path.join(package_share, 'config', 'pc3_robot_status.yaml')

    return LaunchDescription([
        Node(
            package='amr_flow',
            executable='pc1_webcam_worker_node',
            name='pc1_webcam_worker_node',
            output='screen',
            parameters=[pc1_params],
        ),
        Node(
            package='amr_flow',
            executable='pc2_goal_coordinator_node',
            name='pc2_goal_coordinator_node',
            output='screen',
            parameters=[pc2_params],
        ),
        Node(
            package='amr_flow',
            executable='pc3_amr_executor_node',
            name='pc3_amr_executor_node',
            output='screen',
            parameters=[pc3_executor_params],
        ),
        Node(
            package='amr_flow',
            executable='pc3_robot_status_node',
            name='pc3_robot_status_node',
            output='screen',
            parameters=[pc3_status_params],
        ),
    ])
