# person_locator.launch.py
#
# 추론 가능한 머신에 같이 있어야 할 두 노드를 묶음: pose_locator_node
# (YOLO-pose + homography -> map (x, y))와 person_tf_broadcaster_node
# (그 점 -> "map" -> "person" TF).
#
# camera_publisher는 일부러 여기서 실행하지 않음 - 실제 배포에서는 카메라가
# 별도 머신(이 개발 PC)에 남아있고, `ros2 run person_locator camera_publisher`로
# 독립적으로 실행함.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # CLI에서 override할 수 있도록 launch argument로 노출해둠, 예:
    #   ros2 launch person_locator person_locator.launch.py model_path:=/path/to/yolov8n-pose.pt
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='yolov8n-pose.pt',
        description='YOLO-pose weight 경로 (파일명만 주면 자동 다운로드)',
    )
    homography_yaml_arg = DeclareLaunchArgument(
        'homography_yaml_path', default_value='config/person_homography.yaml',
        description='calibrate_homography가 생성한 homography YAML 경로',
    )

    pose_locator = Node(
        package='person_locator',
        executable='pose_locator_node',
        name='pose_locator_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'homography_yaml_path': LaunchConfiguration('homography_yaml_path'),
        }],
    )

    tf_broadcaster = Node(
        package='person_locator',
        executable='person_tf_broadcaster_node',
        name='person_tf_broadcaster_node',
        output='screen',
    )

    return LaunchDescription([
        model_path_arg,
        homography_yaml_arg,
        pose_locator,
        tf_broadcaster,
    ])
