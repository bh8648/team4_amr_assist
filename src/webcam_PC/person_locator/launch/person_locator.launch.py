# person_locator.launch.py
#
# 추론 가능한 머신에서 pose_locator_node(YOLO-pose + homography -> map (x, y))를
# 실행함.
#
# camera_publisher는 일부러 여기서 실행하지 않음 - 실제 배포에서는 카메라가
# 별도 머신(이 개발 PC)에 남아있고, `ros2 run person_locator camera_publisher`로
# 독립적으로 실행함.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# `colcon build`가 models/*.pt를 share/person_locator/models로 복사해두므로
# (setup.py의 data_files 참고), 여기서 그 설치된 경로를 기본값으로 잡는다.
# 다른 가중치를 쓰려면 launch argument로 override하면 됨(파일명만 주면
# Ultralytics가 캐시에서 자동 다운로드도 시도함).
_DEFAULT_POSE_MODEL = os.path.join(
    get_package_share_directory('person_locator'), 'models', 'yolo11n-pose.pt'
)
# 마찬가지로 config/*.yaml도 setup.py의 data_files가 share로 복사해두므로,
# launch를 어느 cwd에서 실행하든 항상 찾을 수 있게 절대경로로 잡는다
# (기존 상대경로 'config/person_homography.yaml'는 워크스페이스 루트 등에서
# 실행하면 못 찾아서 pose_locator_node가 바로 죽는 문제가 있었음)
_DEFAULT_HOMOGRAPHY_YAML = os.path.join(
    get_package_share_directory('person_locator'), 'config', 'person_homography.yaml'
)


def generate_launch_description():
    # CLI에서 override할 수 있도록 launch argument로 노출해둠, 예:
    #   ros2 launch person_locator person_locator.launch.py model_path:=/path/to/other-pose.pt
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value=_DEFAULT_POSE_MODEL,
        description='YOLO-pose weight 경로',
    )
    homography_yaml_arg = DeclareLaunchArgument(
        'homography_yaml_path', default_value=_DEFAULT_HOMOGRAPHY_YAML,
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

    return LaunchDescription([
        model_path_arg,
        homography_yaml_arg,
        pose_locator,
    ])
