# hardhat_detector.launch.py
#
# hardhat_detector_node 하나만 띄움.
#
# 기본 모델은 datasets/detect_warn(cart/helmet/truck, Roboflow 라벨링)으로
# yolo11n_2026-08-05.pt(COCO 사전학습)를 파인튜닝한 결과
# (runs/detect_warn_yolo11n/weights/best.pt를 models/로 복사해둔 것).
# 재학습하면 그 best.pt를 다시 models/로 복사하고 필요시 이 기본값도 갱신.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULT_MODEL = os.path.join(
    get_package_share_directory('hardhat_detector'), 'models', 'detect_warn_yolo11n_best.pt'
)


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value=_DEFAULT_MODEL,
        description='하이바 판별 모델(.pt) 경로 - classify/detect 둘 다 지원 (classify_hardhat 참고)',
    )
    positive_class_name_arg = DeclareLaunchArgument(
        'positive_class_name', default_value='helmet',
        description='모델 클래스 이름 중 "하이바 착용"에 해당하는 이름',
    )

    hardhat_detector = Node(
        package='hardhat_detector',
        executable='hardhat_detector_node',
        name='hardhat_detector_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'positive_class_name': LaunchConfiguration('positive_class_name'),
        }],
    )

    return LaunchDescription([
        model_path_arg,
        positive_class_name_arg,
        hardhat_detector,
    ])
