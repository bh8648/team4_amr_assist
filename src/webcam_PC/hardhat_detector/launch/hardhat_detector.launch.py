# hardhat_detector.launch.py
#
# hardhat_detector_node 하나만 띄움.
#
# 기본 모델은 datasets/detect_warn(cart/helmet/truck, Roboflow 라벨링)으로
# yolov8n(COCO 사전학습)을 튜닝한 결과
# (runs/detect_warn_yolov8n_tuned/weights/best.pt를 models/로 복사해둔 것).
# 재학습하면 그 best.pt를 다시 models/로 복사하고 필요시 이 기본값도 갱신.

import os  # 경로 조합용

from ament_index_python.packages import get_package_share_directory  # 설치된 share 디렉터리 경로 조회
from launch import LaunchDescription  # 최상위 launch 설명 컨테이너
from launch.actions import DeclareLaunchArgument  # CLI에서 override 가능한 launch argument 선언
from launch.substitutions import LaunchConfiguration  # 선언한 argument 값을 참조하는 substitution
from launch_ros.actions import Node  # ROS2 노드 실행 액션

# colcon build가 models/*.pt를 share/hardhat_detector/models로 복사해두므로
# 그 설치된 경로를 기본 모델 경로로 사용
_DEFAULT_MODEL = os.path.join(
    get_package_share_directory('hardhat_detector'), 'models', 'detect_warn_yolov8n_tuned_best.pt'
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
        package='hardhat_detector',  # 실행할 패키지
        executable='hardhat_detector_node',  # setup.py entry_point로 등록된 실행 파일명
        name='hardhat_detector_node',  # ROS2 노드 이름
        output='screen',  # 로그를 터미널에 바로 출력
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),  # 위에서 선언한 argument를 노드 파라미터로 전달
            'positive_class_name': LaunchConfiguration('positive_class_name'),
        }],
    )

    return LaunchDescription([
        model_path_arg,  # launch argument 등록
        positive_class_name_arg,
        hardhat_detector,  # 실제 실행할 노드
    ])
