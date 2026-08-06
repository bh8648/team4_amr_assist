"""
amr_person_tracking.launch.py

AMR 기반 사람 추종 파이프라인(oakd_detector_node -> reid_tracking_node ->
predictive_avoidance_node, 그리고 초근접 구간을 보완하는 leg_detector_bridge_node)
4개 노드를 한 번에 기동하는 launch 파일.
파라미터 조정 로직은 없고, 노드 실행 및 토픽 이름 배선만 담당한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        # 로봇 네임스페이스 (예: robot5). nav2 launch와 맞춰서 넘겨줘야 함
        DeclareLaunchArgument('namespace', default_value='robot5'),
        # OAK-D RGB/Depth/CameraInfo (모두 compressed, 대역폭 절약)
        DeclareLaunchArgument(
            'rgb_topic',
            default_value=PathJoinSubstitution(['/', LaunchConfiguration('namespace'), 'oakd/rgb/image_raw/compressed']),
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value=PathJoinSubstitution(['/', LaunchConfiguration('namespace'), 'oakd/stereo/image_raw/compressedDepth']),
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value=PathJoinSubstitution(['/', LaunchConfiguration('namespace'), 'oakd/stereo/camera_info']),
        ),
        # 웹캠 로컬라이제이션 노드가 발행하는 원거리 검출 스트림 (외부 패키지, 이 워크스페이스 밖)
        DeclareLaunchArgument('webcam_detections_topic', default_value='/vision/webcam/detections_3d'),
        # 라이다 다리검출 패키지(예: ros2_leg_detector)가 발행하는 원본 토픽. 메시지 타입은
        # leg_detector_bridge_node 쪽에서 실제 설치 후 확인 필요
        DeclareLaunchArgument(
            'people_tracked_topic',
            default_value=PathJoinSubstitution(['/', LaunchConfiguration('namespace'), 'people_tracked']),
        ),
        # 예측 회피 노드가 접근 대상 반영 방식을 고를 파라미터: pointcloud | costmap_params | both
        DeclareLaunchArgument('avoidance_mode', default_value='pointcloud'),
    ]

    namespace = LaunchConfiguration('namespace')
    detections_topic = PathJoinSubstitution(['/', namespace, 'vision/detections_3d'])
    leg_detections_topic = PathJoinSubstitution(['/', namespace, 'vision/leg_detections_3d'])
    tracked_topic = PathJoinSubstitution(['/', namespace, 'vision/tracked_detections_3d'])
    target_pose_topic = PathJoinSubstitution(['/', namespace, 'target_person_pose'])
    predicted_points_topic = PathJoinSubstitution(['/', namespace, 'vision/predicted_obstacle_points'])
    costmap_param_service = PathJoinSubstitution(['/', namespace, 'local_costmap/local_costmap/set_parameters'])

    oakd_detector = Node(
        package='amr_person_tracking',
        executable='oakd_detector_node',
        name='oakd_detector_node',
        output='screen',
        parameters=[{
            'namespace': namespace,
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'detections_topic': detections_topic,
        }],
    )

    leg_detector_bridge = Node(
        package='amr_person_tracking',
        executable='leg_detector_bridge_node',
        name='leg_detector_bridge_node',
        output='screen',
        parameters=[{
            'people_tracked_topic': LaunchConfiguration('people_tracked_topic'),
            'leg_detections_topic': leg_detections_topic,
        }],
    )

    reid_tracking = Node(
        package='amr_person_tracking',
        executable='reid_tracking_node',
        name='reid_tracking_node',
        output='screen',
        parameters=[{
            'oakd_detections_topic': detections_topic,
            'webcam_detections_topic': LaunchConfiguration('webcam_detections_topic'),
            'leg_detections_topic': leg_detections_topic,
            'tracked_detections_topic': tracked_topic,
            'target_pose_topic': target_pose_topic,
        }],
    )

    predictive_avoidance = Node(
        package='amr_person_tracking',
        executable='predictive_avoidance_node',
        name='predictive_avoidance_node',
        output='screen',
        parameters=[{
            'tracked_detections_topic': tracked_topic,
            'predicted_points_topic': predicted_points_topic,
            'local_costmap_param_service': costmap_param_service,
            'avoidance_mode': LaunchConfiguration('avoidance_mode'),
        }],
    )

    return LaunchDescription(args + [oakd_detector, leg_detector_bridge, reid_tracking, predictive_avoidance])
