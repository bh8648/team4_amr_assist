"""
amr_person_tracking.launch.py

AMR 기반 사람 추종 파이프라인(oakd_detector_node -> reid_tracking_node ->
predictive_avoidance_node, 초근접 구간을 보완하는 leg_detector_bridge_node,
그리고 오버레이를 눈으로 확인하는 debug_viewer_node)을 한 번에 기동하는 launch 파일.
파라미터 조정 로직은 없고, 노드 실행 및 토픽 이름 배선만 담당한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
            default_value=PathJoinSubstitution(
                ['/', LaunchConfiguration('namespace'), 'oakd/stereo/image_raw/compressedDepth']),
        ),
        # depth가 RGB에 정렬돼 나오고 rgb/stereo의 CameraInfo가 동일하므로 rgb 쪽을 쓴다
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value=PathJoinSubstitution(['/', LaunchConfiguration('namespace'), 'oakd/rgb/camera_info']),
        ),
        # 근접 안전모드 IR 교차확인용 (Create3). 로봇이 발행하지 않으면 노드가 거리 조건만으로 판정한다
        DeclareLaunchArgument(
            'ir_intensity_topic',
            default_value=PathJoinSubstitution(['/', LaunchConfiguration('namespace'), 'ir_intensity']),
        ),
        # 동기 시간차 동안의 로봇 이동량을 검출 covariance에 반영하기 위한 속도 입력
        DeclareLaunchArgument(
            'odom_topic',
            default_value=PathJoinSubstitution(['/', LaunchConfiguration('namespace'), 'odom']),
        ),
        # 검출 노드 디버그 오버레이 발행 여부. true면 debug_viewer_node가 cv2 창을 자동으로 띄운다.
        # 실증(눈으로 검출 확인) 단계라 기본값을 true로 뒀다 - 운영 시엔 false로 낮출 것
        DeclareLaunchArgument('publish_debug_image', default_value='true'),
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
    proximity_alert_topic = PathJoinSubstitution(['/', namespace, 'vision/proximity_alert'])
    diagnostics_topic = PathJoinSubstitution(['/', namespace, 'diagnostics'])
    debug_image_topic = PathJoinSubstitution(['/', namespace, 'vision/oakd_detector/debug/compressed'])

    # tf2의 TransformListener는 노드 네임스페이스와 무관하게 절대경로 /tf, /tf_static을 구독한다.
    # 터틀봇4는 tf를 네임스페이스 아래(/robot5/tf)로 발행하므로 remap이 없으면 tf 버퍼가 비어
    # "base_link ... does not exist"로 모든 좌표 변환이 실패한다. (bag 재생으로 확인함)
    tf_remappings = [
        ('/tf', PathJoinSubstitution(['/', namespace, 'tf'])),
        ('/tf_static', PathJoinSubstitution(['/', namespace, 'tf_static'])),
    ]

    oakd_detector = Node(
        package='amr_person_tracking',
        executable='oakd_detector_node',
        name='oakd_detector_node',
        output='screen',
        remappings=tf_remappings,
        parameters=[{
            'namespace': namespace,
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'detections_topic': detections_topic,
            'ir_intensity_topic': LaunchConfiguration('ir_intensity_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            # 아래 3개는 총괄 매니저 노드가 구독할 출력 채널
            'proximity_alert_topic': proximity_alert_topic,
            'diagnostics_topic': diagnostics_topic,
            'debug_image_topic': debug_image_topic,
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
        }],
    )

    # publish_debug_image가 true일 때만 cv2 창을 자동으로 띄운다. 창을 보려면 X11/Wayland
    # 디스플레이가 있는 세션에서 launch를 실행해야 한다 (SSH라면 -X/-Y 필요).
    debug_viewer = Node(
        package='amr_person_tracking',
        executable='debug_viewer_node',
        name='oakd_detector_debug_viewer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_debug_image')),
        parameters=[{
            'image_topic': debug_image_topic,
            'window_name': 'oakd_detector debug',
        }],
    )

    leg_detector_bridge = Node(
        package='amr_person_tracking',
        executable='leg_detector_bridge_node',
        name='leg_detector_bridge_node',
        output='screen',
        remappings=tf_remappings,
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

    return LaunchDescription(
        args + [oakd_detector, debug_viewer, leg_detector_bridge, reid_tracking, predictive_avoidance])
