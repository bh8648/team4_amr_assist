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
        # 라이다 다리검출 결과 RViz 마커 발행 여부 (leg_detector_bridge_node). 카메라 이미지가
        # 없는 입력이라 debug_viewer_node 대신 RViz Marker로 확인한다
        DeclareLaunchArgument('publish_markers', default_value='true'),
        # 웹캠 로컬라이제이션 노드가 발행하는 원거리 검출 스트림 (외부 패키지, 이 워크스페이스 밖)
        DeclareLaunchArgument('webcam_detections_topic', default_value='/vision/webcam/detections_3d'),
        # LiDAR 드라이버가 발행하는 원본 스캔. leg_detector_bridge_node가 이 토픽을 직접 구독해
        # 다리쌍을 검출한다 (ros2_leg_detector 등 외부 패키지 불필요 - 노드 docstring 참고)
        DeclareLaunchArgument(
            'scan_topic',
            default_value=PathJoinSubstitution(['/', LaunchConfiguration('namespace'), 'scan']),
        ),
        # 예측 회피 노드가 접근 대상 반영 방식을 고를 파라미터: pointcloud | costmap_params | both
        DeclareLaunchArgument('avoidance_mode', default_value='pointcloud'),
        # 전역 좌표계. 각 노드가 tf2 조회/출력 frame_id에 사용한다. AMCL/nav2가 없는 로스백
        # 재생 테스트에서는 map 프레임이 존재하지 않으므로 odom으로 넘겨야 tf lookup이 성공한다.
        DeclareLaunchArgument('map_frame', default_value='map'),
        # 로스백 재생 시 odom/tf가 센서 타임스탬프보다 뒤처지는 구간에서 정확한 시각 대신
        # 최신 tf로 폴백할지 여부. 실물 로봇에선 false 유지, 로스백 테스트에선 true 권장.
        DeclareLaunchArgument('tf_allow_latest_fallback', default_value='false'),
        # debug_viewer_node 창 크기 (원본 프레임이 704x704라 기본값 그대로면 작게 보인다)
        DeclareLaunchArgument('debug_window_width', default_value='960'),
        DeclareLaunchArgument('debug_window_height', default_value='960'),
        # 다리검출 원형적합(곡률) 필터. before/after 증거영상 비교를 위해 끌 수 있게 뒀다 -
        # false면 옛 동작(폭만 검사, 벽 모서리 오검출 재현)으로 돌아간다.
        DeclareLaunchArgument('leg_circularity_filter_enabled', default_value='true'),
        DeclareLaunchArgument('leg_circle_fit_radius_max', default_value='0.20'),
        DeclareLaunchArgument('leg_circle_fit_rms_max', default_value='0.01'),
        # 원형적합만으론 못 거르는 책상/의자 다리(진짜 원통형) 대응 - "계속 같은 자리"라는
        # 시간적 근거로 추가 제외. before/after 비교를 위해 끌 수 있게 뒀다. 개별 다리 후보마다
        # 등속도 칼만필터(LegKalmanTracker)로 속도를 추정해, 누적 나이가 confirm_duration_sec
        # 이상이고 속도가 stationary_speed_threshold 이하면 배경으로 확정한다 - 관측 공백
        # (occlusion)이 있어도 칼만필터 상태가 리셋되지 않아, 사람이 물체 앞을 반복해서 오가도
        # 결국 확정된다 (predictive_utils.LegKalmanTracker 문서 참고).
        DeclareLaunchArgument('background_filter_enabled', default_value='true'),
        DeclareLaunchArgument('background_confirm_duration_sec', default_value='3.0'),
        DeclareLaunchArgument('background_stationary_speed_threshold', default_value='0.01'),
        DeclareLaunchArgument('background_leg_match_gate', default_value='0.10'),
        DeclareLaunchArgument('background_leg_kf_timeout', default_value='5.0'),
        DeclareLaunchArgument('background_kf_process_noise', default_value='0.01'),
        DeclareLaunchArgument('background_kf_measurement_noise', default_value='0.0025'),
        DeclareLaunchArgument('background_cell_size', default_value='0.05'),
        DeclareLaunchArgument('background_exclusion_radius', default_value='0.10'),
        # RGB/Depth 교차검증용 증거영상 녹화 시에만 켠다 (depth_view_republisher_node).
        DeclareLaunchArgument('publish_depth_view', default_value='false'),
        # reid_tracking_node end-to-end 검증 시에만 켠다 (mock_webcam_publisher_node) - 실제
        # 웹캠 로컬라이제이션 스트림이 없는 이 워크스페이스에서, 라이다 다리검출 출력에 노이즈를
        # 더해 "독립 출처"처럼 재발행해 웹캠->OAK-D->라이다 융합 경로를 실제로 돌려본다.
        DeclareLaunchArgument('publish_mock_webcam', default_value='false'),
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
    leg_marker_topic = PathJoinSubstitution(['/', namespace, 'vision/leg_detections/markers'])
    depth_view_topic = PathJoinSubstitution(['/', namespace, 'oakd/stereo/depth_view/compressed'])

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
            'map_frame': LaunchConfiguration('map_frame'),
            'tf_allow_latest_fallback': LaunchConfiguration('tf_allow_latest_fallback'),
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
            'window_width': LaunchConfiguration('debug_window_width'),
            'window_height': LaunchConfiguration('debug_window_height'),
        }],
    )

    # RGB/Depth 교차검증 증거영상 녹화용. 평소엔 필요 없어 기본 꺼짐 - RViz Image 디스플레이가
    # 특수 처리 없이 볼 수 있게 depth를 컬러맵 입힌 평범한 jpeg로 재발행한다.
    depth_view_republisher = Node(
        package='amr_person_tracking',
        executable='depth_view_republisher_node',
        name='depth_view_republisher_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_depth_view')),
        parameters=[{
            'depth_topic': LaunchConfiguration('depth_topic'),
            'depth_view_topic': depth_view_topic,
        }],
    )

    leg_detector_bridge = Node(
        package='amr_person_tracking',
        executable='leg_detector_bridge_node',
        name='leg_detector_bridge_node',
        output='screen',
        remappings=tf_remappings,
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
            'leg_detections_topic': leg_detections_topic,
            'publish_markers': LaunchConfiguration('publish_markers'),
            'marker_topic': leg_marker_topic,
            'map_frame': LaunchConfiguration('map_frame'),
            'tf_allow_latest_fallback': LaunchConfiguration('tf_allow_latest_fallback'),
            'leg_circularity_filter_enabled': LaunchConfiguration('leg_circularity_filter_enabled'),
            'leg_circle_fit_radius_max': LaunchConfiguration('leg_circle_fit_radius_max'),
            'leg_circle_fit_rms_max': LaunchConfiguration('leg_circle_fit_rms_max'),
            'background_filter_enabled': LaunchConfiguration('background_filter_enabled'),
            'background_confirm_duration_sec': LaunchConfiguration('background_confirm_duration_sec'),
            'background_stationary_speed_threshold': LaunchConfiguration('background_stationary_speed_threshold'),
            'background_leg_match_gate': LaunchConfiguration('background_leg_match_gate'),
            'background_leg_kf_timeout': LaunchConfiguration('background_leg_kf_timeout'),
            'background_kf_process_noise': LaunchConfiguration('background_kf_process_noise'),
            'background_kf_measurement_noise': LaunchConfiguration('background_kf_measurement_noise'),
            'background_cell_size': LaunchConfiguration('background_cell_size'),
            'background_exclusion_radius': LaunchConfiguration('background_exclusion_radius'),
        }],
    )

    # reid_tracking_node end-to-end 검증용 - 평소엔 필요 없어 기본 꺼짐. 실제 웹캠 로컬라이제이션
    # 스트림이 없어, 라이다 다리검출 출력에 노이즈+지연을 더해 "독립 출처"처럼 재발행한다.
    mock_webcam_publisher = Node(
        package='amr_person_tracking',
        executable='mock_webcam_publisher_node',
        name='mock_webcam_publisher_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_mock_webcam')),
        parameters=[{
            'leg_detections_topic': leg_detections_topic,
            'webcam_detections_topic': LaunchConfiguration('webcam_detections_topic'),
            'map_frame': LaunchConfiguration('map_frame'),
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
            'map_frame': LaunchConfiguration('map_frame'),
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
            'map_frame': LaunchConfiguration('map_frame'),
        }],
    )

    return LaunchDescription(
        args + [oakd_detector, debug_viewer, depth_view_republisher, leg_detector_bridge,
                mock_webcam_publisher, reid_tracking, predictive_avoidance])
