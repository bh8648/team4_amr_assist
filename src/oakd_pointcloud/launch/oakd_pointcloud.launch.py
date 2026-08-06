from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    args = [
        # 로봇 네임스페이스 (예: robot5). nav2 launch와 맞춰서 넘겨줘야 함
        DeclareLaunchArgument('namespace', default_value='robot5'),
        # depthai_ros_driver가 depth 이미지를 내는 실제 토픽 (raw, 비압축)
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value=PathJoinSubstitution(['/', LaunchConfiguration('namespace'), 'oakd', 'stereo', 'image_raw']),
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value=PathJoinSubstitution(['/', LaunchConfiguration('namespace'), 'oakd', 'stereo', 'camera_info']),
        ),
        # 여기서 만든 PointCloud2가 발행될 절대 토픽. nav2 costmap의 voxel_layer.pointcloud.topic과 일치해야 함
        DeclareLaunchArgument(
            'points_topic',
            default_value=PathJoinSubstitution(['/', LaunchConfiguration('namespace'), 'points']),
        ),
        # depth_image_topic을 raw로 안 받고 image_transport 플러그인으로 압축 해제해서 받음.
        # 로봇<->노트북 WiFi 대역폭 문제로 raw 대신 compressedDepth 사용 (image_rect 리맵은 base topic 그대로 두면
        # image_transport가 알아서 <base>/compressedDepth를 구독하고 내부에서 압축을 풀어줌)
        DeclareLaunchArgument('depth_transport', default_value='compressedDepth'),
    ]

    container = ComposableNodeContainer(
        name='oakd_pointcloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz',
                parameters=[{
                    'image_transport': LaunchConfiguration('depth_transport'),
                    # 구독 쪽도 best_effort로 맞춰서, 로봇 publisher가 best_effort로 바뀌었을 때
                    # reliable 구독자 때문에 매칭이 깨지지 않도록 함. namespace가 'robot5'가 아니면
                    # 아래 토픽 경로도 같이 바꿔줘야 함 (launch substitution을 파라미터 키로는 못 씀).
                    'qos_overrides./robot5/oakd/stereo/image_raw.subscription.reliability': 'best_effort',
                    'qos_overrides./robot5/oakd/stereo/image_raw/compressedDepth.subscription.reliability': 'best_effort',
                    'qos_overrides./robot5/oakd/stereo/camera_info.subscription.reliability': 'best_effort',
                }],
                remappings=[
                    ('image_rect', LaunchConfiguration('depth_image_topic')),
                    ('camera_info', LaunchConfiguration('camera_info_topic')),
                    ('points', LaunchConfiguration('points_topic')),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription(args + [container])
