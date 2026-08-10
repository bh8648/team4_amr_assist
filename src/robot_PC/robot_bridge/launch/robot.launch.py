import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


HMI_PORTS = {'robot5': 8005, 'robot11': 8011}


def _launch_nodes(context):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    if robot_id not in HMI_PORTS:
        raise ValueError('robot_id는 robot5 또는 robot11이어야 합니다.')
    configured_port = LaunchConfiguration('web_port').perform(context).strip()
    web_port = int(configured_port) if configured_port else HMI_PORTS[robot_id]

    nodes = [
        Node(
            package='robot_bridge', executable='robot_bridge_node',
            namespace=robot_id, name='robot_bridge_node', output='screen',
            parameters=[{'robot_id': robot_id}],
        ),
        Node(
            package='robot_bridge', executable='worker_tracking_bridge_node',
            namespace=robot_id, name='worker_tracking_bridge_node', output='screen',
            parameters=[{'robot_id': robot_id}],
        ),
        Node(
            package='robot_hmi_backend', executable='hmi_backend_node',
            namespace=robot_id, name='robot_hmi_backend_node', output='screen',
            parameters=[{'robot_id': robot_id, 'web_port': web_port}],
        ),
    ]

    enable_person_tracking = LaunchConfiguration('enable_person_tracking').perform(context).strip().lower()
    if enable_person_tracking not in ('true', 'false'):
        raise ValueError("enable_person_tracking은 'true' 또는 'false'여야 합니다.")
    if enable_person_tracking == 'true':
        # amr_person_tracking이 빌드/설치되지 않은 PC에서도 robot_bridge_node/hmi_backend_node는
        # 계속 뜨도록, 패키지 조회를 이 분기 안으로 한정한다 (미설치 시 여기서만 실패).
        amr_person_tracking_launch = os.path.join(
            get_package_share_directory('amr_person_tracking'), 'launch', 'amr_person_tracking.launch.py')
        target_person_pose_topic = f'/{robot_id}/target_person_pose'
        target_person_pose_raw_topic = f'/{robot_id}/target_person_pose_raw'
        # reid_tracking_node(amr_person_tracking, 무수정)가 원래 발행하는
        # target_person_pose를 raw 토픽으로 리다이렉트해, worker_tracking_bridge_node가
        # 최종 target_person_pose의 유일한 발행자가 되게 한다. GroupAction으로 감싸지
        # 않으면 이 리맵이 이후 실행되는 다른 노드에도 적용되어 버린다.
        nodes.append(GroupAction(actions=[
            SetRemap(src=target_person_pose_topic, dst=target_person_pose_raw_topic),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(amr_person_tracking_launch),
                launch_arguments={
                    'namespace': robot_id,
                    'publish_debug_image': 'false',
                }.items(),
            ),
        ]))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            description='이 PC가 제어할 로봇 ID: robot5 또는 robot11',
        ),
        DeclareLaunchArgument(
            'web_port', default_value='',
            description='비어 있으면 robot5=8005, robot11=8011을 사용',
        ),
        DeclareLaunchArgument(
            'enable_person_tracking', default_value='true',
            description="amr_person_tracking 추적 파이프라인 기동 여부. 'false'면 "
                        'robot_bridge_node/worker_tracking_bridge_node/hmi_backend_node만 뜬다 '
                        '(amr_person_tracking 미설치 PC에서도 브릿지/HMI 검증 가능).',
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
