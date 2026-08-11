"""Central Task Manager와 robot bridge 사이의 실제 ROS 토픽 통합 테스트."""

import time
from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from robot_bridge.robot_bridge_node import RobotBridgeNode
from robot_manager.task_manager_node import ManagedTask, TaskManagerNode


def _spin_until(executor, predicate, timeout_sec=3.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


@pytest.mark.parametrize('robot_id', ['robot5', 'robot11'])
def test_worker_arrival_starts_following_and_tracking_pose_crosses_dds(robot_id):
    """TO_WORKER 성공이 FOLLOWING을 만들고 이후 target pose가 추종으로 이어지는지 확인한다."""
    rclpy.init(args=['--ros-args', '-p', f'robot_id:={robot_id}'])
    manager = TaskManagerNode()
    bridge = RobotBridgeNode()
    probe = Node(f'{robot_id}_integration_probe')
    executor = SingleThreadedExecutor()
    for node in (manager, bridge, probe):
        executor.add_node(node)

    try:
        # 액션 서버 없이 중앙의 TO_WORKER 성공 콜백부터 DDS 상태 전달까지 검증한다.
        task = ManagedTask(
            task_id=f'TEST_{robot_id}', robot_id=robot_id, state='ASSIGNED',
            goal_type='TO_WORKER')
        manager.tasks[robot_id] = task
        pose_pub = probe.create_publisher(PoseStamped, f'/{robot_id}/target_person_pose', 10)
        bridge.latest_x, bridge.latest_y, bridge.latest_yaw = 0.0, 0.0, 0.0
        bridge.send_follow_goal = Mock()

        # DDS discovery가 끝난 후 성공 상태를 발행해야 volatile TaskState를 놓치지 않는다.
        for _ in range(5):
            executor.spin_once(timeout_sec=0.05)
        manager.handle_navigation_result(robot_id, 'TO_WORKER', True, '')
        assert task.state == 'FOLLOWING'
        assert _spin_until(
            executor,
            lambda: bridge.current_task_state == 'FOLLOWING'
            and bridge.worker_tracking_enabled,
        )

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y = 1.5, -0.5
        pose.pose.orientation.w = 1.0
        pose_pub.publish(pose)

        # FOLLOWING 상태에서 첫 유효 pose가 즉시 안전거리 추종 goal로 이어진다.
        assert _spin_until(executor, lambda: bridge.send_follow_goal.called)
        assert bridge.current_task_id == task.task_id
    finally:
        for node in (probe, bridge, manager):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
