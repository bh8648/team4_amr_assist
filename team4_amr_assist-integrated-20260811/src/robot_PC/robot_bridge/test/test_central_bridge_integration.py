"""Central Task Manager와 robot bridge 사이의 실제 ROS 토픽 통합 테스트."""

import time

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
def test_tracking_pose_crosses_dds_and_changes_central_state(robot_id):
    """TaskState와 target pose가 실제 토픽을 왕복해 FOLLOWING으로 전환되는지 확인한다."""
    rclpy.init(args=['--ros-args', '-p', f'robot_id:={robot_id}'])
    manager = TaskManagerNode()
    bridge = RobotBridgeNode()
    probe = Node(f'{robot_id}_integration_probe')
    executor = SingleThreadedExecutor()
    for node in (manager, bridge, probe):
        executor.add_node(node)

    try:
        # 액션 서버 없이 토픽 계약만 검증하므로 최초 접근이 끝난 상태를 중앙에서 발행한다.
        task = ManagedTask(
            task_id=f'TEST_{robot_id}', robot_id=robot_id, state='ASSIGNED',
            goal_type='TO_WORKER', goal_completed=True)
        manager.tasks[robot_id] = task
        pose_pub = probe.create_publisher(PoseStamped, f'/{robot_id}/target_person_pose', 10)

        # DDS discovery가 끝난 후 상태를 발행해야 volatile TaskState를 놓치지 않는다.
        for _ in range(5):
            executor.spin_once(timeout_sec=0.05)
        manager.publish_state(task, '작업자 위치 도착, 작업자 감지 대기')
        assert _spin_until(
            executor,
            lambda: bridge.current_task_state == 'ASSIGNED'
            and bridge.worker_tracking_enabled,
        )

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y = 1.5, -0.5
        pose.pose.orientation.w = 1.0
        pose_pub.publish(pose)

        # bridge가 WORKER_DETECTED를 발행하고 manager가 FOLLOWING을 회신하는 전체 왕복이다.
        assert _spin_until(executor, lambda: task.state == 'FOLLOWING')
        assert _spin_until(executor, lambda: bridge.current_task_state == 'FOLLOWING')
        assert bridge.current_task_id == task.task_id
    finally:
        for node in (probe, bridge, manager):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
