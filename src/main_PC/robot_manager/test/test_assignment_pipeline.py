from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import PointStamped
from robot_status.msg import RobotStatus

from robot_manager.robot_assignment_node import RobotAssignmentNode


def _call_position(x, y):
    goal = PointStamped()
    goal.header.frame_id = 'map'
    goal.point.x, goal.point.y = x, y
    return goal


def test_webcam_call_is_assigned_and_nearby_repeat_is_filtered():
    rclpy.init()
    node = RobotAssignmentNode()
    try:
        node.assignment_publisher = Mock()
        node.error_publisher = Mock()
        status = RobotStatus()
        status.robot_id, status.battery = 'robot11', 80.0
        status.x, status.y, status.yaw = -2.0, -2.0, 0.0
        node.status_callback('robot11', status)

        node.goal_callback(_call_position(-1.0, -1.0))
        assert node.assignment_publisher.publish.call_count == 1
        assignment = node.assignment_publisher.publish.call_args.args[0]
        assert assignment.assigned is True
        assert assignment.robot_id == 'robot11'
        assert node.assignment_timeout_sec == 30.0

        node.goal_callback(_call_position(-0.8, -1.1))
        assert node.assignment_publisher.publish.call_count == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
