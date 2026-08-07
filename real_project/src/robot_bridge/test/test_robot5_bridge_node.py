import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from unittest.mock import Mock

from std_msgs.msg import Bool

from robot_bridge.robot5_bridge_node import Robot5BridgeNode


def _amcl_msg(x, y, yaw_w=1.0, yaw_z=0.0):
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = yaw_z
    msg.pose.pose.orientation.w = yaw_w
    return msg


def test_build_status_message_none_before_data_received():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        assert node.build_status_message() is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_build_status_message_after_pose_and_battery():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.amcl_pose_callback(_amcl_msg(1.5, -2.0))
        battery_msg = BatteryState()
        battery_msg.percentage = 0.75
        node.battery_callback(battery_msg)

        status = node.build_status_message()
        assert status is not None
        assert status.robot_id == 'robot5'
        assert status.x == 1.5
        assert status.y == -2.0
        assert status.battery == 75.0
        assert status.current_task_id == ''
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pause_true_cancels_active_goal():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        fake_goal_handle = Mock()
        node.nav_goal_handle = fake_goal_handle

        node.pause_callback(Bool(data=True))

        fake_goal_handle.cancel_goal_async.assert_called_once()
        assert node.nav_goal_handle is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pause_true_without_active_goal_does_nothing():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.pause_callback(Bool(data=True))  # 예외 없이 통과해야 함
        assert node.nav_goal_handle is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pause_false_does_not_touch_goal():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        fake_goal_handle = Mock()
        node.nav_goal_handle = fake_goal_handle

        node.pause_callback(Bool(data=False))

        fake_goal_handle.cancel_goal_async.assert_not_called()
        assert node.nav_goal_handle is fake_goal_handle
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dock_request_true_sends_dock_goal():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.dock_client = Mock()
        node.dock_client.wait_for_server.return_value = True
        node.undock_client = Mock()

        node.dock_callback(Bool(data=True))

        node.dock_client.send_goal_async.assert_called_once()
        node.undock_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dock_request_false_sends_undock_goal():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.dock_client = Mock()
        node.undock_client = Mock()
        node.undock_client.wait_for_server.return_value = True

        node.dock_callback(Bool(data=False))

        node.undock_client.send_goal_async.assert_called_once()
        node.dock_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dock_request_skips_when_action_server_not_ready():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.dock_client = Mock()
        node.dock_client.wait_for_server.return_value = False

        node.dock_callback(Bool(data=True))

        node.dock_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


from geometry_msgs.msg import PoseStamped

from robot_status.msg import TaskState


def _valid_person_pose(x=1.0, y=2.0):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    return msg


def _invalid_person_pose():
    msg = PoseStamped()
    msg.pose.orientation.w = 0.0  # orientation 전부 0.0 = 무효
    return msg


def test_task_state_callback_filters_by_robot_id():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        other_robot = TaskState()
        other_robot.robot_id = 'robot11'
        other_robot.state = 'FOLLOWING'
        node.task_state_callback(other_robot)
        assert node.current_task_state == ''

        this_robot = TaskState()
        this_robot.robot_id = 'robot5'
        this_robot.state = 'FOLLOWING'
        node.task_state_callback(this_robot)
        assert node.current_task_state == 'FOLLOWING'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_person_pose_ignored_when_not_following():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.nav_client = Mock()
        node.current_task_state = 'TRANSPORTING'

        node.target_person_pose_callback(_valid_person_pose())

        node.nav_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_person_pose_ignored_when_invalid_quaternion():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.nav_client = Mock()
        node.current_task_state = 'FOLLOWING'

        node.target_person_pose_callback(_invalid_person_pose())

        node.nav_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_person_pose_sends_goal_when_following_and_valid():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.nav_client = Mock()
        node.nav_client.wait_for_server.return_value = True
        node.current_task_state = 'FOLLOWING'

        node.target_person_pose_callback(_valid_person_pose(x=3.0, y=4.0))

        node.nav_client.send_goal_async.assert_called_once()
        sent_goal = node.nav_client.send_goal_async.call_args[0][0]
        assert sent_goal.pose.pose.position.x == 3.0
        assert sent_goal.pose.pose.position.y == 4.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_person_pose_cancels_previous_goal_before_resend():
    rclpy.init()
    node = Robot5BridgeNode()
    try:
        node.nav_client = Mock()
        node.nav_client.wait_for_server.return_value = True
        node.current_task_state = 'FOLLOWING'
        previous_handle = Mock()
        node.nav_goal_handle = previous_handle

        node.target_person_pose_callback(_valid_person_pose())

        previous_handle.cancel_goal_async.assert_called_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()
