import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState

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
