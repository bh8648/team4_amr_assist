import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from unittest.mock import Mock

from std_msgs.msg import Bool

from robot_bridge.robot11_bridge_node import Robot11BridgeNode
from robot_status.msg import TaskState


def _amcl_msg(x, y, yaw_w=1.0, yaw_z=0.0):
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = yaw_z
    msg.pose.pose.orientation.w = yaw_w
    return msg


def test_build_status_message_none_before_data_received():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        assert node.build_status_message() is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_build_status_message_after_pose_and_battery():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.amcl_pose_callback(_amcl_msg(1.5, -2.0))
        battery_msg = BatteryState()
        battery_msg.percentage = 0.75
        node.battery_callback(battery_msg)

        status = node.build_status_message()
        assert status is not None
        assert status.robot_id == 'robot11'
        assert status.x == 1.5
        assert status.y == -2.0
        assert status.battery == 75.0
        assert status.current_task_id == ''
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pause_true_cancels_active_goal():
    rclpy.init()
    node = Robot11BridgeNode()
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
    node = Robot11BridgeNode()
    try:
        node.pause_callback(Bool(data=True))  # 예외 없이 통과해야 함
        assert node.nav_goal_handle is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pause_false_does_not_touch_goal():
    rclpy.init()
    node = Robot11BridgeNode()
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
    node = Robot11BridgeNode()
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
    node = Robot11BridgeNode()
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
    node = Robot11BridgeNode()
    try:
        node.dock_client = Mock()
        node.dock_client.wait_for_server.return_value = False

        node.dock_callback(Bool(data=True))

        node.dock_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _valid_person_pose(x=1.0, y=2.0):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    return msg


def _invalid_person_pose():
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.orientation.w = 0.0  # orientation 전부 0.0 = 무효
    return msg


def _no_target_person_pose():
    # reid_tracking_node가 추적 대상이 없을 때 보내는 실제 메시지:
    # position=(0,0,0), orientation은 IDL 기본값 (0,0,0,1) = 유효한 항등 쿼터니언
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    return msg


def test_task_state_callback_filters_by_robot_id():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        other_robot = TaskState()
        other_robot.robot_id = 'robot5'
        other_robot.state = 'FOLLOWING'
        node.task_state_callback(other_robot)
        assert node.current_task_state == ''

        this_robot = TaskState()
        this_robot.robot_id = 'robot11'
        this_robot.state = 'FOLLOWING'
        node.task_state_callback(this_robot)
        assert node.current_task_state == 'FOLLOWING'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_person_pose_ignored_when_not_following():
    rclpy.init()
    node = Robot11BridgeNode()
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
    node = Robot11BridgeNode()
    try:
        node.nav_client = Mock()
        node.current_task_state = 'FOLLOWING'

        node.target_person_pose_callback(_invalid_person_pose())

        node.nav_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_person_pose_ignored_when_position_is_origin():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.nav_client = Mock()
        node.nav_client.server_is_ready.return_value = True
        node.current_task_state = 'FOLLOWING'

        # 추적 대상 없음: position=(0,0,0), orientation=(0,0,0,1)
        msg = _no_target_person_pose()
        assert msg.pose.orientation.w == 1.0  # 항등 쿼터니언이라 quaternion 검사만으로는 못 거름
        node.target_person_pose_callback(msg)

        node.nav_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_person_pose_sends_goal_when_following_and_valid():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.nav_client = Mock()
        node.nav_client.server_is_ready.return_value = True
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
    node = Robot11BridgeNode()
    try:
        node.nav_client = Mock()
        node.nav_client.server_is_ready.return_value = True
        node.current_task_state = 'FOLLOWING'
        previous_handle = Mock()
        node.nav_goal_handle = previous_handle

        node.target_person_pose_callback(_valid_person_pose())

        previous_handle.cancel_goal_async.assert_called_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_person_pose_ignored_when_frame_id_is_not_map():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.nav_client = Mock()
        node.nav_client.server_is_ready.return_value = True
        node.current_task_state = 'FOLLOWING'

        msg = _valid_person_pose()
        msg.header.frame_id = 'base_link'
        node.target_person_pose_callback(msg)

        node.nav_client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_follow_goal_uses_non_blocking_server_check():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.nav_client = Mock()
        node.nav_client.server_is_ready.return_value = False
        node.current_task_state = 'FOLLOWING'

        node.target_person_pose_callback(_valid_person_pose())

        node.nav_client.send_goal_async.assert_not_called()
        # 고빈도 콜백이므로 블로킹 대기를 쓰면 안 된다
        node.nav_client.wait_for_server.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_follow_goal_does_not_mutate_received_message():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.nav_client = Mock()
        node.nav_client.server_is_ready.return_value = True
        node.current_task_state = 'FOLLOWING'

        msg = _valid_person_pose()
        original_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)

        node.target_person_pose_callback(msg)

        sent_goal = node.nav_client.send_goal_async.call_args[0][0]
        assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == original_stamp
        assert sent_goal.pose is not msg
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pause_during_in_flight_goal_cancels_late_accepted_goal():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.nav_client = Mock()
        node.nav_client.server_is_ready.return_value = True
        node.current_task_state = 'FOLLOWING'

        node.target_person_pose_callback(_valid_person_pose())
        response_callback = node.nav_client.send_goal_async.return_value.add_done_callback.call_args[0][0]

        # goal 응답이 오기 전에 pause 도착 (nav_goal_handle은 아직 None)
        assert node.nav_goal_handle is None
        node.pause_callback(Bool(data=True))

        # 뒤늦게 goal이 수락됨 → 취소되어야 하고 handle로 등록되면 안 된다
        late_handle = Mock()
        late_handle.accepted = True
        future = Mock()
        future.result.return_value = late_handle
        response_callback(future)

        late_handle.cancel_goal_async.assert_called_once()
        assert node.nav_goal_handle is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_follow_goal_handle_cleared_when_navigation_result_arrives():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        node.nav_client = Mock()
        node.nav_client.server_is_ready.return_value = True
        node.current_task_state = 'FOLLOWING'

        node.target_person_pose_callback(_valid_person_pose())
        response_callback = node.nav_client.send_goal_async.return_value.add_done_callback.call_args[0][0]

        goal_handle = Mock()
        goal_handle.accepted = True
        future = Mock()
        future.result.return_value = goal_handle
        response_callback(future)
        assert node.nav_goal_handle is goal_handle

        result_callback = goal_handle.get_result_async.return_value.add_done_callback.call_args[0][0]
        result_callback(Mock())

        assert node.nav_goal_handle is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_battery_subscription_uses_best_effort_qos():
    rclpy.init()
    node = Robot11BridgeNode()
    try:
        assert node.battery_sub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        # amcl_pose는 Nav2 AMCL 기본값(RELIABLE) 그대로 둔다
        assert node.amcl_sub.qos_profile.reliability == ReliabilityPolicy.RELIABLE
    finally:
        node.destroy_node()
        rclpy.shutdown()
