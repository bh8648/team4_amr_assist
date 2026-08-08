#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Dict, Optional, Set, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from robot_status.msg import DeadlockPermission, NavigationResult, RobotAssignment, RobotError, RobotStatus, TaskCommand, TaskState


@dataclass
class ManagedTask:
    task_id: str
    robot_id: str
    state: str = 'DOCKED'
    previous_state: str = ''
    goal_type: str = ''
    target: Optional[Tuple[float, float, float]] = None
    goal_handle: object = None
    pause_reason: str = ''
    nav_generation: int = 0
    goal_pending: bool = False
    goal_completed: bool = False
    awaiting_dock_check: bool = False
    dock_check_started_at: object = None
    undock_requested: bool = False


class TaskManagerNode(Node):
    """AMR 배정 이후 Task 상태 전환과 Nav2 Goal 실행을 중앙에서 관리한다."""

    VALID_ROBOTS = {'robot5', 'robot11'}
    ACTIVE_STATES = {'ASSIGNED', 'FOLLOWING', 'TRANSPORTING', 'RETURNING'}
    COMMAND_REJECTION_CODES = (
        'UNKNOWN_ROBOT_ID', 'TASK_NOT_FOUND', 'STALE_TASK_COMMAND',
        'INVALID_DESTINATION', 'ROBOT_ALREADY_HAS_TASK', 'INVALID_TRANSITION_',
    )
    DOCK_WAIT_TIMEOUT_SEC = 10.0
    DOCK_STATUS_STALE_SEC = 5.0

    def __init__(self):
        super().__init__('task_manager_node')
        self.declare_parameter('robot5_dock_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('robot11_dock_pose', [-2.3, -3.6, -math.pi / 2])
        self.tasks: Dict[str, ManagedTask] = {}
        self.idle_paused: Set[str] = set()
        self.assignment_sub = self.create_subscription(RobotAssignment, '/robot_assignment', self.assignment_callback, 10)
        self.command_sub = self.create_subscription(TaskCommand, '/task/command', self.command_callback, 10)
        self.navigation_result_sub = self.create_subscription(NavigationResult, '/navigation/result', self.navigation_result_callback, 10)
        self.deadlock_sub = self.create_subscription(DeadlockPermission, '/deadlock/permission', self.deadlock_callback, 10)
        self.error_sub = self.create_subscription(RobotError, '/robot_error', self.error_callback, 10)
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.status_subscriptions = [self.create_subscription(RobotStatus, f'/{robot_id}/robot_status', self.robot_status_callback, status_qos) for robot_id in self.VALID_ROBOTS]
        self.robot_dock_states: Dict[str, Tuple[bool, bool, object]] = {}
        self.state_pub = self.create_publisher(TaskState, '/task/state', 10)
        self.error_pub = self.create_publisher(RobotError, '/robot_error', 10)
        self.stop_publishers = {robot_id: self.create_publisher(Bool, f'/{robot_id}/pause/request', 10) for robot_id in self.VALID_ROBOTS}
        self.dock_publishers = {robot_id: self.create_publisher(Bool, f'/{robot_id}/dock/request', 10) for robot_id in self.VALID_ROBOTS}
        self.nav_clients = {robot_id: ActionClient(self, NavigateToPose, f'/{robot_id}/navigate_to_pose') for robot_id in self.VALID_ROBOTS}
        self.nav_retry_timer = self.create_timer(1.0, self.retry_navigation_goals)
        self.get_logger().info('Task Manager 시작: robot5, robot11')

    @staticmethod
    def normalize_robot_id(robot_id: str) -> str:
        value = str(robot_id).strip()
        return value if value.startswith('robot') else f'robot{value}'

    def assignment_callback(self, msg: RobotAssignment) -> None:
        if not msg.assigned:
            return
        robot_id = self.normalize_robot_id(msg.robot_id)
        if robot_id not in self.VALID_ROBOTS:
            self.publish_error(robot_id, '', 'UNKNOWN_ROBOT_ID')
            return
        if robot_id in self.tasks and self.tasks[robot_id].state != 'DOCKED':
            self.publish_error(robot_id, self.tasks[robot_id].task_id, 'ROBOT_ALREADY_HAS_TASK')
            return
        task_id = f'TASK_{msg.assigned_at.sec}_{msg.assigned_at.nanosec}'
        task = ManagedTask(task_id=task_id, robot_id=robot_id, state='ASSIGNED', goal_type='TO_WORKER', target=(float(msg.target_x), float(msg.target_y), 0.0))
        self.tasks[robot_id] = task
        self.publish_state(task, 'AMR 배정 완료')
        self.start_task_navigation(task)

    def get_fresh_dock_state(self, robot_id: str) -> Tuple[bool, bool]:
        """캐시된 도킹 상태가 오래됐으면(브릿지 응답 없음 등) '모름'으로 취급한다."""
        cached = self.robot_dock_states.get(robot_id)
        if cached is None:
            return False, False
        is_docked, dock_status_known, received_at = cached
        elapsed_sec = (self.get_clock().now() - received_at).nanoseconds / 1e9
        if elapsed_sec > self.DOCK_STATUS_STALE_SEC:
            return False, False
        return is_docked, dock_status_known

    def start_task_navigation(self, task: ManagedTask) -> None:
        """배정 직후 주행 시작 전 도킹 상태를 확인한다. dock_status_known이 False면
        (즉 아직 실제 도킹 여부를 모르면) 절대 먼저 주행시키지 않는다."""
        robot_id = task.robot_id
        is_docked, dock_status_known = self.get_fresh_dock_state(robot_id)
        if not dock_status_known:
            task.awaiting_dock_check = True
            task.dock_check_started_at = self.get_clock().now()
            self.get_logger().warn(f'{robot_id} 도킹 상태 미확인 — DockStatus 대기 중')
            return
        if not is_docked:
            self.send_navigation_goal(task)
            return
        self.request_undock_once(task)

    def request_undock_once(self, task: ManagedTask) -> None:
        if task.undock_requested:
            return
        task.undock_requested = True
        task.dock_check_started_at = self.get_clock().now()
        self.dock_publishers[task.robot_id].publish(Bool(data=False))
        self.get_logger().info(f'{task.robot_id} 언도킹 요청 발행, is_docked=False 대기 시작')

    def robot_status_callback(self, msg: RobotStatus) -> None:
        robot_id = self.normalize_robot_id(msg.robot_id)
        if robot_id not in self.VALID_ROBOTS:
            return
        is_docked, dock_status_known = bool(msg.is_docked), bool(msg.dock_status_known)
        self.robot_dock_states[robot_id] = (is_docked, dock_status_known, self.get_clock().now())
        if not dock_status_known:
            return
        task = self.tasks.get(robot_id)
        if task is None or task.state not in ('ASSIGNED', 'TRANSPORTING', 'RETURNING'):
            return
        if task.awaiting_dock_check:
            task.awaiting_dock_check = False
            if is_docked:
                self.request_undock_once(task)
            else:
                self.send_navigation_goal(task)
            return
        if task.undock_requested and not is_docked:
            task.undock_requested = False
            self.send_navigation_goal(task)

    def command_callback(self, msg: TaskCommand) -> None:
        robot_id = self.normalize_robot_id(msg.robot_id)
        task = self.tasks.get(robot_id)
        command = msg.command.strip().upper()
        if robot_id not in self.VALID_ROBOTS:
            self.publish_error(robot_id, msg.task_id, 'UNKNOWN_ROBOT_ID')
            return
        if command == 'PAUSE':
            self.pause_task(robot_id, 'HMI_PAUSE')
        elif command == 'RESUME':
            self.resume_task(robot_id)
        elif command in ('DOCK', 'UNDOCK'):
            self.dock_publishers[robot_id].publish(Bool(data=command == 'DOCK'))
        elif task is None:
            self.publish_error(robot_id, msg.task_id, 'TASK_NOT_FOUND')
        elif msg.task_id and msg.task_id != task.task_id:
            self.publish_error(robot_id, msg.task_id, 'STALE_TASK_COMMAND')
        elif command == 'START_TRANSPORT' and task.state == 'FOLLOWING':
            if not math.isfinite(msg.target_x) or not math.isfinite(msg.target_y):
                self.publish_error(robot_id, task.task_id, 'INVALID_DESTINATION')
                return
            task.goal_type, task.target = 'TO_DESTINATION', (float(msg.target_x), float(msg.target_y), float(msg.target_yaw))
            task.goal_completed = False
            self.transition(task, 'TRANSPORTING', '작업자가 배송 모드로 전환')
            self.send_navigation_goal(task, replace=True)
        elif command == 'CANCEL' and task.state in self.ACTIVE_STATES:
            if task.awaiting_dock_check or task.undock_requested:
                self.transition(task, 'DOCKED', '주행 시작 전 취소 — 도킹 상태 유지')
                return
            task.goal_type, task.target = 'TO_DOCK', tuple(float(value) for value in self.get_parameter(f'{robot_id}_dock_pose').value)
            task.goal_completed = False
            self.transition(task, 'RETURNING', '작업 취소 후 복귀')
            self.send_navigation_goal(task, replace=True)
        else:
            self.publish_error(robot_id, task.task_id, f'INVALID_TRANSITION_{task.state}_{command}')

    def transition(self, task: ManagedTask, new_state: str, detail: str) -> None:
        task.previous_state, task.state = task.state, new_state
        self.publish_state(task, detail)

    def pause_task(self, robot_id: str, detail: str) -> None:
        task = self.tasks.get(robot_id)
        if task is None:
            self.idle_paused.add(robot_id)
            self.stop_publishers[robot_id].publish(Bool(data=True))
            return
        if task.state == 'PAUSED':
            if detail == 'HMI_PAUSE':
                task.pause_reason = detail
                self.publish_state(task, '사용자 일시정지로 전환')
            return
        task.previous_state, task.state = task.state, 'PAUSED'
        task.pause_reason = detail
        self.invalidate_navigation_goal(task)
        self.stop_publishers[robot_id].publish(Bool(data=True))
        self.publish_state(task, detail)

    def resume_task(self, robot_id: str, allow_deadlock_release: bool = False) -> None:
        task = self.tasks.get(robot_id)
        if task is None:
            self.idle_paused.discard(robot_id)
            self.stop_publishers[robot_id].publish(Bool(data=False))
            return
        if task.state != 'PAUSED':
            return
        if task.pause_reason.startswith('DEADLOCK_WAIT:') and not allow_deadlock_release:
            self.get_logger().warn(f'{robot_id}은 교착 대기 중이므로 HMI 재개 요청을 거부합니다.')
            self.publish_state(task, '교착 해제 전 수동 재개 불가')
            return
        task.state, task.previous_state = task.previous_state or 'DOCKED', 'PAUSED'
        task.pause_reason = ''
        self.stop_publishers[robot_id].publish(Bool(data=False))
        self.publish_state(task, '정지 해제 후 이전 상태 복귀')
        if (task.target and not task.goal_completed and task.state in ('ASSIGNED', 'TRANSPORTING', 'RETURNING')
                and not task.awaiting_dock_check and not task.undock_requested):
            self.send_navigation_goal(task)

    def invalidate_navigation_goal(self, task: ManagedTask) -> None:
        task.nav_generation += 1
        task.goal_pending = False
        goal_handle, task.goal_handle = task.goal_handle, None
        if goal_handle is not None:
            goal_handle.cancel_goal_async()

    def send_navigation_goal(self, task: ManagedTask, replace: bool = False) -> None:
        if task.target is None:
            return
        if task.goal_handle is not None or task.goal_pending:
            if not replace:
                return
            self.invalidate_navigation_goal(task)
        client = self.nav_clients[task.robot_id]
        if not client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn(f'{task.robot_id} Nav2 Action 서버 대기 중: {task.goal_type}')
            return
        x, y, yaw = task.target
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        task.nav_generation += 1
        generation = task.nav_generation
        task.goal_pending = True
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda result, rid=task.robot_id, goal_type=task.goal_type, gen=generation: self.goal_response_callback(result, rid, goal_type, gen))

    def goal_response_callback(self, future, robot_id: str, goal_type: str, generation: int) -> None:
        task = self.tasks.get(robot_id)
        if task is None or generation != task.nav_generation:
            return
        task.goal_pending = False
        task.goal_handle = future.result()
        if not task.goal_handle.accepted:
            self.handle_navigation_result(robot_id, goal_type, False, 'NAV_GOAL_REJECTED')
            return
        result_future = task.goal_handle.get_result_async()
        result_future.add_done_callback(lambda result, rid=robot_id, kind=goal_type, gen=generation: self.action_result_callback(result, rid, kind, gen))

    def action_result_callback(self, future, robot_id: str, goal_type: str, generation: int) -> None:
        task = self.tasks.get(robot_id)
        if task is None or generation != task.nav_generation:
            return
        result = future.result()
        self.handle_navigation_result(robot_id, goal_type, result.status == GoalStatus.STATUS_SUCCEEDED, f'NAV_STATUS_{result.status}')

    def navigation_result_callback(self, msg: NavigationResult) -> None:
        robot_id = self.normalize_robot_id(msg.robot_id)
        task = self.tasks.get(robot_id)
        if task is None or (msg.task_id and msg.task_id != task.task_id) or msg.goal_type != task.goal_type:
            return
        self.handle_navigation_result(robot_id, msg.goal_type, msg.success, msg.error_code)

    def handle_navigation_result(self, robot_id: str, goal_type: str, success: bool, error_code: str) -> None:
        task = self.tasks.get(robot_id)
        if task is None:
            return
        task.goal_handle, task.goal_pending = None, False
        if not success:
            self.transition(task, 'ERROR', error_code or 'NAVIGATION_FAILED')
            self.publish_error(robot_id, task.task_id, error_code or 'NAVIGATION_FAILED')
            return
        task.goal_completed = True
        if goal_type == 'TO_WORKER':
            # 웹캠 PC의 실제 작업자 감지 알고리즘은 아직 없다 — 배정 위치에 물리적으로
            # 도착한 것 자체를 감지 완료로 간주하고 대기 없이 바로 FOLLOWING으로 넘어간다.
            self.transition(task, 'FOLLOWING', '작업자 위치 도착, 작업자 감지 완료')
        elif goal_type == 'TO_DESTINATION':
            # 배송확인은 로컬라이제이션(Nav2 액션 성공) 기반으로 자동 처리한다.
            task.goal_type, task.target = 'TO_DOCK', tuple(float(value) for value in self.get_parameter(f'{robot_id}_dock_pose').value)
            task.goal_completed = False
            self.transition(task, 'RETURNING', '배송 위치 도착, 배송 확인 완료')
            self.send_navigation_goal(task, replace=True)
        elif goal_type == 'TO_DOCK':
            self.dock_publishers[robot_id].publish(Bool(data=True))
            self.transition(task, 'DOCKED', '도킹 위치 복귀 완료')

    def retry_navigation_goals(self) -> None:
        now = self.get_clock().now()
        for task in self.tasks.values():
            if (task.state in ('ASSIGNED', 'TRANSPORTING', 'RETURNING') and task.target
                    and not task.goal_completed and task.goal_handle is None and not task.goal_pending
                    and not task.awaiting_dock_check and not task.undock_requested):
                self.send_navigation_goal(task)
            self.check_dock_wait_timeout(task, now)

    def check_dock_wait_timeout(self, task: ManagedTask, now) -> None:
        if not (task.awaiting_dock_check or task.undock_requested):
            return
        if task.dock_check_started_at is None:
            return
        elapsed_sec = (now - task.dock_check_started_at).nanoseconds / 1e9
        if elapsed_sec < self.DOCK_WAIT_TIMEOUT_SEC:
            return
        error_code = 'DOCK_STATUS_UNKNOWN_TIMEOUT' if task.awaiting_dock_check else 'UNDOCK_CONFIRM_TIMEOUT'
        self.publish_error(task.robot_id, task.task_id, error_code)
        self.get_logger().error(f'{task.robot_id} {error_code}: {elapsed_sec:.1f}초 경과, 사람 개입 필요')
        # 타임아웃 에러는 1회만 발행한다 — 플래그(awaiting_dock_check/undock_requested)는
        # 그대로 두어 주행을 계속 막되, dock_check_started_at만 비워 중복 발행을 막는다.
        task.dock_check_started_at = None

    def deadlock_callback(self, msg: DeadlockPermission) -> None:
        robot_id = self.normalize_robot_id(msg.robot_id)
        task = self.tasks.get(robot_id)
        if msg.granted and task and task.state == 'PAUSED' and task.pause_reason.startswith('DEADLOCK_WAIT:'):
            self.resume_task(robot_id, allow_deadlock_release=True)
        elif not msg.granted:
            self.pause_task(robot_id, f'DEADLOCK_WAIT:{msg.zone_id}')

    def error_callback(self, msg: RobotError) -> None:
        if msg.error_code.startswith(self.COMMAND_REJECTION_CODES):
            return
        robot_id = self.normalize_robot_id(msg.robot_id)
        task = self.tasks.get(robot_id)
        if task and task.state != 'ERROR':
            self.invalidate_navigation_goal(task)
            self.stop_publishers[robot_id].publish(Bool(data=True))
            self.transition(task, 'ERROR', msg.error_code)

    def publish_state(self, task: ManagedTask, detail: str) -> None:
        msg = TaskState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.task_id, msg.robot_id, msg.state = task.task_id, task.robot_id, task.state
        msg.previous_state, msg.goal_type, msg.detail = task.previous_state, task.goal_type, detail
        if task.target:
            msg.target_x, msg.target_y, msg.target_yaw = task.target
        self.state_pub.publish(msg)
        self.get_logger().info(f'{task.task_id} · {task.robot_id}: {task.previous_state} -> {task.state} ({detail})')

    def publish_error(self, robot_id: str, task_id: str, error_code: str) -> None:
        msg = RobotError()
        msg.robot_id, msg.task_id, msg.error_code = robot_id, task_id, error_code
        self.error_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
