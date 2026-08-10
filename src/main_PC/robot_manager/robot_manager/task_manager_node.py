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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from robot_status.msg import DeadlockPermission, DestinationList, NavigationResult, RobotAssignment, RobotError, TaskCommand, TaskState


@dataclass
class ManagedTask:  # 로봇의 정보를 모델 클래스
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
    awaiting_undock: bool = False
    canceled: bool = False  # ROS 상태 문자열과 별도로 취소된 작업의 수동 복구 권한을 보존한다.
    destination_id: str = ''  # HMI가 선택한 중앙 목적지 설정 ID를 상태와 DB에 전달한다.


class TaskManagerNode(Node):
    """AMR 배정 이후 Task 상태 전환과 Nav2 Goal 실행을 중앙에서 관리한다."""

    VALID_ROBOTS = {'robot5', 'robot11'}
    ACTIVE_STATES = {'ASSIGNED', 'FOLLOWING', 'TRANSPORTING', 'RETURNING'}  # 로봇의 상태 목록
    CANCELABLE_STATES = ACTIVE_STATES | {'PAUSED'}  # 일시정지한 작업도 HMI에서 취소할 수 있다.
    COMMAND_REJECTION_CODES = (     # 에러 코드
        'UNKNOWN_ROBOT_ID', 'TASK_NOT_FOUND', 'STALE_TASK_COMMAND',
        'INVALID_DESTINATION', 'ROBOT_ALREADY_HAS_TASK', 'INVALID_TRANSITION_',
        'DESTINATION_NOT_FOUND', 'DESTINATIONS_NOT_READY',
        'MANUAL_CONTROL_NOT_ALLOWED',  # 명령 거부 자체가 실제 로봇 ERROR로 전환되지 않게 제외한다.
    )

    def __init__(self):
        super().__init__('task_manager_node')
        self.declare_parameter('robot5_dock_pose', [0.0, 0.0, 0.0])     # 로봇5 도킹 위치
        self.declare_parameter('robot11_dock_pose', [-2.3, -3.6, -math.pi / 2]) # 로봇11 도킹 위치
        self.tasks: Dict[str, ManagedTask] = {}
        self.idle_paused: Set[str] = set()
        self.destinations = {}

        # --------------------------------subscription---------------------------------------------------------------
        self.assignment_sub = self.create_subscription(RobotAssignment, '/robot_assignment', self.assignment_callback, 10)  # 할당된 로봇 및 작업자 위치 받아옴
        self.command_sub = self.create_subscription(TaskCommand, '/task/command', self.command_callback, 10)    # 작업자 상태 변환 수신
        self.navigation_result_sub = self.create_subscription(NavigationResult, '/navigation/result', self.navigation_result_callback, 10)  # 
        self.deadlock_sub = self.create_subscription(DeadlockPermission, '/deadlock/permission', self.deadlock_callback, 10)
        self.error_sub = self.create_subscription(RobotError, '/robot_error', self.error_callback, 10)
        # 늦게 시작해도 중앙 Destination Manager의 마지막 설정 목록을 받는다.
        destination_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.destination_sub = self.create_subscription(
            DestinationList, '/destinations', self.destination_callback, destination_qos)

        # --------------------------------publisher---------------------------------------------------------------
        self.state_pub = self.create_publisher(TaskState, '/task/state', 10)
        self.error_pub = self.create_publisher(RobotError, '/robot_error', 10)
        self.stop_publishers = {robot_id: self.create_publisher(Bool, f'/{robot_id}/pause/request', 10) for robot_id in self.VALID_ROBOTS}
        self.dock_publishers = {robot_id: self.create_publisher(Bool, f'/{robot_id}/dock/request', 10) for robot_id in self.VALID_ROBOTS}

        # --------------------------------Action_client---------------------------------------------------------------
        self.nav_clients = {robot_id: ActionClient(self, NavigateToPose, f'/{robot_id}/navigate_to_pose') for robot_id in self.VALID_ROBOTS}
        self.nav_retry_timer = self.create_timer(1.0, self.retry_navigation_goals)
        self.get_logger().info('Task Manager 시작: robot5, robot11')

    @staticmethod
    def normalize_robot_id(robot_id: str) -> str:       # robot5, robot11 반환
        value = str(robot_id).strip()
        return value if value.startswith('robot') else f'robot{value}'

    def assignment_callback(self, msg: RobotAssignment) -> None:    # 로봇에 작업id와 상태를 부여 후 Nav2에 Action goal 요청
        if not msg.assigned:
            return
        
        robot_id = self.normalize_robot_id(msg.robot_id)
        if robot_id not in self.VALID_ROBOTS:       # 로봇이 5, 11 아닌 다른 로봇일 때
            self.publish_error(robot_id, '', 'UNKNOWN_ROBOT_ID')
            return
        
        if robot_id in self.tasks and self.tasks[robot_id].state != 'DOCKED':   # 로봇이 이미 다른 작업을 수행중일 때
            self.publish_error(robot_id, self.tasks[robot_id].task_id, 'ROBOT_ALREADY_HAS_TASK')
            return
        
        task_id = f'TASK_{msg.assigned_at.sec}_{msg.assigned_at.nanosec}'
        task = ManagedTask(
            task_id=task_id, robot_id=robot_id, state='ASSIGNED',
            goal_type='TO_WORKER', target=(float(msg.target_x), float(msg.target_y), 0.0),
            awaiting_undock=True)
        self.tasks[robot_id] = task
        # 첫 좌표로 이동하기 전에 Create3의 실제 Undock 성공을 확인한다.
        # 브릿지는 False 요청을 Undock 액션으로 변환하고 /navigation/result로 결과를 회신한다.
        self.publish_state(task, 'AMR 배정 완료, 출발 전 언도킹 요청')
        self.dock_publishers[robot_id].publish(Bool(data=False))

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
        elif command in ('DOCK', 'UNDOCK', 'TELEOP_ENABLE'):
            # HMI를 우회한 명령도 막기 위해 Task Manager에서 CANCELED/ERROR 조건을 다시 검증한다.
            if task is None or (not task.canceled and task.state != 'ERROR'):
                self.publish_error(robot_id, msg.task_id, 'MANUAL_CONTROL_NOT_ALLOWED')
                return
            # 취소 복귀 goal 등이 남아 있으면 수동 조작과 충돌하므로 먼저 완전히 중단한다.
            self.invalidate_navigation_goal(task)
            self.stop_publishers[robot_id].publish(Bool(data=True))
            if command in ('DOCK', 'UNDOCK'):
                self.dock_publishers[robot_id].publish(Bool(data=command == 'DOCK'))
        elif task is None:
            self.publish_error(robot_id, msg.task_id, 'TASK_NOT_FOUND')
        elif msg.task_id and msg.task_id != task.task_id:
            self.publish_error(robot_id, msg.task_id, 'STALE_TASK_COMMAND')
        elif command == 'WORKER_DETECTED' and task.state == 'ASSIGNED':
            if not task.goal_completed:
                self.invalidate_navigation_goal(task)
            self.transition(task, 'FOLLOWING', '작업자 추종 시작')
        elif command == 'START_TRANSPORT' and task.state == 'FOLLOWING':
            # 로봇 HMI는 좌표 대신 ID만 보내며, 실제 Nav2 좌표는 중앙 DB 배포 설정만 신뢰한다.
            destination_id = str(msg.destination_id).strip()
            if destination_id:
                if not self.destinations:
                    self.publish_error(robot_id, task.task_id, 'DESTINATIONS_NOT_READY')
                    return
                destination = self.destinations.get(destination_id)
                if destination is None:
                    self.publish_error(robot_id, task.task_id, 'DESTINATION_NOT_FOUND')
                    return
                task.destination_id = destination_id
            else:
                # 기존 CLI 좌표 명령은 호환성을 위해 유지하고, 로봇 HMI API에서는 이 경로를 사용하지 않는다.
                if not math.isfinite(msg.target_x) or not math.isfinite(msg.target_y):
                    self.publish_error(robot_id, task.task_id, 'INVALID_DESTINATION')
                    return
                destination = (float(msg.target_x), float(msg.target_y), float(msg.target_yaw))
            task.goal_type, task.target = 'TO_DESTINATION', destination
            task.goal_completed = False
            self.transition(task, 'TRANSPORTING', '작업자가 배송 모드로 전환')
            self.send_navigation_goal(task, replace=True)
        elif (command == 'DELIVERY_CONFIRMED' and task.state == 'TRANSPORTING'
              and task.goal_type == 'TO_DESTINATION' and task.goal_completed):
            # 로컬 HMI를 우회해도 실제 목적지 Nav2 성공 전에는 복귀를 시작할 수 없다.
            task.goal_type, task.target = 'TO_DOCK', tuple(float(value) for value in self.get_parameter(f'{robot_id}_dock_pose').value)
            task.goal_completed = False
            self.transition(task, 'RETURNING', '배송 확인 완료')
            self.send_navigation_goal(task, replace=True)
        elif command == 'RETURN_TO_DOCK' and task.state in ('ASSIGNED', 'FOLLOWING', 'TRANSPORTING'):
            # 정상 HMI 도킹은 수동 DOCK과 달리 dock pose까지 자율 복귀한 뒤 실제 Dock 액션을 수행한다.
            task.pause_reason = ''
            self.stop_publishers[robot_id].publish(Bool(data=False))
            task.goal_type = 'TO_DOCK'
            task.target = tuple(float(value) for value in self.get_parameter(f'{robot_id}_dock_pose').value)
            task.goal_completed = False
            self.transition(task, 'RETURNING', '로봇 HMI 정상 복귀 요청')
            self.send_navigation_goal(task, replace=True)
        elif command == 'CANCEL' and task.state in self.CANCELABLE_STATES:
            # ROS TaskState에는 CANCELED가 없으므로 별도 플래그로 수동 복구 허가를 유지한다.
            task.canceled = True
            # PAUSED 상태에서 취소한 경우 정지 요청을 해제해야 복귀 Nav2 goal이 실행된다.
            task.pause_reason = ''
            self.stop_publishers[robot_id].publish(Bool(data=False))
            task.goal_type, task.target = 'TO_DOCK', tuple(float(value) for value in self.get_parameter(f'{robot_id}_dock_pose').value)
            task.goal_completed = False
            self.transition(task, 'RETURNING', '작업 취소 후 복귀')
            self.send_navigation_goal(task, replace=True)
        else:
            self.publish_error(robot_id, task.task_id, f'INVALID_TRANSITION_{task.state}_{command}')

    def transition(self, task: ManagedTask, new_state: str, detail: str) -> None:   # 상태 전환
        task.previous_state, task.state = task.state, new_state
        self.publish_state(task, detail)

    def destination_callback(self, msg: DestinationList) -> None:
        """중앙 DB에서 검증되어 배포된 목적지 ID와 map 좌표를 캐시한다."""
        self.destinations = {
            str(item.destination_id): (
                float(item.position_x), float(item.position_y), 0.0)
            for item in msg.destinations
            if (str(item.destination_id).strip()
                and math.isfinite(float(item.position_x))
                and math.isfinite(float(item.position_y)))
        }

    def pause_task(self, robot_id: str, detail: str) -> None:
        task = self.tasks.get(robot_id)
        if task is None:
            self.idle_paused.add(robot_id)
            self.stop_publishers[robot_id].publish(Bool(data=True))
            return
        # ERROR는 이미 goal과 구동이 중단된 최종 상태이므로 PAUSED로 덮어쓰지 않는다.
        if task.state == 'ERROR':
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
        if task.target and not task.goal_completed and task.state in ('ASSIGNED', 'TRANSPORTING', 'RETURNING'):
            self.send_navigation_goal(task)

    def invalidate_navigation_goal(self, task: ManagedTask) -> None:    # Nav2 goal 요청 취소
        task.nav_generation += 1
        task.goal_pending = False
        goal_handle, task.goal_handle = task.goal_handle, None
        if goal_handle is not None:
            goal_handle.cancel_goal_async()

    def send_navigation_goal(self, task: ManagedTask, replace: bool = False) -> None:   # Nav2로 Action goal 요청하기
        if task.target is None:     # goal 목표가 없을 때
            return
        
        if task.goal_handle is not None or task.goal_pending:   # 주행 중 or 서버 응답 대기중 
            if not replace: # 기존 주행 유지
                return
            self.invalidate_navigation_goal(task)   # goal 취소

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
        future = client.send_goal_async(goal)       # Nav2 goal 전송
        future.add_done_callback(lambda result, rid=task.robot_id, goal_type=task.goal_type, gen=generation: self.goal_response_callback(result, rid, goal_type, gen))

    def goal_response_callback(self, future, robot_id: str, goal_type: str, generation: int) -> None:   # Action goal 수락 여부
        task = self.tasks.get(robot_id)
        if task is None or generation != task.nav_generation:
            return
        task.goal_pending = False
        task.goal_handle = future.result()
        if not task.goal_handle.accepted:
            # HMI가 작업 단계별 원인을 표시할 수 있도록 goal 종류가 포함된 코드를 사용한다.
            self.handle_navigation_result(
                robot_id, goal_type, False, self.navigation_error_code(goal_type, 'GOAL_REJECTED'))
            return
        result_future = task.goal_handle.get_result_async()
        result_future.add_done_callback(lambda result, rid=robot_id, kind=goal_type, gen=generation: self.action_result_callback(result, rid, kind, gen))

    def action_result_callback(self, future, robot_id: str, goal_type: str, generation: int) -> None:   # Action goal 결과
        task = self.tasks.get(robot_id)
        if task is None or generation != task.nav_generation:
            return
        result = future.result()
        self.handle_navigation_result(
            robot_id, goal_type, result.status == GoalStatus.STATUS_SUCCEEDED,
            self.navigation_error_code(goal_type, f'FAILED_STATUS_{result.status}'))

    @staticmethod
    def navigation_error_code(goal_type: str, suffix: str) -> str:
        """최초 접근·배송·복귀 Nav2 실패를 HMI/DB에서 구분 가능한 코드로 만든다."""
        prefix = {
            'TO_WORKER': 'WORKER_APPROACH',
            'TO_DESTINATION': 'DESTINATION_NAV',
            'TO_DOCK': 'RETURN_NAV',
        }.get(goal_type, 'NAV')
        return f'{prefix}_{suffix}'

    def navigation_result_callback(self, msg: NavigationResult) -> None:
        robot_id = self.normalize_robot_id(msg.robot_id)
        task = self.tasks.get(robot_id)
        if task is None or (msg.task_id and msg.task_id != task.task_id):
            return
        # 브릿지의 실제 Dock 액션 성공 결과가 도착한 뒤에만 작업을 DOCKED로 완료한다.
        if msg.goal_type == 'DOCK':
            if msg.success and task.goal_type == 'TO_DOCK':
                task.goal_completed = True
                self.transition(task, 'DOCKED', '실제 도킹 액션 성공')
            return
        if msg.goal_type == 'UNDOCK':
            # 작업 배정과 무관한 수동 Undock 결과는 기존처럼 작업 상태에 반영하지 않는다.
            if not task.awaiting_undock:
                return
            task.awaiting_undock = False
            if not msg.success:
                if task.state != 'ERROR':
                    self.transition(task, 'ERROR', msg.error_code or 'UNDOCK_FAILED')
                return
            # Undock 대기 중 일시정지/오류가 발생했으면 즉시 주행하지 않는다.
            # PAUSED는 재개 시 기존 resume_task 경로에서 goal을 전송한다.
            if task.state == 'ASSIGNED':
                self.publish_state(task, '언도킹 완료, 작업자 위치로 이동')
                self.send_navigation_goal(task)
            return
        if msg.goal_type != task.goal_type:
            return
        self.handle_navigation_result(robot_id, msg.goal_type, msg.success, msg.error_code)

    def handle_navigation_result(self, robot_id: str, goal_type: str, success: bool, error_code: str) -> None:
        task = self.tasks.get(robot_id)
        if task is None:
            return
        task.goal_handle, task.goal_pending = None, False
        if not success:
            self.transition(task, 'ERROR', error_code or 'NAVIGATION_FAILED')   # Nav2 도착 실패
            self.publish_error(robot_id, task.task_id, error_code or 'NAVIGATION_FAILED')
        else:
            task.goal_completed = True
        if success and goal_type == 'TO_WORKER':
            self.publish_state(task, '작업자 위치 도착, 작업자 감지 대기')
        elif success and goal_type == 'TO_DESTINATION':
            self.publish_state(task, '배송 위치 도착, 작업자 배송 확인 대기')
        elif success and goal_type == 'TO_DOCK':
            # 도킹 위치 도착은 작업 완료가 아니다. 상태를 유지한 채 실제 Dock 액션 결과를 기다린다.
            self.publish_state(task, '도킹 위치 도착, 실제 도킹 액션 대기')
            self.dock_publishers[robot_id].publish(Bool(data=True))

    def retry_navigation_goals(self) -> None:
        for task in self.tasks.values():
            if (task.state in ('ASSIGNED', 'TRANSPORTING', 'RETURNING')
                    and task.target and not task.awaiting_undock
                    and not task.goal_completed and task.goal_handle is None
                    and not task.goal_pending):
                self.send_navigation_goal(task)

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
        # 로봇 PC 브릿지가 문구 비교 없이 작업자 위치 도착 완료를 판단하도록 명시적으로 전달한다.
        msg.goal_completed = task.goal_completed
        msg.destination_id = task.destination_id
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
