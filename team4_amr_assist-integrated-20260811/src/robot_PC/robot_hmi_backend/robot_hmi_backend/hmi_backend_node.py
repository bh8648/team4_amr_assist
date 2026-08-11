#!/usr/bin/env python3
import asyncio
import threading
import time
from typing import Optional

import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from robot_status.msg import DestinationList, RobotError, RobotStatus, TaskCommand, TaskState


class DestinationRequest(BaseModel):
    destination_id: str


class RobotHmiBackendNode(Node):
    """로봇에 탑재된 HMI 한 대를 해당 robot_id의 ROS 상태와 명령에만 연결한다."""

    def __init__(self):
        super().__init__('robot_hmi_backend_node')
        self.declare_parameter('robot_id', '')
        self.declare_parameter('web_port', 8000)
        self.robot_id = str(self.get_parameter('robot_id').value).strip()
        self.web_port = int(self.get_parameter('web_port').value)
        if self.robot_id not in ('robot5', 'robot11'):
            raise ValueError('robot_id는 robot5 또는 robot11이어야 합니다.')

        self.lock = threading.Lock()
        self.latest_robot_status = None
        self.latest_task_state = None
        self.latest_error_code = ''
        self.last_status_monotonic = 0.0
        self.destinations = {}

        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        destination_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 공통 토픽을 사용하되 콜백에서 자신의 robot_id만 받아 PC별 상태가 섞이지 않게 한다.
        self.status_sub = self.create_subscription(
            RobotStatus, '/robot_status', self.status_callback, status_qos)
        self.task_sub = self.create_subscription(
            TaskState, '/task/state', self.task_state_callback, 10)
        self.error_sub = self.create_subscription(
            RobotError, '/robot_error', self.error_callback, 10)
        self.destination_sub = self.create_subscription(
            DestinationList, '/destinations', self.destination_callback, destination_qos)
        self.command_pub = self.create_publisher(TaskCommand, '/task/command', 10)
        self.get_logger().info(f'{self.robot_id} 탑재 HMI 백엔드 시작: port={self.web_port}')

    def status_callback(self, msg: RobotStatus) -> None:
        if str(msg.robot_id) != self.robot_id:
            return
        with self.lock:
            self.latest_robot_status = msg
            self.last_status_monotonic = time.monotonic()

    def task_state_callback(self, msg: TaskState) -> None:
        if str(msg.robot_id) != self.robot_id:
            return
        with self.lock:
            self.latest_task_state = msg
            # 새 정상 상태가 도착하면 이전 Task의 화면 오류를 남겨두지 않는다.
            if str(msg.state) != 'ERROR':
                self.latest_error_code = ''

    def error_callback(self, msg: RobotError) -> None:
        if str(msg.robot_id) != self.robot_id:
            return
        with self.lock:
            self.latest_error_code = str(msg.error_code)

    def destination_callback(self, msg: DestinationList) -> None:
        with self.lock:
            self.destinations = {
                str(item.destination_id): {
                    'destination_id': str(item.destination_id),
                    'destination_name': str(item.destination_name),
                    'position_x': float(item.position_x),
                    'position_y': float(item.position_y),
                }
                for item in msg.destinations
            }

    def status_payload(self):
        """프론트엔드 버튼 표시까지 가능한 물리 상태와 Task 상태를 하나로 합친다."""
        with self.lock:
            status, task = self.latest_robot_status, self.latest_task_state
            online = bool(status) and time.monotonic() - self.last_status_monotonic < 5.0
            return {
                'robot_id': self.robot_id,
                'online': online,
                'battery': float(status.battery) if status else None,
                'position_x': float(status.x) if status else None,
                'position_y': float(status.y) if status else None,
                'orientation_yaw': float(status.yaw) if status else None,
                'task_id': str(task.task_id) if task else '',
                'state': str(task.state) if task else 'DOCKED',
                'previous_state': str(task.previous_state) if task else '',
                'goal_type': str(task.goal_type) if task else '',
                'goal_completed': bool(task.goal_completed) if task else False,
                'destination_id': str(task.destination_id) if task else '',
                'detail': str(task.detail) if task else '',
                'error_code': self.latest_error_code or None,
            }

    def destination_payload(self):
        with self.lock:
            return list(self.destinations.values())

    def publish_command(self, command_name: str, destination_id: str = '') -> None:
        """HMI 요청을 현재 Task ID가 포함된 중앙 상태 전환 명령으로 발행한다."""
        with self.lock:
            task = self.latest_task_state
        if task is None or not str(task.task_id):
            raise ValueError('현재 작업이 없습니다.')
        command = TaskCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.command, command.robot_id = command_name, self.robot_id
        command.task_id, command.destination_id = str(task.task_id), destination_id
        command.detail = f'{self.robot_id} 탑재 HMI 요청'
        self.command_pub.publish(command)


app = FastAPI(title='Robot-mounted HMI Backend')
app.add_middleware(
    CORSMiddleware, allow_origins=['*'], allow_credentials=True,
    allow_methods=['*'], allow_headers=['*'])
ros_node: Optional[RobotHmiBackendNode] = None


@app.get('/api/status')
def get_status():
    if ros_node is None:
        raise HTTPException(status_code=503, detail='ROS 노드가 준비되지 않았습니다.')
    return ros_node.status_payload()


@app.get('/api/destinations')
def get_destinations():
    if ros_node is None:
        raise HTTPException(status_code=503, detail='ROS 노드가 준비되지 않았습니다.')
    return ros_node.destination_payload()


@app.post('/api/delivery/start')
def start_delivery(request: DestinationRequest):
    if ros_node is None:
        raise HTTPException(status_code=503, detail='ROS 노드가 준비되지 않았습니다.')
    status = ros_node.status_payload()
    destination_id = request.destination_id.strip()
    if status['state'] != 'FOLLOWING':
        raise HTTPException(status_code=409, detail='FOLLOWING 상태에서만 배송 모드로 변경할 수 있습니다.')
    if destination_id not in {item['destination_id'] for item in ros_node.destination_payload()}:
        raise HTTPException(status_code=404, detail='등록되지 않은 목적지입니다.')
    try:
        ros_node.publish_command('START_TRANSPORT', destination_id)
    except ValueError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error
    return {'accepted': True, 'destination_id': destination_id}


@app.post('/api/return-to-dock')
def return_to_dock():
    if ros_node is None:
        raise HTTPException(status_code=503, detail='ROS 노드가 준비되지 않았습니다.')
    if ros_node.status_payload()['state'] not in ('ASSIGNED', 'FOLLOWING', 'TRANSPORTING', 'PAUSED'):
        raise HTTPException(status_code=409, detail='현재 상태에서는 정상 복귀할 수 없습니다.')
    try:
        ros_node.publish_command('RETURN_TO_DOCK')
    except ValueError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error
    return {'accepted': True}


@app.post('/api/task/pause')
def pause_task():
    if ros_node is None:
        raise HTTPException(status_code=503, detail='ROS 노드가 준비되지 않았습니다.')
    if ros_node.status_payload()['state'] not in ('ASSIGNED', 'FOLLOWING', 'TRANSPORTING', 'RETURNING'):
        raise HTTPException(status_code=409, detail='현재 상태에서는 일시정지할 수 없습니다.')
    try:
        ros_node.publish_command('PAUSE')
    except ValueError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error
    return {'accepted': True}


@app.post('/api/task/resume')
def resume_task():
    if ros_node is None:
        raise HTTPException(status_code=503, detail='ROS 노드가 준비되지 않았습니다.')
    if ros_node.status_payload()['state'] != 'PAUSED':
        raise HTTPException(status_code=409, detail='PAUSED 상태에서만 재개할 수 있습니다.')
    try:
        ros_node.publish_command('RESUME')
    except ValueError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error
    return {'accepted': True}


@app.post('/api/delivery/complete')
def complete_delivery():
    if ros_node is None:
        raise HTTPException(status_code=503, detail='ROS 노드가 준비되지 않았습니다.')
    status = ros_node.status_payload()
    if not (status['state'] == 'TRANSPORTING'
            and status['goal_type'] == 'TO_DESTINATION'
            and status['goal_completed']):
        raise HTTPException(status_code=409, detail='목적지 도착 후에만 배송 완료를 확인할 수 있습니다.')
    try:
        ros_node.publish_command('DELIVERY_CONFIRMED')
    except ValueError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error
    return {'accepted': True}


@app.websocket('/ws/status')
async def status_websocket(websocket: WebSocket):
    """탑재 HMI 화면에 로봇 상태를 5Hz로 전달한다."""
    await websocket.accept()
    try:
        while True:
            if ros_node is not None:
                await websocket.send_json(ros_node.status_payload())
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        return


def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = RobotHmiBackendNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()
    try:
        uvicorn.run(app, host='0.0.0.0', port=ros_node.web_port, log_level='info')
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        ros_node.destroy_node()


if __name__ == '__main__':
    main()
