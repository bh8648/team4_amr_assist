import math

from robot_status.msg import RobotStatus


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """z축 회전만 있다고 가정하고 쿼터니언에서 yaw(라디안)를 추출한다."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def is_valid_quaternion(x: float, y: float, z: float, w: float) -> bool:
    """모든 성분이 0인 쿼터니언(추적 대상 없음을 뜻하는 무효 메시지)을 걸러낸다."""
    return not (x == 0.0 and y == 0.0 and z == 0.0 and w == 0.0)


def build_robot_status(robot_id: str, battery_percent: float, x: float, y: float, yaw: float) -> RobotStatus:
    msg = RobotStatus()
    msg.robot_id = robot_id
    msg.battery = float(battery_percent)
    msg.x = float(x)
    msg.y = float(y)
    msg.yaw = float(yaw)
    msg.current_task_id = ''
    return msg
