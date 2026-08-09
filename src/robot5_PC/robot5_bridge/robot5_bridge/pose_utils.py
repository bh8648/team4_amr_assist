import math

from robot_status.msg import RobotStatus


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """쿼터니언을 map 기준 yaw(라디안)로 변환한다."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def is_followable_pose(x: float, y: float, z: float,
                       qx: float, qy: float, qz: float, qw: float) -> bool:
    """추적 대상 없음으로 발행된 원점 pose와 무효 쿼터니언을 차단한다."""
    if x == 0.0 and y == 0.0 and z == 0.0:
        return False
    return not (qx == 0.0 and qy == 0.0 and qz == 0.0 and qw == 0.0)


def build_robot_status(robot_id: str, battery: float, x: float, y: float, yaw: float) -> RobotStatus:
    """공통 /robot_status에 발행할 메시지를 동일 형식으로 생성한다."""
    msg = RobotStatus()
    msg.robot_id, msg.battery = robot_id, float(battery)
    msg.x, msg.y, msg.yaw = float(x), float(y), float(yaw)
    return msg
