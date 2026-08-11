import math

from robot_status.msg import RobotStatus


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """z축 회전만 있다고 가정하고 쿼터니언에서 yaw(라디안)를 추출한다."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw: float) -> tuple:
    """z축 회전만 있다고 가정하고 yaw(라디안)에서 쿼터니언 (z, w)를 만든다."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def is_valid_quaternion(x: float, y: float, z: float, w: float) -> bool:
    """모든 성분이 0인 쿼터니언(추적 대상 없음을 뜻하는 무효 메시지)을 걸러낸다."""
    return not (x == 0.0 and y == 0.0 and z == 0.0 and w == 0.0)


def is_followable_pose(x: float, y: float, z: float,
                       qx: float, qy: float, qz: float, qw: float) -> bool:
    """추종 goal로 보내도 되는 pose인지 판단한다.

    `reid_tracking_node`는 추적 대상이 없을 때도 매 프레임 갓 생성한 `PoseStamped`를
    발행한다. 이때 position은 (0,0,0)이고 orientation은 IDL 기본값 (0,0,0,1) —
    즉 항등 쿼터니언이라 `is_valid_quaternion`만으로는 걸러지지 않는다. 이 pose를
    그대로 Nav2에 넘기면 로봇이 map 원점으로 주행하므로 position이 정확히 원점이면
    거부한다. 전 성분이 0인 쿼터니언도 방어적으로 함께 거부한다.
    """
    if x == 0.0 and y == 0.0 and z == 0.0:
        return False
    return is_valid_quaternion(qx, qy, qz, qw)


def build_robot_status(robot_id: str, battery_percent: float, x: float, y: float, yaw: float) -> RobotStatus:
    msg = RobotStatus()
    msg.robot_id = robot_id
    msg.battery = float(battery_percent)
    msg.x = float(x)
    msg.y = float(y)
    msg.yaw = float(yaw)
    return msg
