builtin_interfaces/Time stamp
string task_id
string robot_id
string state
string previous_state
string goal_type
# 브릿지가 문자열 detail에 의존하지 않고 최초 목적지 도착을 판별하기 위한 완료 값이다.
bool goal_completed
# 현재 배송 목적지를 로봇별 HMI와 DB에 동일하게 전달한다.
string destination_id
float32 target_x
float32 target_y
float32 target_yaw
string detail
