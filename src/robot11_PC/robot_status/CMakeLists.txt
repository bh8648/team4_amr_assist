cmake_minimum_required(VERSION 3.8)
project(robot_status)

# 세 PC에서 동일한 ROS 2 메시지 코드를 생성하는 인터페이스 패키지 복사본이다.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Destination.msg"
  "msg/DestinationList.msg"
  "msg/RobotStatus.msg"
  "msg/RobotAssignment.msg"
  "msg/RobotError.msg"
  "msg/TaskCommand.msg"
  "msg/TaskState.msg"
  "msg/NavigationResult.msg"
  "msg/DeadlockPermission.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
