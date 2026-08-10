cmake_minimum_required(VERSION 3.8)
project(robot_status)

# Robot5/Robot11 PC에서 공통으로 사용하는 ROS 2 인터페이스 패키지다.
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
