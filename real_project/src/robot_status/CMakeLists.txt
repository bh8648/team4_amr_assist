cmake_minimum_required(VERSION 3.8)
project(robot_status)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 커스텀 메시지 등록
rosidl_generate_interfaces(${PROJECT_NAME}
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
