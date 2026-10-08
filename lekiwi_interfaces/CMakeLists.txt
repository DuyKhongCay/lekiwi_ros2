cmake_minimum_required(VERSION 3.10)
project(lekiwi_interfaces)

# Enforce strict compiler warnings across C++ code generators
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find buildtool generators and ROS 2 middleware dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate C++, Python, and C type support for all custom messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraMode.msg"
  "msg/PerceptionContext.msg"
  "msg/ServoTelemetry.msg"
  "msg/DriveStatus.msg"
  "msg/ChessMoveDetails.msg"
  "msg/ChessGameStatus.msg"
  "srv/SetCamMode.srv"
  "srv/SetPerceptionContext.srv"
  "srv/SetTorqueEnabled.srv"
  "srv/ResetMotorBus.srv"
  "srv/CheckMoveFeasibility.srv"
  "action/ComputeBestMove.action"
  "action/ExecuteChessMove.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Export runtime and dependent interfaces to downstream workspace consumers
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs sensor_msgs)
ament_package()
