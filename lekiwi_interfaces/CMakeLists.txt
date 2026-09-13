cmake_minimum_required(VERSION 3.10)
project(lekiwi_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraMode.msg"
  "msg/ServoTelemetry.msg"
  "msg/DriveStatus.msg"
  "srv/SetCamMode.srv"
  "srv/SetTorqueEnabled.srv"
  "srv/ResetMotorBus.srv"
  "srv/CheckReachability.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs sensor_msgs)
ament_package()
