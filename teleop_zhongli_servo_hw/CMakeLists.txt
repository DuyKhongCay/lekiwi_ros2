cmake_minimum_required(VERSION 3.16)
project(teleop_zhongli_servo_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)

pkg_check_modules(SERIAL REQUIRED libserial)

add_library(teleop_zhongli_protocol SHARED
  src/zhongli_protocol.cpp
  src/kinematic_remapper.cpp
  src/follower_dispatcher.cpp
)
target_compile_features(teleop_zhongli_protocol PUBLIC cxx_std_20)
target_include_directories(teleop_zhongli_protocol
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(teleop_zhongli_protocol SYSTEM PUBLIC ${SERIAL_INCLUDE_DIRS})
target_link_libraries(teleop_zhongli_protocol
  PUBLIC
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    trajectory_msgs::trajectory_msgs
    std_msgs::std_msgs
    yaml-cpp
    ${SERIAL_LIBRARIES}
)

add_executable(teleop_uarm_node
  src/teleop_uarm_node.cpp
)
target_compile_features(teleop_uarm_node PUBLIC cxx_std_20)
target_include_directories(teleop_uarm_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(teleop_uarm_node
  teleop_zhongli_protocol
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  diagnostic_updater::diagnostic_updater
  diagnostic_msgs::diagnostic_msgs
  trajectory_msgs::trajectory_msgs
  std_msgs::std_msgs
  yaml-cpp
)

add_executable(calibrate_uarm_node
  src/calibrate_uarm_node.cpp
)
target_compile_features(calibrate_uarm_node PUBLIC cxx_std_20)
target_include_directories(calibrate_uarm_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(calibrate_uarm_node
  teleop_zhongli_protocol
)

install(
  TARGETS teleop_zhongli_protocol
  EXPORT teleop_zhongli_servo_hwTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    teleop_uarm_node
    calibrate_uarm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_command_format test/test_command_format.cpp)
  target_compile_features(test_command_format PUBLIC cxx_std_20)
  target_include_directories(test_command_format PRIVATE include)
  target_link_libraries(test_command_format teleop_zhongli_protocol)

  ament_add_gtest(test_kinematic_conversion test/test_kinematic_conversion.cpp)
  target_compile_features(test_kinematic_conversion PUBLIC cxx_std_20)
  target_include_directories(test_kinematic_conversion PRIVATE include)
  target_link_libraries(test_kinematic_conversion teleop_zhongli_protocol)

  ament_add_gtest(test_kinematic_remapping test/test_kinematic_remapping.cpp)
  target_compile_features(test_kinematic_remapping PUBLIC cxx_std_20)
  target_include_directories(test_kinematic_remapping PRIVATE include)
  target_link_libraries(test_kinematic_remapping teleop_zhongli_protocol)
endif()

ament_export_targets(teleop_zhongli_servo_hwTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs diagnostic_updater diagnostic_msgs trajectory_msgs std_msgs)
ament_package()
