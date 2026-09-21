cmake_minimum_required(VERSION 3.8)
project(lekiwi_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── C++ ament package ───────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)

# ── C++ dependencies ──────────────────────────────────────────────────────────
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(lekiwi_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# ── C++ Component & Executable: tf_gatekeeper ─────────────────────────────────
add_library(tf_gatekeeper_component SHARED
  src/tf_gatekeeper_node.cpp
)
target_include_directories(tf_gatekeeper_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(tf_gatekeeper_component PUBLIC cxx_std_17)
target_link_libraries(tf_gatekeeper_component
  rclcpp::rclcpp
  rclcpp_components::component
  diagnostic_updater::diagnostic_updater
  ${diagnostic_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2_ros::tf2_ros
  ${tf2_geometry_msgs_TARGETS}
)
rclcpp_components_register_node(tf_gatekeeper_component
  PLUGIN "lekiwi_control::TfGatekeeperNode"
  EXECUTABLE tf_gatekeeper_node
)

# ── C++ Component & Executable: workspace_checker ─────────────────────────────
add_library(workspace_checker_component SHARED
  src/workspace_checker_node.cpp
)
target_compile_features(workspace_checker_component PUBLIC cxx_std_17)
target_include_directories(workspace_checker_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(workspace_checker_component
  rclcpp::rclcpp
  rclcpp_components::component
  diagnostic_updater::diagnostic_updater
  urdf::urdf
  ${diagnostic_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2_ros::tf2_ros
  ${tf2_geometry_msgs_TARGETS}
  ${lekiwi_interfaces_TARGETS}
)
rclcpp_components_register_node(workspace_checker_component
  PLUGIN "lekiwi_control::WorkspaceCheckerNode"
  EXECUTABLE workspace_checker_node
)

# ── C++ Component & Executable: torque_manager ───────────────────────────────
add_library(torque_manager_component SHARED src/torque_manager_node.cpp)
target_compile_features(torque_manager_component PUBLIC cxx_std_17)
target_include_directories(torque_manager_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(torque_manager_component
  rclcpp::rclcpp
  rclcpp_components::component
  ${rcl_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${lekiwi_interfaces_TARGETS}
  ${controller_manager_msgs_TARGETS}
)
rclcpp_components_register_node(torque_manager_component
  PLUGIN "lekiwi_control::TorqueManagerNode"
  EXECUTABLE torque_manager_node
)

install(TARGETS
  tf_gatekeeper_component
  tf_gatekeeper_node
  workspace_checker_component
  workspace_checker_node
  torque_manager_component
  torque_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  # C++ pure logic unit tests (không cần ROS runtime)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_torque_command_state test/test_torque_command_state.cpp)
  target_compile_features(test_torque_command_state PUBLIC cxx_std_17)
  target_include_directories(test_torque_command_state PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_torque_command_state ${lekiwi_interfaces_TARGETS})

  ament_add_gtest(test_tf_gatekeeper_pure
    test/test_tf_gatekeeper_pure.cpp
  )
  target_compile_features(test_tf_gatekeeper_pure PUBLIC cxx_std_17)
  target_include_directories(test_tf_gatekeeper_pure PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_workspace_planner_pure
    test/test_workspace_planner_pure.cpp
  )
  target_compile_features(test_workspace_planner_pure PUBLIC cxx_std_17)
  target_include_directories(test_workspace_planner_pure PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_workspace_planner_pure
    urdf::urdf
    ${lekiwi_interfaces_TARGETS}
  )
endif()

ament_package()
