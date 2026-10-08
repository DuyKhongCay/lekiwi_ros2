cmake_minimum_required(VERSION 3.8)
project(lekiwi_motion)

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
find_package(Eigen3 REQUIRED)
find_package(lekiwi_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# ==============================================================================
# SUBMODULE 1: kinematics
# Workspace kinematics, analytical 5-DoF IK, chessboard mapping, reachability
# ==============================================================================
add_library(workspace_model STATIC
  src/kinematics/workspace_kinematics.cpp
  src/kinematics/workspace_planner.cpp
  src/kinematics/chessboard_mapper.cpp
  src/kinematics/feasibility_marker_builder.cpp
)
set_target_properties(workspace_model PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(workspace_model PUBLIC cxx_std_17)
target_include_directories(workspace_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(workspace_model PUBLIC
  Eigen3::Eigen
  urdf::urdf
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${lekiwi_interfaces_TARGETS}
)

add_library(workspace_checker_component SHARED
  src/kinematics/workspace_checker_node.cpp
)
target_compile_features(workspace_checker_component PUBLIC cxx_std_17)
target_include_directories(workspace_checker_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(workspace_checker_component
  workspace_model
  rclcpp::rclcpp
  rclcpp_components::component
  diagnostic_updater::diagnostic_updater
  urdf::urdf
  ${diagnostic_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  tf2_ros::tf2_ros
  ${tf2_geometry_msgs_TARGETS}
  ${lekiwi_interfaces_TARGETS}
)
rclcpp_components_register_node(workspace_checker_component
  PLUGIN "lekiwi_motion::WorkspaceCheckerNode"
  EXECUTABLE workspace_checker_node
)

# ==============================================================================
# SUBMODULE 2: arm
# Unified arm motion planning, cartesian named poses, chess move execution
# ==============================================================================
add_library(arm_motion_planner STATIC
  src/arm/arm_motion_planner.cpp
)
set_target_properties(arm_motion_planner PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(arm_motion_planner PUBLIC cxx_std_17)
target_include_directories(arm_motion_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(arm_motion_planner PUBLIC
  ${trajectory_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

add_library(manipulation_action_component SHARED
  src/arm/manipulation_action_server.cpp
)
target_compile_features(manipulation_action_component PUBLIC cxx_std_17)
target_include_directories(manipulation_action_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(manipulation_action_component
  arm_motion_planner
  workspace_model
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  diagnostic_updater::diagnostic_updater
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  urdf::urdf
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${control_msgs_TARGETS}
  ${lekiwi_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
)
rclcpp_components_register_node(manipulation_action_component
  PLUGIN "lekiwi_motion::ManipulationActionServer"
  EXECUTABLE manipulation_action_server
)

# ==============================================================================
# SUBMODULE 3: supervision
# System readiness monitoring, torque control management, Nav2 startup gate
# ==============================================================================
add_library(system_readiness_state STATIC
  src/supervision/system_readiness_state.cpp
)
set_target_properties(system_readiness_state PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(system_readiness_state PUBLIC cxx_std_17)
target_include_directories(system_readiness_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(system_readiness_component SHARED
  src/supervision/system_readiness_node.cpp
)
target_include_directories(system_readiness_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(system_readiness_component PUBLIC cxx_std_17)
target_link_libraries(system_readiness_component
  system_readiness_state
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
rclcpp_components_register_node(system_readiness_component
  PLUGIN "lekiwi_motion::SystemReadinessNode"
  EXECUTABLE system_readiness_node
)

add_library(torque_manager_component SHARED
  src/supervision/torque_manager_node.cpp
)
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
  PLUGIN "lekiwi_motion::TorqueManagerNode"
  EXECUTABLE torque_manager_node
)

add_library(nav2_startup_gate_component SHARED
  src/supervision/nav2_startup_gate_node.cpp
)
target_compile_features(nav2_startup_gate_component PUBLIC cxx_std_17)
target_include_directories(nav2_startup_gate_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(nav2_startup_gate_component
  rclcpp::rclcpp
  rclcpp_components::component
  tf2_ros::tf2_ros
  ${nav2_msgs_TARGETS}
)
rclcpp_components_register_node(nav2_startup_gate_component
  PLUGIN "lekiwi_motion::Nav2StartupGateNode"
  EXECUTABLE nav2_startup_gate_node
)

# ==============================================================================
# Installation
# ==============================================================================
install(TARGETS
  workspace_model
  workspace_checker_component
  workspace_checker_node
  arm_motion_planner
  manipulation_action_component
  manipulation_action_server
  system_readiness_state
  system_readiness_component
  system_readiness_node
  torque_manager_component
  torque_manager_node
  nav2_startup_gate_component
  nav2_startup_gate_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# ==============================================================================
# Tests
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(lekiwi_description REQUIRED)
  find_program(XACRO_EXECUTABLE xacro REQUIRED)
  get_filename_component(DESCRIPTION_SHARE "${lekiwi_description_DIR}/.." ABSOLUTE)
  file(GLOB DESCRIPTION_INPUTS CONFIGURE_DEPENDS
    "${DESCRIPTION_SHARE}/urdf/*" "${DESCRIPTION_SHARE}/config/calibration/*.yaml")
  set(WORKSPACE_URDF "${CMAKE_CURRENT_BINARY_DIR}/lekiwi_test.urdf")
  add_custom_command(OUTPUT "${WORKSPACE_URDF}"
    COMMAND "${XACRO_EXECUTABLE}" "${DESCRIPTION_SHARE}/urdf/lekiwi_robot.urdf.xacro"
      hardware_type:=mock -o "${WORKSPACE_URDF}"
    DEPENDS ${DESCRIPTION_INPUTS}
    VERBATIM)
  add_custom_target(workspace_urdf_fixture DEPENDS "${WORKSPACE_URDF}")

  # ── Kinematics tests ────────────────────────────────────────────────────────
  ament_add_gtest(test_chessboard_mapper
    test/kinematics/test_chessboard_mapper.cpp
  )
  target_compile_features(test_chessboard_mapper PUBLIC cxx_std_17)
  target_include_directories(test_chessboard_mapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_chessboard_mapper
    workspace_model
    ${geometry_msgs_TARGETS}
  )

  ament_add_gtest(test_workspace_kinematics
    test/kinematics/test_workspace_kinematics.cpp
  )
  target_compile_features(test_workspace_kinematics PUBLIC cxx_std_17)
  target_include_directories(test_workspace_kinematics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_workspace_kinematics
    workspace_model
  )
  add_dependencies(test_workspace_kinematics workspace_urdf_fixture)
  target_compile_definitions(test_workspace_kinematics PRIVATE
    WORKSPACE_URDF_PATH="${WORKSPACE_URDF}")

  ament_add_gtest(test_feasibility_marker_builder
    test/kinematics/test_feasibility_marker_builder.cpp
  )
  target_compile_features(test_feasibility_marker_builder PUBLIC cxx_std_17)
  target_include_directories(test_feasibility_marker_builder PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_feasibility_marker_builder
    workspace_model
    ${visualization_msgs_TARGETS}
    ${lekiwi_interfaces_TARGETS}
    rclcpp::rclcpp
  )

  # ── Arm tests ───────────────────────────────────────────────────────────────
  ament_add_gtest(test_arm_motion_planner
    test/arm/test_arm_motion_planner.cpp
  )
  target_compile_features(test_arm_motion_planner PUBLIC cxx_std_17)
  target_include_directories(test_arm_motion_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_arm_motion_planner
    arm_motion_planner
  )

  ament_add_gtest(test_manipulation_action_server
    test/arm/test_manipulation_action_server.cpp
  )
  target_compile_features(test_manipulation_action_server PUBLIC cxx_std_17)
  target_include_directories(test_manipulation_action_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_manipulation_action_server
    manipulation_action_component
    rclcpp::rclcpp
  )
  add_dependencies(test_manipulation_action_server workspace_urdf_fixture)
  target_compile_definitions(test_manipulation_action_server PRIVATE
    WORKSPACE_URDF_PATH="${WORKSPACE_URDF}")

  # ── Supervision tests ───────────────────────────────────────────────────────
  ament_add_gtest(test_torque_command_state
    test/supervision/test_torque_command_state.cpp
  )
  target_compile_features(test_torque_command_state PUBLIC cxx_std_17)
  target_include_directories(test_torque_command_state PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_torque_command_state
    rclcpp::rclcpp
    ${lekiwi_interfaces_TARGETS}
    ${controller_manager_msgs_TARGETS}
    ${std_msgs_TARGETS}
  )

  ament_add_gtest(test_system_readiness_state
    test/supervision/test_system_readiness_state.cpp
  )
  target_compile_features(test_system_readiness_state PUBLIC cxx_std_17)
  target_include_directories(test_system_readiness_state PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_system_readiness_state
    system_readiness_state
  )

  ament_add_gtest(test_nav2_startup_gate
    test/supervision/test_nav2_startup_gate.cpp
  )
  target_compile_features(test_nav2_startup_gate PUBLIC cxx_std_17)
  target_include_directories(test_nav2_startup_gate PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_nav2_startup_gate
    nav2_startup_gate_component
    rclcpp::rclcpp
  )
endif()

ament_package()
