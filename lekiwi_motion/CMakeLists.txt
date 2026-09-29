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

# ── C++ Domain Library & Component: system_readiness ─────────────────────────
add_library(system_readiness_state STATIC
  src/system_readiness_state.cpp
)
set_target_properties(system_readiness_state PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(system_readiness_state PUBLIC cxx_std_17)
target_include_directories(system_readiness_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(system_readiness_component SHARED
  src/system_readiness_node.cpp
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
  ${nav2_msgs_TARGETS}
  tf2_ros::tf2_ros
  ${tf2_geometry_msgs_TARGETS}
)
rclcpp_components_register_node(system_readiness_component
  PLUGIN "lekiwi_motion::SystemReadinessNode"
  EXECUTABLE system_readiness_node
)

# ── C++ Component & Executable: workspace_checker ─────────────────────────────
add_library(workspace_model STATIC
  src/workspace_kinematics.cpp
  src/workspace_planner.cpp
  src/chessboard_mapper.cpp
  src/feasibility_marker_builder.cpp
)
set_target_properties(workspace_model PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(workspace_model PUBLIC cxx_std_17)
target_include_directories(workspace_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(workspace_model PUBLIC
  Eigen3::Eigen
  urdf::urdf
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${lekiwi_interfaces_TARGETS}
)

add_library(workspace_checker_component SHARED
  src/workspace_checker_node.cpp
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
  PLUGIN "lekiwi_motion::TorqueManagerNode"
  EXECUTABLE torque_manager_node
)

install(TARGETS
  system_readiness_state
  system_readiness_component
  system_readiness_node
  workspace_checker_component
  workspace_checker_node
  torque_manager_component
  torque_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  # C++ pure logic unit tests (không cần ROS runtime)
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

  ament_add_gtest(test_chessboard_mapper test/test_chessboard_mapper.cpp)
  target_compile_features(test_chessboard_mapper PUBLIC cxx_std_17)
  target_include_directories(test_chessboard_mapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_chessboard_mapper
    workspace_model
    ${geometry_msgs_TARGETS}
  )

  ament_add_gtest(test_torque_command_state test/test_torque_command_state.cpp)
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
    test/test_system_readiness_state.cpp
  )
  target_compile_features(test_system_readiness_state PUBLIC cxx_std_17)
  target_include_directories(test_system_readiness_state PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_system_readiness_state
    system_readiness_state
  )

  ament_add_gtest(test_workspace_kinematics
    test/test_workspace_kinematics.cpp
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
    test/test_feasibility_marker_builder.cpp
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
endif()

ament_package()
