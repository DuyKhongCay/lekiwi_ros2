cmake_minimum_required(VERSION 3.8)
project(chess_episode_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lekiwi_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

# StorageOptions::custom_data was added in Iron / Jazzy
if(rosbag2_storage_VERSION VERSION_GREATER_EQUAL "0.22.0")
  add_compile_definitions(HAS_ROSBAG2_CUSTOM_DATA=1)
endif()

# Composable component library
add_library(episode_recorder_component SHARED
  src/episode_recorder_node.cpp
)
target_include_directories(episode_recorder_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(episode_recorder_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
  lekiwi_interfaces
  rosbag2_cpp
  rosbag2_storage
  yaml-cpp
)
rclcpp_components_register_nodes(episode_recorder_component
  "chess_episode_recorder::EpisodeRecorderNode"
)

# Standalone executable generated directly from component
rclcpp_components_register_node(episode_recorder_component
  PLUGIN "chess_episode_recorder::EpisodeRecorderNode"
  EXECUTABLE episode_recorder_node
)

install(TARGETS
  episode_recorder_component
  episode_recorder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_smoke test/test_smoke.py)
endif()

ament_package()
