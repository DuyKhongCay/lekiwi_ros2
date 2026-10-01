cmake_minimum_required(VERSION 3.8)
project(lekiwi_episode_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

# StorageOptions::custom_data was added in Iron / Jazzy
if(rosbag2_storage_VERSION VERSION_GREATER_EQUAL "0.22.0")
  add_compile_definitions(HAS_ROSBAG2_CUSTOM_DATA=1)
endif()

# Core Episode Recorder Node
add_executable(episode_recorder_node
  src/gamepad_controller.cpp
  src/episode_recorder_node.cpp
  src/episode_recorder_main.cpp
)
ament_target_dependencies(episode_recorder_node
  rclcpp
  sensor_msgs
  std_srvs
  rosbag2_cpp
  rosbag2_storage
  yaml-cpp
)

install(TARGETS
  episode_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
