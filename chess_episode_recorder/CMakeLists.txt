cmake_minimum_required(VERSION 3.8)
project(chess_episode_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lekiwi_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(yaml-cpp REQUIRED)

include(FetchContent)
if(EXISTS "/root/docker_ws/chess-library")
  FetchContent_Declare(
    chess_library
    SOURCE_DIR /root/docker_ws/chess-library
  )
else()
  FetchContent_Declare(
    chess_library
    GIT_REPOSITORY https://github.com/Disservin/chess-library.git
    GIT_TAG        master
  )
endif()
FetchContent_MakeAvailable(chess_library)

include_directories(include)

# StorageOptions::custom_data was added in Iron / Jazzy
if(rosbag2_storage_VERSION VERSION_GREATER_EQUAL "0.22.0")
  add_compile_definitions(HAS_ROSBAG2_CUSTOM_DATA=1)
endif()

# Pure Domain Library (Clean Architecture, decoupled from ROS 2)
add_library(collection_planner_domain STATIC
  src/collection_planner_domain.cpp
)
set_target_properties(collection_planner_domain PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(collection_planner_domain PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${chess_library_SOURCE_DIR}/include
)
target_link_libraries(collection_planner_domain
  yaml-cpp
)

# Composable component library
add_library(episode_recorder_component SHARED
  src/episode_recorder_node.cpp
  src/recorder_diagnostics.cpp
)
target_include_directories(episode_recorder_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${chess_library_SOURCE_DIR}/include
)
target_link_libraries(episode_recorder_component
  collection_planner_domain
)
ament_target_dependencies(episode_recorder_component
  rclcpp
  rclcpp_components
  diagnostic_updater
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
  collection_planner_domain
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
  find_package(ament_cmake_gtest REQUIRED)

  # 1. Pure Domain unit tests
  ament_add_gtest(test_collection_planner_domain test/test_collection_planner_domain.cpp)
  if(TARGET test_collection_planner_domain)
    target_include_directories(test_collection_planner_domain PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${chess_library_SOURCE_DIR}/include
    )
    target_link_libraries(test_collection_planner_domain
      collection_planner_domain
      yaml-cpp
    )
  endif()

  # 2. Recorder Diagnostics unit tests
  ament_add_gtest(test_recorder_diagnostics test/test_recorder_diagnostics.cpp)
  if(TARGET test_recorder_diagnostics)
    target_include_directories(test_recorder_diagnostics PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${chess_library_SOURCE_DIR}/include
    )
    target_link_libraries(test_recorder_diagnostics
      episode_recorder_component
      collection_planner_domain
    )
    ament_target_dependencies(test_recorder_diagnostics
      rclcpp
      diagnostic_updater
    )
  endif()

  # 3. EpisodeRecorderNode integration tests
  ament_add_gtest(test_episode_recorder_node test/test_episode_recorder_node.cpp)
  if(TARGET test_episode_recorder_node)
    target_include_directories(test_episode_recorder_node PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${chess_library_SOURCE_DIR}/include
    )
    target_link_libraries(test_episode_recorder_node
      episode_recorder_component
      collection_planner_domain
    )
    ament_target_dependencies(test_episode_recorder_node
      rclcpp
      diagnostic_updater
      sensor_msgs
      std_srvs
      lekiwi_interfaces
      rosbag2_cpp
      rosbag2_storage
      yaml-cpp
    )
  endif()
endif()

ament_package()
