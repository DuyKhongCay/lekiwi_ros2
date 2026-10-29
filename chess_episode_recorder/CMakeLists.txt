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
find_package(rclcpp_action REQUIRED)
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

# 1. Pure C++17 Domain Library (Quota & Planning)
add_library(chess_episode_recorder_domain STATIC
  src/campaign_quota_domain.cpp
  src/move_planning_domain.cpp
)
set_target_properties(chess_episode_recorder_domain PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(chess_episode_recorder_domain PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${chess_library_SOURCE_DIR}/include
)

# 2. Storage & MCAP Recording Library
add_library(chess_episode_recorder_storage STATIC
  src/dataset_storage.cpp
)
set_target_properties(chess_episode_recorder_storage PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(chess_episode_recorder_storage PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${chess_library_SOURCE_DIR}/include
)
target_link_libraries(chess_episode_recorder_storage
  chess_episode_recorder_domain
  yaml-cpp
)
ament_target_dependencies(chess_episode_recorder_storage
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  yaml-cpp
)

# 3. Campaign Conductor FSM & Diagnostics Library
add_library(chess_episode_recorder_conductor STATIC
  src/campaign_conductor.cpp
)
set_target_properties(chess_episode_recorder_conductor PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(chess_episode_recorder_conductor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${chess_library_SOURCE_DIR}/include
)
target_link_libraries(chess_episode_recorder_conductor
  chess_episode_recorder_domain
  chess_episode_recorder_storage
)
ament_target_dependencies(chess_episode_recorder_conductor
  rclcpp
  diagnostic_updater
)

# 4. Composable Component Library
add_library(episode_recorder_component SHARED
  src/episode_recorder_node.cpp
)
target_include_directories(episode_recorder_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${chess_library_SOURCE_DIR}/include
)
target_link_libraries(episode_recorder_component
  chess_episode_recorder_conductor
  chess_episode_recorder_storage
  chess_episode_recorder_domain
)
ament_target_dependencies(episode_recorder_component
  rclcpp
  rclcpp_action
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
  chess_episode_recorder_domain
  chess_episode_recorder_storage
  chess_episode_recorder_conductor
  episode_recorder_component
  episode_recorder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 1. Campaign Quota Domain Unit Tests
  ament_add_gtest(test_campaign_quota_domain test/test_campaign_quota_domain.cpp)
  if(TARGET test_campaign_quota_domain)
    target_include_directories(test_campaign_quota_domain PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${chess_library_SOURCE_DIR}/include
    )
    target_link_libraries(test_campaign_quota_domain
      chess_episode_recorder_domain
    )
  endif()

  # 2. Move Planning Domain Unit Tests
  ament_add_gtest(test_move_planning_domain test/test_move_planning_domain.cpp)
  if(TARGET test_move_planning_domain)
    target_include_directories(test_move_planning_domain PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${chess_library_SOURCE_DIR}/include
    )
    target_link_libraries(test_move_planning_domain
      chess_episode_recorder_domain
    )
  endif()

  # 3. Dataset Storage Unit Tests
  ament_add_gtest(test_dataset_storage test/test_dataset_storage.cpp)
  if(TARGET test_dataset_storage)
    target_include_directories(test_dataset_storage PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${chess_library_SOURCE_DIR}/include
    )
    target_link_libraries(test_dataset_storage
      chess_episode_recorder_storage
      chess_episode_recorder_domain
      yaml-cpp
    )
    ament_target_dependencies(test_dataset_storage
      rclcpp
      rosbag2_cpp
      rosbag2_storage
    )
  endif()

  # 4. Campaign Conductor Unit Tests
  ament_add_gtest(test_campaign_conductor test/test_campaign_conductor.cpp)
  if(TARGET test_campaign_conductor)
    target_include_directories(test_campaign_conductor PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${chess_library_SOURCE_DIR}/include
    )
    target_link_libraries(test_campaign_conductor
      chess_episode_recorder_conductor
      chess_episode_recorder_storage
      chess_episode_recorder_domain
    )
    ament_target_dependencies(test_campaign_conductor
      rclcpp
      diagnostic_updater
    )
  endif()

  # 5. EpisodeRecorderNode Integration Tests
  ament_add_gtest(test_episode_recorder_node test/test_episode_recorder_node.cpp)
  if(TARGET test_episode_recorder_node)
    target_include_directories(test_episode_recorder_node PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${chess_library_SOURCE_DIR}/include
    )
    target_link_libraries(test_episode_recorder_node
      episode_recorder_component
    )
    ament_target_dependencies(test_episode_recorder_node
      rclcpp
      rclcpp_action
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
