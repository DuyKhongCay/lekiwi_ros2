cmake_minimum_required(VERSION 3.10)
project(lekiwi_chess_master)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lekiwi_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

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

add_library(lekiwi_chess_master_components SHARED
  src/stockfish_driver.cpp
  src/chess_engine_action_component.cpp
  src/chess_game_state_tracker_component.cpp
  src/chessboard_visualizer_component.cpp
)
target_include_directories(lekiwi_chess_master_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${chess_library_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(lekiwi_chess_master_components
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(lekiwi_chess_master_components
  rclcpp
  rclcpp_components
  rclcpp_action
  lekiwi_interfaces
  std_msgs
  std_srvs
  sensor_msgs
  ament_index_cpp
)

rclcpp_components_register_node(lekiwi_chess_master_components
  PLUGIN "lekiwi_chess_master::ChessGameStateTrackerComponent"
  EXECUTABLE chess_game_state_tracker_node
)

rclcpp_components_register_node(lekiwi_chess_master_components
  PLUGIN "lekiwi_chess_master::ChessEngineActionComponent"
  EXECUTABLE chess_engine_action_node
)

rclcpp_components_register_node(lekiwi_chess_master_components
  PLUGIN "lekiwi_chess_master::ChessboardVisualizerComponent"
  EXECUTABLE chessboard_visualizer_node
)

install(TARGETS
  lekiwi_chess_master_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS
  chess_game_state_tracker_node
  chess_engine_action_node
  chessboard_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_stockfish_driver test/test_stockfish_driver.cpp)
  if(TARGET test_stockfish_driver)
    target_link_libraries(test_stockfish_driver lekiwi_chess_master_components)
  endif()

  ament_add_gtest(test_game_state_tracker test/test_game_state_tracker.cpp)
  if(TARGET test_game_state_tracker)
    target_include_directories(test_game_state_tracker PRIVATE ${chess_library_SOURCE_DIR}/include)
    target_link_libraries(test_game_state_tracker lekiwi_chess_master_components)
  endif()

  ament_add_gtest(test_chess_engine_action test/test_chess_engine_action.cpp)
  if(TARGET test_chess_engine_action)
    target_link_libraries(test_chess_engine_action lekiwi_chess_master_components)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(lekiwi_chess_master_components)
ament_export_dependencies(rclcpp rclcpp_components rclcpp_action lekiwi_interfaces std_msgs std_srvs sensor_msgs ament_index_cpp)
ament_package()

