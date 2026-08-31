cmake_minimum_required(VERSION 3.10)
project(lekiwi_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(lekiwi_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn highgui)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)
pkg_check_modules(HAILO_TAPPAS REQUIRED IMPORTED_TARGET
  hailo-tappas-core
  gsthailometa
)

set(HAILO_EXTRA_INCLUDES
  /usr/include/hailo
  /usr/include/hailo/tappas
)

add_library(lekiwi_chessboard_postprocess MODULE
  src/hailo/chessboard_postprocess.cpp
)
target_include_directories(lekiwi_chessboard_postprocess PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${HAILO_EXTRA_INCLUDES}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(lekiwi_chessboard_postprocess PRIVATE
  PkgConfig::HAILO_TAPPAS
  ${OpenCV_LIBRARIES}
)

add_library(lekiwi_pieces_postprocess MODULE
  src/hailo/yolo_hailortpp.cpp
)
target_include_directories(lekiwi_pieces_postprocess PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${HAILO_EXTRA_INCLUDES}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(lekiwi_pieces_postprocess PRIVATE
  PkgConfig::HAILO_TAPPAS
  ${OpenCV_LIBRARIES}
)

add_library(lekiwi_perception_component SHARED
  src/hailo/hailo_gst_pipeline.cpp
  src/hailo/chess_vision_mapper.cpp
  src/chess_visualizer_component.cpp
  src/hailo_chess_inference_component.cpp
  src/camera_streamer_component.cpp
)
target_include_directories(lekiwi_perception_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  ${HAILO_EXTRA_INCLUDES}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(lekiwi_perception_component
  PkgConfig::GSTREAMER
  PkgConfig::HAILO_TAPPAS
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(lekiwi_perception_component
  ament_index_cpp
  camera_info_manager
  cv_bridge
  diagnostic_updater
  lekiwi_interfaces
  lifecycle_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  vision_msgs
)

rclcpp_components_register_node(lekiwi_perception_component
  PLUGIN "lekiwi_perception::HailoChessInferenceComponent"
  EXECUTABLE hailo_chess_inference_node
)

rclcpp_components_register_node(lekiwi_perception_component
  PLUGIN "lekiwi_perception::ChessVisualizerComponent"
  EXECUTABLE chess_visualizer_node
)

rclcpp_components_register_node(lekiwi_perception_component
  PLUGIN "lekiwi_perception::CameraStreamerComponent"
  EXECUTABLE camera_streamer_node
)

install(TARGETS
  lekiwi_chessboard_postprocess
  lekiwi_pieces_postprocess
  lekiwi_perception_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  hailo_chess_inference_node
  chess_visualizer_node
  camera_streamer_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_hailo_gst_pipeline test/test_hailo_gst_pipeline.cpp)
  if(TARGET test_hailo_gst_pipeline)
    target_link_libraries(test_hailo_gst_pipeline lekiwi_perception_component)
  endif()

  ament_add_gtest(test_camera_streamer_component test/test_camera_streamer_component.cpp)
  if(TARGET test_camera_streamer_component)
    target_link_libraries(test_camera_streamer_component lekiwi_perception_component)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(cv_bridge lekiwi_interfaces sensor_msgs vision_msgs)
ament_package()
