cmake_minimum_required(VERSION 3.10)
project(lekiwi_tag_localization)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d imgproc)

add_library(lekiwi_tag_localization SHARED
  src/chessboard_pose_estimator.cpp
)
target_include_directories(lekiwi_tag_localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(lekiwi_tag_localization
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(lekiwi_tag_localization
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  apriltag_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_geometry
)

rclcpp_components_register_node(lekiwi_tag_localization
  PLUGIN "lekiwi_tag_localization::ChessboardPoseEstimator"
  EXECUTABLE chessboard_pose_estimator_node
)

install(TARGETS
  lekiwi_tag_localization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  chessboard_pose_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_solver test/test_pose_solver.cpp)
  if(TARGET test_pose_solver)
    target_link_libraries(test_pose_solver lekiwi_tag_localization ${OpenCV_LIBRARIES})
    ament_target_dependencies(test_pose_solver
      rclcpp
      sensor_msgs
      geometry_msgs
      std_msgs
      apriltag_msgs
      tf2
      tf2_ros
      tf2_geometry_msgs
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(lekiwi_tag_localization)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  apriltag_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
ament_package()

