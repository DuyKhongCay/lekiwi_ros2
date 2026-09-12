cmake_minimum_required(VERSION 3.10)
project(apriltag_localizer)

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
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lekiwi_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d imgproc aruco)
find_package(Eigen3 REQUIRED)

add_library(apriltag_localizer SHARED
  src/pose_solver.cpp
  src/chessboard_pose_estimator.cpp
)
target_include_directories(apriltag_localizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(apriltag_localizer
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(apriltag_localizer
  rclcpp
  rclcpp_components
  diagnostic_updater
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  lekiwi_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_geometry
  apriltag_msgs
)

rclcpp_components_register_node(apriltag_localizer
  PLUGIN "apriltag_localizer::ChessboardPoseEstimator"
  EXECUTABLE chessboard_pose_estimator_node
)

install(TARGETS
  apriltag_localizer
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

install(PROGRAMS
  scripts/calibrate_chessboard_tags.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_solver test/test_pose_solver.cpp)
  if(TARGET test_pose_solver)
    target_link_libraries(test_pose_solver apriltag_localizer ${OpenCV_LIBRARIES} Eigen3::Eigen)
    ament_target_dependencies(test_pose_solver
      rclcpp
      diagnostic_updater
      sensor_msgs
      geometry_msgs
      std_msgs
      lekiwi_interfaces
      tf2
      tf2_ros
      tf2_geometry_msgs
      cv_bridge
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(apriltag_localizer)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  diagnostic_updater
  sensor_msgs
  geometry_msgs
  std_msgs
  lekiwi_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  apriltag_msgs
  Eigen3
)
ament_package()


