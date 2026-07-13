cmake_minimum_required(VERSION 3.10)
project(lekiwi_cameras)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)

# Find libcamera via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

# Add Executables
add_executable(imx219_stereo_camera_node
  src/imx219_stereo_camera_node.cpp
)

target_include_directories(imx219_stereo_camera_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(imx219_stereo_camera_node
  ${OpenCV_LIBRARIES}
  PkgConfig::LIBCAMERA
)

ament_target_dependencies(imx219_stereo_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

add_executable(opencv_camera_node
  src/opencv_camera_node.cpp
)

target_include_directories(opencv_camera_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(opencv_camera_node
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(opencv_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

# Install rules
install(TARGETS 
  imx219_stereo_camera_node
  opencv_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
