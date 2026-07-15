cmake_minimum_required(VERSION 3.10)
project(imx219_stereo_camera)

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
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)

# Find libcamera via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

# Find GStreamer libraries (built from source and installed into conda prefix)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-video-1.0
  gstreamer-app-1.0
)

# Build Components Library (includes libcamera, OpenCV, and GStreamer camera nodes)
add_library(imx219_stereo_camera_components SHARED
  src/base_camera_node.cpp
  src/imx219_stereo_camera.cpp
  src/gstreamer_imx219_camera.cpp
)

target_include_directories(imx219_stereo_camera_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(imx219_stereo_camera_components
  ${OpenCV_LIBRARIES}
  PkgConfig::LIBCAMERA
  PkgConfig::GSTREAMER
)

# ROS2 Dependencies for Component Library
ament_target_dependencies(imx219_stereo_camera_components
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

rclcpp_components_register_node(imx219_stereo_camera_components
  PLUGIN "lekiwi_cameras::IMX219StereoCamera"
  EXECUTABLE imx219_stereo_camera
)

rclcpp_components_register_node(imx219_stereo_camera_components
  PLUGIN "lekiwi_cameras::GStreamerIMX219Camera"
  EXECUTABLE gstreamer_imx219_camera
)

# Install targets
install(TARGETS imx219_stereo_camera_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS imx219_stereo_camera gstreamer_imx219_camera
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
