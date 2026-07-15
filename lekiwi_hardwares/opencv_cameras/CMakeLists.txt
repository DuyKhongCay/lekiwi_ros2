cmake_minimum_required(VERSION 3.10)
project(opencv_cameras)

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

# Build Component Library
add_library(opencv_camera_component SHARED
  src/opencv_camera.cpp
)

target_include_directories(opencv_camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(opencv_camera_component
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(opencv_camera_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

# Register Component & Build Standalone Executable
rclcpp_components_register_node(opencv_camera_component
  PLUGIN "opencv_cameras::OpenCVCamera"
  EXECUTABLE opencv_camera
)

# Install targets
install(TARGETS opencv_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS opencv_camera
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
