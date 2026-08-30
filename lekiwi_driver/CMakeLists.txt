cmake_minimum_required(VERSION 3.16)
project(lekiwi_driver)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

add_library(lekiwi_feetech_hardware SHARED
  src/lekiwi_feetech_hardware.cpp
  src/sts_protocol.cpp)
target_compile_features(lekiwi_feetech_hardware PUBLIC cxx_std_20)
target_include_directories(lekiwi_feetech_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(lekiwi_feetech_hardware SYSTEM PUBLIC ${SERIAL_INCLUDE_DIRS})
target_link_libraries(lekiwi_feetech_hardware
  PUBLIC
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    yaml-cpp
    ${SERIAL_LIBRARIES})

pluginlib_export_plugin_description_file(
  hardware_interface lekiwi_feetech_hardware_plugin.xml)

install(
  TARGETS lekiwi_feetech_hardware
  EXPORT lekiwi_driverTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES lekiwi_feetech_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_velocity_codec test/test_velocity_codec.cpp)
  target_compile_features(test_velocity_codec PUBLIC cxx_std_20)
  target_include_directories(test_velocity_codec PRIVATE include)
endif()

ament_export_targets(lekiwi_driverTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle yaml-cpp)
ament_package()
