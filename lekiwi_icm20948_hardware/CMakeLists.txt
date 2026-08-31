cmake_minimum_required(VERSION 3.16)
project(lekiwi_icm20948_hardware)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(lekiwi_icm20948_hardware SHARED
  src/icm20948_driver.cpp
  src/icm20948_sensor_hardware.cpp)

target_compile_features(lekiwi_icm20948_hardware PUBLIC cxx_std_20)

target_include_directories(lekiwi_icm20948_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(lekiwi_icm20948_hardware
  PUBLIC
    diagnostic_updater::diagnostic_updater
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle)

pluginlib_export_plugin_description_file(
  hardware_interface lekiwi_icm20948_hardware_plugin.xml)

install(
  TARGETS lekiwi_icm20948_hardware
  EXPORT lekiwi_icm20948_hardwareTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(FILES lekiwi_icm20948_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_icm20948_math test/test_icm20948_math.cpp src/icm20948_driver.cpp)
  target_compile_features(test_icm20948_math PUBLIC cxx_std_20)
  target_include_directories(test_icm20948_math PRIVATE include)
endif()

ament_export_targets(lekiwi_icm20948_hardwareTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(diagnostic_updater hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()

