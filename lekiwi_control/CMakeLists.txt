cmake_minimum_required(VERSION 3.8)
project(lekiwi_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/workspace_checker
    scripts/torque_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_workspace_checker test/test_workspace_checker.py)
  ament_add_pytest_test(test_torque_manager test/test_torque_manager.py)
endif()

ament_package()
