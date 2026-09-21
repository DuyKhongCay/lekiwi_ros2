cmake_minimum_required(VERSION 3.8)
project(lekiwi_manipulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/lerobot_arm_bridge
    scripts/mock_policy_server
    scripts/smolvla_policy_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_trajectory_generator test/test_trajectory_generator.py)
  ament_add_pytest_test(test_lerobot_arm_bridge test/test_lerobot_arm_bridge.py)
  ament_add_pytest_test(test_mock_policy_server test/test_mock_policy_server.py)
endif()

ament_package()

