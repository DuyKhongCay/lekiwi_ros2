cmake_minimum_required(VERSION 3.8)
project(lekiwi_manipulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/mock_policy_server
    scripts/cartesian_service_node
    scripts/manipulation_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mock_policy_server test/test_mock_policy_server.py)
  ament_add_pytest_test(test_arm_constants test/test_arm_constants.py)
  ament_add_pytest_test(test_trajectory_planner test/test_trajectory_planner.py)
endif()

ament_package()

