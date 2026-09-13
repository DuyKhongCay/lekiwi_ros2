cmake_minimum_required(VERSION 3.8)
project(lekiwi_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/camera_mode_demo
    scripts/task_orchestrator
    scripts/lerobot_arm_bridge
    scripts/calibrate_omni_base
    scripts/tf_gatekeeper
    scripts/workspace_checker
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fsm test/test_fsm.py)
  ament_add_pytest_test(test_task_orchestrator test/test_task_orchestrator.py)
  ament_add_pytest_test(test_omni_calib test/test_omni_calib.py)
  ament_add_pytest_test(test_tf_gatekeeper test/test_tf_gatekeeper.py)
  ament_add_pytest_test(test_workspace_checker test/test_workspace_checker.py)
endif()

ament_package()
