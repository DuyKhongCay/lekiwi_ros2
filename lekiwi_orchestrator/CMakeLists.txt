cmake_minimum_required(VERSION 3.8)
project(lekiwi_orchestrator)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/chess_mission_orchestrator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fsm test/test_fsm.py)
  ament_add_pytest_test(test_chess_mission_orchestrator test/test_chess_mission_orchestrator.py)
  ament_add_pytest_test(test_health_supervisor test/test_health_supervisor.py)
  ament_add_pytest_test(test_move_sequencer test/test_move_sequencer.py)
  ament_add_pytest_test(test_perception_context test/test_perception_context.py)
  ament_add_pytest_test(test_motion_client test/test_motion_client.py)
  ament_add_pytest_test(test_status_marker_builder test/test_status_marker_builder.py)
  ament_add_pytest_test(test_viewpoint_scheduler test/test_viewpoint_scheduler.py)
  ament_add_pytest_test(test_architecture_contracts test/test_architecture_contracts.py)
endif()

ament_package()
