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
  # Core layer tests
  ament_add_pytest_test(test_fsm test/core/test_fsm.py)

  # Planning layer tests
  ament_add_pytest_test(test_viewpoint_scheduler test/planning/test_viewpoint_scheduler.py)

  # Execution layer tests
  ament_add_pytest_test(test_motion_client test/execution/test_motion_client.py)
  ament_add_pytest_test(test_move_sequencer test/execution/test_move_sequencer.py)

  # Supervision layer tests
  ament_add_pytest_test(test_chess_mission_orchestrator test/supervision/test_chess_mission_orchestrator.py)
  ament_add_pytest_test(test_health_supervisor test/supervision/test_health_supervisor.py)
  ament_add_pytest_test(test_perception_context test/supervision/test_perception_context.py)
  ament_add_pytest_test(test_status_marker_builder test/supervision/test_status_marker_builder.py)
endif()

ament_package()
