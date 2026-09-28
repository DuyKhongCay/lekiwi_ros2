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

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fsm test/test_fsm.py)
  ament_add_pytest_test(test_chess_mission_orchestrator test/test_chess_mission_orchestrator.py)
  ament_add_pytest_test(test_health_monitor test/test_health_monitor.py)
  ament_add_pytest_test(test_move_pipeline test/test_move_pipeline.py)
  ament_add_pytest_test(test_perception_manager test/test_perception_manager.py)
  ament_add_pytest_test(test_motion_dispatcher test/test_motion_dispatcher.py)
  ament_add_pytest_test(test_visualizer test/test_visualizer.py)
endif()

ament_package()
