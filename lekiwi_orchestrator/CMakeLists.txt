cmake_minimum_required(VERSION 3.8)
project(lekiwi_orchestrator)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/chess_mission_orchestrator
    scripts/task_orchestrator
    scripts/tf_gatekeeper
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fsm test/test_fsm.py)
  ament_add_pytest_test(test_chessboard_mapper test/test_chessboard_mapper.py)
  ament_add_pytest_test(test_tf_gatekeeper test/test_tf_gatekeeper.py)
  ament_add_pytest_test(test_task_orchestrator test/test_task_orchestrator.py)
  ament_add_pytest_test(test_chess_mission_orchestrator test/test_chess_mission_orchestrator.py)
endif()

ament_package()

