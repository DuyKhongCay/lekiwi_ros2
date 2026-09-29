cmake_minimum_required(VERSION 3.10)
project(lekiwi_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf assets config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/chessboard_3d_visualizer.py
  scripts/recenter_chess_pieces.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
