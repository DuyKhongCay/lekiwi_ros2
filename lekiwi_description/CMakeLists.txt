cmake_minimum_required(VERSION 3.10)
project(lekiwi_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf assets config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
