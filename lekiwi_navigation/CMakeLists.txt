cmake_minimum_required(VERSION 3.10)
project(lekiwi_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
