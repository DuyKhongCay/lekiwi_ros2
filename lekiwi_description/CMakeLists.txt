cmake_minimum_required(VERSION 3.10)
project(lekiwi_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf assets launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
