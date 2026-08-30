cmake_minimum_required(VERSION 3.10)
project(lekiwi_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES udev/99-lekiwi.rules
  DESTINATION share/${PROJECT_NAME}/udev
)

ament_package()
