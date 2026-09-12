cmake_minimum_required(VERSION 3.8)
project(ridgeback_autonomy_isaac)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch sim DESTINATION share/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
