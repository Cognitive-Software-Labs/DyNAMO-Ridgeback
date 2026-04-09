cmake_minimum_required(VERSION 3.8)
project(ridgeback_slam_exploration)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/G1Detection.msg"
  DEPENDENCIES std_msgs
)

# Qt for MOC
find_package(Qt5 REQUIRED COMPONENTS Core Quick)
set(CMAKE_AUTOMOC ON)
cmake_policy(SET CMP0100 NEW)

# SpawnG1 Gazebo GUI plugin
qt5_add_resources(SPAWN_G1_RSC gz_plugins/SpawnG1.qrc)
add_library(SpawnG1 SHARED
  gz_plugins/SpawnG1.cc
  gz_plugins/SpawnG1.hh
  ${SPAWN_G1_RSC}
)
target_link_libraries(SpawnG1
  gz-gui8::gz-gui8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
  Qt5::Core
  Qt5::Quick
)
target_include_directories(SpawnG1 PRIVATE gz_plugins)

install(TARGETS SpawnG1
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/g1_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
