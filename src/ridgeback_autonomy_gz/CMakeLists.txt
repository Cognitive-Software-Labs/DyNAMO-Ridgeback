cmake_minimum_required(VERSION 3.8)
project(ridgeback_autonomy_gz)

find_package(ament_cmake REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick)

set(CMAKE_AUTOMOC ON)
cmake_policy(SET CMP0100 NEW)

qt5_add_resources(SPAWN_G1_RSC sim/gz_plugins/SpawnG1.qrc)
add_library(SpawnG1 SHARED
  sim/gz_plugins/SpawnG1.cc
  sim/gz_plugins/SpawnG1.hh
  ${SPAWN_G1_RSC}
)
target_link_libraries(SpawnG1
  gz-gui8::gz-gui8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
  Qt5::Core
  Qt5::Quick
)
target_include_directories(SpawnG1 PRIVATE sim/gz_plugins)

install(TARGETS SpawnG1 LIBRARY DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch sim DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_benchmark_assets test/test_benchmark_assets.py)
  ament_add_pytest_test(test_gazebo_launch test/test_gazebo_launch.py)
  # Starts a real gzserver on a physics-only world.
  ament_add_pytest_test(test_scene_control_provider test/test_scene_control_provider.py
    TIMEOUT 180)
endif()

ament_package()
