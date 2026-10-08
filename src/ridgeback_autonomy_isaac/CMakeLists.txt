cmake_minimum_required(VERSION 3.8)
project(ridgeback_autonomy_isaac)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch sim DESTINATION share/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_benchmark_assets test/test_benchmark_assets.py)
  ament_add_pytest_test(test_scene_command_queue test/test_scene_command_queue.py)
  ament_add_pytest_test(test_scene_provider test/test_scene_provider.py)
  ament_add_pytest_test(test_scene_control_service test/test_scene_control_service.py
    TIMEOUT 120)
  # Skips without pxr; the Isaac interpreter provides it without Kit.
  ament_add_pytest_test(test_usd_scene test/test_usd_scene.py)
endif()

ament_package()
