cmake_minimum_required(VERSION 3.8)
project(ridgeback_leader_follow)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ridgeback_leader_follow/leader_identity_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME leader_identity)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_lock_on test/test_lock_on.py)
  ament_add_pytest_test(test_camera_people test/test_camera_people.py)
  ament_add_pytest_test(test_leader_identity test/test_leader_identity.py)
endif()

ament_package()
