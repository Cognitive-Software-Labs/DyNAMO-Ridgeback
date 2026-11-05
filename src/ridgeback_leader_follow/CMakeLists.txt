cmake_minimum_required(VERSION 3.8)
project(ridgeback_leader_follow)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(gz-sim8 QUIET)
if(gz-sim8_FOUND)
  find_package(gz-transport13 REQUIRED)
  find_package(gz-plugin2 REQUIRED COMPONENTS register)
  add_library(CrowdMotion SHARED sim/CrowdMotion.cc)
  target_compile_features(CrowdMotion PRIVATE cxx_std_17)
  target_link_libraries(CrowdMotion gz-sim8::gz-sim8 gz-transport13::gz-transport13
    gz-plugin2::gz-plugin2)
  install(TARGETS CrowdMotion LIBRARY DESTINATION lib/${PROJECT_NAME})
endif()
install(PROGRAMS ridgeback_leader_follow/leader_identity_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME leader_identity)
install(PROGRAMS ridgeback_leader_follow/crowd_runner.py
  DESTINATION lib/${PROJECT_NAME} RENAME crowd_scene)
install(PROGRAMS ridgeback_leader_follow/crowd_qualify.py
  DESTINATION lib/${PROJECT_NAME} RENAME qualify_crowd)
install(PROGRAMS ridgeback_leader_follow/benchmarking/evaluate.py
  DESTINATION lib/${PROJECT_NAME} RENAME evaluate_people)
install(PROGRAMS ridgeback_leader_follow/benchmarking/replay.py
  DESTINATION lib/${PROJECT_NAME} RENAME replay_people)
install(PROGRAMS ridgeback_leader_follow/people_tracker_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME people_tracker)
install(PROGRAMS ridgeback_leader_follow/benchmarking/corpus.py
  DESTINATION lib/${PROJECT_NAME} RENAME record_people_corpus)
install(PROGRAMS ridgeback_leader_follow/benchmarking/sweep.py
  DESTINATION lib/${PROJECT_NAME} RENAME benchmark_people)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY sim benchmark_native DESTINATION share/${PROJECT_NAME})
install(FILES ridgeback_leader_follow/benchmarking/candidate-sources.json
  ridgeback_leader_follow/benchmarking/requirements-camera.txt
  ridgeback_leader_follow/benchmarking/requirements-tracking.txt
  DESTINATION share/${PROJECT_NAME}/benchmarking)
install(DIRECTORY ridgeback_leader_follow/benchmarking/patches
  DESTINATION share/${PROJECT_NAME}/benchmarking)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_realistic_scene test/test_realistic_scene.py)
  ament_add_pytest_test(test_stationary_anchor test/test_stationary_anchor.py)
  ament_add_pytest_test(test_recognition_scenarios test/test_recognition_scenarios.py)
  ament_add_pytest_test(test_lock_metrics test/test_lock_metrics.py)
  ament_add_pytest_test(test_crowd_appearance test/test_crowd_appearance.py)
  ament_add_pytest_test(test_reid test/test_reid.py)
  ament_add_pytest_test(test_appearance test/test_appearance.py)
  ament_add_pytest_test(test_lab_scene test/test_lab_scene.py)
  ament_add_pytest_test(test_image_association test/test_image_association.py)
  ament_add_pytest_test(test_people_tracker_node test/test_people_tracker_node.py)
  ament_add_pytest_test(test_image_metrics test/test_image_metrics.py)
  ament_add_pytest_test(test_tracking_metrics test/test_tracking_metrics.py)
  ament_add_pytest_test(test_fusion test/test_fusion.py)
  ament_add_pytest_test(test_lock_on test/test_lock_on.py)
  ament_add_pytest_test(test_camera_people test/test_camera_people.py)
  ament_add_pytest_test(test_leader_identity test/test_leader_identity.py)
  ament_add_pytest_test(test_crowd_scenarios test/test_crowd_scenarios.py)
  ament_add_pytest_test(test_people_benchmark test/test_people_benchmark.py)
endif()

ament_package()
