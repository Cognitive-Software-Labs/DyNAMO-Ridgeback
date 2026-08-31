cmake_minimum_required(VERSION 3.8)
project(ridgeback_autonomy)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetDetections.msg"
  "msg/TargetMeasurements.msg"
  DEPENDENCIES std_msgs
)

# Qt for MOC
find_package(Qt5 REQUIRED COMPONENTS Core Quick)
set(CMAKE_AUTOMOC ON)
cmake_policy(SET CMP0100 NEW)

# SpawnG1 Gazebo GUI plugin
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

install(TARGETS SpawnG1
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

ament_get_python_install_dir(python_install_dir)

install(DIRECTORY
  ridgeback_autonomy/
  DESTINATION ${python_install_dir}/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

install(PROGRAMS
  ridgeback_autonomy/perception/target_localization/detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_detector_node
)

install(PROGRAMS
  ridgeback_autonomy/perception/target_localization/pointcloud_measurement_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_pointcloud_measurement_node
)

install(PROGRAMS
  ridgeback_autonomy/perception/target_localization/overlay_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_overlay_node
)

install(PROGRAMS
  ridgeback_autonomy/perception/target_localization/mask_measurement_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_mask_measurement_node
)

install(PROGRAMS
  ridgeback_autonomy/benchmarking/target_distance_benchmark_runner_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_distance_benchmark_runner
)

install(PROGRAMS
  ridgeback_autonomy/perception/target_localization/visualization_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_visualization_node
)

install(PROGRAMS
  ridgeback_autonomy/benchmarking/target_benchmark_sweep.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_benchmark_sweep
)

install(PROGRAMS
  ridgeback_autonomy/frontier_explorer/frontier_explorer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME frontier_explorer_node.py
)

install(PROGRAMS
  ridgeback_autonomy/diagnostics/velocity_overlay_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME velocity_overlay_node
)

install(PROGRAMS
  ridgeback_autonomy/common/launch_wait.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME launch_wait
)

install(PROGRAMS
  ridgeback_autonomy/diagnostics/hud_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hud_node
)

install(PROGRAMS
  ridgeback_autonomy/diagnostics/coverage_overlay_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME coverage_overlay_node
)

install(DIRECTORY
  launch
  config
  sim
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_camera_inputs test/test_camera_inputs.py)
  ament_add_pytest_test(test_artifacts test/test_artifacts.py)
  ament_add_pytest_test(test_depth_common test/test_depth_common.py)
  ament_add_pytest_test(test_depth_sources test/test_depth_sources.py)
  ament_add_pytest_test(test_detection test/test_detection.py)
  ament_add_pytest_test(test_detector_node test/test_detector_node.py)
  ament_add_pytest_test(test_image_utils test/test_image_utils.py)
  ament_add_pytest_test(test_imports test/test_imports.py)
  ament_add_pytest_test(test_intrinsics test/test_intrinsics.py)
  ament_add_pytest_test(test_isolation_2d test/test_isolation_2d.py)
  ament_add_pytest_test(test_isolation_3d test/test_isolation_3d.py)
  ament_add_pytest_test(test_mask test/test_mask.py)
  ament_add_pytest_test(test_mask_region test/test_mask_region.py)
  ament_add_pytest_test(test_mask_region_allocation test/test_mask_region_allocation.py)
  ament_add_pytest_test(test_mask_region_parity test/test_mask_region_parity.py)
  ament_add_pytest_test(test_mask_measurement_node test/test_mask_measurement_node.py)
  ament_add_pytest_test(test_shared_defaults test/test_shared_defaults.py)
  ament_add_pytest_test(test_tf_utils test/test_tf_utils.py)
  ament_add_pytest_test(test_miss_reason test/test_miss_reason.py)
  ament_add_pytest_test(test_launch_layout test/test_launch_layout.py)
  ament_add_pytest_test(test_markers test/test_markers.py)
  ament_add_pytest_test(test_messages test/test_messages.py)
  ament_add_pytest_test(test_projective_ranging test/test_projective_ranging.py)
  ament_add_pytest_test(test_rendering test/test_rendering.py)
  ament_add_pytest_test(test_robot_config test/test_robot_config.py)
  ament_add_pytest_test(test_segmentation test/test_segmentation.py)
  ament_add_pytest_test(test_status_attribution test/test_status_attribution.py)
  ament_add_pytest_test(test_euclidean_reconstruction test/test_euclidean_reconstruction.py)
  ament_add_pytest_test(test_polar_profiling test/test_polar_profiling.py)
  ament_add_pytest_test(test_pointcloud_ranging test/test_pointcloud_ranging.py)
  ament_add_pytest_test(test_vehicle_frame test/test_vehicle_frame.py)
  ament_add_pytest_test(test_benchmark_runner test/test_benchmark_runner.py)
  ament_add_pytest_test(test_scenarios test/test_scenarios.py)
  ament_add_pytest_test(test_association test/test_association.py)
  ament_add_pytest_test(test_scoring test/test_scoring.py)
  ament_add_pytest_test(test_report test/test_report.py)
  ament_add_pytest_test(test_target_visualization test/test_target_visualization.py)
  ament_add_pytest_test(test_recording test/test_recording.py)
  ament_add_pytest_test(test_sweep test/test_sweep.py)
  ament_add_pytest_test(test_sweep_report test/test_sweep_report.py)
  ament_add_pytest_test(test_hud_node test/test_hud_node.py)
endif()

ament_package()
