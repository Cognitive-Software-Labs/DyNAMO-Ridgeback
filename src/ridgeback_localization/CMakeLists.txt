cmake_minimum_required(VERSION 3.8)
project(ridgeback_localization)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  ridgeback_localization/detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_detector_node
)
install(PROGRAMS
  ridgeback_localization/pointcloud_measurement_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_pointcloud_measurement_node
)
install(PROGRAMS
  ridgeback_localization/overlay_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_overlay_node
)
install(PROGRAMS
  ridgeback_localization/mask_measurement_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_mask_measurement_node
)
install(PROGRAMS
  ridgeback_localization/visualization_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_visualization_node
)

ament_package()
