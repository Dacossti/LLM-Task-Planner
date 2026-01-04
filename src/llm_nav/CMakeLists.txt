cmake_minimum_required(VERSION 3.5)
project(llm_nav)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav2_simple_commander REQUIRED)

# Install Python package
install(
  DIRECTORY llm_nav
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install Python packaging config files
install(
  FILES
    setup.py
    setup.cfg
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install supporting resources
install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/maps
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/params
)

install(
  DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(
  DIRECTORY test
  DESTINATION lib/${PROJECT_NAME}/test
)

# Install data folder inside Python package
install(
  DIRECTORY llm_nav/data
  DESTINATION lib/${PROJECT_NAME}/llm_nav
)

# Register as ament package
ament_package()
