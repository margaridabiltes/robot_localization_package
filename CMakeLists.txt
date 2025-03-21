cmake_minimum_required(VERSION 3.5)
project(robot_localization_package)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Besides the package specific dependencies we also need the `pluginlib` and `webots_ros2_driver`
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


## Fake feature extractor
include_directories(
  include
)


add_executable(particle_filter
  src/particle_filter.cpp
)
target_include_directories(particle_filter PRIVATE include)
ament_target_dependencies(particle_filter
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  nav_msgs
  tf2_geometry_msgs
  tf2
  visualization_msgs
  Eigen3
)
install(TARGETS
  particle_filter
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(fake_feature_extractor
  src/fake_feature_extractor.cpp
)
ament_target_dependencies(fake_feature_extractor
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)
install(TARGETS
  fake_feature_extractor
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

## MyRobotDriver 
# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver my_robot_driver.xml)


# MyRobotDriver library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/MyRobotDriver.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
  nav_msgs
)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install additional directories.
install(DIRECTORY
  launch
  resource
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}/
)


ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()