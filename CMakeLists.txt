cmake_minimum_required(VERSION 3.5)
project(robot_localization_package)

# Set C++ standard if not already specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ---------------- Dependencies ----------------
# ROS 2 core dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ROS 2 TF dependencies
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Third-party libraries
find_package(yaml-cpp REQUIRED)

# Custom messages
find_package(robot_msgs REQUIRED)

# ---------------- Particle Filter Node ----------------
add_executable(particle_filter
  src/particle_filter.cpp
  src/MapLoader.cpp
)

# Include directories for particle_filter
target_include_directories(particle_filter PRIVATE include)

# Link dependencies for particle_filter
ament_target_dependencies(particle_filter
  rclcpp
  geometry_msgs
  tf2_ros
  nav_msgs
  tf2_geometry_msgs
  tf2
  visualization_msgs
  robot_msgs
)

target_link_libraries(particle_filter
  yaml-cpp
)

# ---------------- Installation ----------------
# Install the particle_filter executable
install(TARGETS
  particle_filter
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional directories (e.g., launch files, RViz configs)
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ---------------- Finalize Package ----------------
ament_package()