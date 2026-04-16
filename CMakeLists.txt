cmake_minimum_required(VERSION 3.22)

# C must be included because of inclusion of MPI
project(lidar_odometry_node
  VERSION 0.5.0
  LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------
# ROS2
# ------------------------------

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# ------------------------------
# PCL (minimal components)
# ------------------------------
find_package(MPI REQUIRED COMPONENTS C CXX)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common
  filters
  registration
)

# ============================
# Include directories
# ============================
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# ------------------------------
# Executable
# ------------------------------

add_executable(lidar_odometry_node
  src/lidar_odometry_node.cpp
  include/lidar_odometry_node.h
  config/lidar_odometry_node.yaml
  src/lidar_base_imu.cpp src/odom_TFs_publishers.cpp
)

target_include_directories(lidar_odometry_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

# Link only needed PCL modules
target_link_libraries(lidar_odometry_node
  pcl_common
  pcl_filters
  pcl_registration
)

ament_target_dependencies(lidar_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
)

# ------------------------------
# Install
# ------------------------------

install(TARGETS
  lidar_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
