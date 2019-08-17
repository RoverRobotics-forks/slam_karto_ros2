cmake_minimum_required(VERSION 3.5)
project(slam_karto)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#find_package(catkin REQUIRED
#  COMPONENTS
#    cmake_modules
#    message_filters
#    nav_msgs
#    open_karto
#    rosconsole
#    roscpp
#    sensor_msgs
#    sparse_bundle_adjustment
#    tf
#    visualization_msgs
#)
find_package(ament_cmake REQUIRED)
find_package(open_karto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sparse_bundle_adjustment REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(slam_karto src/slam_karto.cpp src/spa_solver.cpp)
ament_target_dependencies(slam_karto
  Eigen3
  rclcpp
  tf2_ros
  open_karto
  sensor_msgs
  visualization_msgs
  sparse_bundle_adjustment
  )

install(
  TARGETS slam_karto
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

ament_package()
