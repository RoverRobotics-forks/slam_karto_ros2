cmake_minimum_required(VERSION 3.5)
project(slam_karto)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(open_karto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sparse_bundle_adjustment REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread) # < this should be pulled in from open_karto. Not sure why it's not

add_executable(slam_karto src/slam_karto.cpp src/spa_solver.cpp)
ament_target_dependencies(slam_karto
  Boost # < this should be pulled in from open_karto. Not sure why it's not
  Eigen3
  open_karto
  rclcpp
  nav_msgs
  sensor_msgs
  sparse_bundle_adjustment
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  )
#target_link_libraries(slam_karto sparse_bundle_adjustment::sba)
install(
  TARGETS slam_karto
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

ament_package()
