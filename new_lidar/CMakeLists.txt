cmake_minimum_required(VERSION 3.0.2)
project(new_lidar)

find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_circle_detector
  src/lidar_circle_detector.cpp
)

target_link_libraries(lidar_circle_detector
  ${catkin_LIBRARIES}
  yaml-cpp
)
