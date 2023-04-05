cmake_minimum_required(VERSION 3.5)
project(lidar_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS visualization)
find_package(yaml-cpp REQUIRED)
find_package(calibration_common REQUIRED)
find_package(Eigen3 REQUIRED)

set(deps
  rclcpp
  pcl_ros
  sensor_msgs
  visualization_msgs
  Eigen3
  PCL
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# include_directories(
# 	include/${PROJECT_NAME}
# 	${catkin_INCLUDE_DIRS}
# 	${PCL_INCLUDE_DIRS}
# 	${YAML_CPP_INCLUDE_DIRS}
# )

# link_directories(
# 	${PCL_LIBRARY_DIRS}
# )

# add_definitions(
# 	${PCL_DEFINITIONS}
# )

# Yaml parsing library for custom lidar detector types
add_library(${PROJECT_NAME}_yaml src/lib/yaml.cpp)
target_link_libraries(${PROJECT_NAME}_yaml ${YAML_CPP_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_yaml ${deps})

# Library without any ros dependencies
add_library(${PROJECT_NAME} src/lib/keypoint_detection.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_yaml)
ament_target_dependencies(${PROJECT_NAME} ${deps} calibration_common)
# target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_yaml ${PCL_LIBRARIES})

# Library containing a ros wrapper around the detection library
add_library(${PROJECT_NAME}_ros src/lib/node_lib.cpp)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_ros ${deps})

# Ros node, depending on the ros detection library
add_executable(${PROJECT_NAME}_node src/node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${deps} calibration_common)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros ${PCL_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}_yaml ${PROJECT_NAME} ${PROJECT_NAME}_ros
  DESTINATION lib
)
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Disable auto copyright
  set(ament_cmake_copyright_FOUND TRUE)
  # Disable auto uncrustify
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
