cmake_minimum_required(VERSION 3.5)
project(accumulator)

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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(calibration_interfaces REQUIRED)

set(deps
  pcl_ros
  rclcpp
  sensor_msgs
  visualization_msgs
  std_msgs
  std_srvs
  calibration_interfaces
)

# include_directories(
#   include/${PROJECT_NAME}
#   ${catkin_INCLUDE_DIRS}
#   ${YAML_CPP_INCLUDE_DIRS}
# )
include_directories(include)

add_executable(${PROJECT_NAME}_node src/node.cpp src/yaml.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${deps})
target_link_libraries(${PROJECT_NAME}_node ${YAML_CPP_LIBRARIES})
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Disable auto copyright
  set(ament_cmake_copyright_FOUND TRUE)
  # Disable auto uncrustify
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
