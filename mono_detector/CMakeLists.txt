cmake_minimum_required(VERSION 3.5)
project(mono_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)

set(deps
  rclcpp
  cv_bridge
  image_geometry
  pcl_ros
  sensor_msgs
  yaml-cpp)

include_directories(include)

# Library without any ros dependencies
add_library(${PROJECT_NAME}_lib SHARED src/lib/detector.cpp src/lib/pnp.cpp src/lib/yaml.cpp src/lib/util.cpp)
ament_target_dependencies(${PROJECT_NAME}_lib ${deps})

# Library containing a ros wrapper around the detection library
add_library(${PROJECT_NAME}_ros src/lib/node_lib.cpp)
ament_target_dependencies(${PROJECT_NAME}_ros ${deps})
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME}_lib ${YAML_CPP_LIBRARIES})

# Ros node, depending on the ros detection library
add_executable(${PROJECT_NAME}_node src/node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${deps})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros)

install(TARGETS
  ${PROJECT_NAME}_lib ${PROJECT_NAME}_ros
  DESTINATION lib)
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_yaml_test test/yaml.cpp)
  ament_target_dependencies(${PROJECT_NAME}_yaml_test ${deps})
  target_link_libraries(${PROJECT_NAME}_yaml_test ${PROJECT_NAME}_ros)
endif()

ament_package()
