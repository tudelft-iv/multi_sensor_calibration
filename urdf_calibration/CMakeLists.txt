cmake_minimum_required(VERSION 2.8.3)
project(urdf_calibration)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
	kdl_parser
	urdf
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
	${YAML_CPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/urdf_calibration.cpp
)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_cli
	src/main.cpp
	src/yaml.cpp
)
target_link_libraries(${PROJECT_NAME}_cli
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	${YAML_CPP_LIBRARIES}
)


