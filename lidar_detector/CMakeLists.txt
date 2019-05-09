cmake_minimum_required(VERSION 2.8.3)
project(lidar_detector)

find_package(catkin REQUIRED COMPONENTS
	pcl_ros
	roscpp
	sensor_msgs
	visualization_msgs
	common
)

find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
	INCLUDE_DIRS include
)

include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${YAML_CPP_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(
	${PCL_DEFINITIONS}
)

# Yaml parsing library for custom lidar detector types
add_library(${PROJECT_NAME}_yaml src/lib/yaml.cpp)
target_link_libraries(${PROJECT_NAME}_yaml ${YAML_CPP_LIBRARIES})

# Library without any ros dependencies
add_library(${PROJECT_NAME} src/lib/keypoint_detection.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_yaml ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Command line interface for the library (no ros)
add_executable(${PROJECT_NAME}_cli src/cli.cpp)
target_link_libraries(${PROJECT_NAME}_cli ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Library containing a ros wrapper around the detection library
add_library(${PROJECT_NAME}_ros src/lib/node_lib.cpp)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Ros node, depending on the ros detection library
add_executable(${PROJECT_NAME}_node src/node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES})

