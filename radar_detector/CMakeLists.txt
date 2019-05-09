cmake_minimum_required(VERSION 2.8.3)
project(radar_detector)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
	cmake_modules
	pcl_conversions
	pcl_ros
	roscpp
	sensor_msgs
	visualization_msgs
	radar_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
)

include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

# Library without any ros dependencies
add_library(${PROJECT_NAME} src/lib/keypoint_detection.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES})

# Library containing a ros wrapper around the detection library
add_library(${PROJECT_NAME}_ros src/lib/node_lib.cpp)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Ros node, depending on the ros detection library
add_executable(${PROJECT_NAME}_node src/node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
