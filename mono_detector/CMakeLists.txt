cmake_minimum_required(VERSION 2.8.3)
project(mono_detector)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	camera_calibration_parsers
	cv_bridge
	image_geometry
	pcl_ros
	roscpp
	sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS}
	LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_ros ${OpenCV_LIBS}
	DEPENDS cv_bridge
)

include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${YAML_CPP_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

# Library without any ros dependencies
add_library(${PROJECT_NAME} src/lib/detector.cpp src/lib/pnp.cpp src/lib/yaml.cpp src/lib/util.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Eigen_LIBRARIES})

# Command line interface for the library (no ros)
add_executable(${PROJECT_NAME}_cli src/cli.cpp)
target_link_libraries(${PROJECT_NAME}_cli ${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

# Library containing a ros wrapper around the detection library
add_library(${PROJECT_NAME}_ros src/lib/node_lib.cpp)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Ros node, depending on the ros detection library
add_executable(${PROJECT_NAME}_node src/node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# Add tests
catkin_add_gtest(${PROJECT_NAME}_yaml_test test/yaml.cpp)
target_link_libraries(${PROJECT_NAME}_yaml_test ${PROJECT_NAME})
