cmake_minimum_required(VERSION 2.8.3)
project(accumulator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	pcl_ros
	roscpp
	sensor_msgs
	visualization_msgs
)

find_package(yaml-cpp REQUIRED)

add_message_files(
	FILES
	AccumulatedPatterns.msg
)

add_service_files(
	FILES
	SendPatterns.srv
	SendString.srv
	SendUInt.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	sensor_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime
)

include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
	${YAML_CPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/node.cpp src/yaml.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
