cmake_minimum_required(VERSION 2.8.3)
project(common)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(INCLUDE_DIRS
    include
    DEPENDS
    LIBRARIES ${PROJECT_NAME}
)
include_directories(
    include/${PROJECT_NAME}
 )

add_library(${PROJECT_NAME} src/robust_least_squares.cpp)
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})
