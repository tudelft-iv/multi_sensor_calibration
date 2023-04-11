cmake_minimum_required(VERSION 3.5)
project(multi_sensor_calibration_launch)

find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
