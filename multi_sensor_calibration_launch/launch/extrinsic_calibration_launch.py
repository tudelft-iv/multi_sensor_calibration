import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    detectors = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('multi_sensor_calibration_launch'), 'launch'),
         '/detectors_launch.py'])
      )

    accumulator = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('multi_sensor_calibration_launch'), 'launch'),
         '/accumulator_launch.py'])
      )

    optimizer = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('multi_sensor_calibration_launch'), 'launch'),
         '/optimizer_launch.py'])
    )

    return LaunchDescription([
        detectors,
        accumulator,
        optimizer,
    ])
