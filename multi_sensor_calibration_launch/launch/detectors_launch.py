from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_detector',
            executable='lidar_detector_node',
            name='lidar_detector',
        ),
        Node(
            package='stereo_detector',
            executable='stereo_detector_node',
            name='stereo_detector',
        ),
        Node(
            package='radar_detector',
            executable='radar_detector_node',
            name='radar_detector',
            parameters=[{
		'minimum_RCS': -10.0,
		'maximum_RCS': 15.0,
		'max_range_object': 5.0,
		'selection_basis': 'range',
		'selection_criterion': 'min',
	    }]
        ),
    ])
