# DETECTORS
This file contains some additional information on calbration board detectors.

# Lidar detector
The _lidar_detector_ node subscribes to /velodyne_points ROS topic, which is a sensors_msgs::PointCloud2 message. This ROS message contains a point cloud with the Velodyne Point type. This point type additionally contains the ring number of each point. Note: if you do not have a Velodyne Lidar, then you may want to write a converter that adds the ring number.

The _lidar_detector_ publishes two messages:
- _lidar_pattern_: point cloud with the four circle centers
- _lidar_pattern_markers_: visualization marker with the 4 circle centers in order to visualize the detections in RVIZ.

Note:
- In the _lidar_detector_, PCL library constantly throws warnings. These warnings may be ignored if the four circle centers are detected correctly (visulise the detection in RViz).

# Stereo detector
The _stereo_detector_ node subscribes to four ROS topics:
- /ueye/left/image_rect_color: type: sensor_msgs::Image message
- /ueye/left/camera_info:      type: sensor_msgs::CameraInfo message
- /ueye/right/camera_info:     type: sensor_msgs::CameraInfo message
- /ueye/disparity:             type: stereo_msgs::DisparityImage message

The _stereo_detector_ publishes two messages:
- _stereo_pattern_: point cloud with the four circle centers
- _stereo_pattern_markers_: visualization marker with the 4 circle centers in order to visualize the detections in RVIZ.

# Radar detector (2D or 3D)
The _radar_detector_ node subscribes to a ROS topic with the name: /radar_converter/detections. This ROS topic has a message type: radar_msgs::RadarDetectionArray. We assume that X longitudinal and Y is lateral in the radar_msgs::RadarDetectionArray.
The detector can work with both 2D and 3D radars, but range is determined only by longitudinal (X) and lateral (Y) position. 

You can define a minimum and maximum allowed range and RCS using ROS parameters. Within that RCS+range window, you can choose to select the actual detection either based on lowest/highest range, or lowest/higher RCS. For an example, see the launch file [here](multi_sensor_calibration_launch/launch/detectors.launch). The full list of parameters is as follows:
```
minimum_RCS:float, default: -inf
maximum_RCS:float, default: inf
min_range_object:float, default: -inf
max_range_object:float, default: inf
selection_basis:[rcs or range], default:range
selection_criterion:[min or max], default:min
```


This node publishes two messages:
- _radar_pattern_: point cloud with the calibration board detection (of reflector)
- _radar_marker_: visualization marker with the radar detection. This marker can be visualized in RViz. The vertical shape indicates the detection of the reflector at elevation angle equal 0 degrees.

# Monocular detector
The _mono_detector_ node subscribes to four ROS topics:
- /ueye/left/image_rect_color: type: sensor_msgs::Image message
- /ueye/left/camera_info:      type: sensor_msgs::CameraInfo message

The _mono_detector_ publishes one message:
- _mono_pattern_: point cloud with the four circle centers

# Explanation of inputs
This section explains the inputs to the detectors and optimizer.
The _config.yaml_ files for the detectors contain explanations of the input parameters.

NOTE: Make sure that the geometry of the calibration board is known to the detectors:
- detectors: min_radius, max_radius, radius, width and height in YAML files
- optimizer: see class geometry_calibration_board in calibration_board.py.

Inputs to optimizer are arrays with x,y,z for lidar and camera and x,y for radar.
