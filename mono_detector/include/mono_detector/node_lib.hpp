/*
  multi_sensor_calibration
  Copyright (C) 2019 Intelligent Vehicles, Delft University of Technology

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include "types.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h> // ToDo: Remove ros dependency

namespace mono_detector {

/// Class with a node, subscribing to image, camera info and publishes point cloud of calibration pattern
class MonoDetectorNode {
public:
	/// Constructor taking the node handle as a member variable
	MonoDetectorNode(ros::NodeHandle & nh);

private:

	/// Ros Node Handle to communicate with ros server
	ros::NodeHandle nh_;

	/// Publisher for resulting pattern point cloud
	ros::Publisher point_cloud_publisher_;

	/// Subscriber for input images
	ros::Subscriber image_subscriber_;

	/// Subscriber for camera info
	ros::Subscriber camera_info_subscriber_;

	/// Configuration of parameters, intrinsics, visualization
	Configuration config_;

	/// Intrinsics
	image_geometry::PinholeCameraModel intrinsics_;

	/// Object points
	std::vector<cv::Point3f> object_points_;

	/// Image callback, subscribes to image, detects pattern and publishes pattern point cloud
	void imageCallback(sensor_msgs::ImageConstPtr const & in);

	/// Camera info callback
	void cameraInfoCallback(sensor_msgs::CameraInfo const & camera_info);
};

}

