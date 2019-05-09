/*
  multi_sensor_calibration
  Copyright (C) 2019  Intelligent Vehicles Delft University of Technology

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along
  with this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>
#include "config.hpp"

namespace stereo_detector {

/// Class with a node, subscribing to an organized point cloud and publishes point cloud of calibration pattern
class StereoDetectorNode {
public:

	/// Constructor taking the node handle as a member variable
	StereoDetectorNode(ros::NodeHandle & nh);

private:

	/// Ros Node Handle to communicate with ros server
	ros::NodeHandle nh_;

	/// Publisher for resulting pattern point cloud
	ros::Publisher point_cloud_publisher_;

	/// Publisher for visualizing the resulting pattern locations in rviz
	ros::Publisher sphere_marker_publisher_;

	/// Image subscriber
	message_filters::Subscriber<sensor_msgs::Image> image_subscriber_;

	/// Left camera info subscriber
	message_filters::Subscriber<sensor_msgs::CameraInfo> left_camera_info_subscriber_;

	/// right camera info subscriber
	message_filters::Subscriber<sensor_msgs::CameraInfo> right_camera_info_subscriber_;

	/// Disparity subscriber
	message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_subscriber_;

	/// Create message time synchronizer
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, stereo_msgs::DisparityImage> sync_;

	/// Parameters for processing the data
	Configuration config_;

	/// Point cloud callback function
	void callback(
		sensor_msgs::ImageConstPtr const & image,
		sensor_msgs::CameraInfoConstPtr const & left_camera_info,
		sensor_msgs::CameraInfoConstPtr const & right_camera_info,
		stereo_msgs::DisparityImageConstPtr const & disparity
	);

};


}
