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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>

#include "stereo_detector/config.hpp"

namespace stereo_detector {

/// Class with a node, subscribing to an organized point cloud and publishes point cloud of calibration pattern
class StereoDetectorNode : public rclcpp::Node {
public:
	StereoDetectorNode();

private:
	/// Publisher for resulting pattern point cloud
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

	/// Publisher for visualizing the resulting pattern locations in rviz
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sphere_marker_publisher_;

	/// Image subscriber
	message_filters::Subscriber<sensor_msgs::msg::Image> image_subscriber_;

	/// Left camera info subscriber
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_camera_info_subscriber_;

	/// right camera info subscriber
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> right_camera_info_subscriber_;

	/// Disparity subscriber
	message_filters::Subscriber<stereo_msgs::msg::DisparityImage> disparity_subscriber_;

	/// Create message time synchronizer
	message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo, stereo_msgs::msg::DisparityImage> sync_;

	/// Parameters for processing the data
	Configuration config_;

	/// Point cloud callback function
	void callback(
		sensor_msgs::msg::Image::ConstSharedPtr const & image,
		sensor_msgs::msg::CameraInfo::ConstSharedPtr const & left_camera_info,
		sensor_msgs::msg::CameraInfo::ConstSharedPtr const & right_camera_info,
		stereo_msgs::msg::DisparityImage::ConstSharedPtr const & disparity
	);

};


}
