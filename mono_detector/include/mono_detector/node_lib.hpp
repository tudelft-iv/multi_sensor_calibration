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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mono_detector/types.hpp"

namespace mono_detector {

/// Class with a node, subscribing to image, camera info and publishes point cloud of calibration pattern
class MonoDetectorNode : public rclcpp::Node {
public:
	/// Constructor taking the node handle as a member variable
	MonoDetectorNode();

private:
	/// Publisher for resulting pattern point cloud
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

	/// Subscriber for input images
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

	/// Subscriber for camera info
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;

	/// Configuration of parameters, intrinsics, visualization
	Configuration config_;

	/// Intrinsics
	CameraModel intrinsics_;
	bool intrinsics_received_ = false;

	/// Object points
	std::vector<cv::Point3f> object_points_;

	/// Image callback, subscribes to image, detects pattern and publishes pattern point cloud
	void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const & in);

	/// Camera info callback
	void cameraInfoCallback(sensor_msgs::msg::CameraInfo const & camera_info);
};

}

