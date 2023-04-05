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
#include <visualization_msgs/msg/marker_array.hpp>
// #include <pcl_ros/point_cloud.h>

#include "lidar_detector/keypoint_detection.hpp"


namespace lidar_detector {

/// Class with a node, subscribing to lidar point cloud and publishing detected pattern
class LidarDetectorNode : public rclcpp::Node {
public:
	LidarDetectorNode();

private:
	/// Subscriber to velodyne point clouds
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;

	/// Publisher of the detected pattern
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

	/// Publisher of the visualization markers of the pattern
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sphere_marker_publisher_;

	/// Configuration of the detection algorithm
	Configuration config_;

	void callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const & in);

	/// Publisher of the pattern as sphere markers to visualize the result in rviz
	void publishMarker(
		pcl::PointCloud<pcl::PointXYZ> const & pattern,
		std_msgs::msg::Header const & header
	);
};

}  // namespace lidar_detector
