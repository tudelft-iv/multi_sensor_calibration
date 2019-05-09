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
#include <pcl_ros/point_cloud.h>
#include "keypoint_detection.hpp"


namespace lidar_detector {

/// Class with a node, subscribing to lidar point cloud and publishing detected pattern
class LidarDetectorNode {
public:

	// Constructor, taking a node handle
	LidarDetectorNode(ros::NodeHandle & nh);

private:
	/// Ros Node Handle
	ros::NodeHandle nh_;

	/// Subscriber to velodyne point clouds
	ros::Subscriber point_cloud_subscriber_;

	/// Publisher of the detected pattern
	ros::Publisher point_cloud_publisher_;

	/// Publisher of the visualization markers of the pattern
	ros::Publisher sphere_marker_publisher_;

	/// Configuration of the detection algorithm
	Configuration config_;

	void callback(sensor_msgs::PointCloud2ConstPtr const & in);

	/// Publisher of the pattern as sphere markers to visualize the result in rviz
	void publishMarker(
		pcl::PointCloud<pcl::PointXYZ> const & pattern,
		std_msgs::Header const & header
	);
};

} // namespace
