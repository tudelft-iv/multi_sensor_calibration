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
#include <ros/ros.h>
#include <radar_msgs/RadarDetectionArray.h>
#include <pcl_ros/point_cloud.h>

namespace radar_detector {

/// Class with a node, subscribing to a radar measurement and publishes transformed calibration pattern
class RadarDetectorNode {
public:

	/// Constructor taking the node handle as a member variable
	RadarDetectorNode(ros::NodeHandle & nh);

private:

	/// Define specfic RCS values range
	float min_RCS_;
	float max_RCS_;
	float min_range_object_;
	float max_range_object_;
	bool select_range_;
	bool select_min_;

	/// Ros Node Handle to communicate with ros server
	ros::NodeHandle nh_;

	/// Subscriber for radar messages point cloud
	ros::Subscriber radar_subscriber_;

	/// Publisher for resulting pattern point cloud
	ros::Publisher pattern_publisher_;

	/// Publisher for resulting pattern as a marker for rviz
	ros::Publisher marker_publisher_;

	/// Function to convert to a point cloud type and publish pattern
	void publishPattern(pcl::PointXYZ const & point, std_msgs::Header const & header);

	/// Function to convert to a marker and publish pattern
	void publishMarker(pcl::PointXYZ const & point, std_msgs::Header const & header);

	/// Point cloud callback function
	void callback(radar_msgs::RadarDetectionArray const & in);

};


}
