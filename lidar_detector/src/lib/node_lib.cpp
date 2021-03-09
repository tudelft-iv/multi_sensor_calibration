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

#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <ros/package.h>
#include "yaml.hpp"
#include "node_lib.hpp"

namespace lidar_detector {

namespace {
	visualization_msgs::Marker toMarker(pcl::PointXYZ const & point, std_msgs::Header const & header) {
		visualization_msgs::Marker marker;
		marker.header = header;
		marker.action = visualization_msgs::Marker::ADD;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.color.g = 1;
		marker.color.a = 1.0;
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		marker.pose.position.x = point.x;
		marker.pose.position.y = point.y;
		marker.pose.position.z = point.z;
		marker.pose.orientation.w = 1.0;
		return marker;
	}
}

LidarDetectorNode::LidarDetectorNode(ros::NodeHandle & nh) : nh_(nh) {
	ROS_INFO("Initialized lidar detector.");

	// TODO: Optionally load params to from parameter server instead of yaml file
	std::string yaml_config;
	nh_.param<std::string>("path_to_yaml_config", yaml_config, ros::package::getPath("lidar_detector") + "/" + "config/config.yaml");
	config_ = YAML::LoadFile(yaml_config).as<lidar_detector::Configuration>();

	point_cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &LidarDetectorNode::callback, this);
	point_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("lidar_pattern", 100);
	sphere_marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("lidar_pattern_markers", 100);
}

void LidarDetectorNode::callback(sensor_msgs::PointCloud2ConstPtr const & in) {
	ROS_INFO_ONCE("Receiving lidar point clouds.");
	try {
		pcl::PointCloud<Lidar::PointWithDist> cloud;
		pcl::fromROSMsg(*in, cloud);
		pcl::PointCloud<pcl::PointXYZ> pattern = keypointDetection(cloud, config_);
		sensor_msgs::PointCloud2 out;
		pcl::toROSMsg(pattern, out);
		out.header = in->header;
		point_cloud_publisher_.publish(out);
		publishMarker(pattern, in->header);
	} catch (pcl::PCLException & e) {
		ROS_INFO_ONCE("Ignoring exceptions thrown by pcl in at least one frame.");
	}
	ROS_INFO_ONCE("Publishing patterns.");
}

void LidarDetectorNode::publishMarker(
	pcl::PointCloud<pcl::PointXYZ> const & pattern,
	std_msgs::Header const & header
) {
	visualization_msgs::MarkerArray markers;
	for (std::size_t i = 0; i < pattern.size(); ++i) {
	    auto marker = toMarker(pattern.at(i), header);
	    marker.id = i;
		markers.markers.push_back(marker);
	}
	sphere_marker_publisher_.publish(markers);
}

} // namespace
