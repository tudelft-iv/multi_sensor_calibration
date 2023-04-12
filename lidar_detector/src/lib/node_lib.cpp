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

#include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include "lidar_detector/yaml.hpp"
#include "lidar_detector/node_lib.hpp"

using std::placeholders::_1;

namespace lidar_detector {

namespace {
	visualization_msgs::msg::Marker toMarker(pcl::PointXYZ const & point, std_msgs::msg::Header const & header) {
		visualization_msgs::msg::Marker marker;
		marker.header = header;
		marker.action = visualization_msgs::msg::Marker::ADD;
		marker.type = visualization_msgs::msg::Marker::SPHERE;
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
}  // namespace

LidarDetectorNode::LidarDetectorNode() : Node("lidar_detector") {
	RCLCPP_INFO(get_logger(), "Initialized lidar detector.");
  std::string package_share = ament_index_cpp::get_package_share_directory("lidar_detector");

	// TODO: Optionally load params to from parameter server instead of yaml file
  this->declare_parameter("path_to_yaml_config", package_share + "/config/config.yaml");
  std::string yaml_config = this->get_parameter("path_to_yaml_config").get_parameter_value().get<std::string>();
	config_ = YAML::LoadFile(yaml_config).as<lidar_detector::Configuration>();

	point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points", 10, std::bind(&LidarDetectorNode::callback, this, _1)
	);

	point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_pattern", 100);
	sphere_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("lidar_pattern_markers", 100);
}

void LidarDetectorNode::callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const & in) {
	RCLCPP_INFO_ONCE(get_logger(), "Receiving lidar point clouds.");
	try {
		pcl::PointCloud<Lidar::PointWithDist> cloud;
		pcl::fromROSMsg(*in, cloud);
		pcl::PointCloud<pcl::PointXYZ> pattern = keypointDetection(cloud, config_);
		sensor_msgs::msg::PointCloud2 out;
		pcl::toROSMsg(pattern, out);
		out.header = in->header;
		point_cloud_publisher_->publish(out);
		publishMarker(pattern, in->header);
	} catch (pcl::PCLException & e) {
		RCLCPP_INFO_ONCE(get_logger(), "Ignoring exceptions thrown by pcl in at least one frame.");
	}
	RCLCPP_INFO_ONCE(get_logger(), "Publishing patterns.");
}

void LidarDetectorNode::publishMarker(
	pcl::PointCloud<pcl::PointXYZ> const & pattern,
	std_msgs::msg::Header const & header
) {
	visualization_msgs::msg::MarkerArray markers;
	for (std::size_t i = 0; i < pattern.size(); ++i) {
	    auto marker = toMarker(pattern.at(i), header);
	    marker.id = i;
		markers.markers.push_back(marker);
	}
	sphere_marker_publisher_->publish(markers);
}

}  // namespace lidar_detector
