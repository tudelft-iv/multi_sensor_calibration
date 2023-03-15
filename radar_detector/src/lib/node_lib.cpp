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

#include "radar_detector/node_lib.hpp"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "radar_detector/keypoint_detection.hpp"

using std::placeholders::_1;

bool isValidDetection(pcl::PointXYZ point) {
  return !(point.x == 0 && point.y == 0);
}

bool is_3d(pcl::PointXYZ point) {
  return point.z != 0;
}

namespace radar_detector {

// Convert a pcl point to a geometry msgs point
geometry_msgs::msg::Point toRos(pcl::PointXYZ const & point) {
  geometry_msgs::msg::Point out;
  out.x = point.x;
  out.y = point.y;
  out.z = point.z;
  return out;
}

// Convert a point cloud to a marker of spheres
visualization_msgs::msg::Marker toPoint(pcl::PointXYZ const & point, std_msgs::msg::Header const & header) {
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.header = header;
  marker.pose.position = toRos(point);
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  return marker;
}

// Convert a point cloud to a specific point
visualization_msgs::msg::Marker toArc(pcl::PointXYZ const & point, std_msgs::msg::Header const & header, float & maximum_elevation_degrees) {
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.header = header;

  // Get point
  pcl::PointXYZ point2d = point;
  point2d.z = 0;
  // Transform to polar:
  float range = sqrt(point2d.x*point2d.x + point2d.y*point2d.y);
  float azimuth = atan2(point2d.y, point2d.x);

  float max_elevation_angle_radians = maximum_elevation_degrees*M_PI/180; // TODO: should be to config
  unsigned int nr_line_segements = 25;
  float delta = 2*max_elevation_angle_radians/nr_line_segements;
  for (size_t n = 0; n < nr_line_segements; n++) {
    // Get elevation angle
    float elevation = -max_elevation_angle_radians + delta*n;
    // Covert back to x,y,z
    pcl::PointXYZ t;
    t.x = range * cos(azimuth) * cos(elevation);
    t.y = range * sin(azimuth) * cos(elevation);
    t.z = range * sin(elevation);
    // Store point
    marker.points.push_back(toRos(t));
  }

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  return marker;
}

RadarDetectorNode::RadarDetectorNode() : Node("radar_detector") {
  bool initialization_errors = false;

  this->declare_parameter("minimum_RCS", std::numeric_limits<float>::lowest());
  this->declare_parameter("maximum_RCS", std::numeric_limits<float>::max());
  this->declare_parameter("min_range_object", std::numeric_limits<float>::lowest());
  this->declare_parameter("max_range_object", std::numeric_limits<float>::max());
  this->declare_parameter("selection_basis", radar_detector::RANGE_BASED_SELECTION);
  this->declare_parameter("selection_criterion", radar_detector::SELECT_MIN);

  min_RCS_ = this->get_parameter("minimum_RCS").get_parameter_value().get<float>();
  max_RCS_ = this->get_parameter("maximum_RCS").get_parameter_value().get<float>();
  min_range_object_ = this->get_parameter("min_range_object").get_parameter_value().get<float>();
  max_range_object_ = this->get_parameter("max_range_object").get_parameter_value().get<float>();
  std::string selection_basis = this->get_parameter("selection_basis").get_parameter_value().get<std::string>();
  std::string selection_criterion = this->get_parameter("selection_criterion").get_parameter_value().get<std::string>();

  if (selection_basis == radar_detector::RANGE_BASED_SELECTION) {
    select_range_ = true;
  } else if (selection_basis == radar_detector::RCS_BASED_SELECTION) {
    select_range_ = false;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "selection_basis parameter must either be " << radar_detector::RANGE_BASED_SELECTION
                        << " or " << radar_detector::RCS_BASED_SELECTION << " (current value: " << selection_basis <<  ")");
    initialization_errors = true;
  }

  if (selection_criterion == radar_detector::SELECT_MIN) {
    select_min_ = true;
  } else if (selection_criterion == radar_detector::SELECT_MAX) {
    select_min_ = false;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "selection_criterion parameter must either be " << radar_detector::SELECT_MIN
                        << " or " << radar_detector::SELECT_MAX << " (current value: " << selection_criterion <<  ")");
    initialization_errors = true;
  }

  if (initialization_errors) {
      throw std::exception();
  }

  radar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/radar_converter/detections", 10, std::bind(&RadarDetectorNode::callback, this, _1));
  pattern_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("radar_pattern", 10);
  marker_publisher_  = this->create_publisher<visualization_msgs::msg::Marker>("radar_marker", 10);
  RCLCPP_INFO(get_logger(), "Initialized radar detector.");
}

void RadarDetectorNode::publishMarker(pcl::PointXYZ const & point, std_msgs::msg::Header const & header) {
  float el = 9; // TODO move to ROS parameter

  // Get arc for radar detection
  visualization_msgs::msg::Marker marker;
  if (is_3d(point)) {
    marker = toPoint(point, header);
  }else{
    marker = toArc(point, header, el);
  }

  // Publish marker
  marker_publisher_->publish(marker);
}

void RadarDetectorNode::publishPattern(pcl::PointXYZ const & point, std_msgs::msg::Header const & header) {
  pcl::PointCloud<pcl::PointXYZ> pattern;
  pattern.push_back(point);
  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(pattern, out);
  out.header = header;
  pattern_publisher_->publish(out);
}

void RadarDetectorNode::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& in) {
  RCLCPP_INFO_ONCE(get_logger(), "Receiving radar messages.");
  // Find reflection of calibration board
  pcl::PointXYZ point = keypointDetection(in, min_RCS_, max_RCS_,min_range_object_, max_range_object_, select_range_, select_min_);

  // Publish results if detected point is valid (so not in origin of sensor)
  if (isValidDetection(point)) {
    publishMarker(point, (*in).header);
    publishPattern(point, (*in).header);
  }
}

}
