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

#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace radar_detector {

// Node, subscribing to a radar measurement and publishes transformed calibration pattern
class RadarDetectorNode : public rclcpp::Node {
public:
  RadarDetectorNode();

private:
  // Define specfic RCS values range
  float min_RCS_;
  float max_RCS_;
  float min_range_object_;
  float max_range_object_;
  bool select_range_;
  bool select_min_;

  // Subscriber for radar messages point cloud
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_subscriber_;

  // Publisher for resulting pattern point cloud
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pattern_publisher_;

  // Publisher for resulting pattern as a marker for rviz
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  // Function to convert to a point cloud type and publish pattern
  void publishPattern(pcl::PointXYZ const & point, std_msgs::msg::Header const & header);

  // Function to convert to a marker and publish pattern
  void publishMarker(pcl::PointXYZ const & point, std_msgs::msg::Header const & header);

  // Point cloud callback function
  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& in);
};

}  // namespace radar_detector
