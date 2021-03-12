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

#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include "node_lib.hpp"
#include "keypoint_detection.hpp"

bool isValidDetection(pcl::PointXYZ point) {
	return !(point.x == 0 && point.y == 0);
}

bool is_3d(pcl::PointXYZ point) {
	return point.z != 0;
}

namespace radar_detector {

/// Convert a pcl point to a geometry msgs point
geometry_msgs::Point toRos(pcl::PointXYZ const & point) {
	geometry_msgs::Point out;
	out.x = point.x;
	out.y = point.y;
	out.z = point.z;
	return out;
}

/// Convert a point cloud to a marker of spheres
visualization_msgs::Marker toPoint(pcl::PointXYZ const & point, std_msgs::Header const & header) {
	visualization_msgs::Marker marker;
	marker.action          = visualization_msgs::Marker::ADD;
	marker.type            = visualization_msgs::Marker::SPHERE;
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

/// Convert a point cloud to a specific point
visualization_msgs::Marker toArc(pcl::PointXYZ const & point, std_msgs::Header const & header, float & maximum_elevation_degrees) {
	visualization_msgs::Marker marker;
	marker.action          = visualization_msgs::Marker::ADD;
	marker.type            = visualization_msgs::Marker::LINE_STRIP;
	marker.header = header;

	// Get point
	pcl::PointXYZ point2d = point;
	point2d.z = 0;
	// Transform to polar:
	float range = sqrt(point2d.x*point2d.x + point2d.y*point2d.y);
	float azimuth = atan2(point2d.y, point2d.x);

	float max_elevation_angle_radians = maximum_elevation_degrees*M_PI/180; // TODO: should be to config
	int nr_line_segements = 25;
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

void RadarDetectorNode::publishMarker(pcl::PointXYZ const & point, std_msgs::Header const & header) {
	float el = 9; // TODO move to ROS parameter

	// Get arc for radar detection
	visualization_msgs::Marker marker;
	if(is_3d(point)){
        marker = toPoint(point, header);
	}else{
	    marker = toArc(point, header, el);
	}

	// Publish marker
	marker_publisher_.publish(marker);
}

void RadarDetectorNode::publishPattern(pcl::PointXYZ const & point, std_msgs::Header const & header) {
	pcl::PointCloud<pcl::PointXYZ> pattern;
	pattern.push_back(point);
	sensor_msgs::PointCloud2 out;
	pcl::toROSMsg(pattern, out);
	out.header = header;
	pattern_publisher_.publish(out);
}

RadarDetectorNode::RadarDetectorNode(ros::NodeHandle & nh) :
	nh_(nh)
{
    bool initialization_errors = false;
	nh_.param<float>("minimum_RCS", min_RCS_, std::numeric_limits<float>::lowest());
	nh_.param<float>("maximum_RCS", max_RCS_, std::numeric_limits<float>::max());
	nh_.param<float>("min_range_object", min_range_object_, std::numeric_limits<float>::lowest());
	nh_.param<float>("max_range_object", max_range_object_, std::numeric_limits<float>::max());
	std::string selection_basis;
	nh_.param<std::string>("selection_basis", selection_basis, radar_detector::RANGE_BASED_SELECTION);
	if(selection_basis == radar_detector::RANGE_BASED_SELECTION){
        select_range_ = true;
	}else if(selection_basis == radar_detector::RCS_BASED_SELECTION)
	{
	    select_range_ = false;
	}else
	{
	    ROS_ERROR_STREAM("selection_basis parameter must either be " << radar_detector::RANGE_BASED_SELECTION
                         << " or " << radar_detector::RCS_BASED_SELECTION << " (current value: " << selection_basis <<  ")");
        initialization_errors = true;
	}
	std::string selection_criterion;
	nh_.param<std::string>("selection_criterion", selection_criterion, radar_detector::SELECT_MIN);
	if(selection_criterion == radar_detector::SELECT_MIN){
        select_min_ = true;
	}else if(selection_criterion == radar_detector::SELECT_MAX)
	{
	    select_min_ = false;
	}else
	{
	    ROS_ERROR_STREAM("selection_criterion parameter must either be " << radar_detector::SELECT_MIN
                         << " or " << radar_detector::SELECT_MAX << " (current value: " << selection_criterion <<  ")");
        initialization_errors = true;
	}

	if(initialization_errors)
	{
	    throw std::exception();
	}

	radar_subscriber_ = nh_.subscribe("/radar_converter/detections", 10, &RadarDetectorNode::callback, this);
	pattern_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("radar_pattern", 10);
	marker_publisher_  = nh_.advertise<visualization_msgs::Marker>("radar_marker", 10);
	ROS_INFO("Initialized radar detector.");
}

void RadarDetectorNode::callback(radar_msgs::RadarDetectionArray const & in) {
	ROS_INFO_ONCE("Receiving radar messages.");
	// Find reflection of calibration board
	pcl::PointXYZ point = keypointDetection(in, min_RCS_, max_RCS_,min_range_object_, max_range_object_, select_range_, select_min_);

	// Publish results if detected point is valid (so not in origin of sensor)
	if (isValidDetection(point)) {
		publishMarker(point, in.header);
		publishPattern(point, in.header);
	}
}



}
