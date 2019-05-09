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

#include "node_lib.hpp"
#include "detector.hpp"
#include "pnp.hpp"
#include "util.hpp"
#include "yaml.hpp"
#include <ros/console.h>
#include <ros/package.h>
#include <string>
#include <image_geometry/pinhole_camera_model.h> // ToDo: Remove ros dependency
#include <pcl/common/transforms.h>

namespace mono_detector {


MonoDetectorNode::MonoDetectorNode(ros::NodeHandle & nh) : nh_(nh) {
	ROS_INFO("Initialized mono detector.");

	// Load object points from ros parameter server
	std::string object_points_file;
	nh_.param<std::string>("object_points_file", object_points_file, ros::package::getPath("mono_detector") + "/" + "config/object_points.yaml");
	object_points_ = YAML::LoadFile(object_points_file).as<std::vector<cv::Point3f>>();
	cv::Point3f center = calculateCenter(object_points_);
	std::sort(object_points_.begin(), object_points_.end(), [center](cv::Point3f a, cv::Point3f b) {
		return std::atan((a.y - center.y) / (a.x - center.x)) > std::atan((b.y - center.y) / (b.x - center.x));
	});

	// Load configuration from file
	std::string yaml_file;
	nh_.param<std::string>("yaml_file", yaml_file, ros::package::getPath("mono_detector") + "/" + "config/image_processing.yaml");
	config_ = YAML::LoadFile(yaml_file).as<mono_detector::Configuration>();

	// Setup subscriber and publisher
	image_subscriber_       = nh_.subscribe("/ueye/left/image_raw", 1, &MonoDetectorNode::imageCallback, this);
	camera_info_subscriber_ = nh_.subscribe("/ueye/left/camera_info", 1, &MonoDetectorNode::cameraInfoCallback, this); // ToDo: Replace with camera subscriber for sync camera info
	point_cloud_publisher_  = nh_.advertise<sensor_msgs::PointCloud2>("mono_pattern", 100);
}

void MonoDetectorNode::imageCallback(sensor_msgs::ImageConstPtr const & in) {
	ROS_INFO_ONCE("Receiving images.");

	try {
		// Call to do image processing
		std::vector<cv::Point2f> image_points;
		std::vector<float> radi;
		detectMono(toOpencv(in), config_, image_points, radi);

		// Call to solve pnp
		Eigen::Isometry3f isometry = solvePose(image_points, object_points_, intrinsics_);

		// Transform pattern
		pcl::PointCloud<pcl::PointXYZ> transformed_pattern;
		pcl::transformPointCloud(toPcl(object_points_), transformed_pattern, isometry);

		// Publish pattern
		ROS_INFO_ONCE("Detected a mono detector pattern point cloud at least once.");
		sensor_msgs::PointCloud2 out;
		pcl::toROSMsg(transformed_pattern, out);
		out.header = in->header;
		point_cloud_publisher_.publish(out);
	} catch (std::exception & e) {
		ROS_ERROR_STREAM("Exception thrown: '" << e.what() << "'.");
	}
}

void MonoDetectorNode::cameraInfoCallback(sensor_msgs::CameraInfo const & camera_info) {
	ROS_INFO_ONCE("Receiving camera info.");
	// ToDo: Use message filters to make sure to get both camera info and image
	if (!intrinsics_.fromCameraInfo(camera_info)) {
		throw std::runtime_error("Unable to convert camera info to pinhole camera model.");
	}
}

}
