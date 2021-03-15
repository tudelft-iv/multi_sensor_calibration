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

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <image_geometry/stereo_camera_model.h>
#include <visualization_msgs/MarkerArray.h>

#include "node_lib.hpp"
#include "yaml.hpp"
#include "keypoint_detection.hpp"

namespace stereo_detector {

namespace {
	cv::Mat toOpencv(const sensor_msgs::Image & in) {
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(in);
		return cv_ptr->image;
	}

	visualization_msgs::Marker toMarker(pcl::PointXYZRGB const & point, std_msgs::Header const & header) {
		visualization_msgs::Marker marker;
		marker.header = header;
		marker.action = visualization_msgs::Marker::ADD;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.color.r = (float) std::rand()/RAND_MAX;
		marker.color.g = (float) std::rand()/RAND_MAX;
		marker.color.b = (float) std::rand()/RAND_MAX;
		marker.color.a = 1.0;
		marker.pose.position.x = point.x;
		marker.pose.position.y = point.y;
		marker.pose.position.z = point.z;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		return marker;
	}

	visualization_msgs::MarkerArray toMarkers(pcl::PointCloud<pcl::PointXYZRGB> const & pattern, std_msgs::Header const & header) {
		visualization_msgs::MarkerArray markers;
		for (std::size_t i = 0; i < pattern.size(); ++i) {
		    auto marker = toMarker(pattern.at(i), header);
		    marker.id = i;
			markers.markers.push_back(marker);
		}
		return markers;
	}

	pcl::PointCloud<pcl::PointXYZRGB> toCloud(
		cv::Mat const & image,
		sensor_msgs::CameraInfoConstPtr const & left_camera_info,
		sensor_msgs::CameraInfoConstPtr const & right_camera_info,
		cv::Mat const & disparity
) {
		// Get 3d locations
		image_geometry::StereoCameraModel model;
		model.fromCameraInfo(left_camera_info, right_camera_info);
		cv::Mat xyz;
		model.projectDisparityImageTo3d(disparity, xyz);

		// Build the point cloud
		pcl::PointCloud<pcl::PointXYZRGB> out;
		out.is_dense = true;
		for (int y = 0; y < image.rows; ++y) {
			for (int x = 0; x < image.cols; ++x) {
				cv::Point3f point3f = xyz.at<cv::Point3f>(y, x);
				pcl::PointXYZRGB point_xyzrgb;
				point_xyzrgb.x = point3f.x;
				point_xyzrgb.y = point3f.y;
				point_xyzrgb.z = point3f.z;
				cv::Vec3b color = image.at<cv::Vec3b>(y, x);
				point_xyzrgb.b = color[0];
				point_xyzrgb.g = color[1];
				point_xyzrgb.r = color[2];
				out.push_back(point_xyzrgb);
			}
		}
		out.width = image.cols;
		out.height = image.rows;
		return out;
	}

}

StereoDetectorNode::StereoDetectorNode(ros::NodeHandle & nh)
:
	nh_(nh),
	image_subscriber_(nh_, "/ueye/left/image_rect_color", 1),
	left_camera_info_subscriber_(nh_, "/ueye/left/camera_info", 1),
	right_camera_info_subscriber_(nh_, "/ueye/right/camera_info", 1),
	disparity_subscriber_(nh_, "/ueye/disparity", 1),
	sync_(image_subscriber_, left_camera_info_subscriber_, right_camera_info_subscriber_, disparity_subscriber_, 10)
{

	ROS_INFO("Initialized stereo detector.");

	// Load configuration from file
	std::string yaml_file;
	nh_.param<std::string>("yaml_file", yaml_file, ros::package::getPath("stereo_detector") + "/" + "config/config.yaml");
	config_ = YAML::LoadFile(yaml_file).as<stereo_detector::Configuration>();

	// Setup subscriber and publisher
	point_cloud_publisher_  = nh_.advertise<sensor_msgs::PointCloud2>("stereo_pattern", 100);
	sphere_marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("stereo_pattern_markers", 100);

	// Get synchronized image and point cloud
	sync_.registerCallback(boost::bind(&StereoDetectorNode::callback, this, _1, _2, _3, _4));
}

void StereoDetectorNode::callback(
	sensor_msgs::ImageConstPtr const & image,
	sensor_msgs::CameraInfoConstPtr const & left_camera_info,
	sensor_msgs::CameraInfoConstPtr const & right_camera_info,
	stereo_msgs::DisparityImageConstPtr const & disparity
) {
	// convert cloud
	ROS_INFO_ONCE("Receiving synchronized rectified image, camera info and disparity image.");
	try {

		pcl::PointCloud<pcl::PointXYZRGB> cloud = toCloud(toOpencv(*image), left_camera_info, right_camera_info, toOpencv(disparity->image));

		// process
		pcl::PointCloud<pcl::PointXYZRGB> processed = keypointDetection(
			toOpencv(*image),
			cloud,
			config_
		);
		sensor_msgs::PointCloud2 out;
		pcl::toROSMsg(processed, out);
		out.header = image->header;
		point_cloud_publisher_.publish(out);
		sphere_marker_publisher_.publish(toMarkers(processed, image->header));
	} catch (std::exception & e) {
		ROS_INFO_ONCE("Ignoring exceptions in at least one frame. Reason: %s", e.what());
	}
}

}
