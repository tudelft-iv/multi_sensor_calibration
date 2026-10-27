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
	cv::Mat toOpencv(const sensor_msgs::Image& in, const std::string& topic_name) {
		cv_bridge::CvImagePtr cv_ptr;

		try {
			// Imprimir el tópico
			

			// Convertir la imagen
			cv_ptr = cv_bridge::toCvCopy(in);

			// Verificar tipo de imagen y dimensiones
			//int image_type = cv_ptr->image.type();
			

			/*// Guardar la imagen para depuración
			
			

			// Validar tipo de imagen y convertir si es necesario
			if (image_type == CV_8UC1) {
				ROS_INFO("Converting grayscale image to color.");
				cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_GRAY2BGR);
			} else if (image_type == CV_16UC1) {
				ROS_INFO("Processing image from topic: %s", topic_name.c_str());
				
				ROS_INFO("Saved image to /tmp/debug_image_before_error.png");
				cv::imwrite("/tmp/debug_image_before_error.png", cv_ptr->image);
				ROS_INFO("Normalizing disparity image.");
				cv::Mat disparity_normalized;
				cv_ptr->image.convertTo(disparity_normalized, CV_32F, 1.0 / 256.0);
				disparity_normalized.convertTo(cv_ptr->image, CV_8UC1, 255.0);
			} else if (image_type == CV_8UC3) {
				ROS_INFO("Image is already in color.");
			} else {
				ROS_ERROR("Unexpected image type encountered: %d", image_type);
				return cv::Mat();
			}*/

			return cv_ptr->image;

		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("Error converting image from topic: %s, reason: %s", topic_name.c_str(), e.what());
			return cv::Mat();
		}
	}

	visualization_msgs::Marker toMarker(pcl::PointXYZRGB const& point, std_msgs::Header const& header) {
		visualization_msgs::Marker marker;
		marker.header = header;
		marker.action = visualization_msgs::Marker::ADD;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.color.r = (float)std::rand() / RAND_MAX;
		marker.color.g = (float)std::rand() / RAND_MAX;
		marker.color.b = (float)std::rand() / RAND_MAX;
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

	visualization_msgs::MarkerArray toMarkers(pcl::PointCloud<pcl::PointXYZRGB> const& pattern, std_msgs::Header const& header) {
		visualization_msgs::MarkerArray markers;
		for (std::size_t i = 0; i < pattern.size(); ++i) {
			auto marker = toMarker(pattern.at(i), header);
			marker.id = i;
			markers.markers.push_back(marker);
		}
		return markers;
	}

	pcl::PointCloud<pcl::PointXYZRGB> toCloud(
		cv::Mat const& image,
		sensor_msgs::CameraInfoConstPtr const& left_camera_info,
		sensor_msgs::CameraInfoConstPtr const& right_camera_info,
		cv::Mat const& disparity
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

StereoDetectorNode::StereoDetectorNode(ros::NodeHandle& nh)
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
	point_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("stereo_pattern", 100);
	sphere_marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("stereo_pattern_markers", 100);

	// Get synchronized image and point cloud
	sync_.registerCallback(boost::bind(&StereoDetectorNode::callback, this, _1, _2, _3, _4));
}

void StereoDetectorNode::callback(
	sensor_msgs::ImageConstPtr const& image,
	sensor_msgs::CameraInfoConstPtr const& left_camera_info,
	sensor_msgs::CameraInfoConstPtr const& right_camera_info,
	stereo_msgs::DisparityImageConstPtr const& disparity
) {
	// Registrar el nombre del frame y del tópico
	ROS_INFO("Processing frame from: %s", image->header.frame_id.c_str());
	ROS_INFO("Processing message from topic: %s", image_subscriber_.getTopic().c_str());

	// Convertir la imagen a OpenCV
	cv::Mat cv_image = toOpencv(*image, image_subscriber_.getTopic());

	// Validar que la imagen no esté vacía
	if (cv_image.empty()) {
		ROS_WARN("Input image is empty. Skipping frame.");
		return;
	}

	// Validar dimensiones de la imagen
	if (cv_image.rows <= 0 || cv_image.cols <= 0) {
		ROS_WARN("Invalid image dimensions: rows=%d, cols=%d", cv_image.rows, cv_image.cols);
		return;
	}

	// Mostrar la imagen en pantalla
	cv::imshow("Input Image", cv_image);
	cv::waitKey(1);

	// Convertir la imagen a nube de puntos
	try {
		cv::Mat disparity_image = toOpencv(disparity->image, "/ueye/disparity");
		int image_type = disparity_image.type();
		//ROS_INFO("Image type: %d, dimensions: [%d x %d]", image_type, disparity->image->image.rows, disparity->image->image.cols);
		ROS_INFO("Image type: %d, dimensions: [%d x %d]", image_type, disparity_image.rows, disparity_image.cols);
		pcl::PointCloud<pcl::PointXYZRGB> cloud = toCloud(cv_image, left_camera_info, right_camera_info, disparity_image);
		ROS_INFO("A");

		// Procesar y publicar resultados
		pcl::PointCloud<pcl::PointXYZRGB> processed = keypointDetection(cv_image, cloud, config_);
		ROS_INFO("B");
		sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(processed, out);
		ROS_INFO("C");
        out.header = image->header;
        point_cloud_publisher_.publish(out);
		ROS_INFO("D");
        sphere_marker_publisher_.publish(toMarkers(processed, image->header));
		ROS_INFO("E");
    } catch (std::exception & e) {
        ROS_ERROR("Error processing frame from topic: %s, reason: %s", image_subscriber_.getTopic().c_str(), e.what());
    }
}



}
