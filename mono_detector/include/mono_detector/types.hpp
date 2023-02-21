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
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace mono_detector {

struct CameraModel {
	enum DistortionModel {
		PINHOLE,
		FISHEYE
	};

	DistortionModel distortion_model;
	cv::Mat camera_matrix;
	cv::Mat distortion_parameters;

	void fromCameraInfo(sensor_msgs::CameraInfo camera_info) {
		if (camera_info.distortion_model == sensor_msgs::distortion_models::PLUMB_BOB) {
			distortion_model = DistortionModel::PINHOLE;
			distortion_parameters = cv::Mat(1, 5, CV_32F, camera_info.D.data()).clone();
		} else if (camera_info.distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
			distortion_model = DistortionModel::FISHEYE;
			distortion_parameters = cv::Mat(1, 4, CV_32F, camera_info.D.data()).clone();
		} else {
			throw std::runtime_error("Unsupported distortion model: " + camera_info.distortion_model);
		}

		camera_matrix = cv::Mat(3, 3, CV_32F, camera_info.K.data()).clone();
	}
};

struct GaussConfig {
	bool apply;
	int ksize_x;
	int ksize_y;
	float sigma_x;
	float sigma_y;
};

struct CannyConfig {
	bool apply;
	int min_threshold;
	int max_threshold;
};

struct HoughConfig {
	double dp;
	double min_dist;
	double param1;
	double param2;
	int min_radius;
	int max_radius;
};

struct Configuration {
	GaussConfig pre_blur;
	CannyConfig edge_detection;
	GaussConfig post_blur;
	HoughConfig hough_config;
	cv::Rect roi;
	bool visualize;
};

}
