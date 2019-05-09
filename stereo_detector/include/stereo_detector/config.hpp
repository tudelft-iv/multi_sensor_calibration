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

#pragma once
#include <opencv2/opencv.hpp>

namespace stereo_detector {

struct PassThroughFilter {
	std::string dim;
	float min;
	float max;
};

struct Canny {
	int min;
	int max;
};

struct PlaneFilter {
	float threshold;
	float eps_angle;
	int iterations;
};

struct CircleDetection {
	float threshold;
	int iterations;
	float min_radius;
	float max_radius;
	int cluster_iterations;
	float radius_max_points;
	int max_points_within_radius;
};

struct  Refinement {
	bool refine;
	float width;
	float height;
	float threshold_inlier;
};

struct Configuration {
	std::vector<PassThroughFilter> pass_through_filter;
	Canny canny;
	cv::Rect roi;
	PlaneFilter plane_filter;
	CircleDetection circle_detection;
	double plane_distance;
	bool visualize;
	Refinement refinement;
};

}
