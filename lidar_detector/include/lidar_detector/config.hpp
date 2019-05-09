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
#include <Eigen/Dense>

namespace lidar_detector {

struct PassThroughFilter {
	std::string dim;
	float min;
	float max;
};

struct PlaneFilter {
	float distance_threshold;
	Eigen::Vector3f axis;
	float eps_angle;
	int max_iterations;
	bool set_negative;
	int model_type;
	bool return_projected;
};

struct CloudEdgeFilter {
	float threshold;
	float radius;
};

struct CircleDetection {
	float distance_threshold;
	int max_iterations;
	float min_radius;
	float max_radius;
	int cluster_iterations;
	int max_points_within_radius;
	float radius_max_points;
};

struct  Refinement {
	bool refine;
	float width;
	float height;
	float threshold_inlier;
};

struct LidarParameters {
	int number_layers;
};

struct Configuration {
	bool visualize;
	LidarParameters lidar_parameters;
	std::vector<PassThroughFilter> pass_through_filter;
	PlaneFilter ground_floor_filter;
	PlaneFilter calibration_board_filter;
	CircleDetection circle_detection;
	CloudEdgeFilter cloud_edge_filter;
	Refinement refinement;
};

}
