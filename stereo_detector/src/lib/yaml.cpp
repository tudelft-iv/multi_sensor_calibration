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

#include "yaml.hpp"

namespace YAML {

Node convert<stereo_detector::PassThroughFilter>::encode(const stereo_detector::PassThroughFilter & config) {
	Node node;
	node["dim"] = Node(config.dim);
	node["min"] = Node(config.min);
	node["max"] = Node(config.max);
	return node;
}
bool convert<stereo_detector::PassThroughFilter>::decode(const Node & node, stereo_detector::PassThroughFilter & config) {
	config.dim = node["dim"].as<std::string>();
	config.min = node["min"].as<float>();
	config.max = node["max"].as<float>();
	return true;
}


Node convert<stereo_detector::Canny>::encode(const stereo_detector::Canny & config) {
	Node node;
	node["min"] = std::to_string(config.min);
	node["max"] = std::to_string(config.max);
	return node;
}
bool convert<stereo_detector::Canny>::decode(const Node & node, stereo_detector::Canny & config) {
	config.min = node["min"].as<int>();
	config.max = node["max"].as<int>();
	return true;
}

Node convert<cv::Rect>::encode(const cv::Rect & config) {
	Node node;
	node["x"]           = Node(config.x);
	node["y"]           = Node(config.y);
	node["width"]       = Node(config.width);
	node["height"]      = Node(config.height);
	return node;
}
bool convert<cv::Rect>::decode(const Node & node, cv::Rect & config) {
	config.x              = node["x"].as<int>();
	config.y              = node["y"].as<int>();
	config.width          = node["width"].as<int>();
	config.height         = node["height"].as<int>();
	return true;
}


Node convert<stereo_detector::PlaneFilter>::encode(const stereo_detector::PlaneFilter & config) {
	Node node;
	node["threshold"] = std::to_string(config.threshold);
	node["eps_angle"] = std::to_string(config.eps_angle);
	return node;
}
bool convert<stereo_detector::PlaneFilter>::decode(const Node & node, stereo_detector::PlaneFilter & config) {
	config.threshold  = node["threshold"].as<float>();
	config.eps_angle  = node["eps_angle"].as<float>();
	config.iterations = node["iterations"].as<int>();
	return true;
}



Node convert<stereo_detector::CircleDetection>::encode(const stereo_detector::CircleDetection & config) {
	Node node;
	node["threshold"]                = std::to_string(config.threshold);
	node["iterations"]               = std::to_string(config.iterations);
	node["min_radius"]               = std::to_string(config.min_radius);
	node["max_radius"]               = std::to_string(config.max_radius);
	node["cluster_iterations"]       = std::to_string(config.cluster_iterations);
	node["radius_max_points"]        = std::to_string(config.radius_max_points);
	node["max_points_within_radius"] = std::to_string(config.max_points_within_radius);
	return node;
}
bool convert<stereo_detector::CircleDetection>::decode(const Node & node, stereo_detector::CircleDetection & config) {
	config.threshold                = node["threshold"].as<float>();
	config.iterations               = node["iterations"].as<int>();
	config.min_radius               = node["min_radius"].as<float>();
	config.max_radius               = node["max_radius"].as<float>();
	config.cluster_iterations       = node["cluster_iterations"].as<int>();
	config.radius_max_points        = node["radius_max_points"].as<float>();
	config.max_points_within_radius = node["max_points_within_radius"].as<int>();
	return true;
}


Node convert<stereo_detector::Refinement>::encode(const stereo_detector::Refinement & config) {
	Node node;
	node["refine"] = Node(config.refine);
	node["width"] = Node(config.width);
	node["height"] = Node(config.height);
	node["threshold_inlier"] = Node(config.threshold_inlier);
	return node;
}

bool convert<stereo_detector::Refinement>::decode(const Node & node, stereo_detector::Refinement & config) {
	config.refine = node["refine"].as<bool>();
	config.width = node["width"].as<float>();
	config.height = node["height"].as<float>();
	config.threshold_inlier = node["threshold_inlier"].as<float>();
	return true;
}

Node convert<stereo_detector::Configuration>::encode(const stereo_detector::Configuration & config) {
	Node node;
	node["pass_through_filter"] = Node(config.pass_through_filter);
	node["canny"]               = Node(config.canny);
	node["roi"]                 = Node(config.roi);
	node["plane_filter"]        = Node(config.plane_filter);
	node["circle_detection"]    = Node(config.circle_detection);
	node["plane_distance"]      = Node(config.plane_distance);
	node["visualize"]           = config.visualize ? "true" : "false";
	node["refinement"]      = Node(config.refinement);
	return node;
}
bool convert<stereo_detector::Configuration>::decode(const Node & node, stereo_detector::Configuration & config) {
	config.pass_through_filter  = node["pass_through_filter"].as<std::vector<stereo_detector::PassThroughFilter> >();
	config.canny                = node["canny"].as<stereo_detector::Canny>();
	config.roi                  = node["roi"].as<cv::Rect>();
	config.plane_filter         = node["plane_filter"].as<stereo_detector::PlaneFilter>();
	config.circle_detection     = node["circle_detection"].as<stereo_detector::CircleDetection>();
	config.plane_distance       = node["plane_distance"].as<float>();
	config.visualize            = node["visualize"].as<bool>();
	config.refinement            = node["refinement"].as<stereo_detector::Refinement>();
	return true;
}

}
