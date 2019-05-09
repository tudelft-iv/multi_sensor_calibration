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
#include <pcl/segmentation/sac_segmentation.h> // to be able to parse model type

namespace YAML {

Node convert<lidar_detector::Configuration>::encode(const lidar_detector::Configuration & config) {
	Node node;
	node["visualize"]                = Node(config.visualize);
	node["lidar_parameters"]         = Node(config.lidar_parameters);
	node["pass_through_filter"]      = Node(config.pass_through_filter);
	node["ground_floor_filter"]      = Node(config.ground_floor_filter);
	node["calibration_board_filter"] = Node(config.calibration_board_filter);
	node["cloud_edge_filter"]        = Node(config.cloud_edge_filter);
	node["circle_detection"]         = Node(config.circle_detection);
	node["refinement"]                = Node(config.refinement);
	return node;
}
bool convert<lidar_detector::Configuration>::decode(const Node & node, lidar_detector::Configuration & config) {
	config.visualize                = node["visualize"].as<bool>();
	config.lidar_parameters         = node["lidar_parameters"].as<lidar_detector::LidarParameters>();
	config.pass_through_filter      = node["pass_through_filter"].as<std::vector<lidar_detector::PassThroughFilter> >();
	config.ground_floor_filter      = node["ground_floor_filter"].as<lidar_detector::PlaneFilter>();
	config.calibration_board_filter = node["calibration_board_filter"].as<lidar_detector::PlaneFilter>();
	config.cloud_edge_filter        = node["cloud_edge_filter"].as<lidar_detector::CloudEdgeFilter>();
	config.circle_detection         = node["circle_detection"].as<lidar_detector::CircleDetection>();
	config.refinement         = node["refinement"].as<lidar_detector::Refinement>();
	return true;
}

Node convert<lidar_detector::LidarParameters>::encode(const lidar_detector::LidarParameters & config) {
	Node node;
	node["number_layers"] = Node(config.number_layers);
	return node;
}

bool convert<lidar_detector::LidarParameters>::decode(const Node & node, lidar_detector::LidarParameters & config) {
	config.number_layers = node["number_layers"].as<int>();
	return true;
}

Node convert<lidar_detector::Refinement>::encode(const lidar_detector::Refinement & config) {
	Node node;
	node["refine"] = Node(config.refine);
	node["width"] = Node(config.width);
	node["height"] = Node(config.height);
	node["threshold_inlier"] = Node(config.threshold_inlier);
	return node;
}

bool convert<lidar_detector::Refinement>::decode(const Node & node, lidar_detector::Refinement & config) {
	config.refine = node["refine"].as<bool>();
	config.width = node["width"].as<float>();
	config.height = node["height"].as<float>();
	config.threshold_inlier = node["threshold_inlier"].as<float>();
	return true;
}

Node convert<lidar_detector::PassThroughFilter>::encode(const lidar_detector::PassThroughFilter & config) {
	Node node;
	node["dim"] = Node(config.dim);
	node["min"] = Node(config.min);
	node["max"] = Node(config.max);
	return node;
}
bool convert<lidar_detector::PassThroughFilter>::decode(const Node & node, lidar_detector::PassThroughFilter & config) {
	config.dim = node["dim"].as<std::string>();
	config.min = node["min"].as<float>();
	config.max = node["max"].as<float>();
	return true;
}


Node convert<Eigen::Vector3f>::encode(const Eigen::Vector3f & config) {
	Node node;
	node["x"] = Node(config.x());
	node["y"] = Node(config.y());
	node["z"] = Node(config.z());
	return node;
}
bool convert<Eigen::Vector3f>::decode(const Node & node, Eigen::Vector3f & config) {
	config.x() = node["x"].as<float>();
	config.y() = node["y"].as<float>();
	config.z() = node["z"].as<float>();
	return true;
}


Node convert<lidar_detector::PlaneFilter>::encode(const lidar_detector::PlaneFilter & config) {
	Node node;
	node["distance_threshold"] = Node(config.distance_threshold);
	node["axis"]               = Node(config.axis);
	node["eps_angle"]          = Node(config.eps_angle);
	node["max_iterations"]     = Node(config.max_iterations);
	node["set_negative"]       = Node(config.set_negative);
	node["return_projected"]   = Node(config.return_projected);
	if (config.model_type == pcl::SACMODEL_PARALLEL_PLANE) { node["model_type"] = "sacmodel_parallel_plane"; }
	else if (config.model_type == pcl::SACMODEL_PERPENDICULAR_PLANE) { node["model_type"] = "sacmodel_perpendicular_plane"; }
	else { throw std::runtime_error("Conversion of SAC Model type to yaml not yet implemented."); }
	return node;
}
bool convert<lidar_detector::PlaneFilter>::decode(const Node & node, lidar_detector::PlaneFilter & config) {
	config.distance_threshold  = node["distance_threshold"].as<float>();
	config.axis                = node["axis"].as<Eigen::Vector3f>();
	config.eps_angle           = node["eps_angle"].as<float>();
	config.max_iterations      = node["max_iterations"].as<int>();
	config.set_negative        = node["set_negative"].as<bool>();
	config.return_projected    = node["return_projected"].as<bool>();
	if (node["model_type"].as<std::string>() == "sacmodel_parallel_plane") { config.model_type = pcl::SACMODEL_PARALLEL_PLANE; }
	else if (node["model_type"].as<std::string>() == "sacmodel_perpendicular_plane") { config.model_type = pcl::SACMODEL_PERPENDICULAR_PLANE; }
	else { throw std::runtime_error("Parser of yaml SAC Model type '" + node["model_type"].as<std::string>() + "' is not implemented yet."); }
	return true;
}


Node convert<lidar_detector::CloudEdgeFilter>::encode(const lidar_detector::CloudEdgeFilter & config) {
	Node node;
	node["threshold"]          = Node(config.threshold);
	node["radius"]             = Node(config.radius);
	return node;
}
bool convert<lidar_detector::CloudEdgeFilter>::decode(const Node & node, lidar_detector::CloudEdgeFilter & config) {
	config.threshold           = node["threshold"].as<float>();
	config.radius              = node["radius"].as<float>();
	return true;
}


Node convert<lidar_detector::CircleDetection>::encode(const lidar_detector::CircleDetection & config) {
	Node node;
	node["distance_threshold"]       = Node(config.distance_threshold);
	node["max_iterations"]           = Node(config.max_iterations);
	node["min_radius"]               = Node(config.min_radius);
	node["max_radius"]               = Node(config.max_radius);
	node["cluster_iterations"]       = Node(config.cluster_iterations);
	node["max_points_within_radius"] = Node(config.max_points_within_radius);
	node["radius_max_points"]        = Node(config.radius_max_points);
	return node;
}
bool convert<lidar_detector::CircleDetection>::decode(const Node & node, lidar_detector::CircleDetection & config) {
	config.distance_threshold        = node["distance_threshold"].as<float>();
	config.max_iterations            = node["max_iterations"].as<int>();
	config.min_radius                = node["min_radius"].as<float>();
	config.max_radius                = node["max_radius"].as<float>();
	config.cluster_iterations        = node["cluster_iterations"].as<int>();
	config.max_points_within_radius  = node["max_points_within_radius"].as<int>();
	config.radius_max_points         = node["radius_max_points"].as<float>();
	return true;
}




}
