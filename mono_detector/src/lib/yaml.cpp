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

#include "yaml.hpp"

namespace YAML {

Node convert<cv::Point3f>::encode(const cv::Point3f & config) {
	Node node;
	node["x"]             = config.x;
	node["y"]             = config.y;
	node["z"]             = config.z;
	return node;
}
bool convert<cv::Point3f>::decode(const Node & node, cv::Point3f & config) {
	config.x              = node["x"].as<float>();
	config.y              = node["y"].as<float>();
	config.z              = node["z"].as<float>();
	return true;
}

Node convert<mono_detector::CannyConfig>::encode(const mono_detector::CannyConfig & config) {
	Node node;
	node["apply"]         = config.apply ? "true" : "false";
	node["min_threshold"] = std::to_string(config.min_threshold);
	node["max_threshold"] = std::to_string(config.max_threshold);
	return node;
}
bool convert<mono_detector::CannyConfig>::decode(const Node & node, mono_detector::CannyConfig & config) {
	config.apply          = node["apply"].as<bool>();
	config.min_threshold  = node["min_threshold"].as<int>();
	config.max_threshold  = node["max_threshold"].as<int>();
	return true;
}

Node convert<mono_detector::GaussConfig>::encode(const mono_detector::GaussConfig & config) {
	Node node;
	node["apply"]         = config.apply ? "true" : "false";
	node["ksize_x"]       = std::to_string(config.ksize_x);
	node["ksize_y"]       = std::to_string(config.ksize_y);
	node["sigma_x"]       = std::to_string(config.sigma_x);
	node["sigma_y"]       = std::to_string(config.sigma_y);
	return node;
}
bool convert<mono_detector::GaussConfig>::decode(const Node & node, mono_detector::GaussConfig & config) {
	config.apply          = node["apply"].as<bool>();
	config.ksize_x        = node["ksize_x"].as<int>();
	config.ksize_y        = node["ksize_y"].as<int>();
	config.sigma_x        = node["sigma_x"].as<float>();
	config.sigma_y        = node["sigma_y"].as<float>();
	return true;
}

Node convert<mono_detector::HoughConfig>::encode(const mono_detector::HoughConfig & config) {
	Node node;
	node["dp"]            = std::to_string(config.dp);
	node["min_dist"]      = std::to_string(config.min_dist);
	node["param1"]        = std::to_string(config.param1);
	node["param2"]        = std::to_string(config.param2);
	node["min_radius"]    = std::to_string(config.min_radius);
	node["max_radius"]    = std::to_string(config.max_radius);
	return node;
}
bool convert<mono_detector::HoughConfig>::decode(const Node & node, mono_detector::HoughConfig & config) {
	config.dp             = node["dp"].as<double>();
	config.min_dist       = node["min_dist"].as<double>();
	config.param1         = node["param1"].as<double>();
	config.param2         = node["param2"].as<double>();
	config.min_radius     = node["min_radius"].as<int>();
	config.max_radius     = node["max_radius"].as<int>();
	return true;
}

Node convert<mono_detector::Configuration>::encode(const mono_detector::Configuration & config) {
	Node node;
	node["pre_gauss"]     = Node(config.pre_blur);
	node["canny"]         = Node(config.edge_detection);
	node["post_gauss"]    = Node(config.post_blur);
	node["hough"]         = Node(config.hough_config);
	node["roi"]           = Node(config.roi);
	node["visualize"]     = config.visualize ? "true" : "false";
	return node;
}
bool convert<mono_detector::Configuration>::decode(const Node & node, mono_detector::Configuration & config) {
	config.pre_blur       = node["pre_gauss"].as<mono_detector::GaussConfig>();
	config.edge_detection = node["canny"].as<mono_detector::CannyConfig>();
	config.post_blur      = node["post_gauss"].as<mono_detector::GaussConfig>();
	config.hough_config   = node["hough"].as<mono_detector::HoughConfig>();
	config.roi            = node["roi"].as<cv::Rect>();
	config.visualize      = node["visualize"].as<bool>();
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


}
