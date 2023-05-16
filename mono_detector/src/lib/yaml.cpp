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

#include "mono_detector/yaml.hpp"

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

Node convert<mono_detector::Configuration>::encode(const mono_detector::Configuration & config) {
  Node node;
  node["marker_size"]   = config.marker_size;
  node["roi"]           = Node(config.roi);
  node["visualize"]     = config.visualize ? "true" : "false";
  return node;
}
bool convert<mono_detector::Configuration>::decode(const Node & node,
                                                   mono_detector::Configuration & config) {
  config.marker_size    = node["marker_size"].as<float>();
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

}  // namespace YAML
