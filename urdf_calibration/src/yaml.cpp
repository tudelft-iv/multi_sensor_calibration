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

Node convert<urdf_calibration::Calibration>::encode(const urdf_calibration::Calibration & config) {
	Node node;
	// TODO: Implement yaml conversion for calibration
	throw std::runtime_error("conversion from calibration to yaml node not implemented yet");
	return node;
}
bool convert<urdf_calibration::Calibration>::decode(const Node & node, urdf_calibration::Calibration & config) {
	config.source    = node["source"].as<std::string>();
	config.target    = node["target"].as<std::string>();
	config.transform = node["transform"].as<Eigen::Isometry3d>();
	return true;
}

Node convert<Eigen::Isometry3d>::encode(const Eigen::Isometry3d & config) {
	Node node;
	// TODO: Implement yaml conversion for calibration
	throw std::runtime_error("conversion from isometry to yaml node not implemented yet");
	return node;
}
bool convert<Eigen::Isometry3d>::decode(const Node & node, Eigen::Isometry3d & config) {
	Eigen::Matrix4d m;
	for (std::size_t i = 0; i < node.size(); ++i) {
		YAML::Node row = node[i];
		for (std::size_t j = 0; j < row.size(); ++j) {
			m(i,j) = row[j].as<double>();
		}
	}
	config.matrix() = m;
	return true;
}

}
