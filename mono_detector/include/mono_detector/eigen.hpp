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
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace mono_detector {

/// Convert a vector to YAML.
inline std::string toYaml(const Eigen::Vector3f & vector) {
	std::string result;
	result.reserve(50);

	result += "{x: ";
	result += std::to_string(vector.x());

	result += ", y: ";
	result += std::to_string(vector.y());

	result += ", z: ";
	result += std::to_string(vector.z());

	result += "}";
	return result;
}

/// Convert a quaternion to YAML.
inline std::string toYaml(const Eigen::Quaternionf & quaternion) {
	std::string result;
	result.reserve(50);

	result += "{x: ";
	result += std::to_string(quaternion.x());

	result += ", y: ";
	result += std::to_string(quaternion.y());

	result += ", z: ";
	result += std::to_string(quaternion.z());

	result += ", w: ";
	result += std::to_string(quaternion.w());
	result += "}";
	return result;
}

/// Convert an isometry to YAML.
inline std::string toYaml(const Eigen::Isometry3f & pose, std::string const & indent = "") {
	std::string result;
	result.reserve(100);

	result.append(indent);
	result.append("position:    ");
	result.append(toYaml(pose.translation()));
	result.push_back('\n');
	result.append(indent);
	result.append("orientation: ");
	result.append(toYaml(Eigen::Quaternionf(pose.rotation())));
	return result;
}

}
