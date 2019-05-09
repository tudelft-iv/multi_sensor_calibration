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
#include <string>


namespace urdf_calibration {

/// Convert a vector3d to a string of three space separated values
inline std::string toString(Eigen::Vector3d const & vector) {
	return std::to_string(vector.x()) + " " + std::to_string(vector.y()) + " " + std::to_string(vector.z());
}

/// Convert quaternion to string
inline std::string toString(Eigen::Translation3d const & t) {
	std::string out("translation part: x: " + std::to_string(t.x()) + " y: " + std::to_string(t.y()) + " z: " + std::to_string(t.z()));
	return out;
}

/// Convert quaternion to string
inline std::string toString(Eigen::Quaterniond const & q) {
	std::string out("rotation part: x: " + std::to_string(q.x()) + " y: " + std::to_string(q.y()) + " z: " + std::to_string(q.z()) + " w: " + std::to_string(q.w()));
	return out;
}

/// Convert quaternion to string
inline std::string toString(Eigen::Isometry3d const & isometry) {
	return toString(Eigen::Translation3d(isometry.translation())) + "\n" + toString(Eigen::Quaterniond(isometry.rotation()));
}


}
