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
#include <kdl/tree.hpp>
#include <boost/algorithm/string.hpp>

namespace urdf_calibration {

	namespace {

		/// Split the string in separate values (space separated)
		std::vector<std::string> split(std::string const & in) {
			std::vector<std::string> split_vec;
		boost::split( split_vec, in, boost::is_any_of(" "));
		return split_vec;
		}

	}

/// Convert a KDL rotation to an Eigen quaternion.
inline Eigen::Quaterniond toEigen(KDL::Rotation const & input) {
	Eigen::Quaterniond result;
	input.GetQuaternion(result.x(), result.y(), result.z(), result.w());
	return result;
}

/// Convert a KDL vector to an Eigen vector.
inline Eigen::Vector3d toEigen(KDL::Vector const & input) {
	return Eigen::Vector3d{input[0], input[1], input[2]};
}

/// Convert a KDL frame to an Eigen isometry.
inline Eigen::Isometry3d toEigen(KDL::Frame const & input) {
	return Eigen::Translation3d(toEigen(input.p)) * toEigen(input.M);
}

/// Convert the xyz and rpy from string to isometry3d
inline Eigen::Isometry3d toEigen(std::string const & xyz, std::string const & rpy) {
	std::vector<std::string> xyz_vector = split(xyz);
	std::vector<std::string> rpy_vector = split(rpy);

	Eigen::Quaterniond q =
		Eigen::AngleAxisd(std::stod(rpy_vector.at(2)), Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(std::stod(rpy_vector.at(1)), Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(std::stod(rpy_vector.at(0)), Eigen::Vector3d::UnitX());

	Eigen::Translation3d t(std::stod(xyz_vector.at(0)), std::stod(xyz_vector.at(1)), std::stod(xyz_vector.at(2)));
	return t * q;
}

}
