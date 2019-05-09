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
#include <yaml-cpp/yaml.h>
#include "types.hpp"

namespace YAML {

template <>
struct convert<urdf_calibration::Calibration> {
	static Node encode(const urdf_calibration::Calibration & config);
	static bool decode(const Node & node, urdf_calibration::Calibration & config);
};

template <>
struct convert<Eigen::Isometry3d> {
	static Node encode(const Eigen::Isometry3d & config);
	static bool decode(const Node & node, Eigen::Isometry3d & config);
};


}
