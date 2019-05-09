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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <map>
#include <vector>

namespace YAML {

template <>
struct convert<pcl::PointXYZ> {
	static Node encode(const pcl::PointXYZ & config);
	static bool decode(const Node & node, pcl::PointXYZ & config);
};

template <>
struct convert<pcl::PointCloud<pcl::PointXYZ> > {
	static Node encode(const pcl::PointCloud<pcl::PointXYZ> & config);
	static bool decode(const Node & node, pcl::PointCloud<pcl::PointXYZ> & config);
};

}
