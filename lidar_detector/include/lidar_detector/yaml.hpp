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

#pragma once
#include <yaml-cpp/yaml.h>
#include "config.hpp"

namespace YAML {

template <>
struct convert<lidar_detector::Configuration> {
	static Node encode(const lidar_detector::Configuration & config);
	static bool decode(const Node & node, lidar_detector::Configuration & config);
};

template <>
struct convert<lidar_detector::PassThroughFilter> {
	static Node encode(const lidar_detector::PassThroughFilter & config);
	static bool decode(const Node & node, lidar_detector::PassThroughFilter & config);
};

template <>
struct convert<Eigen::Vector3f> {
	static Node encode(const Eigen::Vector3f & config);
	static bool decode(const Node & node, Eigen::Vector3f & config);
};

template <>
struct convert<lidar_detector::PlaneFilter> {
	static Node encode(const lidar_detector::PlaneFilter & config);
	static bool decode(const Node & node, lidar_detector::PlaneFilter & config);
};

template <>
struct convert<lidar_detector::CircleDetection> {
	static Node encode(const lidar_detector::CircleDetection & config);
	static bool decode(const Node & node, lidar_detector::CircleDetection & config);
};

template <>
struct convert<lidar_detector::CloudEdgeFilter> {
	static Node encode(const lidar_detector::CloudEdgeFilter & config);
	static bool decode(const Node & node, lidar_detector::CloudEdgeFilter & config);
};

template <>
struct convert<lidar_detector::Refinement> {
	static Node encode(const lidar_detector::Refinement & config);
	static bool decode(const Node & node, lidar_detector::Refinement & config);
};

template <>
struct convert<lidar_detector::LidarParameters> {
	static Node encode(const lidar_detector::LidarParameters & config);
	static bool decode(const Node & node, lidar_detector::LidarParameters & config);
};
}
