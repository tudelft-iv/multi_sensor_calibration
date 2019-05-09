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
struct convert<cv::Point3f> {
	static Node encode(const cv::Point3f & config);
	static bool decode(const Node & node, cv::Point3f & config);
};

template <>
struct convert<mono_detector::CannyConfig> {
	static Node encode(const mono_detector::CannyConfig & config);
	static bool decode(const Node & node, mono_detector::CannyConfig & config);
};

template <>
struct convert<mono_detector::GaussConfig> {
	static Node encode(const mono_detector::GaussConfig & config);
	static bool decode(const Node & node, mono_detector::GaussConfig & config);
};

template <>
struct convert<mono_detector::HoughConfig> {
	static Node encode(const mono_detector::HoughConfig & config);
	static bool decode(const Node & node, mono_detector::HoughConfig & config);
};

template <>
struct convert<mono_detector::Configuration> {
	static Node encode(const mono_detector::Configuration & config);
	static bool decode(const Node & node, mono_detector::Configuration & config);
};

template <>
struct convert<cv::Rect> {
	static Node encode(const cv::Rect & config);
	static bool decode(const Node & node, cv::Rect & config);
};

}
