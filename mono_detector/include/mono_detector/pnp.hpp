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
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include "types.hpp"
#include "util.hpp"
#include <pcl/common/common.h>
#include <image_geometry/pinhole_camera_model.h> // ToDo: Remove ros dependency

namespace mono_detector {

// Replace by point cloud
Eigen::Isometry3f solvePose(
	std::vector<cv::Point2f> const & image_points,
	std::vector<cv::Point3f> const & object_points,
	image_geometry::PinholeCameraModel const & intrinsics
);

}
