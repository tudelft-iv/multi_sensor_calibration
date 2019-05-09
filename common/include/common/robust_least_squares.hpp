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

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <iterator>
#include <limits>

// 3D refinement
Eigen::Matrix3Xd refine3D(Eigen::Matrix3Xd calibration_board, Eigen::Matrix3Xd detections, double threshold_inlier = 0.05);

// 2D Refinement
Eigen::Matrix3Xd refine2D(Eigen::Matrix3Xd calibration_board, Eigen::Matrix3Xd detections, double threshold_inlier = 0.05);
