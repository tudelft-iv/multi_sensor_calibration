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
#include <array>
#include "types.hpp"

namespace mono_detector {

/// Function to calculate the location of the calibration board in the image (pixel coordinates)
void detectMono(
	cv::Mat const & image,
	Configuration const & configuration,
	std::vector<cv::Point2f> & centers,
	std::vector<float> & radi
);

}
