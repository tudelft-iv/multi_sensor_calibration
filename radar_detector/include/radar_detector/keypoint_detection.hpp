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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <radar_msgs/RadarDetectionArray.h>
#include <radar_msgs/RadarDetection.h>


namespace radar_detector {


// TODO: min should be std::numeric_limits<float>::lowest()
pcl::PointXYZ keypointDetection(radar_msgs::RadarDetectionArray const & in, float const min = -std::numeric_limits<float>::max(), float const max = std::numeric_limits<float>::max(), float const min_range = -std::numeric_limits<float>::max(), float const max_range = std::numeric_limits<float>::max());


}
