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

#include "keypoint_detection.hpp"


namespace radar_detector {

pcl::PointXYZ keypointDetection(radar_msgs::RadarDetectionArray const & in, float const min, float const max, float const min_range, float const max_range) {
	pcl::PointXYZ point;
	float closest_range = std::numeric_limits<float>::max();
	std::vector<radar_msgs::RadarDetection> vector_detections = in.detections;
	for (std::size_t i = 0; i < vector_detections.size(); ++i) {
		float x = vector_detections.at(i).position.x;
		float y = vector_detections.at(i).position.y;
		double range = sqrt(x*x+y*y); // compute range using x and y
		float rcs = vector_detections.at(i).amplitude; // RCS value
		if (rcs > min && rcs < max && range < closest_range && range > min_range && range < max_range) {
			closest_range = range;
			point.x = x;
			point.y = y;
			point.z = rcs;
		}
	}
	return point;
}

}
