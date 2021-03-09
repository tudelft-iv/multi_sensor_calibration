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

pcl::PointXYZ keypointDetection(radar_msgs::RadarDetectionArray const & in, float const min, float const max,
                                        float const min_range, float const max_range,
                                        bool const select_range, bool const select_min) {
	pcl::PointXYZ point;
	float best_candidate_value = select_min ? std::numeric_limits<float>::max():std::numeric_limits<float>::lowest();
	std::vector<radar_msgs::RadarDetection> vector_detections = in.detections;
	for (auto detection:vector_detections) {

		float x = detection.position.x;
		float y = detection.position.y;
		float range = sqrt(x*x+y*y); // compute range using x and y
		float rcs = detection.amplitude; // RCS value
		// select the best candidate either based on range or on rcs
		float selection = select_range ? range : rcs;
		// the best candidate can be either the lowest or the highest value
		bool is_best_candidate = select_min ? selection < best_candidate_value : selection > best_candidate_value;
		if (is_best_candidate && rcs > min && rcs < max && range < max_range && range > min_range) {
			best_candidate_value = selection;
			point.x = x;
			point.y = y;
			point.z = detection.position.z;
		}
	}
	return point;
}

}
