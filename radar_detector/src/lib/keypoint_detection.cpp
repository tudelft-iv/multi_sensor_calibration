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

#include "radar_detector/keypoint_detection.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace radar_detector {

pcl::PointXYZ keypointDetection(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& in,
																float const min, float const max,
																float const min_range, float const max_range,
																bool const select_range, bool const select_min) {
	pcl::PointXYZ point;
	float best_candidate_value = select_min ? std::numeric_limits<float>::max() : std::numeric_limits<float>::lowest();

	std::size_t n_points = (*in).width;
	sensor_msgs::PointCloud2ConstIterator<float> iter_x(*in, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(*in, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(*in, "z");
	sensor_msgs::PointCloud2ConstIterator<float> iter_rcs(*in, "RCS");
	for (std::size_t i = 0; i < n_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rcs) {

		float x = *iter_x;
		float y = *iter_y;
		float range = sqrt(x*x + y*y); // compute range using x and y
		float rcs = *iter_rcs; // RCS value
		// select the best candidate either based on range or on rcs
		float selection = select_range ? range : rcs;
		// the best candidate can be either the lowest or the highest value
		bool is_best_candidate = select_min ? selection < best_candidate_value : selection > best_candidate_value;
		if (is_best_candidate && rcs > min && rcs < max && range < max_range && range > min_range) {
			best_candidate_value = selection;
			point.x = x;
			point.y = y;
			point.z = *iter_z;
		}
	}
	return point;
}

}
