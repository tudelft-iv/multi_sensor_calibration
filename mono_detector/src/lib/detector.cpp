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

#include "detector.hpp"
#include "util.hpp"
#include "types.hpp"
#include <stdexcept>

namespace mono_detector {

void detectMono(
	cv::Mat const & image,
	Configuration const & configuration,
	std::vector<cv::Point2f> & centers,
	std::vector<float> & radi
) {
	// Continue with a deep copy to prevent modification of original data
	cv::Mat processed = image.clone();

	// Optionally blur the image
	if (configuration.pre_blur.apply) {
		processed = gaussianBlur(processed, configuration.pre_blur);
	}

	// Optionally do edge detection on the image
	if (configuration.edge_detection.apply) {
		cv::Mat canny_filtered; // source and destination cannot be the same in canny filter
		cv::Canny(processed, canny_filtered, configuration.edge_detection.min_threshold, configuration.edge_detection.max_threshold);
		processed = canny_filtered;
	}

	// Optionally blur the image (AGAIN?)
	if (configuration.post_blur.apply) {
		processed = gaussianBlur(processed, configuration.post_blur);
	}

	// Detect circles with hough (because that's available in opencv)
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(
		processed,
		circles,
		CV_HOUGH_GRADIENT,
		configuration.hough_config.dp,
		configuration.hough_config.min_dist,
		configuration.hough_config.param1,
		configuration.hough_config.param2,
		configuration.hough_config.min_radius,
		configuration.hough_config.max_radius
	);

	// ToDo: Filter the 4 best scoring circles
	
	// visualize result
	if (configuration.visualize) {
		visualize(image, circles, configuration.roi);
	}

	// Filter out circles with it's center outside the roi
	circles.erase(std::remove_if(circles.begin(), circles.end(), [configuration](cv::Vec3f p){return !configuration.roi.contains(cv::Point(p[0], p[1]));}), circles.end());

	// Filter circles based on median closest to median x and y:
	if (circles.size() > 4) {
		// Compute median radius of circles
		std::vector<double> median_circle = compute_median_circle(circles); 
		double median_x = median_circle[0]; // Compute median for first dimension of cicle: aka x
		double median_y = median_circle[1]; // Compute median for first dimension of cicle: aka y
		
		// Compute distances wrt median
		std::vector<double> distances_from_median;
		for (const auto & circle : circles ) {
			distances_from_median.push_back(sqrt((circle[0]-median_x)*(circle[0]-median_x)+(circle[1]-median_y)*(circle[1]-median_y)));
		} 

		// Remove untill we have 4 detections
		while (circles.size() > 4) {
			// Compute index of largest element:
			auto result = std::max_element(distances_from_median.begin(), distances_from_median.end());
			auto index = std::distance(distances_from_median.begin(), result);

			// Erase cirlce at this index
			distances_from_median.erase(distances_from_median.begin() + index);
			circles.erase(circles.begin() + index);
		}
	}

	// Only continue if the number of circles is exactly 4
	if (circles.size() != 4) {
		throw std::runtime_error("Number of circles found: '" + std::to_string(circles.size()) + "', but should be exactly 4.");
	}

	// Convert to vector of cv point 2f
	centers = toCvPoint2fVector(circles);

	// Sort in order to match correspondences with object points
	cv::Point2f origin = calculateCenter(centers);
	std::sort(centers.begin(), centers.end(), [origin](cv::Point2f a, cv::Point2f b) {
		return std::atan((a.y - origin.y) / (a.x - origin.x)) > std::atan((b.y - origin.y) / (b.x - origin.x));
	});

	// Convert to radi (float)
	for (const auto & circle : circles) {
		radi.push_back(circle[2]);
	}
}


}
