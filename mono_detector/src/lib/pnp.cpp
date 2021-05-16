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

#include "pnp.hpp"
#include "util.hpp"
#include "types.hpp"
#include <pcl/common/transforms.h>

namespace mono_detector {

namespace {

	/// Rotate a cv point
	cv::Point3f rotate(cv::Mat const & rvec, cv::Point3f const & point) {
		cv::Mat rotation;
		cv::Rodrigues(rvec, rotation);
		cv::Mat mat(1, 3, CV_32FC1); // Or should this be 3x1?
		mat.at<float>(0) = point.x;
		mat.at<float>(1) = point.y;
		mat.at<float>(2) = point.z;
		mat = rotation * mat;
		return {mat.at<float>(0), mat.at<float>(1), mat.at<float>(2)};
	}

	/// Translate a cv point
	cv::Point3f translate(cv::Mat const & tvec, cv::Point3f const & point) {
		return {
			tvec.at<float>(0) + point.x,
			tvec.at<float>(1) + point.y,
			tvec.at<float>(2) + point.z
		};
	}

	/// Isometry (rigid) transformation of a point using rvec, tvec
	cv::Point3f transform(cv::Mat const & rvec, cv::Mat const & tvec, cv::Point3f const & point) {
		return translate(tvec, rotate(rvec, point));
	}

	/// Transform a vector of points using an opencv rvec, tvec isometry
	std::vector<cv::Point3f> transform(cv::Mat const & rvec, cv::Mat const & tvec, std::vector<cv::Point3f> const & points) {
		std::vector<cv::Point3f> out;
		for (const auto & point : points) {
			out.push_back(transform(rvec, tvec, point));
		}
		return out;
	}


	/// Function to convert a vector of points to a 2-channel Mat for use in e.g. solvePnP
	cv::Mat detectionToMat(std::vector<cv::Point2f> const & detection) {
		cv::Mat result{cv::Size{1, 4}, CV_32FC2};
		for (std::size_t i = 0; i < 4; ++i) {
			result.at<cv::Point2f>(i) = {detection.at(i).x, detection.at(i).y};
		}
		return result;
	}

}

/// Solves the pose using opencv pnp and returns the transformed point cloud of object points
Eigen::Isometry3f solvePose(
	std::vector<cv::Point2f> const & image_points,
	std::vector<cv::Point3f> const & object_points,
	image_geometry::PinholeCameraModel const & intrinsics
) {

	std::vector<double> rvec, tvec; // Unfortunately we can only work with double in solvepnp or otherwise assertion violated

	// Get projection matrix from intrinsics 
	cv::Mat intrinsic_matrix = cv::Mat(intrinsics.intrinsicMatrix()); //Note: we assume rectified images

	// Solve pose using known points
	if (!solvePnP(object_points, detectionToMat(image_points), intrinsic_matrix, 0*intrinsics.distortionCoeffs(), rvec, tvec, false)) {
		throw std::runtime_error("Unable to solve PnP");
	}

	return toEigen(rvec, tvec);
}


}
