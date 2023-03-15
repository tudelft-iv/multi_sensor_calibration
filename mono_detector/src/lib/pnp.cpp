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

#include "mono_detector/pnp.hpp"

#include <pcl/common/transforms.h>

#include "mono_detector/util.hpp"
#include "mono_detector/types.hpp"

namespace mono_detector {

namespace {

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
  CameraModel const & intrinsics
) {
  // We first undistort points manually and then pass identity and zeros matrices to solvePnP,
  // as this way we can support both pinhole and fisheye models.
  cv::Mat undistorted = cv::Mat::zeros(1, 4, CV_32FC2);
  if (intrinsics.distortion_model == CameraModel::DistortionModel::PINHOLE) {
    cv::undistortPoints(detectionToMat(image_points), undistorted, intrinsics.camera_matrix, intrinsics.distortion_parameters);
  } else {
    cv::fisheye::undistortPoints(detectionToMat(image_points), undistorted, intrinsics.camera_matrix, intrinsics.distortion_parameters);
  }

  std::vector<double> rvec, tvec; // Unfortunately we can only work with double in solvepnp or otherwise assertion violated
  cv::Mat camera_matrix_eye = cv::Mat::eye(3, 3, CV_32F);
  cv::Mat distortion_parameters_zeros = cv::Mat::zeros(1, 5, CV_32F);

  // Solve pose using known points
  if (!solvePnP(object_points, undistorted, camera_matrix_eye, distortion_parameters_zeros, rvec, tvec, false)) {
    throw std::runtime_error("Unable to solve PnP");
  }

  return toEigen(rvec, tvec);
}

}
