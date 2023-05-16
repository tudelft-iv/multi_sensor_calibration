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

#include "mono_detector/detector.hpp"

#include <stdexcept>

#include <opencv2/aruco.hpp>

#include "mono_detector/util.hpp"
#include "mono_detector/types.hpp"

namespace mono_detector {

Eigen::Isometry3f detectMono(
  cv::Mat const & image,
  Configuration const & configuration,
  CameraModel const & intrinsics
) {
  // Continue with a deep copy to prevent modification of original data
  cv::Mat processed = image.clone();

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  cv::aruco::detectMarkers(processed, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  // Only continue if we detect exactly one marker
  if (markerCorners.size() != 1) {
    throw DetectionException("Number of markers found: " + std::to_string(markerCorners.size()) +
                             ", but should be exactly 1.");
  }

  // We first undistort points manually and then pass identity and zeros matrices,
  // as this way we can support both pinhole and fisheye models.
  std::vector<std::vector<cv::Point2f>> undistorted(1);
  if (intrinsics.distortion_model == CameraModel::DistortionModel::PINHOLE) {
    cv::undistortPoints(markerCorners[0], undistorted[0],
                        intrinsics.camera_matrix, intrinsics.distortion_parameters);
  } else {
    cv::fisheye::undistortPoints(markerCorners[0], undistorted[0],
                                 intrinsics.camera_matrix, intrinsics.distortion_parameters);
  }

  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::Mat camera_matrix_eye = cv::Mat::eye(3, 3, CV_32F);
  cv::Mat distortion_parameters_zeros = cv::Mat::zeros(1, 5, CV_32F);

  cv::aruco::estimatePoseSingleMarkers(
    undistorted, configuration.marker_size,
    camera_matrix_eye, distortion_parameters_zeros,
    rvecs, tvecs);

  // visualize result
  if (configuration.visualize) {
    visualize(image, markerCorners, markerIds, configuration.roi, tvecs[0], intrinsics);
  }

  // std::cout << "Point: " << tvecs[0] << std::endl;
  std::vector<double> rvec(3), tvec(3);
  for (size_t i = 0; i < 3; ++i) {
    rvec[i] = rvecs[0][i];
    tvec[i] = tvecs[0][i];
  }
  return toEigen(rvec, tvec);
}

}  // namespace mono_detector
