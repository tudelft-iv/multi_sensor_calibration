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

#include "mono_detector/util.hpp"

#include <opencv2/aruco.hpp>

namespace mono_detector {

std::string toYaml(pcl::PointCloud<pcl::PointXYZ> const & cloud) {
  std::string out;
  for (const auto & p : cloud) {
    out += "- {x: ";
    out += std::to_string(p.x);
    out += ", y: ";
    out += std::to_string(p.y);
    out += ", z: ";
    out += std::to_string(p.z);
    out += "}\n";
  }
  return out;
}

Eigen::Isometry3f toEigen(std::vector<double> const & rvec, std::vector<double> const & tvec) {
  cv::Mat rotation_mat;
  cv::Rodrigues(rvec, rotation_mat);

  Eigen::Matrix3d rotation_eigen;
  cv::cv2eigen(rotation_mat, rotation_eigen);

  Eigen::Quaterniond quaternion(rotation_eigen);
  Eigen::Translation3d translation(tvec.at(0), tvec.at(1), tvec.at(2));

  Eigen::Isometry3d isometry = translation * quaternion;

  return isometry.cast<float>();
}

pcl::PointXYZ toPcl(cv::Point3f const & point) {
  return {point.x, point.y, point.z};
}

pcl::PointCloud<pcl::PointXYZ> toPcl(std::vector<cv::Point3f> const & points) {
  pcl::PointCloud<pcl::PointXYZ> out;
  for (const auto & point : points) {
    out.push_back(toPcl(point));
  }
  return out;
}

cv::Mat toOpencv(const sensor_msgs::msg::Image::ConstSharedPtr & in) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(in, in->encoding);
  return cv_ptr->image;
}

std::vector<cv::Point2f> toCvPoint2fVector(std::vector<cv::Vec3f> const & circles) {
  std::vector<cv::Point2f> result;
  for (auto const & c : circles) {
    result.push_back(cv::Point2f(c[0], c[1]));
  }
  return result;
}

cv::Point2f calculateCenter(std::vector<cv::Point2f> const & in) {
  cv::Point2f out(0, 0);
  for (const auto & point : in) {
    out.x += point.x;
    out.y += point.y;
  }
  out.x /= in.size();
  out.y /= in.size();
  return out;
}

cv::Point3f calculateCenter(std::vector<cv::Point3f> const & in) {
  cv::Point3f out(0, 0, 0);
  for (const auto & point : in) {
    out.x += point.x;
    out.y += point.y;
    out.z += point.z;
  }
  out.x /= in.size();
  out.y /= in.size();
  out.z /= in.size();
  return out;
}

void visualize(cv::Mat const & image, std::vector<std::vector<cv::Point2f>> const & corners,
               std::vector<int> const & markerIds, cv::Rect const & roi,
               cv::Vec3d const & detection, CameraModel const & intrinsics) {
  cv::Mat outputImage = image.clone();
  cv::aruco::drawDetectedMarkers(outputImage, corners, markerIds);

  std::vector<cv::Point2f> projected;
  std::vector<cv::Point3f> objectPoints{cv::Point3f(detection[0], detection[1], detection[2])};
  std::vector<double> empty(3, 0.);

  if (intrinsics.distortion_model == CameraModel::DistortionModel::PINHOLE) {
    cv::projectPoints(objectPoints, projected, empty, empty,
      intrinsics.camera_matrix, intrinsics.distortion_parameters);
  } else {
    cv::fisheye::projectPoints(objectPoints, projected, empty, empty,
      intrinsics.camera_matrix, intrinsics.distortion_parameters);
  }

  cv::circle(outputImage, projected[0], 7, cv::Scalar(150, 0, 200), -1);

  if (roi != cv::Rect()) {
    cv::rectangle(outputImage, roi.tl(), roi.br(), cv::Scalar(255, 0, 0), 3);
  }
  std::string window_name = "result";
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::imshow(window_name, outputImage);
  cv::waitKey(100);
}

}  // namespace mono_detector
