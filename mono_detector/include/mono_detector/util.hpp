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

#include <cv_bridge/cv_bridge.h>
#include <pcl/common/common.h>

#include <array>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "mono_detector/detector.hpp"
#include "mono_detector/eigen.hpp"
#include <opencv2/core/eigen.hpp>  // needs to be after including Eigen

namespace mono_detector {

// cloud to yaml string conversion function (ToDo: Move to separate package, avoid code duplication)
std::string toYaml(pcl::PointCloud<pcl::PointXYZ> const & cloud);

// Convert opencv rvec tvec (double, for solvepnp) to an eigen isometry (float, for pcl)
Eigen::Isometry3f toEigen(std::vector<double> const & rvec, std::vector<double> const & tvec);

// Helper function to convert between opencv and pcl
pcl::PointXYZ toPcl(cv::Point3f const & point);

// Helper function to convert between opencv and pcl
pcl::PointCloud<pcl::PointXYZ> toPcl(std::vector<cv::Point3f> const & points);

// Helper function to convert ROS image to OpenCV image
cv::Mat toOpencv(const sensor_msgs::msg::Image::ConstSharedPtr & in);

// Conversion function
std::vector<cv::Point2f> toCvPoint2fVector(std::vector<cv::Vec3f> const & circles);

// Calculate the center of a vector of points
cv::Point2f calculateCenter(std::vector<cv::Point2f> const & in);

// Calculate the center of a vector of points
cv::Point3f calculateCenter(std::vector<cv::Point3f> const & in);

// visualize (blocking window until key press)
void visualize(cv::Mat const & image, std::vector<std::vector<cv::Point2f>> const & corners,
               std::vector<int> const & markerIds, cv::Rect const & roi,
               cv::Vec3d const & detection, CameraModel const & intrinsics);

}  // namespace mono_detector
