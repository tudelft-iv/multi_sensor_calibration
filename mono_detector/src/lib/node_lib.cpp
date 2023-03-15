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

#include "mono_detector/node_lib.hpp"

#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "mono_detector/detector.hpp"
#include "mono_detector/pnp.hpp"
#include "mono_detector/util.hpp"
#include "mono_detector/yaml.hpp"

using std::placeholders::_1;

namespace mono_detector {


MonoDetectorNode::MonoDetectorNode() : Node("mono_detector") {
  RCLCPP_INFO(get_logger(), "Initialized mono detector.");

  std::string package_share = ament_index_cpp::get_package_share_directory("mono_detector");
  // Load object points from ros parameter server
  this->declare_parameter("object_points_file", package_share + "/config/object_points.yaml");
  std::string object_points_file = this->get_parameter("object_points_file").get_parameter_value().get<std::string>();
  object_points_ = YAML::LoadFile(object_points_file).as<std::vector<cv::Point3f>>();
  cv::Point3f center = calculateCenter(object_points_);
  std::sort(object_points_.begin(), object_points_.end(), [center](cv::Point3f a, cv::Point3f b) {
    return std::atan((a.y - center.y) / (a.x - center.x)) > std::atan((b.y - center.y) / (b.x - center.x));
  });

  // Load configuration from file
  this->declare_parameter("yaml_file", package_share + "/config/image_processing.yaml");
  std::string yaml_file = this->get_parameter("yaml_file").get_parameter_value().get<std::string>();
  config_ = YAML::LoadFile(yaml_file).as<mono_detector::Configuration>();

  // Setup subscriber and publisher
  image_subscriber_       = this->create_subscription<sensor_msgs::msg::Image>(
    "/ueye/left/image_raw", 1, std::bind(&MonoDetectorNode::imageCallback, this, _1)
  );
  camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/ueye/left/camera_info", 1, std::bind(&MonoDetectorNode::cameraInfoCallback, this, _1)
  );
  point_cloud_publisher_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("mono_pattern", 100);
}

void MonoDetectorNode::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const & in) {
  RCLCPP_INFO_ONCE(get_logger(), "Receiving images.");
  if (!intrinsics_received_) {
    return;
  }

  try {
    // Call to do image processing
    std::vector<cv::Point2f> image_points;
    std::vector<float> radi;
    detectMono(toOpencv(in), config_, image_points, radi);

    // Call to solve pnp
    Eigen::Isometry3f isometry = solvePose(image_points, object_points_, intrinsics_);

    // Transform pattern
    pcl::PointCloud<pcl::PointXYZ> transformed_pattern;
    pcl::transformPointCloud(toPcl(object_points_), transformed_pattern, isometry);

    // Publish pattern
    RCLCPP_INFO_ONCE(get_logger(), "Detected a mono detector pattern point cloud at least once.");
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(transformed_pattern, out);
    out.header = in->header;
    point_cloud_publisher_->publish(out);
  } catch (DetectionException & e) {
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 10, "Detection failed: '" << e.what() << "'.");
  } catch (std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Exception thrown: '" << e.what() << "'.");
  }
}

void MonoDetectorNode::cameraInfoCallback(sensor_msgs::msg::CameraInfo const & camera_info) {
  RCLCPP_INFO_ONCE(get_logger(), "Receiving camera info.");
  // ToDo: Use message filters to make sure to get both camera info and image
  intrinsics_.fromCameraInfo(camera_info);
  intrinsics_received_ = true;
}

}
