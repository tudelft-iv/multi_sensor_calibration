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

#include "node_lib.hpp"
#include "detector2.hpp"
#include "pnp.hpp"
#include "util.hpp"
#include "yaml.hpp"
#include <ros/console.h>
#include <ros/package.h>
#include <string>
#include <image_geometry/pinhole_camera_model.h> // ToDo: Remove ros dependency
#include <pcl/common/transforms.h>

namespace stereo_detector_alterno {

StereoDetectorAlternoNode::StereoDetectorAlternoNode(ros::NodeHandle & nh) : nh_(nh) {
    ROS_INFO("Initialized stereo detector alterno.");

    // Load object points from ros parameter server
    std::string object_points_file;
    nh_.param<std::string>("object_points_file", object_points_file, ros::package::getPath("stereo_detector_alterno") + "/config/object_points.yaml");
    object_points_ = YAML::LoadFile(object_points_file).as<std::vector<cv::Point3f>>();
    cv::Point3f center = calculateCenter(object_points_);
    std::sort(object_points_.begin(), object_points_.end(), [center](cv::Point3f a, cv::Point3f b) {
        return std::atan((a.y - center.y) / (a.x - center.x)) > std::atan((b.y - center.y) / (b.x - center.x));
    });

    // Load configuration from file
    std::string yaml_file;
    nh_.param<std::string>("yaml_file", yaml_file, ros::package::getPath("stereo_detector_alterno") + "/config/image_processing.yaml");
    config_ = YAML::LoadFile(yaml_file).as<stereo_detector_alterno::Configuration>();

    // Load topic names from parameters
    std::string image_topic, camera_info_topic, point_cloud_topic;
    nh_.param<std::string>("image_topic", image_topic, "/zed/zed_node/left/image_rect_color");
    nh_.param<std::string>("camera_info_topic", camera_info_topic, "/zed/zed_node/left/camera_info");
    nh_.param<std::string>("point_cloud_topic", point_cloud_topic, "stereo_pattern_alterno");

    // Setup subscriber and publisher
    image_subscriber_       = nh_.subscribe(image_topic, 1, &StereoDetectorAlternoNode::imageCallback, this);
    camera_info_subscriber_ = nh_.subscribe(camera_info_topic, 1, &StereoDetectorAlternoNode::cameraInfoCallback, this);
    point_cloud_publisher_  = nh_.advertise<sensor_msgs::PointCloud2>(point_cloud_topic, 100);
}

void StereoDetectorAlternoNode::imageCallback(sensor_msgs::ImageConstPtr const & in) {
    ROS_INFO_ONCE("Receiving images.");

    if (intrinsics_.fx() != 0 && intrinsics_.fy() != 0 && intrinsics_.cx() != 0 && intrinsics_.cy() != 0) { // Verifica si las intrínsecas son válidas
        try {
            std::vector<cv::Point2f> image_points;
            std::vector<float> radi;
            detectStereoAlterno(toOpencv(in), config_, image_points, radi);
            ROS_INFO("A");

            Eigen::Isometry3f isometry = solvePose(image_points, object_points_, intrinsics_);
            ROS_INFO("B");

            pcl::PointCloud<pcl::PointXYZ> transformed_pattern;
            pcl::transformPointCloud(toPcl(object_points_), transformed_pattern, isometry);
            ROS_INFO("C");

            ROS_INFO_ONCE("Detected a stereo detector alterno pattern point cloud at least once.");
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(transformed_pattern, out);
            out.header = in->header;
            point_cloud_publisher_.publish(out);
            ROS_INFO("D");
        } catch (std::exception & e) {
            ROS_ERROR_STREAM("Exception thrown: '" << e.what() << "'.");
            ROS_INFO("E");
        }
    } else {
        ROS_WARN("Waiting for valid camera info.");
    }
}



void StereoDetectorAlternoNode::cameraInfoCallback(sensor_msgs::CameraInfo const & camera_info) {
    ROS_INFO_ONCE("Receiving camera info.");
    
    // Verifica que los parámetros intrínsecos no estén vacíos o invalidos
    for (int i = 0; i < 9; i++) {
        ROS_INFO("Camera info K[%d]: %f", i, camera_info.K[i]);
    }

    if (camera_info.K[0] == 0 && camera_info.K[4] == 0 && camera_info.K[8] == 0) {
        ROS_ERROR("Camera info contains an invalid intrinsic matrix.");
        return;
    }


	if (!intrinsics_.fromCameraInfo(camera_info)) {
		ROS_ERROR("Failed to convert camera info.");
		//throw std::runtime_error("Conversion failed.");
	}

}

} // namespace stereo_detector_alterno

