/*
  multi_sensor_calibration
  Copyright (C) 2019  Intelligent Vehicles Delft University of Technology

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along
  with this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include "config.hpp"

namespace stereo_detector {

/// Main function that does all stereo processing (circlr, plane fitting, transforming point cloud
pcl::PointCloud<pcl::PointXYZRGB> keypointDetection(
	cv::Mat const & image,
	pcl::PointCloud<pcl::PointXYZRGB> const & cloud,
	Configuration const & config
);

/// Simply visualize a cloud for debugging purposes
inline void pclVisualize(pcl::PointCloud<pcl::PointXYZRGB> const & cloud) {
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud.makeShared(), "cloud");
	viewer.addCoordinateSystem();
	viewer.spin();
}



}
