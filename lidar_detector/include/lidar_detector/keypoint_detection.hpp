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
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// We will need to pass header files for the algorithms with which we want the new custom point types to work with
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>

#include "config.hpp"

/// Velodyne point type containing ring information like in original guindal code and in velodyne driver
namespace Velodyne {
	struct Point {
		PCL_ADD_POINT4D; ///< quad-word XYZ
		float intensity; ///< laser intensity reading
		uint16_t ring; ///< laser ring number
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW ///< ensure proper alignment
	} EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
	Velodyne::Point, (float, x, x) (float, y, y) (float, z, z)
		(float, intensity, intensity) (uint16_t, ring, ring));

namespace lidar_detector {

/// Main function to delete calibration board pose from lidar (velodyne) point cloud
pcl::PointCloud<pcl::PointXYZ> keypointDetection(pcl::PointCloud<Velodyne::Point> const & cloud, Configuration const & config);

}
