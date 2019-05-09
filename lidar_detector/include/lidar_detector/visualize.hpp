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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_detector {

/// Simply visualize a cloud for debugging purposes
template <typename T>
void visualize(pcl::PointCloud<T> const & cloud) {
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud<T>(cloud.makeShared(), "cloud");
	viewer.addCoordinateSystem();
	viewer.spin();
}

/// Simply visualize a cloud for debugging purposes
template <typename T, typename Y, typename Z>
void visualize(pcl::PointCloud<T> const & cloud, pcl::PointCloud<Y> const & pattern, pcl::PointCloud<Z> const & edges_cloud) {
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud<T>(cloud.makeShared(), "cloud");
	viewer.addCoordinateSystem();
	for (std::size_t i = 0; i < pattern.size(); ++i) {
		viewer.addSphere(pattern.at(i), 0.01, 1, 0, 0, boost::to_string(i));
	}
	for (std::size_t i = 0; i < edges_cloud.size(); ++i) {
		viewer.addSphere(edges_cloud.at(i), 0.01, 0, 1, 0, boost::to_string(i+100));
	}
	viewer.spin();
}


}
