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

#include "keypoint_detection.hpp"
#include "visualize.hpp"

#include <pcl/octree/octree_search.h>
#include <common/robust_least_squares.hpp>

namespace lidar_detector {
namespace {

/// Passthrough filter of point cloud
pcl::PointCloud<Lidar::PointWithDist> passThrough(
	pcl::PointCloud<Lidar::PointWithDist> const & cloud,
	PassThroughFilter const & config
) {
	pcl::PassThrough<Lidar::PointWithDist> pass;
	pass.setInputCloud(cloud.makeShared());
	pass.setFilterFieldName(config.dim);
	pass.setFilterLimits(config.min, config.max);
	pass.setFilterLimitsNegative(false);
	pass.setKeepOrganized(false);
	pcl::PointCloud<Lidar::PointWithDist> out;
	pass.filter(out);
	return out;
}

/// Multiple passthrough filters of point cloud
pcl::PointCloud<Lidar::PointWithDist> passThrough(
	pcl::PointCloud<Lidar::PointWithDist> const & cloud,
	std::vector<PassThroughFilter> const & config
) {
	pcl::PointCloud<Lidar::PointWithDist> out = cloud;
	for (std::size_t i = 0; i < config.size(); ++i) {
		out = passThrough(out, config.at(i));
	}
	return out;
}

/// Project points on plane given by coefficients
void projectOnPlane(pcl::ModelCoefficients::ConstPtr const & coefficients, pcl::PointCloud<Lidar::PointWithDist>::Ptr & cloud) {
	pcl::ProjectInliers<Lidar::PointWithDist> projection;
	projection.setModelType(pcl::SACMODEL_PLANE);
	projection.setInputCloud(cloud);
	projection.setModelCoefficients(coefficients);
	projection.filter(*cloud);
}

/// Apply plane fit and return filtered point cloud
pcl::PointCloud<Lidar::PointWithDist> filterPlane(
	pcl::PointCloud<Lidar::PointWithDist> const & in,
	PlaneFilter const & config
) {
	// Compute coefficients of ground floor
	pcl::PointCloud<Lidar::PointWithDist>::Ptr out (new pcl::PointCloud<Lidar::PointWithDist>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::SACSegmentation<Lidar::PointWithDist> plane_segmentation;
	plane_segmentation.setModelType(config.model_type);
	plane_segmentation.setDistanceThreshold(config.distance_threshold);
	plane_segmentation.setMethodType(pcl::SAC_RANSAC);
	plane_segmentation.setAxis(config.axis);
	plane_segmentation.setEpsAngle(config.eps_angle);
	plane_segmentation.setOptimizeCoefficients(true);
	plane_segmentation.setMaxIterations(config.max_iterations);
	plane_segmentation.setInputCloud(in.makeShared());
	plane_segmentation.segment(*inliers, *coefficients);

	// Extract non ground floor PCL
	pcl::ExtractIndices<Lidar::PointWithDist> extract;
	extract.setInputCloud(in.makeShared());
	extract.setIndices(inliers);
	extract.setNegative(config.set_negative);
	extract.filter(*out);

	// Optionally project to plane before returning
	if (config.return_projected) {
		projectOnPlane(coefficients, out);
	}

	return *out;
}

/// Get rings as a vector of pointers to the Velodyne Points to keep mapping from rings to original velodyne points
std::vector<std::vector<Lidar::PointWithDist*> > getRings(pcl::PointCloud<Lidar::PointWithDist> & in, LidarParameters const & lidar_parameters) {
	std::vector<std::vector<Lidar::PointWithDist*> > rings(lidar_parameters.number_layers);
	for (std::size_t i = 0; i < in.size(); ++i) {
		rings[in.at(i).ring].push_back(&(in.at(i)));
	}
	return rings;
}

pcl::PointCloud<pcl::PointXYZ> toCloud(std::vector<Lidar::PointWithDist*> const & ring) {
	pcl::PointCloud<pcl::PointXYZ> out;
	for (std::size_t i = 0; i < ring.size(); ++i) {
		out.push_back(pcl::PointXYZ(ring.at(i)->x, ring.at(i)->y, ring.at(i)->z));
	}
	return out;
}


// Visualize rings to check if OK
void visualize(std::vector<std::vector<Lidar::PointWithDist*> > const & rings) {
	pcl::visualization::PCLVisualizer viewer;
	for (std::size_t i = 0; i < rings.size(); ++i) {
		pcl::PointCloud<pcl::PointXYZ> cloud = toCloud(rings.at(i));
		viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), boost::to_string(i));
		viewer.spin();
	}
	viewer.addCoordinateSystem();
}


/// Returns true if the number of points is too small (c++03 compatible)
bool isFewPoints(std::vector<Lidar::PointWithDist*> const in) {
	return (in.size() < 4) ? true : false;
}

/// Detect edges based on distances with respect to neighbouring points in the filtered velodyne point cloud
std::vector<std::vector<Lidar::PointWithDist*> > toDistanceRing(pcl::PointCloud<Lidar::PointWithDist> & in, float & average_distance_ring, LidarParameters const & lidar_parameters) {
	// Loop over the rings
	int max_points_ring = 0;
	std::vector<std::vector<Lidar::PointWithDist*> > rings = getRings(in, lidar_parameters);
	rings.erase(std::remove_if(rings.begin(), rings.end(), isFewPoints), rings.end()); // c++03 compatible
	for (std::vector<std::vector<Lidar::PointWithDist*> >::iterator ring = rings.begin(); ring < rings.end(); ring++){
		if (ring->size() < 10) {
			std::cerr << "Ring too small, continue.." << std::endl;
			continue;
		}
		(*ring->begin())->distance = 0;
		(*(ring->end()-1))->distance = 0;
		// Loop over the points in the ring
		float total_distance_ring = 0;
		for (std::vector<Lidar::PointWithDist*>::iterator pt = ring->begin() + 1; pt < ring->end() - 1; pt++) {
			Lidar::PointWithDist* next_point = *(pt + 1);
			Lidar::PointWithDist* prev_point = *(pt - 1);
			float dx = prev_point->x - next_point->x;
			float dy = prev_point->y - next_point->y;
			float dz = prev_point->z - next_point->z;
			(*pt)->distance = sqrt(dx*dx+dy*dy+dz*dz)/2;
			total_distance_ring += (*pt)->distance;
		}
		if (ring->size() > max_points_ring) {
			max_points_ring = ring->size();
			average_distance_ring = total_distance_ring/max_points_ring;
		}
	}
	return rings;
}

/// Create edges point cloud
pcl::PointCloud<Lidar::PointWithDist> createEdgeCloud(pcl::PointCloud<Lidar::PointWithDist> const & cloud, CloudEdgeFilter const & config, float const average_distance_ring) {
	pcl::PointCloud<Lidar::PointWithDist> edges_cloud;
	float min_threshold = 3*average_distance_ring; // min gap is 3 times resolution
	float max_threshold = (2*config.radius + 2*average_distance_ring); // Gap cannot be larger than diameter plus 2 times resolution
	for (std::size_t i = 0; i < cloud.size(); ++i) {
		if (cloud.at(i).distance > min_threshold && cloud.at(i).distance < max_threshold ) { // jump should be smaller or equal to diameter + 2* angeluar resuoltion
			edges_cloud.push_back(cloud.at(i));
		}
	}
	if (edges_cloud.size() == 0) {
		throw pcl::PCLException("Lidar calibration board detection edge cloud does not contain any points.");
	}
	return edges_cloud;
}

// Filter outliers and return PointXYZ since velodyne specific info is no longer required from now on
pcl::PointCloud<pcl::PointXYZ> filterOutliers(std::vector<std::vector<Lidar::PointWithDist*> > const & rings) {
	pcl::PointCloud<pcl::PointXYZ> out;
	int rings_with_circle = 0;
	for (std::size_t i = 0; i < rings.size(); ++i) {
		if(rings.at(i).size() >= 4){
			rings_with_circle++;
			for (std::size_t j = 0; j < rings.at(i).size(); ++j){
				pcl::PointXYZ point_xyz;
				point_xyz.x = rings.at(i).at(j)->x;
				point_xyz.y = rings.at(i).at(j)->y;
				point_xyz.z = rings.at(i).at(j)->z;
				out.push_back(point_xyz);
			}
		}
	}
	return out;
}

/// Calculates the number of points within a certain radius
int pointsWithinRadius(pcl::PointXYZ const & point, pcl::PointCloud<Lidar::PointWithDist> const & cloud, float const radius) {
	// Convert cloud from velodyne point to pcl PointXYZ
	pcl::PointCloud<pcl::PointXYZ> out;
	for (std::size_t i = 0; i < cloud.size(); ++i) {
		pcl::PointXYZ point_xyz;
		point_xyz.x = cloud.at(i).x;
		point_xyz.y = cloud.at(i).y;
		point_xyz.z = cloud.at(i).z;
		out.push_back(point_xyz);
	}

	// Indices and distances output
	std::vector<int> point_indices;
	std::vector<float> distances;

	// Setup octree
	float resolution = 0.02f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
	octree.setInputCloud(out.makeShared());
	octree.addPointsFromInputCloud();

	// Perform search
	return octree.radiusSearch(point, radius, point_indices, distances);
}

/// Process to find a circle
bool processCircle(pcl::PointCloud<pcl::PointXYZ> & cloud, pcl::PointCloud<Lidar::PointWithDist> & plane, CircleDetection const & config, pcl::PointXYZ & point) {
	// Create circle segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setModelType(pcl::SACMODEL_CIRCLE3D);
	seg.setDistanceThreshold(config.distance_threshold);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setOptimizeCoefficients(true);
	seg.setMaxIterations(config.max_iterations);
	seg.setRadiusLimits(config.min_radius, config.max_radius);

	// Find circle
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	seg.setInputCloud(cloud.makeShared());
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	seg.segment(*inliers, *coefficients);



	if (inliers->indices.size() == 0) {
	    return false;
		//throw pcl::PCLException("Lidar calibration board detection could not estimate a circle fit for lidar camera edge cloud.");
	}

	// Return center
	point.x = *coefficients->values.begin();
	point.y = *(coefficients->values.begin() + 1);
	point.z = *(coefficients->values.begin() + 2);

	if (pointsWithinRadius(point, plane, config.radius_max_points) <= config.max_points_within_radius) {
		// Filter point cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(cloud);
		return true;
	}
	return false;
}

/// Detect circles and return point cloud with centroids of calibration pattern
pcl::PointCloud<pcl::PointXYZ> processCircles(pcl::PointCloud<pcl::PointXYZ> & cloud, pcl::PointCloud<Lidar::PointWithDist> & plane, CircleDetection const & config) {
	pcl::PointCloud<pcl::PointXYZ> out;
	std::size_t iteration = 0;
	while (out.size() < 4 && iteration < config.cluster_iterations) { // The calibration board consists of four circles that need to be detected
		iteration++;
		pcl::PointXYZ point;
		if (processCircle(cloud, plane, config, point)) {
			out.push_back(point);
		}
	}
	return out;
}


} // anonymous namespace

pcl::PointCloud<pcl::PointXYZ> convertEigentoPointcloud(Eigen::Matrix3Xd matrix){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width  = matrix.cols();
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = matrix.col(i)[0];
    cloud.points[i].y = matrix.col(i)[1];
    cloud.points[i].z = matrix.col(i)[2];
  }

  return cloud;
}

Eigen::Matrix3Xd convertPointcloudToEigen(pcl::PointCloud<pcl::PointXYZ> cloud) {
  Eigen::Matrix3Xd out = Eigen::Matrix3Xd::Zero(3,cloud.points.size()) ;

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    // Convert to eigen matrix:
    out(0,i) = cloud.points[i].x;
    out(1,i) = cloud.points[i].y;
    out(2,i) = cloud.points[i].z;
  }

  return out;
}

/// Apply plane fit and return filtered point cloud
pcl::ModelCoefficients getPlaneCoefficients(
	pcl::PointCloud<Lidar::PointWithDist> const & in,
	PlaneFilter const & config
) {
	// Compute coefficients of ground floor
	pcl::PointCloud<Lidar::PointWithDist>::Ptr out (new pcl::PointCloud<Lidar::PointWithDist>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::SACSegmentation<Lidar::PointWithDist> plane_segmentation;
	plane_segmentation.setModelType(config.model_type);
	plane_segmentation.setDistanceThreshold(config.distance_threshold);
	plane_segmentation.setMethodType(pcl::SAC_RANSAC);
	plane_segmentation.setAxis(config.axis);
	plane_segmentation.setEpsAngle(config.eps_angle);
	plane_segmentation.setOptimizeCoefficients(true);
	plane_segmentation.setMaxIterations(config.max_iterations);
	plane_segmentation.setInputCloud(in.makeShared());
	plane_segmentation.segment(*inliers, *coefficients);

	return *coefficients;
}

Eigen::Matrix3Xd projectToCalibrationBoardPlane(Eigen::Matrix3Xd in, pcl::ModelCoefficients coefficients_plane) {
	Eigen::Vector3d normal_vector;
	normal_vector << coefficients_plane.values[0], coefficients_plane.values[1], coefficients_plane.values[2];

	Eigen::Matrix3Xd out = Eigen::Matrix3Xd::Zero(in.rows(), in.cols());
	for (size_t i = 0; i < in.cols(); i++) {
		// compute disntance
		double distance = normal_vector.dot(in.col(i)) + coefficients_plane.values[3];
		// Correct point using normal and distance
		out.col(i) = in.col(i) - distance*normal_vector;
	}

	return out;
}
pcl::PointCloud<pcl::PointXYZ> refinement(pcl::PointCloud<pcl::PointXYZ> pattern, Configuration const & config) {
	// Define calibration board pattern
	 Eigen::Matrix3Xd  calibration_board(3,4);
	 double width_calibration_board = config.refinement.width;
	 double heigth_calibration_board = config.refinement.height;
	 calibration_board << 0, width_calibration_board, 0, width_calibration_board,
	         0, 0, heigth_calibration_board, heigth_calibration_board,
	         0, 0, 0, 0;

	// Convert detections to Eigen
	Eigen::Matrix3Xd detections = convertPointcloudToEigen(pattern);
	// Refine circle centers
	Eigen::Matrix3Xd results = refine2D(calibration_board, detections, config.refinement.threshold_inlier);
	// Convert Eigen to PCL
	pcl::PointCloud<pcl::PointXYZ> refined_pattern = convertEigentoPointcloud(results);

	return refined_pattern;
}
pcl::PointCloud<pcl::PointXYZ> keypointDetection(pcl::PointCloud<Lidar::PointWithDist> const & in, Configuration const & config) {
	// Passthrough filter
	pcl::PointCloud<Lidar::PointWithDist> passthrough_cloud = passThrough(in, config.pass_through_filter);

	// Compute coefficients of ground floor
	pcl::PointCloud<Lidar::PointWithDist> cloud_without_ground_floor = filterPlane(passthrough_cloud, config.ground_floor_filter);

	// Compute coefficients of vertical plane and extract points of vertical plane
	pcl::PointCloud<Lidar::PointWithDist> cloud_calibration_board = filterPlane(cloud_without_ground_floor, config.calibration_board_filter);

	// Edge detection: Loop over all rings, and then over all points, and calculate distance w.r.t. neighbors.
	float average_distance_ring;
	std::vector<std::vector<Lidar::PointWithDist*> > rings = toDistanceRing(cloud_calibration_board, average_distance_ring, config.lidar_parameters);
	if (config.visualize) { visualize(rings); }

	// Create unorganized point cloud of edge points
	pcl::PointCloud<Lidar::PointWithDist> edges_cloud = createEdgeCloud(cloud_calibration_board, config.cloud_edge_filter, average_distance_ring);

	// Remove edge points from edge cloud if corresponding ring contains only a < 4 points
	std::vector<std::vector<Lidar::PointWithDist*> > edges_rings = getRings(edges_cloud, config.lidar_parameters);
	pcl::PointCloud<pcl::PointXYZ> circles_cloud = filterOutliers(edges_rings);

	// Circle3d fit for all four circles
	pcl::PointCloud<pcl::PointXYZ> pattern = processCircles(circles_cloud, cloud_without_ground_floor, config.circle_detection);
	if (config.visualize) { visualize(cloud_calibration_board, pattern, edges_cloud); }

	// Refine circle centers using calibration board geometry
	if (config.refinement.refine) {
		// Refinement using kabsch
		pattern = refinement(pattern, config);
	}

	return pattern;
}

} // lidar_detector namespace
