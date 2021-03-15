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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/octree/octree_search.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/passthrough.h>
#include <stdexcept>
#include <limits>

#include <common/robust_least_squares.hpp>

namespace stereo_detector {

namespace {

/// Passthrough filter of point cloud
pcl::PointCloud<pcl::PointXYZRGB> passThrough(
	pcl::PointCloud<pcl::PointXYZRGB> const & cloud,
	PassThroughFilter const & config
) {
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(cloud.makeShared());
	pass.setFilterFieldName(config.dim);
	pass.setFilterLimits(config.min, config.max);
	pass.setFilterLimitsNegative(false);
	pass.setKeepOrganized(false);
	pcl::PointCloud<pcl::PointXYZRGB> out;
	pass.filter(out);
	return out;
}

/// Multiple passthrough filters of point cloud
pcl::PointCloud<pcl::PointXYZRGB> passThrough(
	pcl::PointCloud<pcl::PointXYZRGB> const & cloud,
	std::vector<PassThroughFilter> const & config
) {
	pcl::PointCloud<pcl::PointXYZRGB> out = cloud;
	for (std::size_t i = 0; i < config.size(); ++i) {
		out = passThrough(out, config.at(i));
	}
	return out;
}

/// Crop the point cloud based on an image based region-of-interest
pcl::PointCloud<pcl::PointXYZRGB> crop(pcl::PointCloud<pcl::PointXYZRGB> const & in, cv::Rect const & roi) {
	pcl::PointCloud<pcl::PointXYZRGB> out;
	out.header = in.header;
	out.reserve(roi.width * roi.height);
	out.is_dense = false;
	for (std::size_t y = roi.y; y < roi.y + roi.height; ++y) {
		for (std::size_t x = roi.x; x < roi.x + roi.width; ++x) {
			out.push_back(in.at(x, y));
		}
	}
	out.width  = roi.width;
	out.height = roi.height;
	return out;
}

/// Visualize the image based region-of-interest
void drawRoi(cv::Mat const & image, cv::Rect const & roi) {
	cv::namedWindow("roi", CV_WINDOW_NORMAL);
	cv::Mat draw = image.clone();
	if (draw.channels() == 1) {
		cv::cvtColor(draw, draw, cv::COLOR_GRAY2BGR);
	}
	cv::rectangle(draw, roi, cv::Scalar(255, 0, 0), 2);
	cv::imshow("roi", draw);
	cv::waitKey();
}

/// Visualize the circles in a pcl visualizer
void pclVisualizeCircles(pcl::PointCloud<pcl::PointXYZRGB> const & cloud, pcl::PointCloud<pcl::PointXYZRGB> const & circles) {
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud.makeShared(), "cloud");
	viewer.addCoordinateSystem();
	viewer.addSphere(circles.at(0), 0.05, "sphere0");
	viewer.addSphere(circles.at(1), 0.05, "sphere1");
	viewer.addSphere(circles.at(2), 0.05, "sphere2");
	viewer.addSphere(circles.at(3), 0.05, "sphere3");
	viewer.spin();
}

/// Get rotation matrix from two vectors
Eigen::Isometry3d getRotationMatrix(Eigen::Vector3d source, Eigen::Vector3d target){
	Eigen::Vector3d rotation_vector = target.cross(source);
	rotation_vector.normalize();
	double theta = acos(source[2]/sqrt( pow(source[0],2)+ pow(source[1],2) + pow(source[2],2)));
	Eigen::Matrix3d rotation = Eigen::AngleAxis<double>(theta, rotation_vector).matrix();
	Eigen::Isometry3d rot(rotation);
	return rot;
}

/// Calculate plane model of calibration board
void getPlane(
	pcl::PointCloud<pcl::PointXYZRGB> const & cloud,
	pcl::ModelCoefficients & coefficients,
	pcl::PointCloud<pcl::PointXYZRGB> & plane_cloud,
	float const plane_distance_threshold,
	float const plane_eps_angle,
	int const plane_max_iterations
) {
	// Setup plane segmentation
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
	seg.setDistanceThreshold(plane_distance_threshold);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setAxis(Eigen::Vector3f::UnitY());
	seg.setOptimizeCoefficients(true);
	seg.setEpsAngle(plane_eps_angle);
	seg.setMaxIterations(plane_max_iterations);

	// Remove NaN points from point cloud
	std::vector<int> indices;
	pcl::PointCloud<pcl::PointXYZRGB> filtered;
	pcl::removeNaNFromPointCloud(cloud, filtered, indices);

	// Perform segmentation
	seg.setInputCloud(filtered.makeShared());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	seg.segment(*inliers, coefficients); // Gets the plane coefficients (a*x + b*y + c*z = d)

	// Check if a plane is found
	if (inliers->indices.size() <= 1) {
		throw std::runtime_error("Could not estimate a planar model for the given point cloud.");
	}

	// Filter based on indices
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud.makeShared());
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(plane_cloud);
}

cv::Vec3b toVec3b(pcl::PointXYZRGB const & point) {
	cv::Vec3b out;
	out[0] = point.b * 255;
	out[1] = point.g * 255;
	out[2] = point.r * 255;
	return out;
}

/// Convert pcl coefficients to Eigen vector
Eigen::Vector4f toEigen(pcl::ModelCoefficients const & in) {
	Eigen::Vector4f out;
	out(0) = in.values[0];
	out(1) = in.values[1];
	out(2) = in.values[2];
	out(3) = in.values[3];
	return out;
}

/// Filter points close to the plane
pcl::PointCloud<pcl::PointXYZRGB> segmentPlane(pcl::ModelCoefficients const & coefficients, pcl::PointCloud<pcl::PointXYZRGB> const & cloud, float const plane_distance) {
	pcl::PointCloud<pcl::PointXYZRGB> filtered;
	pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> dit(cloud.makeShared());
	std::vector<int> plane_inliers;
	dit.selectWithinDistance(toEigen(coefficients), plane_distance, plane_inliers);
	pcl::copyPointCloud<pcl::PointXYZRGB>(cloud, plane_inliers, filtered);
	return filtered;
}



/// Segment point cloud by replacing points with NaN if corresponding image point is zero
pcl::PointCloud<pcl::PointXYZRGB> keep(cv::Mat const & image, pcl::PointCloud<pcl::PointXYZRGB> const & cloud, bool visualize) {
	pcl::PointCloud<pcl::PointXYZRGB> out;
	if (cloud.size() != image.total()) {
		throw std::runtime_error(std::string("Image and point cloud are not the same size, ") + std::to_string(image.total()) + " and " + std::to_string(cloud.size()) + " respectively.");
	}
	for (std::size_t x = 0; x < image.size().width; ++x) {
		for (std::size_t y = 0; y < image.size().height; ++y) {
			if (image.at<uchar>(y, x) > 0) {
				out.push_back(cloud.at(x, y));
			}
		}
	}
	return out;
}

/// Rotate point cloud
Eigen::Isometry3d getRotation(pcl::ModelCoefficients const & cam_plane_coeffs) {
	Eigen::Vector3d xy_plane_normal_vector, floor_plane_normal_vector;
	xy_plane_normal_vector[0] = 0.0;
	xy_plane_normal_vector[1] = 0.0;
	xy_plane_normal_vector[2] = -1.0;

	floor_plane_normal_vector[0] = cam_plane_coeffs.values[0];
	floor_plane_normal_vector[1] = cam_plane_coeffs.values[1];
	floor_plane_normal_vector[2] = cam_plane_coeffs.values[2];

	return getRotationMatrix(floor_plane_normal_vector, xy_plane_normal_vector);
}

/// Calculates the number of points within a certain radius
int pointsWithinRadius(pcl::PointXYZRGB const & point, pcl::PointCloud<pcl::PointXYZRGB> const & cloud, float const radius) {

	// Indices and distances output
	std::vector<int> point_indices;
	std::vector<float> distances;

	// Setup octree
	float resolution = 0.02f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution);
	octree.setInputCloud(cloud.makeShared());
	octree.addPointsFromInputCloud();

	// Perform search
	return octree.radiusSearch(point, radius, point_indices, distances);
}

/// Processes a single circle, returning the center and filtering the cloud
bool processCircle(
	pcl::PointCloud<pcl::PointXYZRGB> & cloud,
	float const circle_distance_threshold,
	int const circle_max_iterations,
	float const min_circle_radius,
	float const max_circle_radius,
	float const radius_max_points,
	int const max_points_within_radius,
	pcl::PointXYZRGB & center
) {
	// Create circle segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setModelType(pcl::SACMODEL_CIRCLE3D);
	seg.setDistanceThreshold(circle_distance_threshold);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setOptimizeCoefficients(true);
	seg.setMaxIterations(circle_max_iterations);
	seg.setRadiusLimits(min_circle_radius, max_circle_radius);

	// Find circle
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	seg.setInputCloud(cloud.makeShared());
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    try	{
	    seg.segment(*inliers, *coefficients);
    }catch (std::exception & e) {
        //Segmentation failed!
        return false;
    }


	if (inliers->indices.size() == 0) {
	    return false;
		//throw std::runtime_error("Could not estimate a circle fit for stereo camera edge cloud.");
	}

	// Return center
	center.x = *coefficients->values.begin();
	center.y = *(coefficients->values.begin()+1);
	center.z = *(coefficients->values.begin()+2);

	if (pointsWithinRadius(center, cloud, radius_max_points) <= max_points_within_radius) {
		// Filter point cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(cloud);
		return true;
	}
	return false;
}

/// Processes all four circles
pcl::PointCloud<pcl::PointXYZRGB> processCircles(
	pcl::PointCloud<pcl::PointXYZRGB> & cloud,
	float const circle_distance_threshold,
	int const circle_max_iterations,
	float const min_circle_radius,
	float const max_circle_radius,
	int const cluster_iterations,
	float const radius_max_points,
	int const max_points_within_radius
) {
	pcl::PointCloud<pcl::PointXYZRGB> out;
	std::size_t iteration = 0;
	while (out.size() < 4 && iteration < cluster_iterations) { // The calibration board consists of four circles that need to be detected
		iteration++;
		pcl::PointXYZRGB point;
		if (processCircle(cloud, circle_distance_threshold, circle_max_iterations, min_circle_radius, max_circle_radius, radius_max_points, max_points_within_radius, point)) {
			out.push_back(point);
		}
	}
	return out;
}

/// Project points on plane
void projectOnPlane(pcl::PointCloud<pcl::PointXYZRGB> & cloud, pcl::ModelCoefficients::ConstPtr const & coefficients) {
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud.makeShared());
	proj.setModelCoefficients(coefficients);
	proj.filter(cloud);
}

}

pcl::PointCloud<pcl::PointXYZRGB> convertEigentoPointcloud(Eigen::Matrix3Xd matrix){
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
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

Eigen::Matrix3Xd convertPointcloudToEigen(pcl::PointCloud<pcl::PointXYZRGB> cloud) {
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

pcl::PointCloud<pcl::PointXYZRGB> refinement(pcl::PointCloud<pcl::PointXYZRGB> circles, Configuration const & config) {
	// Define calibration board pattern
	 Eigen::Matrix3Xd  calibration_board(3,4);
	 double width_calibration_board = config.refinement.width;
	 double heigth_calibration_board = config.refinement.height;
	 calibration_board << 0, width_calibration_board, 0, width_calibration_board,
	         0, 0, heigth_calibration_board, heigth_calibration_board,
	         0, 0, 0, 0;

	// Convert detections to Eigen
	Eigen::Matrix3Xd detections = convertPointcloudToEigen(circles);
	// Refine circle centers
	Eigen::Matrix3Xd results = refine2D(calibration_board, detections, config.refinement.threshold_inlier);
	// Convert Eigen to PCL
	circles = convertEigentoPointcloud(results);

	return circles;
}
/// Keypoint detection using image, canny and cloud processing
pcl::PointCloud<pcl::PointXYZRGB> keypointDetection(
	cv::Mat const & image,
	pcl::PointCloud<pcl::PointXYZRGB> const & cloud,
	Configuration const & config
) {
	// Apply a ROI
	if (config.visualize) {
		drawRoi(image, config.roi);
	}
	cv::Mat image_roi = image(config.roi);
	pcl::PointCloud<pcl::PointXYZRGB> cloud_roi = crop(cloud, config.roi);

	// Edge detection in image left
	cv::Mat edge_image;
	cv::Canny(image_roi, edge_image, config.canny.min, config.canny.max);
	if (config.visualize) {
		cv::namedWindow("image", CV_WINDOW_NORMAL);
		cv::imshow("image", edge_image);
		cv::waitKey();
	}

	// Extract vertical plane of calibration board to get plane coefficients 'cam_plane_coeffs'
	pcl::ModelCoefficients::Ptr cam_plane_coeffs (new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZRGB> plane_cloud;
	getPlane(passThrough(cloud_roi, config.pass_through_filter), *cam_plane_coeffs, plane_cloud, config.plane_filter.threshold, config.plane_filter.eps_angle, config.plane_filter.iterations);

	// Extract pointcloud with edges
	pcl::PointCloud<pcl::PointXYZRGB> edge_cloud = keep(edge_image, cloud_roi, config.visualize);

	// Segment points close to plane
	edge_cloud = segmentPlane(*cam_plane_coeffs, edge_cloud, config.plane_distance);

	// Apply a passthrough filter to remove points
	edge_cloud = passThrough(edge_cloud, config.pass_through_filter);

	// Project edge_cloud on plane
	projectOnPlane(edge_cloud, cam_plane_coeffs);

	// Get rotation which is required for 2D fitting
	Eigen::Affine3d rotation = getRotation(*cam_plane_coeffs);

	// Transform edge point cloud to xy plane
	pcl::PointCloud<pcl::PointXYZRGB> edge_cloud_xy;
	pcl::transformPointCloud(edge_cloud, edge_cloud_xy, rotation);

	// Two-dimensional circle fitting of remaining point cloud
	pcl::PointCloud<pcl::PointXYZRGB> circles = processCircles(
		edge_cloud_xy,
		config.circle_detection.threshold,
		config.circle_detection.iterations,
		config.circle_detection.min_radius,
		config.circle_detection.max_radius,
		config.circle_detection.cluster_iterations,
		config.circle_detection.radius_max_points,
		config.circle_detection.max_points_within_radius
	);

	// Rotate resulting point cloud back to original frame
	pcl::transformPointCloud(circles, circles, rotation.inverse());

	// Visualize circles optionally
	if (config.visualize) {
		pclVisualizeCircles(passThrough(edge_cloud, config.pass_through_filter), circles);
	}

	// Refine circle centers using calibration board geometry
	if (config.refinement.refine) {
		circles = refinement(circles, config);
	}

	return circles;
}

} // stereo_detector namespace
