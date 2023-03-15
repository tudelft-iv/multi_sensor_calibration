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

#include "accumulator/yaml.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <calibration_interfaces/msg/accumulated_patterns.hpp>
#include <calibration_interfaces/srv/send_patterns.hpp>
#include <calibration_interfaces/srv/send_string.hpp>
#include <calibration_interfaces/srv/send_u_int.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace accumulator {

using PointCloud2 = sensor_msgs::msg::PointCloud2;

/// Configuration of euclidean cluster extraction
struct EuclideanConfig {
	float cluster_tolerance;
	int min_cluster_size;
	int max_cluster_size;
};

namespace {
	// Unfortunately the c++03 (for pcl) compatible to string is needed
	std::string toString(float const value) {
		std::ostringstream out;
		out << value;
		return out.str();
	}

	// (pattern) cloud to yaml string conversion function (ToDo: Move to separate package, avoid code duplication)
	inline std::string toYaml(pcl::PointCloud<pcl::PointXYZ> const & cloud) {
		std::string out;
		for (std::size_t i = 0; i < cloud.size(); ++i) {
			out += "- {x: ";
			out += toString(cloud.at(i).x);
			out += ", y: ";
			out += toString(cloud.at(i).y);
			out += ", z: ";
			out += toString(cloud.at(i).z);
			out += "}\n";
		}
		return out;
	}

	/// Creates a PointCloud with Nan
	pcl::PointCloud<pcl::PointXYZ> createNanPointcloud(unsigned int nr_detections) {
		pcl::PointCloud<pcl::PointXYZ> clusters;
		const float nan_point = std::numeric_limits<float>::quiet_NaN();
		for (size_t i =  0; i < nr_detections; i++) {
			pcl::PointXYZ pointXYZ;
			pointXYZ.x = nan_point;
			pointXYZ.y = nan_point;
			pointXYZ.z = nan_point;
			clusters.push_back(pointXYZ);
		}

		return clusters;
	}

	/// Convert a pcl point to a geometry msgs point
	geometry_msgs::msg::Point toRos(pcl::PointXYZ const & point) {
		geometry_msgs::msg::Point out;
		out.x = point.x;
		out.y = point.y;
		out.z = point.z;
		return out;
	}

	/// Conversion between pcl and ros point cloud type
	PointCloud2 toRos(pcl::PointCloud<pcl::PointXYZ> const & in) {
		PointCloud2 out;
		pcl::toROSMsg(in, out);
		return out;
	}

	/// Conversion between pcl and ros point cloud type
	pcl::PointCloud<pcl::PointXYZ> toPcl(PointCloud2 const & in) {
		pcl::PointCloud<pcl::PointXYZ> out;
		pcl::fromROSMsg(in, out);
		return out;
	}

	/// Conversion between c++ struct of accumulated board locations and message to be used with service call to optimizer. Returns a vector (multiple sensors), and for each sensor an array of clouds (since we have multiple board locations)
	std::vector<calibration_interfaces::msg::AccumulatedPatterns> toRos(std::map<std::string, std::vector<pcl::PointCloud<pcl::PointXYZ> > > const & in) {
		std::vector<calibration_interfaces::msg::AccumulatedPatterns> out;
		for (const auto & sensor : in) {
			calibration_interfaces::msg::AccumulatedPatterns msg;
			msg.sensor.data = sensor.first;
			for (const auto & cloud : sensor.second) {
				msg.patterns.push_back(toRos(cloud));
			}
			out.push_back(msg);
		}
		return out;
	}

	/// Calculate the centroid of a point cloud
	pcl::PointXYZ centroid(pcl::PointCloud<pcl::PointXYZ> const & in) {
		pcl::CentroidPoint<pcl::PointXYZ> centroid;
		for (const auto & point : in) {
			centroid.add(point);
		}
		pcl::PointXYZ out;
		centroid.get(out);
		return out;
	}

	/// Some final processing on the accumulated detections for more robustness
	pcl::PointCloud<pcl::PointXYZ> mergePointclouds(std::vector<pcl::PointCloud<pcl::PointXYZ> > const & in) {
		// Combine separate clouds in a single cloud
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.header = in.front().header;
		for (const auto & c : in) {
			cloud += c;
		}

		return cloud;
	}

	std::vector<pcl::PointIndices> findClusterIndices(pcl::PointCloud<pcl::PointXYZ> const & cloud, EuclideanConfig const & config) {
		// Create search method to add to euclidean clustering for speedup
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud.makeShared());

		// Do euclidean clustering
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster;
		euclidean_cluster.setClusterTolerance(config.cluster_tolerance);
		euclidean_cluster.setMinClusterSize(config.min_cluster_size);
		euclidean_cluster.setMaxClusterSize(config.max_cluster_size);
		euclidean_cluster.setSearchMethod(tree);
		euclidean_cluster.setInputCloud(cloud.makeShared());
		euclidean_cluster.extract(cluster_indices);

		return cluster_indices;
	}

	pcl::PointCloud<pcl::PointXYZ> extractClusterCenters(pcl::PointCloud<pcl::PointXYZ> const & cloud, std::vector<pcl::PointIndices> const & cluster_indices, unsigned int nr_output_clusters) {
		// Create centers cloud
		pcl::PointCloud<pcl::PointXYZ> centers;
		centers.header = cloud.header;
		for (const auto & cluster : cluster_indices) {
			pcl::PointXYZ center(0, 0, 0);
			for (std::vector<int>::const_iterator it = cluster.indices.begin(); it < cluster.indices.end(); ++it) {
				center.x += cloud.at(*it).x;
				center.y += cloud.at(*it).y;
				center.z += cloud.at(*it).z;
			}
			center.x /= cluster.indices.size();
			center.y /= cluster.indices.size();
			center.z /= cluster.indices.size();
			centers.push_back(center);
			if (centers.size() == nr_output_clusters) {
				break;
			}
		}
		return centers;
	}

	pcl::PointCloud<pcl::PointXYZ> removeRCSdimension(pcl::PointCloud<pcl::PointXYZ> const & cloud) {
		pcl::PointCloud<pcl::PointXYZ> cloud_xy;
		copyPointCloud(cloud, cloud_xy);
		for (size_t i = 0; i < cloud_xy.points.size(); i++) {
			cloud_xy.points[i].z = 0;
		}

		return cloud_xy;
	}
	/// Convert a point cloud to a marker of spheres
	visualization_msgs::msg::Marker toArc(PointCloud2 const & cloud, std_msgs::msg::ColorRGBA const & color, float & maximum_elevation_degrees) {
		visualization_msgs::msg::Marker marker;
		marker.action          = visualization_msgs::msg::Marker::ADD;
		marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
		marker.header.frame_id = cloud.header.frame_id;
		marker.header.stamp = cloud.header.stamp;
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud = toPcl(cloud);
		for (const auto & point : pcl_cloud) {
			// Get point
			pcl::PointXYZ point2d = point;
			point2d.z = 0;
			// Transform to polar:
			float range = sqrt(point2d.x*point2d.x + point2d.y*point2d.y);
			float azimuth = atan2(point2d.y, point2d.x);

			float max_el = maximum_elevation_degrees*M_PI/180; // TODO: should be to config
			unsigned int nr_line_segements = 25;
			float delta = 2*max_el/nr_line_segements;
			for (size_t n = 0; n < nr_line_segements; n++) {
				// Get elevation angle
				float elevation = -max_el + delta*n;
				// Covert back to x,y,z
				pcl::PointXYZ t;
				t.x = range * cos(azimuth) * cos(elevation);
				t.y = range * sin(azimuth) * cos(elevation);
				t.z = range * sin(elevation);
				// Store point
				marker.points.push_back(toRos(t));
			}
		}
		marker.color = color;
		marker.scale.x = 0.02;
		marker.scale.y = 0.02;
		marker.scale.z = 0.02;
		return marker;
	}

	/// Convert a point cloud to a marker of spheres
	visualization_msgs::msg::Marker toMarker(PointCloud2 const & cloud, std_msgs::msg::ColorRGBA const & color) {
		visualization_msgs::msg::Marker marker;
		marker.action          = visualization_msgs::msg::Marker::ADD;
		marker.type            = visualization_msgs::msg::Marker::POINTS;
		marker.header.frame_id = cloud.header.frame_id;
		marker.header.stamp = cloud.header.stamp;
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud = toPcl(cloud);
		for (const auto & point : pcl_cloud) {
			marker.points.push_back(toRos(point));
		}
		marker.color = color;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		return marker;
	}

	/// Create a text marker to visualize the index of the collected pattern
	visualization_msgs::msg::Marker toMarker(std::string const & frame_id, PointCloud2 const & cloud, int const index, std_msgs::msg::ColorRGBA const & color) {
		visualization_msgs::msg::Marker marker;
		marker.action          = visualization_msgs::msg::Marker::ADD;
		marker.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
		marker.header.frame_id = frame_id;
		marker.header.stamp = cloud.header.stamp;
		marker.color = color;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.text            = std::to_string(index);
		marker.pose.position   = toRos(centroid(toPcl(cloud)));
		return marker;
	}

	/// Returns a color based on the hash of a string to make it more or less random so that each sensor, regardless of amount and name, has a different color with respect to each other
	std_msgs::msg::ColorRGBA getColor(std::string const & sensor) {
		std::hash<std::string> hasher;
		std::size_t hashed = hasher(sensor);
		std_msgs::msg::ColorRGBA color;
		color.r = float(hashed % 2) / 2;
		color.g = float(hashed % 3) / 3;
		color.b = float(hashed % 3) / 4;
		color.a = 1;
		return color;
	}

}

/// Accumulates calibration pattern point clouds, cluster extraction after accumulation and sends the patterns to the optimizer to get the calibration
class AccumulatorNode : public rclcpp::Node {
public:
	/// Constructor taking the node handle as a member variable
	AccumulatorNode() : Node("accumlator"), accumulate_(false) {
		// Load dynamic number of sensor topics, to be configured on the ros parameter server, defaulting to the values listed
		// NOTE: topic name is important because the optimizer needs to know which sensor it is
		// For instance, the topic name requirement is that a lidar sensor should contain 'lidar' in the name.
		// Furthermore, stereo camera should contain 'stereo', monocular camera should contain 'mono' and radar sensor should contain 'radar')
		this->declare_parameter("sensor_topics",
			std::vector<std::string> {
				"/stereo_detector/stereo_pattern",
				"/lidar_detector/lidar_pattern",
				"/radar_detector/radar_pattern"});
		std::vector<std::string> sensor_pattern_topics = this->get_parameter("sensor_topics").as_string_array();

		// Load config for euclidean clustering of accumulated detections
		this->declare_parameter("cluster_tolerance", 0.03);
		euclidean_config_.cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
		this->declare_parameter("min_cluster_size", 1);
		euclidean_config_.min_cluster_size = this->get_parameter("min_cluster_size").as_int();
		this->declare_parameter("max_cluster_size", 1000);
		euclidean_config_.max_cluster_size = this->get_parameter("max_cluster_size").as_int();
		this->declare_parameter("maximum_elevation_angle", 9.0);
		maximum_elevation_angle_ = this->get_parameter("maximum_elevation_angle").as_double();

		// Identifier of the optimizer to call service
		this->declare_parameter("optimize_srv", "/optimizer/optimize");
		optimize_srv_ = this->get_parameter("optimize_srv").as_string();

		// number of output clusters of radar and non-radar (lidar and camera)
		nr_output_clusters_radar_ = 1;
		nr_output_clusters_non_radar_ = 4;

		service_save_              = this->create_service<calibration_interfaces::srv::SendString>(
			"save", std::bind(&AccumulatorNode::save, this, _1, _2));
		service_load_              = this->create_service<calibration_interfaces::srv::SendString>(
			"load", std::bind(&AccumulatorNode::load, this, _1, _2));
		service_toggle_accumulate_ = this->create_service<std_srvs::srv::Empty>(
			"toggle_accumulate", std::bind(&AccumulatorNode::toggleRecord, this, _1, _2));
		service_optimize_          = this->create_service<std_srvs::srv::Empty>(
			"optimize", std::bind(&AccumulatorNode::optimize, this, _1, _2));
		service_remove_            = this->create_service<calibration_interfaces::srv::SendUInt>(
			"remove", std::bind(&AccumulatorNode::remove, this, _1, _2));

		client_optimize_					 = this->create_client<calibration_interfaces::srv::SendPatterns>(
			optimize_srv_);

		// Create the dynamic amount of ros subscribers for the detected patterns on various topics, one topic for each sensor to be calibrated
		for (const std::string & topic : sensor_pattern_topics) {
			current_location_patterns_.insert({topic, {}});
			accumulated_locations_.insert({topic, {}});

			// needs to be std::function, otherwise create_subscription doesn't recognize type
			std::function<void(PointCloud2::ConstSharedPtr)> fun = std::bind(
				&AccumulatorNode::callback, this, _1, topic);
			auto subscription = this->create_subscription<PointCloud2>(topic, 1, fun);
			subscribers_.push_back(subscription);
		}

		// Setup visualization markers publisher
  	markers_publisher_  = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"accumulated_patterns", 10);

		RCLCPP_INFO_STREAM(get_logger(), "Accumulator node initialized.");
	}

private:
	/// Vector of subscribers, one for every sensor to be calibrated
	std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> subscribers_;

	/// Visualization marker array to visualize accumulated patterns
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;

	/// Record (accumulate) if user sends a start accumulate service call until user sends a stop accumulate service call
	bool accumulate_;

	/// Service server exposed to let the user control accumulating
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_toggle_accumulate_;

	/// Service server exposed to let the user start optimization
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_optimize_;

	/// Service server exposed to save accumulated board locations to file
	rclcpp::Service<calibration_interfaces::srv::SendString>::SharedPtr service_save_;

	/// Service server exposed to load accumulated board locations from file (does not append to what's collected so far)
	rclcpp::Service<calibration_interfaces::srv::SendString>::SharedPtr service_load_;

	/// Service server exposed to load accumulated board locations from file (does not append to what's collected so far)
	rclcpp::Service<calibration_interfaces::srv::SendUInt>::SharedPtr service_remove_;

	// Service client to call optimization with accumulated patterns
	rclcpp::Client<calibration_interfaces::srv::SendPatterns>::SharedPtr client_optimize_;

	/// Contains for each sensor (string) a vector of different board locations
	std::map<std::string, std::vector<pcl::PointCloud<pcl::PointXYZ> > > accumulated_locations_;

	/// Contains for each sensor the accumulated point clouds corresponding to the current board location, will be cleared after stop service call and empty upon starting at new board location
	std::map<std::string, std::vector<pcl::PointCloud<pcl::PointXYZ> > > current_location_patterns_;

	/// Configuration for euclidean clustering
	EuclideanConfig euclidean_config_;

	/// Identifier of the service server to call optimization
	std::string optimize_srv_;

	// Number of output detections for radar
	unsigned int nr_output_clusters_radar_;
	unsigned int nr_output_clusters_non_radar_;
	float maximum_elevation_angle_;

	/// Check if the pattern is from a radar
	bool isRadar(std::vector<pcl::PointCloud<pcl::PointXYZ> > const & patterns) {
		if (patterns.front().size() == nr_output_clusters_radar_) { // Only checks the first item, but actually all elements should have only a single pattern point for radar
			return true;
		}
		return false;
	}

	/// Check if the pattern is from a radar
	bool isRadarPcl(pcl::PointCloud<pcl::PointXYZ> const & patterns) {
		if (patterns.size() == nr_output_clusters_radar_) {
			return true;
		}
		return false;
	}

	// Returns an integer with number of detections for a certain sensor type
	int nrDetections(std::string sensor) {
		int nr_detections;
		std::string radar_string("radar");
		bool is_radar = sensor.find(radar_string) !=  std::string::npos;

		// In this case the sensors has no detections
		if (is_radar) {
			// This sensor is radar
			nr_detections = nr_output_clusters_radar_;
		}
		else {
			// This sensor is non-radar
			nr_detections = nr_output_clusters_non_radar_;
		}

		return nr_detections;
	}

	/// Record function to be switched when service is called by user
	void toggleRecord(std_srvs::srv::Empty::Request::ConstSharedPtr /*req*/, std_srvs::srv::Empty::Response::SharedPtr /*res*/) {
		accumulate_ = !accumulate_;
		RCLCPP_INFO_STREAM(get_logger(), "Recording " << (accumulate_ ? "true" : "false"));
		if (accumulate_) {
			RCLCPP_INFO_STREAM(get_logger(), "New board location, starting accumulating with an empty 'current location accumulator'.");
			for (auto & c : current_location_patterns_) {
				c.second.clear();
			}
		} else { // stop accumulate
			RCLCPP_INFO_STREAM(get_logger(), "Done with current calibration board location.");
			RCLCPP_INFO_STREAM(get_logger(), "Currently accumulated: " << accumulated_locations_.at(accumulated_locations_.begin()->first).size() << " calibration board locations.");
			// There is at least one detection for each sensor. Proceeding with cluster extraction.
			for (auto & c : current_location_patterns_) { // Calculate center based on multiple frames using cluster extraction and store it in the accumulated locations
				RCLCPP_INFO_STREAM(get_logger(), "Recorded " << c.second.size() << " detections for sensor " << c.first);
				RCLCPP_INFO_STREAM(get_logger(), "Calculating and adding euclidean cluster for each sensor to the accumulated locations.");

				// Check if there are zero detections for a sensor
				if (current_location_patterns_.at(c.first).size() == 0) {
					// Save a Pointcloud with NaN in case there are no detections
					accumulated_locations_.at(c.first).push_back(createNanPointcloud(nrDetections(c.first)));
					// Print to terminal:
					std::cout << toYaml(createNanPointcloud(nrDetections(c.first))) << std::endl;
					// Skip to next sensor
					continue;
				}

				if (isRadar(current_location_patterns_.at(c.first))) {
					// Merge all PCLs to obtain pointcloud with all detections
					pcl::PointCloud<pcl::PointXYZ> merged_pcl = mergePointclouds(current_location_patterns_.at(c.first));
					// Radar should be clustered based on XY (and not on RCS):
					pcl::PointCloud<pcl::PointXYZ> merged_pcl_XY = removeRCSdimension(merged_pcl);
					// Find cluster indices using eucledian clustering
					std::vector<pcl::PointIndices> cluster_indices = findClusterIndices(merged_pcl_XY, euclidean_config_);
					// Extract clusters indices from accumulated pcl
					pcl::PointCloud<pcl::PointXYZ> clusters = extractClusterCenters(merged_pcl, cluster_indices, nr_output_clusters_radar_);

					if (clusters.size() != nr_output_clusters_radar_) {
						RCLCPP_WARN_STREAM(get_logger(), "Extracting clusters failed for sensor '" << c.first << "', adding an empty board location for this sensor.");
					}
					accumulated_locations_.at(c.first).push_back(clusters);

					// Print to terminal:
					std::cout << toYaml(clusters) << std::endl;
				} else {
					// Merge all PCLs to obtain pointcloud with all detections
					pcl::PointCloud<pcl::PointXYZ> merged_pcl = mergePointclouds(current_location_patterns_.at(c.first));
					// Perform eucledian clustering to find indices of clusters
					std::vector<pcl::PointIndices> cluster_indices = findClusterIndices(merged_pcl, euclidean_config_);
					// Extract clusters indices from accumulated pcl
					pcl::PointCloud<pcl::PointXYZ> clusters = extractClusterCenters(merged_pcl, cluster_indices, nr_output_clusters_non_radar_);

					if (clusters.size() != nr_output_clusters_non_radar_) {
						RCLCPP_WARN_STREAM(get_logger(), "Extracting clusters failed for sensor '" << c.first << "', adding an empty board location for this sensor.");
					}
					accumulated_locations_.at(c.first).push_back(clusters);

					// Print to terminal:
					std::cout << toYaml(clusters) << std::endl;
				}
			}
		}
	}

	/// Calibrate with patterns collected so far
	void optimize(std_srvs::srv::Empty::Request::ConstSharedPtr /*req*/, std_srvs::srv::Empty::Response::SharedPtr /*res*/) {
		using namespace std::chrono_literals;

		// Calibrate with accumulated patterns
		auto request = std::make_shared<calibration_interfaces::srv::SendPatterns::Request>();
		request->accumulated_patterns = toRos(accumulated_locations_);
		RCLCPP_INFO(get_logger(), "Publishing markers.");
		publishMarkers(request->accumulated_patterns);

		if (client_optimize_->wait_for_service(1s)) {
			client_optimize_->async_send_request(request);
			RCLCPP_INFO_STREAM(get_logger(), "Optimization service called");
		} else {
			RCLCPP_ERROR_STREAM(get_logger(), "Optimization service '"<< optimize_srv_  << "' not available.");
		}
	}

	/// Save accumulated board locations to file
	void save(calibration_interfaces::srv::SendString::Request::ConstSharedPtr req, calibration_interfaces::srv::SendString::Response::SharedPtr /*res*/) {
		std::ofstream file(req->data.data);
		file << YAML::Node(accumulated_locations_);
	}

	/// Load accumulated board locations from file (does not append to what's collected so far)
	void load(calibration_interfaces::srv::SendString::Request::ConstSharedPtr req, calibration_interfaces::srv::SendString::Response::SharedPtr /*res*/) {
		accumulated_locations_ = YAML::LoadFile(req->data.data).as<std::map<std::string, std::vector<pcl::PointCloud<pcl::PointXYZ> > > >();
	}

	/// Remove accumulated board location of a certain index
	void remove(calibration_interfaces::srv::SendUInt::Request::ConstSharedPtr req, calibration_interfaces::srv::SendUInt::Response::SharedPtr /*res*/) {
		unsigned int index = req->data.data;
		for (auto & sensor : accumulated_locations_) {
			if (sensor.second.size() < index) {
				RCLCPP_WARN_STREAM(get_logger(), "Cannot remove index '" << index << "', only '" << sensor.second.size() << "' accumulated board locations available.");
				return;
			}
			sensor.second.erase(sensor.second.begin() + index);
		}
	}

	/// Point cloud callback function
	void callback(PointCloud2::ConstSharedPtr in, const std::string &sensor_pattern_topic) {
		// pcl::PointCloud<pcl::PointXYZ>::ConstPtr const & in_pcl;
		RCLCPP_INFO_ONCE(get_logger(), "Receiving pattern.");
		if (accumulate_) {
			current_location_patterns_.at(sensor_pattern_topic).push_back(toPcl(*in));
		}
	}

	/// Publish markers of the accumulated patterns
	void publishMarkers(std::vector<calibration_interfaces::msg::AccumulatedPatterns> const & accumulated_locations_) {
		visualization_msgs::msg::MarkerArray marker_array;
		size_t marker_id = 0; // Used to plot all detections
		for (auto const & sensor : accumulated_locations_) {
			std_msgs::msg::ColorRGBA color = getColor(sensor.sensor.data);
			// Check if current sensor is the radar sensor
			bool is_radar = isRadarPcl(toPcl(sensor.patterns.at(0)));

			for (std::size_t i = 0; i < sensor.patterns.size(); ++i) {
				// Plot detections
				visualization_msgs::msg::Marker detections = toMarker(sensor.patterns.at(i), color);
				detections.id = marker_id; // Each marker requires different ID
				marker_id++;

				// Plot
				visualization_msgs::msg::Marker text = toMarker(sensor.patterns.at(i).header.frame_id, sensor.patterns.at(i), i, color);
				text.id = marker_id; // Each marker requires different ID
				marker_id++;

				// The radar sensor is a special case:
				if (is_radar) {
					// Z field for radar contains RCS value
					// If this sensor is a radar, Z field should be equal to 0 for visualisation of pointcloud
					detections.points.at(0).z = 0;
					text.pose.position.z = -0.1;

					visualization_msgs::msg::Marker arc = toArc(sensor.patterns.at(i), color, maximum_elevation_angle_);
					arc.id = marker_id; // Each marker requires different ID
					marker_id++;
					marker_array.markers.push_back(arc);
				}

				// Push back markers:
				marker_array.markers.push_back(detections);
				marker_array.markers.push_back(text);
			}
		}
		markers_publisher_->publish(marker_array);
	}
};

} // namespace
