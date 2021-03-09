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
#include "yaml.hpp"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

#include <boost/filesystem.hpp>

/*
 * Get File extension from File path or File Name
 */
std::string getFileExtension(std::string filePath)
{
	// Create a Path object from given string
	boost::filesystem::path pathObj(filePath);
	// Check if file name in the path object has extension
	if (pathObj.has_extension()) {
		// Fetch the extension from path object and return
		return pathObj.extension().string();
	}
	// In case of no extension return empty string
	return "";
}

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

}

int main(int argc, char * * argv) {
	using namespace lidar_detector;
	std::cerr << "Starting: '" << argv[0] << "'." << std::endl;
	// Parse arguments
	if (argc != 4) {
		std::cerr << "Usage: " << argv[0] <<
			" <input_point_cloud_file.pcd>" <<
			" <configuration_yaml.yaml>" <<
			" <output_file.pcd/output_file.yaml>" <<
			"\nNumber arguments given: '" << argc << "'." << std::endl;
		return 1;
	}
	std::string point_cloud_file = argv[1];
	Configuration config = YAML::LoadFile(argv[2]).as<Configuration>();

	// Load point cloud
	pcl::PointCloud<Lidar::PointWithDist> cloud;
	if (pcl::io::loadPCDFile<Lidar::PointWithDist>(point_cloud_file, cloud) != 0) {
		std::cerr << "Could not open file: '" << point_cloud_file << "'." << std::endl;
		return 1;
	}

	// Do the actual keypoint detection
	std::cerr << "starting detection." << std::endl;
	pcl::PointCloud<pcl::PointXYZ> detected_pattern = keypointDetection(cloud, config);

	// Check file extension
	std::string file_extension = getFileExtension(argv[3]);
	if (file_extension == ".yaml") {
		// Write result as yaml
		std::ofstream file(argv[3]);
		file << toYaml(detected_pattern);
		std::cerr << "Output written to file: '" << argv[3] << "'." << std::endl;
	}
	else if (file_extension == ".pcd") {
		// Save pointcloud to .pcd file
		pcl::io::savePCDFileASCII (argv[3], detected_pattern);
	}
	else {
		// Cannot save the data because file extension is not known
		std::cerr << "Output file extension should be either .yaml or .pcd." << std::endl;
	}

}
