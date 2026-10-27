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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "keypoint_detection.hpp"
#include "yaml.hpp"

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

namespace stereo_detector {
namespace {

// (pattern) cloud to yaml string conversion function (ToDo: Move to separate package, avoid code duplication)
inline std::string toYaml(pcl::PointCloud<pcl::PointXYZRGB> const & cloud) {
	std::string out;
	for (const auto & p : cloud) {
		out += "- {x: ";
		out += std::to_string(p.x);
		out += ", y: ";
		out += std::to_string(p.y);
		out += ", z: ";
		out += std::to_string(p.z);
		out += "}\n";
	}
	return out;
}

}}

int main(int argc, char * * argv) {
	using namespace stereo_detector;

	// Parse arguments
	if (argc != 5) {
		std::cerr << "Usage: " << argv[0] << " <input_image_file.png> <input_point_cloud_file.pcd> <config.yaml> <output_file.pcd/output_file.yaml>"
			<< "\nNumber of arguments given: '" << argc << "'." << std::endl;
		return 1;
	}
	std::string image_file = argv[1];
	std::string point_cloud_file = argv[2];
	Configuration config = YAML::LoadFile(argv[3]).as<Configuration>();
	std::string output = std::string(argv[4]);

	// Load point cloud
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(point_cloud_file, cloud) != 0) {
		std::cerr << "Could not open file: '" << point_cloud_file << "'." << std::endl;
		return 1;
	}

	// Load image
	cv::Mat image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
	if (image.empty()) {
		std::cout << "Could not open file: '" << image_file << "'." << std::endl;
		return 1;
	}

	// Do the actual processing
	pcl::PointCloud<pcl::PointXYZRGB> detected_pattern = keypointDetection(
		image,
		cloud,
		config
	);

	// Check file extension
	std::string file_extension = getFileExtension(argv[4]);
	if (file_extension == ".yaml") {
		// Write result as yaml
		std::ofstream file(argv[4]);
		file << toYaml(detected_pattern);
		std::cerr << "Output written to file: '" << argv[4] << "'." << std::endl;
	}
	else if (file_extension == ".pcd") {
		// Save pointcloud to .pcd file
		pcl::io::savePCDFileASCII (argv[4], detected_pattern);
	}
	else {
		// Cannot save the data because file extension is not known
		std::cerr << "Output file extension should be either .yaml or .pcd." << std::endl;
	}

	return 0;
}
