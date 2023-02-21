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

#include "yaml.hpp"
#include "detector.hpp"
#include "pnp.hpp"
#include "types.hpp"
#include "eigen.hpp"

#include <camera_calibration_parsers/parse.h> // ToDo: Remove ros dependency
#include <pcl/common/transforms.h>
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

namespace mono_detector {
namespace {

/// Loads intrinsics from file (to be replaced by a non-ros method to avoid ros dependency in cli)
CameraModel loadIntrinsics(std::string const & file_name) {
	// Load intrinsics yaml/ini file as camera info
	sensor_msgs::CameraInfo camera_info;
	std::string camera_name;
	if (!camera_calibration_parsers::readCalibration(file_name, camera_name, camera_info)) {
		throw std::runtime_error("Unable to read camera parameters from: '" + file_name + "'.");
	}

	// Use image geometry to get cv::Mat from camera info
	CameraModel model;
	model.fromCameraInfo(camera_info);
	return model;
}

/// Write as yaml (result and some additional metadata)
void write(
	Eigen::Isometry3f const & isometry,
	pcl::PointCloud<pcl::PointXYZ> const & object_points,
	pcl::PointCloud<pcl::PointXYZ> const & transformed_object_points,
	std::string const & image_file_name,
	std::string const & intrinsics_file_name,
	Configuration const & config
) {
	std::cerr
		<< "object points\n" << toYaml(object_points) << "\n"
		<< "transformed object points\n" << toYaml(transformed_object_points) << "\n"
		<< "Isometry:\n" << toYaml(isometry) << "\n"
		<< "image file name: " << image_file_name << "\n"
		<< "intrinsics file name: " << intrinsics_file_name << "\n"
		<< YAML::Node(config) << std::endl;
}
}

}

int main(int argc, char * * argv) {
	using namespace mono_detector;

	// Check input
	if (argc != 6) {
		std::cerr << "Usage: " << argv[0] << " <input_image.png> <image_processing.yaml> <intrinsics.ini> <object_points.yaml> <output_file.pcd/output_file.yaml" << std::endl;
		return 1;
	}

	// Read image
	std::string image_file_name = argv[1];
	cv::Mat image = cv::imread(image_file_name, cv::IMREAD_GRAYSCALE);
	if (image.empty()) {
		std::cout << "Cannot open file: '" << image_file_name << "'." << std::endl;
		return 1;
	}

	// Load configuration
	Configuration config = YAML::LoadFile(argv[2]).as<Configuration>();

	// Load intrinsics (parsing the ini is a dependency on a ros package, // ToDo: Remove dependency
	std::string intrinsics_file_name = argv[3];
	CameraModel intrinsics = loadIntrinsics(intrinsics_file_name);

	// Load object points
	std::vector<cv::Point3f> object_points = YAML::LoadFile(argv[4]).as<std::vector<cv::Point3f>>();
	cv::Point3f center = calculateCenter(object_points);
	std::sort(object_points.begin(), object_points.end(), [center](cv::Point3f a, cv::Point3f b) {
		return std::atan((a.y - center.y) / (a.x - center.x)) > std::atan((b.y - center.y) / (b.x - center.x));
	});

	// Run detection algorithm
	std::vector<cv::Point2f> image_points;
	std::vector<float> radi;
	detectMono(image, config, image_points, radi);

	// Run solvePnP
	Eigen::Isometry3f isometry = solvePose(image_points, object_points, intrinsics);

	// Transform
	pcl::PointCloud<pcl::PointXYZ> transformed_object_points;
	pcl::transformPointCloud(toPcl(object_points), transformed_object_points, isometry);

	// Write to stdout
	write(isometry, toPcl(object_points), transformed_object_points, image_file_name, intrinsics_file_name, config);

	// Check file extension
	std::string file_extension = getFileExtension(argv[5]);
	if (file_extension == ".yaml") {
		// Write result as yaml
		std::ofstream file(argv[5]);
		file << toYaml(transformed_object_points);
		std::cerr << "Output written to file: '" << argv[5] << "'." << std::endl;
	}
	else if (file_extension == ".pcd") {
		// Save pointcloud to .pcd file
		pcl::io::savePCDFileASCII (argv[5], transformed_object_points);
	}
	else {
		// Cannot save the data because file extension is not known
		std::cerr << "Output file extension should be either .yaml or .pcd." << std::endl;
	}

	return 0;
}
