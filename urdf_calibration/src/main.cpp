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

#include <boost/filesystem.hpp>
#include "urdf_calibration.hpp"

int main(int argc, char * * argv) {
	using namespace urdf_calibration;
	if (argc != 6) {
		std::cerr << "usage: " << argv[0] << " <input_urdf_file> <calibration_urdf_or_yaml_file> <output_urdf_file> <link_to_update> <joint_to_update>" << std::endl;
		return 1;
	}

	std::string input_urdf_file        = argv[1];
	std::string calibration_file       = argv[2];
	std::string output_urdf_file       = argv[3];
	std::string link_to_update         = argv[4];
	std::string joint_to_update        = argv[5];

	// Check if files exist
	if (!boost::filesystem::exists(input_urdf_file))  { throw std::runtime_error("input_urdf_file "  + input_urdf_file  + " does not exist"); }
	if (!boost::filesystem::exists(calibration_file)) { throw std::runtime_error("calibration_file " + calibration_file + " does not exist"); }

	// Load calibration urdf file with source, target and transform
	Calibration calibration = loadCalibration(calibration_file);

	// Load XML document
	TiXmlDocument xml = loadTiXmlFromFile(input_urdf_file);

	// Extract urdf
	urdf::Model urdf = extractUrdf(xml);

	// Load Kdl Tree
	KDL::Tree kdl_tree = loadKdlTree(urdf);

	// ToDo: Check if parent is on chain, otherwise abort..
	std::string parent = getParent(urdf, link_to_update);

	// ToDo: Calculate transform to update for link_to_update
	Eigen::Isometry3d pose = calculateCalibratedTransform(kdl_tree, link_to_update, parent, calibration);

	// Update Xml Document
	updateXml(xml, joint_to_update, pose);

	// Save to file
	xml.SaveFile(output_urdf_file);
	std::cerr << "New urdf written to: " << output_urdf_file << std::endl;
	return 0;
}
