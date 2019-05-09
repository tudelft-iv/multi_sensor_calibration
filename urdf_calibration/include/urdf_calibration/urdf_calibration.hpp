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
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <tinyxml.h>
#include <Eigen/Dense>
#include "yaml.hpp"
#include "types.hpp"

namespace urdf_calibration {

/// Load calibration (delegate to urdf or yaml)
Calibration loadCalibration(std::string const & file);

/// Loads the xml document from a file
TiXmlDocument loadTiXmlFromFile(std::string const & file);

/// Extracts the urdf from the xml (lossy)
urdf::Model extractUrdf(TiXmlDocument & ti_xml_doc);

/// Create a kdl tree from the urdf model
KDL::Tree loadKdlTree(urdf::Model const & urdf);

/// Get the parent of a joint
std::string getParent(urdf::Model const & urdf, std::string const & joint);

/// Lookup the poses and calculate the new pose for update_joint
Eigen::Isometry3d calculateCalibratedTransform(
	KDL::Tree const & kdl_tree,
	std::string const & update_joint,
	std::string const & parent,
	Calibration const & calibration
);

/// Update the xml document with the new pose at update_joint
void updateXml(
	TiXmlDocument & doc,
	std::string const & joint_to_update,
	Eigen::Isometry3d const & pose
);


}
