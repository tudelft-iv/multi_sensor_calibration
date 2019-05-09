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

#include "urdf_calibration.hpp"
#include "eigen_kdl.hpp"
#include "eigen_string.hpp"
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

namespace urdf_calibration {

namespace {

	/// Get the kdl chain
	KDL::Chain getChain(KDL::Tree const & kdl_tree, std::string const & start, std::string const & end) {
		KDL::Chain chain;
		if (kdl_tree.getChain(start, end, chain)) {
			return chain;
		} else {
			throw std::runtime_error("No chain found from frame `" + start + "' to frame `" + end + "'.");
		}
	};

	/// Function to load calibration from urdf file
	Calibration loadCalibrationUrdf(std::string const & file) {
		TiXmlDocument doc = loadTiXmlFromFile(file);
		TiXmlElement * root = doc.FirstChildElement("joint");
		std::string source, target, xyz, rpy;
		for (TiXmlElement * n = root->FirstChildElement(); n; n=n->NextSiblingElement()) {
			if (n->ValueStr() == "child") {
				target = n->Attribute("link");
			} else if (n->ValueStr() == "parent") {
				source = n->Attribute("link");
			} else if (n->ValueStr() == "origin") {
				xyz = n->Attribute("xyz");
				rpy = n->Attribute("rpy");
			}
		}
		Eigen::Isometry3d transform = toEigen(xyz, rpy);
		Calibration result;
		result.transform = transform;
		result.source    = source;
		result.target    = target;
		return result;
	}

	/// Prints the xml document to cout
	void printXml(TiXmlDocument const & doc) {
		TiXmlPrinter printer;
		doc.Accept(&printer);
		std::cout << printer.Str() << std::endl;
	}

	/// Get a the pose of the end frame relative to the start frame of a chain
	Eigen::Isometry3d getPose(KDL::Chain const & chain) {
		KDL::Frame transform = KDL::Frame::Identity();
		for (auto const & segment : chain.segments) {
			// Make sure we're only dealing with fixed joints.
			if (segment.getJoint().getType() != KDL::Joint::JointType::None) {
				throw std::runtime_error("Non-fixed joint `" + segment.getName() + "' found in chain, but no joint positions are given.");
			}
			transform = transform * segment.pose(0);
		}
		return toEigen(transform);
	}

	/// Get the transform between source and target of kdl tree
	Eigen::Isometry3d getPose(KDL::Tree const & kdl_tree, std::string const & source, std::string const & target) {
		KDL::Chain chain = getChain(kdl_tree, source, target);
		return getPose(chain);
	}

	/// Check if the joint is on the chain
	bool jointOnChain(KDL::Tree const & tree, std::string const & joint, std::string const & source, std::string const & target) {
		KDL::Chain chain;
		tree.getChain(source, target, chain);
		bool found = false;
		for (std::size_t i = 0; i < chain.getNrOfSegments(); ++i) {
			if (joint == chain.getSegment(i).getName()) {
				found = true;
			}
		}
		return found;
	}

	// From output of static tf publisher
	Eigen::Isometry3d fromTf(double const x, double const y, double const z, double const yaw, double const pitch, double const roll) {
		Eigen::Quaterniond q =
			Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
			* Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
		Eigen::Translation3d t(x, y, z);
		Eigen::Isometry3d static_tf_result = t * q;
		return static_tf_result;
	}

	/// Convert a quaternion to rpy
	Eigen::Vector3d quaternionToRpy(Eigen::Quaterniond const & quaternion) {
		// zyx_euler angle rotation, in order yaw, pitch, roll
		Eigen::Vector3d zyx_euler_angle_rotation = quaternion.matrix().eulerAngles(2, 1, 0);
		// rpy is rotation around x, y, z (applied in reversed order) (https://math.stackexchange.com/questions/147028/are-euler-angles-the-same-as-pitch-roll-and-yaw)
		Eigen::Vector3d rpy(zyx_euler_angle_rotation[2], zyx_euler_angle_rotation[1], zyx_euler_angle_rotation[0]);
		return rpy;
	}

	/// Update the pose for a joint
	void setPose(TiXmlElement & element, Eigen::Isometry3d const & pose) {
		for (TiXmlElement * n = element.FirstChildElement(); n; n=n->NextSiblingElement()) {
			if (n->ValueStr() == "origin") {
				std::string xyz = n->Attribute("xyz");
				std::string rpy = n->Attribute("rpy");
				n->SetAttribute("xyz", toString(pose.translation()));
				n->SetAttribute("rpy", toString(quaternionToRpy(Eigen::Quaterniond(pose.rotation()))));
			}
		}
	}


} // namespace


Calibration loadCalibration(std::string const & file) {
	std::string extension = boost::filesystem::extension(file);
	std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower); // Check extension in a case insensitive manner
	if (extension == ".yaml" || extension == ".yml") {
		return YAML::LoadFile(file).as<Calibration>();
	}
	if (extension == ".urdf") {
		return loadCalibrationUrdf(file);
	}
	throw std::runtime_error("Could not load calibration file " + file + ", file extension shoud be yml, yaml, urdf or URDF.");
}

TiXmlDocument loadTiXmlFromFile(std::string const & file) {
	TiXmlDocument doc(file);
	doc.LoadFile();
	return doc;
}

urdf::Model extractUrdf(TiXmlDocument & ti_xml_doc) {
	urdf::Model urdf;
	if (!urdf.initXml(&ti_xml_doc)) {
		throw std::runtime_error("Failed to parse urdf robot model from XML");
	}
	return urdf;
}

KDL::Tree loadKdlTree(urdf::Model const & urdf) {
	KDL::Tree kdl_tree;
	if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree)){
		throw std::runtime_error("Failed to construct kdl tree from urdf model");
	}
	return kdl_tree;
}

std::string getParent(urdf::Model const & urdf, std::string const & joint) {
	return urdf.getLink(joint)->getParent()->name;
}

Eigen::Isometry3d calculateCalibratedTransform(
	KDL::Tree const & kdl_tree,
	std::string const & update_joint,
	std::string const & parent,
	Calibration const & calibration
) {
	if (!jointOnChain(kdl_tree, parent, calibration.source, calibration.target)) {
		throw std::runtime_error("joint '" + parent + "' is not part of the chain between '" + calibration.source + "' and '" + calibration.target + "'.");
	}

	bool update_joint_above_target = jointOnChain(kdl_tree, update_joint, parent, calibration.target);
	Eigen::Isometry3d pose_of_joint_to_update_in_parent_frame;
	if (!update_joint_above_target) {
		ROS_INFO_STREAM("Update joint '" + update_joint + "' it not part of the chain between '" + parent + "' and '" + calibration.target + "', inverting both transform, source and target of the calibration.");

		// Transformation from B to A is the same as the the pose of B expressed in A, therefore we apply the inverse here
		Eigen::Isometry3d source_to_update_joint = getPose(kdl_tree, calibration.source, update_joint).inverse();
		Eigen::Isometry3d parent_to_target       = getPose(kdl_tree, parent, calibration.target).inverse();

		// Calculate the transform from joint to parent
		Eigen::Isometry3d parent_to_joint =
			source_to_update_joint
			* calibration.transform.inverse()
			* parent_to_target;

		// The pose of the joint to update will be written to the urdf, which is the same as the inverse of the transformation from joint to parent
		pose_of_joint_to_update_in_parent_frame = parent_to_joint.inverse();

	} else {

		// Transformation from B to A is the same as the the pose of B expressed in A, therefore we apply the inverse here
		Eigen::Isometry3d target_to_update_joint = getPose(kdl_tree, calibration.target, update_joint).inverse();
		Eigen::Isometry3d parent_to_source       = getPose(kdl_tree, parent, calibration.source).inverse();

		// Calculate the transform from joint to parent
		Eigen::Isometry3d parent_to_joint =
			target_to_update_joint
			* calibration.transform
			* parent_to_source;

		// The pose of the joint to update will be written to the urdf, which is the same as the inverse of the transformation from joint to parent
		pose_of_joint_to_update_in_parent_frame = parent_to_joint.inverse();
	}
	return pose_of_joint_to_update_in_parent_frame;
}

void updateXml(
	TiXmlDocument & doc,
	std::string const & joint_to_update,
	Eigen::Isometry3d const & pose
) {
	TiXmlElement * root = doc.FirstChildElement("robot");
	for (TiXmlElement  * n = root->FirstChildElement(); n; n=n->NextSiblingElement()) { // loop over all links, joints and gazbo tags
		if (n->ValueStr() == "joint") {
			std::string name = n->Attribute("name");
			if (name == joint_to_update) {
				setPose(*n, pose);
			}
		}
	}
	return;
}


} // urdf_calibration namespace
