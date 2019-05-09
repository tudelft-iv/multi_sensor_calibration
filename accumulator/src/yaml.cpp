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

namespace YAML {

namespace {

	std::vector<pcl::PointXYZ> toVector(pcl::PointCloud<pcl::PointXYZ> const & cloud) {
		std::vector<pcl::PointXYZ> out;
		for (const auto & p : cloud) {
			out.push_back(p);
		}
		return out;
	}

	pcl::PointCloud<pcl::PointXYZ> toCloud(std::vector<pcl::PointXYZ> const & vector) {
		pcl::PointCloud<pcl::PointXYZ> out;
		for (const auto & p : vector) {
			out.push_back(p);
		}
		return out;
	}
}


Node convert<pcl::PointXYZ>::encode(pcl::PointXYZ const & config) {
	Node node;
	if (std::isnan(config.x)) {
		// Newer version of yamlcpp already has solution for nan and infitity:
		// https://github.com/jbeder/yaml-cpp/issues/507
		node["x"]      = ".nan";
		node["y"]      = ".nan";
		node["z"]      = ".nan";
	}
	else
	{	
		// Normal case
		node["x"]      = config.x;
		node["y"]      = config.y;
		node["z"]      = config.z;
	}

	return node;
}
bool convert<pcl::PointXYZ>::decode(const Node & node, pcl::PointXYZ & config) {
	config.x       = node["x"].as<float>();
	config.y       = node["y"].as<float>();
	config.z       = node["z"].as<float>();
	return true;
}



Node convert<pcl::PointCloud<pcl::PointXYZ> >::encode(pcl::PointCloud<pcl::PointXYZ> const & config) {
	Node node;
	node["frame_id"] = config.header.frame_id;
	node["points"]   = toVector(config);
	return node;
}
bool convert<pcl::PointCloud<pcl::PointXYZ> >::decode(const Node & node, pcl::PointCloud<pcl::PointXYZ> & config) {
	config                 = toCloud(node["points"].as<std::vector<pcl::PointXYZ> >());
	config.header.frame_id = node["frame_id"].as<std::string>();
	return true;
}

}
