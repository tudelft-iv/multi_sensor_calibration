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

#include "util.hpp"


namespace mono_detector {

std::string toYaml(pcl::PointCloud<pcl::PointXYZ> const & cloud) {
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

Eigen::Isometry3f toEigen(std::vector<double> const & rvec, std::vector<double> const & tvec) {
	cv::Mat rotation_mat;
	cv::Rodrigues(rvec, rotation_mat);

	Eigen::Matrix3d rotation_eigen;
	cv::cv2eigen(rotation_mat, rotation_eigen);

	Eigen::Quaterniond quaternion(rotation_eigen);
	Eigen::Translation3d translation(tvec.at(0), tvec.at(1), tvec.at(2));

	Eigen::Isometry3d isometry = translation * quaternion;

	return isometry.cast<float>();
}

pcl::PointXYZ toPcl(cv::Point3f const & point) {
	return {point.x, point.y, point.z};
}

pcl::PointCloud<pcl::PointXYZ> toPcl(std::vector<cv::Point3f> const & points) {
	pcl::PointCloud<pcl::PointXYZ> out;
	for (const auto & point : points) {
		out.push_back(toPcl(point));
	}
	return out;
}

cv::Mat toOpencv(const sensor_msgs::ImageConstPtr & in) {
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(in, in->encoding);
	return cv_ptr->image;
}

std::vector<cv::Point2f> toCvPoint2fVector(std::vector<cv::Vec3f> const & circles) {
	std::vector<cv::Point2f> result;
	for (auto const & c : circles) {
		result.push_back(cv::Point2f(c[0], c[1]));
	}
	return result;
}

cv::Mat gaussianBlur(cv::Mat const & image, GaussConfig const & config) {
	cv::Mat result;
	cv::GaussianBlur(image, result, cv::Size(config.ksize_x, config.ksize_y), config.sigma_x, config.sigma_y);
	return result;
}

cv::Point2f calculateCenter(std::vector<cv::Point2f> const & in) {
	cv::Point2f out(0, 0);
	for (const auto & point : in) {
		out.x += point.x;
		out.y += point.y;
	}
	out.x /= in.size();
	out.y /= in.size();
	return out;
}

cv::Point3f calculateCenter(std::vector<cv::Point3f> const & in) {
	cv::Point3f out(0, 0, 0);
	for (const auto & point : in) {
		out.x += point.x;
		out.y += point.y;
		out.z += point.z;
	}
	out.x /= in.size();
	out.y /= in.size();
	out.z /= in.size();
	return out;
}

void visualize(cv::Mat const & image, std::vector<cv::Vec3f> const & circles, cv::Rect const & roi) {
	cv::Mat draw = image.clone();
	if (draw.channels() == 1) {
		cv::cvtColor(draw, draw, cv::COLOR_GRAY2BGR);
	}
	for (auto const & c : circles) {
		cv::circle(draw, cv::Point(c[0], c[1]), c[2], cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
		cv::circle(draw, cv::Point(c[0], c[1]), 2, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
	}
	if (roi != cv::Rect()) {
		cv::rectangle(draw, roi.tl(), roi.br(), cv::Scalar(255, 0, 0), 3);
	}
	std::string window_name = "result";
	cv::namedWindow(window_name, CV_WINDOW_NORMAL);
	cv::imshow(window_name, draw);
	cv::waitKey(100);
}

std::vector<double> compute_median_circle(std::vector<cv::Vec3f> const & circles) { 
	// Output vector
	std::vector<double> median_vector;

	// Loop over the 3D vector of circle: x,y,radius:
	for (int dim = 0; dim < 3; dim++) {
		std::vector<double> v;
		for (const auto & circle : circles) {
			v.push_back(circle[dim]);
		}
		// Find median
		std::nth_element(v.begin(), v.begin() + v.size()/2, v.end());

		// Push back current median
		median_vector.push_back(v[v.size()/2]);
	}

	return median_vector;
}

}
