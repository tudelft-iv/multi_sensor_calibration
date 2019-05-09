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
#include <gtest/gtest.h>

TEST(Yaml, parseConfig) {

	// Setup some particular configuration
	mono_detector::Configuration expected;
	expected.pre_blur.ksize_x = 5;
	expected.roi = cv::Rect(5, 4, 7, 9);

	// Convert the configuration to yaml
	YAML::Node node(expected);

	// Convert back to configuration
	mono_detector::Configuration result = node.as<mono_detector::Configuration>();

	// Compare expectation with result
	EXPECT_TRUE(expected.pre_blur.ksize_x == result.pre_blur.ksize_x);
	EXPECT_TRUE(expected.roi.x            == result.roi.x);
	EXPECT_TRUE(expected.roi.y            == result.roi.y);
	EXPECT_TRUE(expected.roi.width        == result.roi.width);
	EXPECT_TRUE(expected.roi.height       == result.roi.height);
}


int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
