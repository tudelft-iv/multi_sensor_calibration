#!/usr/bin/env python3
# multi_sensor_calibration
# Copyright (C) 2019  Intelligent Vehicles Delft University of Technology

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys
import os
import traceback

import rclpy
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node

from calibration_interfaces.srv import SendPatterns
from optimization.optimization.io import *
from optimization.optimization.calibration_board import *
from optimization.optimization.helper_functions import *
from optimization.optimization.config import *
from optimization.optimization.optimize import joint_optimization,remove_non_visible_detections_in_reference_sensor


def convert_tuple_to_list(pattern, nr_detections):
    reshaped_pattern = np.zeros((3, nr_detections))
    for k in range(len(pattern)):
        for j in range(3):
            reshaped_pattern[j, k] = pattern[k][j]

    return reshaped_pattern


def get_pcl_sensor(msg, nr_detections):
    data = np.zeros((3, len(msg.patterns) * nr_detections))
    for i in range(len(msg.patterns)):
        # Convert ROS pointlcoud2 message to numpy array
        pattern = pc2.read_points_numpy(msg.patterns[i])
        # Add into data numpy array - with conversion from numpy tuples to numpy list
        data[:, i * nr_detections:(i + 1) * nr_detections] = convert_tuple_to_list(pattern, nr_detections)

    return data


class OptimizerNode(Node):
    """ Optimizer node that receives service call from accumulator with all the detections and calls optimizer to compute all sensor poses

    ROS parameters:
        - calibration_mode

    """

    def __init__(self):
        super().__init__('optimizer')

        self.declare_parameter("calibration_mode", 3)
        self.declare_parameter("correspondences", "known")    # For lidar and stereo, 4 circles are detected. If 'known' the correpondences are known between lidar and stereo, else not.
        self.declare_parameter("reference_sensor", "velodyne")    # reference sensor
        self.declare_parameter("export_detections_to_files", True)
        self.declare_parameter("visualise", True)  # visualise result using matplotlib 3D visualisation
        self.declare_parameter("results_folder", "results")  # os.path.join(pkg_dir, "results")
        self.declare_parameter("detections_folder", "data")  # os.path.join(pkg_dir, "data")
        self.declare_parameter("reordering", "based_on_reference_sensor")
        self.declare_parameter("outlier_removal", "remove_board_locations")

        # Service call receiver
        s = self.create_service(SendPatterns, '/optimizer/optimize', self.optimize)
        self.get_logger().info('Optimizer ready!')

    def optimize(self, req, response):
        try:
            # Notifiy that service call is received:
            self.get_logger().info(f'Received service call!')

            # Parameters for Joint Optimization:
            calibration_mode = self.get_parameter("calibration_mode").get_parameter_value().integer_value
            # Possible calibration modes:
                # 0: Pose and Structure Estimation (PSE) with unknown observation covariance matrices
                # 1: Pose and Structure Estimation (PSE) with known observation covariance matrices
                # 2: Minimally Connected Pose Estimation (MCPE)
                # 3: Fully Connected Pose Estimation (FCPE)
            correspondences = self.get_parameter("correspondences").get_parameter_value().string_value   # For lidar and stereo, 4 circles are detected. If 'known' the correpondences are known between lidar and stereo, else not.
            reference_sensor = self.get_parameter("reference_sensor").get_parameter_value().string_value    # reference sensor
            export_detections_to_files = self.get_parameter("export_detections_to_files").get_parameter_value().bool_value
            visualise = self.get_parameter("visualise").get_parameter_value().bool_value  # visualise result using matplotlib 3D visualisation
            results_folder = self.get_parameter("results_folder").get_parameter_value().string_value
            detections_folder = self.get_parameter("detections_folder").get_parameter_value().string_value
            reordering_method = self.get_parameter("reordering").get_parameter_value().string_value
            outlier_removal_method = self.get_parameter("outlier_removal").get_parameter_value().string_value

            os.makedirs(results_folder, exist_ok=True)
            os.makedirs(detections_folder, exist_ok=True)

            # Convert ROS service call to sensors struct
            sensors, nr_calib_boards = self.convert_service_to_sensors_struct(req, correspondences, reference_sensor, reordering_method, outlier_removal_method)

            if export_detections_to_files:
                # Save as CSV
                export_sensor_data_to_csv(sensors, detections_folder)
                # Save as YAML
                export_sensor_data_to_yaml(sensors, detections_folder)

            # Joint Optimization
            Tms = joint_optimization(sensors, calibration_mode, correspondences, reference_sensor, visualise, results_folder)

            # Send back response
            return response
        except Exception:
            self.get_logger().warn(f'Exception while running the optimization: {traceback.format_exc()}')
            return response

    def convert_service_to_sensors_struct(self, req, correspondences, reference_sensor, reordering_method, outlier_removal_method):
        # Retrieve numpy arrays
        sensors = []
        for i_sensor in range(len(req.accumulated_patterns)):
            # Get sensor type from ros service all
            sensor_topic_type = req.accumulated_patterns[i_sensor].sensor.data # This parameter is used to determine if the sensor is a lidar, camera or radar
            self.get_logger().info('Sensor topic name: ' + sensor_topic_type)
            # Based on sensor_topic_type we known which sensor it is
            # This means that for instance a lidar should contain lidar in the topic name
            if 'stereo' in sensor_topic_type:
                Xc = get_pcl_sensor(req.accumulated_patterns[i_sensor], get_nr_detection('camera'))

                # Convert to sensor struct:
                sensor = get_camera(Xc)
            elif 'mono' in sensor_topic_type:
                Xc = get_pcl_sensor(req.accumulated_patterns[i_sensor], get_nr_detection('camera'))

                # Convert to sensor struct:
                sensor = get_camera(Xc)
            elif 'lidar' in sensor_topic_type:
                Xl = get_pcl_sensor(req.accumulated_patterns[i_sensor], get_nr_detection('lidar'))

                # Convert to sensor struct:
                sensor = get_lidar(Xl)
            elif 'radar' in sensor_topic_type:
                pcl = get_pcl_sensor(req.accumulated_patterns[i_sensor], get_nr_detection('radar'))
                # the third axis will be zero for 2D radars
                if np.all(pcl[2, :] == 0):
                    Xr = pcl[:2, :]
                else:
                    Xr = pcl
                #rcs = pcl[2, :]

                # Setup sensors struct
                sensor = get_radar(Xr, None)
            else:
                raise ValueError('ROS topic name should contain one the the four keywords: stereo, mono, lidar, radar')

            # Get sensor link from ROS service call
            for i in range(len(req.accumulated_patterns[i_sensor].patterns)):
                # Get frame_id
                frame_id = req.accumulated_patterns[i_sensor].patterns[i].header.frame_id
                if frame_id:  # If frame id is not empty
                    sensor.link = frame_id  # Link name is derived from frame_id message
                    sensor.name = sensor.link  # Name of the sensor is set to link name
                    # Link name is defined so let's continue:
                    break

            # Append sensor in list
            sensors.append(sensor)

        # Get nr calibration boards in this recording
        nr_calib_boards = int(len(sensors[0].mu) / get_nr_detection(sensors[0].type))

        # Outlier removal
        sensors, nr_calib_boards = remove_outlier_detections(sensors, nr_calib_boards, outlier_removal_method)

        # Reorder detections
        try:
            sensors = reorder_detections_sensors(sensors, reordering_method, reference_sensor)
        except ValueError as msg_value_error:
            self.get_logger().info('----------------------------------------------------------')
            self.get_logger().info(msg_value_error)
            self.get_logger().info('----------------------------------------------------------')
            # ValueError 1: reference sensor is not defined
            # ValueError 2: reference sensor contains non visible detection therefore reordering cannot be done for those calibration board locations

            # Pick sensor as reference sensor for reindexing:
            for i in range(len(sensors)):
                if sensors[i].type != 'radar':
                    index_reference_sensor = i
                    break
            # Remove all non visible detections in reference sensors such that we can reorder based on that
            sensors = remove_non_visible_detections_in_reference_sensor(sensors, sensors[index_reference_sensor].name)
            # Retry reordering based on reindex using reference sensor
            sensors = reorder_detections_sensors(sensors, 'based_on_reference_sensor', sensors[index_reference_sensor].name)

        return sensors, nr_calib_boards


def main():
    rclpy.init(args=sys.argv)
    optimizer_node = OptimizerNode()

    rclpy.spin(optimizer_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
