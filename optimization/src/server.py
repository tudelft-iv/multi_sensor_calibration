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
import rospkg
NAME = 'optimizer'
pkg_name = rospkg.get_package_name(os.path.dirname(os.path.realpath(__file__)))
pkg_dir = rospkg.RosPack().get_path(pkg_name)
sys.path.append(pkg_dir + '/lib/icp') #Added for lib/icp folder in ROS
from accumulator.srv import *
import rospy
import numpy as np
from optimization.io import *
from optimization.calibration_board import *
from optimization.helper_functions import *
from optimization.config import *
from optimization.optimize import joint_optimization,remove_non_visible_detections_in_reference_sensor
import ros_numpy

def print_service_call_message(msg):
    for i_sensor in range(len(msg.accumulated_patterns)):
        print('Sensor type: ' + msg.accumulated_patterns[i_sensor].sensor.data)
        for i in range(len(msg.accumulated_patterns[i_sensor].patterns)):
            pcl_cb = msg.accumulated_patterns[i_sensor].patterns[i]
            print('Calibration board location: ' + str(i))
            print(ros_numpy.point_cloud2.pointcloud2_to_array(pcl_cb).shape)


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
        pattern = ros_numpy.point_cloud2.pointcloud2_to_array(msg.patterns[i])
        # Add into data numpy array - with conversion from numpy tuples to numpy list
        data[:, i * nr_detections:(i + 1) * nr_detections] = convert_tuple_to_list(pattern, nr_detections)

    return data

class optimizer_node():
    """ Optimizer node that receives service call from accumulator with all the detections and calls optimizer to compute all sensor poses
    
    ROS parameters:
        - calibration_mode
        
    """

    def __init__(self):
        # Initialise ROS node
        rospy.init_node(NAME)

        # Service call receiver
        s = rospy.Service('/optimizer/optimize', SendPatterns, self.optimize)

        # ROS parameters
        rospy.set_param("~calibration_mode", 3)
        rospy.set_param("~reference_sensor", 'velodyne')
        rospy.set_param("~correspondences", 'known')
        rospy.set_param("~export_detections_to_files", True)
        rospy.set_param("~visualise", True)
        rospy.set_param("~reordering", 'based_on_reference_sensor') #Either 'based_on_reference_sensor' or 'based_on_definition'
        rospy.set_param("~outlier_removal", 'remove_board_locations')

        # spin() keeps Python from exiting until node is shutdown
        rospy.spin()

    def optimize(self, req):
        # Notifiy that service call is received:
        print('Received service call!')

        # Parameters for Joint Optimization:
        calibration_mode = rospy.get_param("~calibration_mode")  
        # Possible calibration modes:
            # 0: Pose and Structure Estimation (PSE) with unknown observation covariance matrices
            # 1: Pose and Structure Estimation (PSE) with known observation covariance matrices
            # 2: Minimally Connected Pose Estimation (MCPE)
            # 3: Fully Connected Pose Estimation (FCPE)
        correspondences = rospy.get_param("~correspondences")    # For lidar and stereo, 4 circles are detected. If 'known' the correpondences are known between lidar and stereo, else not.
        reference_sensor = rospy.get_param("~reference_sensor")    # reference sensor
        export_detections_to_files = rospy.get_param("~export_detections_to_files")
        visualise = rospy.get_param("~visualise")  # visualise result using matplotlib 3D visualisation
        save_folder_yaml = os.path.join(pkg_dir, 'results')
        reordering_method = rospy.get_param("~reordering")
        outlier_removal_method = rospy.get_param("~outlier_removal")

        # Convert ROS service call to sensors struct
        sensors, nr_calib_boards = self.convert_service_to_sensors_struct(req, correspondences, reference_sensor, reordering_method, outlier_removal_method)

        if export_detections_to_files:
            # Define folder to save CSV and YAML files with detections
            save_folder = os.path.join(pkg_dir, 'data')
            # Save as CSV
            export_sensor_data_to_csv(sensors, save_folder)
            # Save as YAML
            export_sensor_data_to_yaml(sensors, save_folder)

        # Joint Optimization
        Tms = joint_optimization(sensors, calibration_mode, correspondences, reference_sensor, visualise, save_folder_yaml)

        # Send back response
        return SendPatternsResponse()

    def convert_service_to_sensors_struct(self, req, correspondences, reference_sensor, reordering_method, outlier_removal_method):
        # For debugging: print service call message
        if False:
            print_service_call_message(req)

        # Retrieve numpy arrays
        sensors = []
        for i_sensor in range(len(req.accumulated_patterns)):
            # Get sensor type from ros service all
            sensor_topic_type = req.accumulated_patterns[i_sensor].sensor.data # This parameter is used to determine if the sensor is a lidar, camera or radar
            print('Sensor topic name: ' + sensor_topic_type)
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
            print('----------------------------------------------------------')
            print(msg_value_error)
            print('----------------------------------------------------------')
            # ValueError 1: reference sensor is not defined
            # ValueError 2: reference sensor contains non visible detection therefore reordering cannot be done for those calibration board locations

            # Pick sensor as reference sensor for reindexing:
            for i in range(len(sensors)):
                if sensors[i].type is not 'radar':
                    index_reference_sensor = i
                    break
            # Remove all non visible detections in reference sensors such that we can reorder based on that
            sensors = remove_non_visible_detections_in_reference_sensor(sensors, sensors[index_reference_sensor].name)
            # Retry reordering based on reindex using reference sensor
            sensors = reorder_detections_sensors(sensors, 'based_on_reference_sensor', sensors[index_reference_sensor].name)
        
        return sensors, nr_calib_boards
    
if __name__ == "__main__":
    # Start optimizer ROS node:
    optimizer_node()
