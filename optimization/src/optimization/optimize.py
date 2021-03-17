#!/usr/bin/python3
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
import numpy as np
from .config import *
from .PSE import PSE
from .FCPE import FCPE
from .MCPE import MCPE
from .plotting import *
import matplotlib
import matplotlib.pyplot as plt
from .ransac import *
import itertools
import pickle
import argparse
import timeit
import copy
import scipy.io
from .iterative_covariance_estimation import iterative_covariance_estimation
import threading

# List with names for feedback to the user
all_optimization_modes = ['PSE (with unknown observation covariance matrices)', 
                          'PSE (with known observation covariance matrices)',
                          'MCPE',
                          'FPCE']

def get_mu_union_pcl_and_pcl(mu1, mu2):
    # Assumption: both lidar and camera have same number of detections
    mu = np.logical_and(mu1, mu2)
    return mu

def get_mu_union_radar_and_pcl(mu_radar, mu_pcl, nr_detections_lidar_camera):
    # Assumption: radar has one detection per calibration board.
    mu_radar_out = np.full((len(mu_radar)), False, dtype=bool)
    mu_pcl_out = np.full((len(mu_pcl)), False, dtype=bool)
    for indices_mu in range(len(mu_radar)):
        if np.all(mu_pcl[indices_mu * nr_detections_lidar_camera: indices_mu * nr_detections_lidar_camera + nr_detections_lidar_camera]) and mu_radar[indices_mu]:
            # Remove element for mus
            mu_pcl_out[indices_mu * nr_detections_lidar_camera: indices_mu * nr_detections_lidar_camera + nr_detections_lidar_camera] = True
            mu_radar_out[indices_mu] = True
        else:
            # Remove element for mus
            mu_pcl_out[indices_mu * nr_detections_lidar_camera: indices_mu * nr_detections_lidar_camera + nr_detections_lidar_camera] = False
            mu_radar_out[indices_mu] = False
    return mu_radar_out, mu_pcl_out

def print_RMSE_calibration_errors(sensors, reference_sensor, Tms):
    # initialise list for output
    errors = []

    # Compute all pairwise errors
    for l in range(len(Tms)):
        for m in range(l + 1, len(Tms)):
            this_Tms = np.linalg.inv(np.dot(Tms[l], np.linalg.inv(Tms[m])))
            sensor1 = sensors[m]
            sensor2 = sensors[l]
            Y1, Y2 = get_aligned_sensor_data(sensor1, sensor2, valid_measurements_only=True)
            if sensor1.type == 'radar' and sensor2.type == 'radar':
                # TODO: implement radar to radar errors
                print("Cannot show error from %s to %s, radar to radar error not yet implemented." % (sensor1.name, sensor2.name))
                continue
            elif sensor1.type == 'radar':
                # Tms is always from sensor to radar
                rmse, error_per_item = compute_rmse_pcl2radar2(Y2, Y1, this_Tms, return_per_item_error=True)
                sensor_from = sensor2
                sensor_to = sensor1
            elif sensor2.type == 'radar':
                # Tms is always from sensor to radar
                rmse, error_per_item = compute_rmse_pcl2radar2(Y1, Y2, np.linalg.inv(this_Tms), return_per_item_error=True)
                sensor_from = sensor1
                sensor_to = sensor2
            else:
                # Transformation matrix might be defined from l to m or from m to l --> pick smallest
                # TODO: use as argument which of the two it is for lidar to stereo
                # Use all common detections to compute RMSE
                rmse1 = compute_rmse_pcl2pcl(Y1, Y2, this_Tms)
                rmse2 = compute_rmse_pcl2pcl(Y1, Y2, np.linalg.inv(this_Tms))

                # Print smallest error to terminal
                if rmse1 < rmse2:
                    rmse = rmse1
                    sensor_from = sensor2
                    sensor_to = sensor1
                    error_per_item = np.sqrt(square_dist_pcl2pcl(Y1, Y2, this_Tms))
                else:
                    rmse = rmse2
                    sensor_from = sensor1
                    sensor_to = sensor2
                    error_per_item = np.sqrt(square_dist_pcl2pcl(Y1, Y2, np.linalg.inv(this_Tms)))

            print('RMSE ', sensor_from.name, 'to', sensor_to.name, '=', '{: .5f}'.format(rmse))
            print("    Error per item:", {index:round(error_per_item[index],3) for index in np.argsort(-error_per_item)[:20]})

            # Store RMSE in errors list:
            errors.append(rmse)
    
    return errors

def convert_Tms_FPCE(sensors, reference_sensor, Tms_cPE, edges):
    # Convert results in order to visualise results
    # Find index of base sensor
    index_reference_sensor = -1
    for i in range(len(sensors)):
        if sensors[i].name == reference_sensor:
            index_reference_sensor = i

    # Initialise Tms list
    Tms = [None] * len(sensors)
    # Assign identity to reference sensor
    Tms[index_reference_sensor] = np.identity(4)

    # Loop over all sensors
    for i_sensor in range(len(sensors)):
        # All except the reference sensors (is origin)
        if i_sensor != index_reference_sensor:
            # Find edge related to this sensor and connected to reference sensor
            for j in range(len(edges)):
                # Check in which direction it is defined
                if edges[j] == (i_sensor, index_reference_sensor):
                    Tms[edges[j][0]] = np.linalg.inv(Tms_cPE[j])
                elif edges[j] == (index_reference_sensor, i_sensor):
                    Tms[edges[j][1]] = Tms_cPE[j]

    return Tms

def remove_non_visible_detections_in_reference_sensor(sensors, reference_sensor):
    # Find index reference sensor:
    for i in range(len(sensors)):
        if sensors[i].name == reference_sensor:
            index_reference_sensor = i
            break

    # Find all indices of detections for reference sensor that are visible:  
    indices = []
    nr_detections = get_nr_detection(sensors[index_reference_sensor].type)
    nr_calib_boards = int(len(sensors[index_reference_sensor].mu) / nr_detections)
    for i in range(nr_calib_boards):
        if all(sensors[index_reference_sensor].mu[i*nr_detections:(i+1)*nr_detections]):
            indices.append(i)
        
    # Remove all detections that contain outlier:
    n = len(indices)
    for j in range(len(sensors)):
        # Get number of detections
        nr_detections_board = get_nr_detection(sensors[j].type)
        if j is not index_reference_sensor:    
            # Set all detections to invisible
            sensors[j].mu[:] = False
            # Set all visible detections to true
            for k in indices:
                sensors[j].mu[k * nr_detections_board:k * nr_detections_board + nr_detections_board] = True
                
            # Update sensor data and mus
            sensors[j].sensor_data = sensors[j].sensor_data[:, sensors[j].mu]
            sensors[j].mu = np.full(nr_detections_board * n, True, dtype=bool)
        else:
            # Current sensor is the reference sensors
            sensors[j].mu = np.full(nr_detections_board * n, True, dtype=bool)
    
    return sensors

def check_valid_inputs(sensors, mode, correspondences, reference_sensor):
    # Check if correspondences is valid
    if correspondences not in ['known', 'unknown']:
        raise ValueError('Variable correspondences should be "known" or "unknown"')

    # Check if calibration mode is valid
    if mode not in [0,1,2,3]:
        raise ValueError('Variable mode should be equal to [0,1,2,3]')
    
    # Check if reference sensor is valid
    valid_reference_sensor = False
    list_sensor_names = []
    for i in range(len(sensors)):
        list_sensor_names.append(sensors[i].name)
        if sensors[i].name == reference_sensor:
            valid_reference_sensor = True

    if not valid_reference_sensor:
        raise ValueError('Reference sensor should be one of the following sensors: ' +  str(list_sensor_names))
       
def joint_optimization(sensors, mode, correspondences, reference_sensor, visualise=False, folder=None, sensors_to_number=[], sensor_correspondence_to_plot=[], attempt_retry=True):
    """ Estimate sensors transformation matrices based on detection from sensors in struct

    Args:
        sensors (struct): struct as defined in get_sensor_setup()
        mode (int):
            0: Pose and Structure Estimation (PSE) with unknown observation covariance matrices
            1: Pose and Structure Estimation (PSE) with known observation covariance matrices
            2: Minimally Connected Pose Estimation (MCPE)
            3: Fully Connected Pose Estimation (FCPE)
        correspondences (string):  'known' if correspondes of sensor observations are known (are in same order for every calibration board detection)
        reference_sensor (string): name of sensor that is used as reference_sensor. This also defines in which reference frame the results are visualised.
        visualise (bool): if true then the results are visualised
        folder: output directory for results (YAML and launch files)

    Returns:
        list with transformation matrices corresponding to input sensors (output[0] corresponds to sensor[0], etc.)
            Equals identity for reference sensor. For non reference sensors, it returns transformation matrix with respect to reference sensor.
            For instance, reference sensor is with index 0, then the transformation matrix for sensor[1] is with respect to sensor[0]
    """

    # Calibration
    try:
        # Check for valid inputs:
        check_valid_inputs(sensors, mode, correspondences, reference_sensor)

        # Provide feedback to user:
        print("\n----------------------------------------------------------------")
        print("Optimizing your sensor setup using", all_optimization_modes[mode])
        print('----------------------------------------------------------------\n')

        if mode == 0:
            # The configuration used in this calibration mode is: Pose and Structure Estimation (PSE)
            # This mode iteratively joinlty estimates all sensor poses, calibration board poses and observation noise of the sensors.
            # 1) First  finds optimal sensor poses and calibration board poses with minimizing squared error (aka all covariance matrices equal I)
            # 2) Using the sensor poses and calibration poses, estimate the observation covariance matrices.
            # 3) Repeat 1 and 2 until convergence.
            calibration = iterative_covariance_estimation(sensors, reference_sensor, True, correspondences)
            Tms = calibration.convertXtoTms(calibration.getX())
        elif mode == 1:
            # The configuration used in this calibration mode is: Pose and Structure Estimation (PSE)
            # This mode assumes that the observations covariance matrices (W^-1) are known and defined in get_sensor_setup()
            calibration = PSE(sensors, reference_sensor, True, correspondences)
            calibration.optimize('constrained')
            Tms = calibration.convertXtoTms(calibration.getX())
        elif mode == 2:
            # Find index of reference sensor:
            index_reference_sensor = np.nan # we know the reference sensor is in sensors since it passed the test in check_valid_inputs()
            for i in range(len(sensors)):
                if sensors[i].name == reference_sensor:
                    index_reference_sensor = i

            # Check if radar is the reference sensor
            if sensors[index_reference_sensor].type == 'radar':
                # In that case we compute the transformation from non-reference sensor to reference sensor
                Tms = [] # Create a empty list with all transformation matrices
                # Loop over all sensors to calibrate them with respect to reference sensor
                for i in range(len(sensors)):
                    if sensors[i].name == reference_sensor:
                        # Set reference sensor at initial pose
                        Tms.append(sensors[i].T)
                    else:
                        # Compute pose from non-reference sensor to reference sensor:
                        # The configuration used in this calibration mode is: Minimally Connected Pose Estimation (MCPE)
                        calibration = MCPE([sensors[i],sensors[index_reference_sensor]], sensors[i].name, True, correspondences)
                        calibration.optimize()
                        T = calibration.convertXtoTms(calibration.getX()) # T[1] contains transformation matrix from non-reference sensor to reference sensor
                        Tms.append(np.linalg.inv(T[1])) # We store the transformation from reference to non reference
            else:
                # The configuration used in this calibration mode is: Minimally Connected Pose Estimation (MCPE)
                calibration = MCPE(sensors, reference_sensor, True, correspondences)
                calibration.optimize()
                Tms = calibration.convertXtoTms(calibration.getX())
        elif mode == 3:
            # The configuration used in this calibration mode is: Fully Connected Pose Estimation (MCPE)

            # Jointly estimate all sensor poses
            calibration = FCPE(sensors, reference_sensor, True, correspondences)
            calibration.optimize()
            Tms_cPE = calibration.convertXtoTms(calibration.getX())

            Tms = convert_Tms_FPCE(sensors, reference_sensor, Tms_cPE, calibration.edges)
        else:
            raise ValueError('Unknown calibration mode')

        # Print RMSE to terminal
        sensors_test = copy.deepcopy(sensors)
        print_RMSE_calibration_errors(sensors_test, reference_sensor, Tms)

        # Export Results to Yaml if folder is defined
        if folder is not None:
            write_tms_to_yaml(sensors, Tms, reference_sensor, folder)
            write_tms_to_launch_file(sensors, Tms, reference_sensor, folder)

        if visualise:
            plot_3D_calibration_result(sensors, Tms, sensors_to_number, sensor_correspondence_to_plot)

            if isinstance(threading.current_thread(), threading._MainThread):
                # Visualise all figures
                plt.show()
            else:
                print("Saving figure in %s" % folder)
                plt.savefig(folder + "/figure.png")
            
    except NotImplementedError as msg_not_implemented_error:
        print('----------------------------------------------------------')
        print(msg_not_implemented_error)
        print('Solution: optimize using calibration mode option MPCE')
        print('----------------------------------------------------------')

        # Run calibration with MCPE
        if attempt_retry:
            Tms = joint_optimization(sensors, 2, correspondences, reference_sensor, visualise, folder,
                                     sensors_to_number=sensors_to_number,
                                     sensor_correspondence_to_plot=sensor_correspondence_to_plot, attempt_retry=False)

    except ValueError as msg_value_error:
        print('----------------------------------------------------------')
        print(msg_value_error)
        print('Solution: optimize using calibration mode option MPCE with selecting first non radar as reference sensor.')
        print('----------------------------------------------------------')  

        # Solutions: 
        # - define reference sensors by selecting one from list (MCPE & PSE)
        # - remove for this reference sensor all mus = False (MCPE & PSE)

        # Select first sensor that is not radar as reference sensor
        for i in range(len(sensors)):
            if sensors[i].type != 'radar':
                index_reference_sensor = i

        # Remove non visible detection from reference sensor
        sensors = remove_non_visible_detections_in_reference_sensor(sensors, sensors[index_reference_sensor].name)
            
        # Run calibration with MCPE
        if attempt_retry:
            Tms = joint_optimization(sensors, 2, 'unknown', sensors[index_reference_sensor].name, visualise, folder,
                                     sensors_to_number=sensors_to_number,
                                     sensor_correspondence_to_plot=sensor_correspondence_to_plot, attempt_retry=False)

    return Tms


if __name__ == '__main__':
    # Python 3 should be used:
    assert sys.version_info.major == 3, 'Python 3 should be used'

    # Instantiate the parser
    parser = argparse.ArgumentParser(description='Experiments for extrinsic calibration')

    # Required positional argument
    parser.add_argument('--calibration-mode', required=True, type=int, help='0: Pose and Structure Estimation (PSE) with unknown observation covariance matrices, 1: Pose and Structure Estimation (PSE) with known observation covariance matrices, 2: Minimally Connected Pose Estimation (MCPE), 3: Fully Connected Pose Estimation (FCPE)')

    # path to csv files
    parser.add_argument('--lidar', type=str, required=True, help='Path to lidar csv file')
    parser.add_argument('--camera', type=str, required=True, help='Path to camera csv file')
    parser.add_argument('--radar', type=str, required=True, help='Path to radar csv file')
    parser.add_argument('--rcs', type=str, default=None, required=False, help='Path to rcs csv file')

    # test set csv files, only used with use_test set mode
    parser.add_argument('--use-test-set', action='store_true', help='If true the test set is loaded')
    parser.add_argument('--lidar-test', type=str, help='Path to lidar test csv file')
    parser.add_argument('--camera-test', type=str, help='Path to camera test csv file')
    parser.add_argument('--radar-test', type=str, help='Path to radar test csv file')
    parser.add_argument('--rcs-test', type=str, help='Path to rcs test csv file')

    # output directory to save yaml
    parser.add_argument('--output-directory', type=str, default="results", help='Path to save output yaml file')

    # optional settings for plotting
    parser.add_argument('--mayavi-plot', action='store_true', help='If true mayavi plotting will be used')
    args = parser.parse_args()

    # Retrieve sensors setup:
    sensors, nr_calib_boards = get_sensor_setup(args.lidar, args.camera, args.radar, args.rcs)

    # Settings:TODO: should be params
    mode = args.calibration_mode

    Tms = joint_optimization(sensors, mode, 'known', 'lidar1', True)
