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

import numpy as np
from .helper_functions import *
from .config import *
from .PSE import *
import copy


def update_sensor_data_with_indices(sensors0, indices):
    sensors = copy.deepcopy(sensors0)
    n = len(indices)

    for j in range(len(sensors)):
        nr_detections_board = get_nr_detection(sensors[j].type)
        sensors[j].mu[:] = False
        for k in indices:
            sensors[j].mu[k * nr_detections_board:k * nr_detections_board + nr_detections_board] = True

        sensors[j].sensor_data = sensors[j].sensor_data[:, sensors[j].mu]
        sensors[j].mu = np.full(nr_detections_board * n, True)

    return sensors


def select_random_calibration_boards(sensors0, nr_calib_boards, n_ransac):
    this_round_indices = np.random.permutation(nr_calib_boards)[0:n_ransac]
    sensors = copy.deepcopy(sensors0)
    sensors = update_sensor_data_with_indices(sensors, this_round_indices)

    return sensors


def compute_ransac_inliers(errors, threshold, type):
    nr_detections_board = get_nr_detection(type)
    nr_targets = int(len(errors) / nr_detections_board)
    inliers = np.full(nr_targets, False)
    for i in range(nr_targets):
        inliers[i] = np.all(errors[i * nr_detections_board:i * nr_detections_board + nr_detections_board] < threshold[type])  # TODO: move the fours into sensor struct

    nr_inliers = inliers.sum()
    return inliers, nr_inliers


def ransac(sensors0, nr_calibration_boards, reference_sensor, init_poses=True):
    # NOTE: Assumes all calibration boards are visible for all sensors

    # Get RANSAC parameters
    ransac_parameters = get_ransac_parameters()

    # Set random seed for reproduceability
    np.random.seed(0)

    # Init ransac parameters
    best_nr_inliers = 0
    best_inliers = []
    nr_rounds = math.ceil(math.log(1 - ransac_parameters.probability_success) / math.log(1 - ransac_parameters.probability_inlier**ransac_parameters.nr_selected_points))
    for i in range(nr_rounds):
        # Sample calibration boards
        if True:
            # Check if the current set has enough calibration boards to compute poses
            while True:
                has_required_nr_obs = True
                sensors = select_random_calibration_boards(sensors0, nr_calibration_boards, ransac_parameters.nr_selected_points)
                for i in range(len(sensors)):
                    # Compute number of calibration boards in this set
                    nr_boards = np.sum(sensors[i].mu is True) / get_nr_detection(type)
                    # Check if it exceeds minimum values
                    if nr_boards >= ransac_parameters.min_nr_boards[sensors[i].type]:
                        has_required_nr_obs = False

                if has_required_nr_obs:
                    break

        else:
            sensors = select_random_calibration_boards(sensors0, nr_calibration_boards, ransac_parameters.nr_selected_points)

        # Optimize
        proposal = PSE(sensors, reference_sensor, init_poses)
        proposal.optimize('constrained')
        X_ransac = proposal.getX()

        # Compute errors
        test_proposal = PSE(sensors0, reference_sensor, False)
        xmap = test_proposal.convertX2Xmap(test_proposal.getX())
        Tms = test_proposal.convertXtoTms(proposal.getX())
        errors = test_proposal.compute_calibration_errors(xmap, Tms)  # Squared error for every points, TODO: change it?

        # Count inliers
        sensor_inliers = []
        sum_inliers = 0
        for j in range(len(sensors0)):
            if sensors0[j].name is not reference_sensor:  # NOTE: why not base sensor? --> because we will map all datapoints from base sensor to the other sensors to define the error. We do not have GT.
                inliers, nr_inliers = compute_ransac_inliers(errors[j], ransac_parameters.threshold, sensors0[j].type)
                sensor_inliers.append(inliers)
                sum_inliers = sum_inliers + nr_inliers

        if sum_inliers / (len(sensors0) - 1) > best_nr_inliers:
            best_nr_inliers = sum_inliers / (len(sensors0) - 1)
            best_inliers = np.all(sensor_inliers, axis=0)

    # Print results of ransac
    print('Outcome of ransac')
    print('Number of inliers = ', best_inliers)

    # Get sensor data from all inliers
    indices_inliers = [i for i, x in enumerate(best_inliers) if x]
    sensors_filtered = update_sensor_data_with_indices(sensors0, indices_inliers)

    # Refine using all inliers
    calibration_ransac = PSE(sensors_filtered, reference_sensor, init_poses)
    calibration_ransac.optimize('constrained')
    X_ransac = calibration_ransac.getX()
    Tms = calibration_ransac.convertXtoTms(X_ransac)

    # compute errors outliers
    indices_outliers = [i for i, x in enumerate(best_inliers) if not x]
    sensors_outliers = update_sensor_data_with_indices(sensors0, indices_outliers)
    test_outliers = PSE(sensors_outliers, reference_sensor, False)
    outlier_errors = test_outliers.compute_calibration_errors(test_outliers.convertX2Xmap(test_outliers.getX()), test_outliers.convertXtoTms(X_ransac))

    return Tms, X_ransac, indices_inliers, outlier_errors, calibration_ransac
