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
import itertools
import pickle
import argparse
import timeit
import copy
import scipy.io
from optimization.optimize import joint_optimization
from optimization.config import *
import matplotlib.pyplot as plt


def get_RMSE_calibration_errors(sensors, reference_sensor, Tms):
    # Compute all pairwise errors
    errors = []
    for l in range(len(Tms)):
        for m in range(l + 1, len(Tms)):
            this_Tms = np.linalg.inv(np.dot(Tms[l], np.linalg.inv(Tms[m])))
            if sensors[m].type == 'radar':
                # Tms is always from sensor to radar
                rmse = compute_rmse_pcl2radar(sensors[l].sensor_data, sensors[m].sensor_data, this_Tms)
            elif sensors[l].type == 'radar':
                # Tms is always from sensor to radar
                rmse = compute_rmse_pcl2radar(sensors[m].sensor_data, sensors[l].sensor_data, np.linalg.inv(this_Tms))
            else:
                # Transformation matrix might be defined from l to m or from m to l --> pick smallest
                # TODO: use as argument which of the two it is for lidar to stereo
                rmse1 = compute_rmse_pcl2pcl_unknown_assignment(sensors[l].sensor_data, sensors[m].sensor_data, this_Tms)
                rmse2 = compute_rmse_pcl2pcl_unknown_assignment(sensors[l].sensor_data, sensors[m].sensor_data, np.linalg.inv(this_Tms))
                rmse = min(rmse1, rmse2)

            errors.append(rmse)

    return errors


def plot_heat_map(x, y, data):
    ax = plt.gca()
    ax.imshow(data)

    # # We want to show all ticks...
    ax.set_xticks(np.arange(len(x)))
    ax.set_yticks(np.arange(len(y)))
    # ... and label them with the respective list entries
    ax.set_xticklabels(np.array(x))
    ax.set_yticklabels(np.array(y))

    # Loop over data dimensions and create text annotations.
    for i in range(len(y)):
        for j in range(len(x)):
            ax.text(j, i, '{:.4f}'.format(data[i, j]), ha="center", va="center", color="w")


def find_best_optimization_parameters(sensors, sensors_test, all_combinations, calibration_mode):
    all_errors = np.zeros((len(all_combinations), 3))
    elapsed = np.zeros((len(all_combinations), 1))

    # Joint optimization
    for i in range(len(all_combinations)):
        # Set parameters to current combination
        optimizer_parameters.stopping_tolerance = all_combinations[i][0]
        optimizer_parameters.maximum_iterations = all_combinations[i][1]

        # Perform calibration
        start_time = timeit.default_timer()
        Tms = joint_optimization(sensors, calibration_mode, correpondences, reference_sensor, False, None)
        elapsed[i] = timeit.default_timer() - start_time

        # Compute errors
        errors = get_RMSE_calibration_errors(sensors_test, 'lidar', Tms)

        # Save all errors
        all_errors[i, 0] = errors[0]
        all_errors[i, 1] = errors[1]
        all_errors[i, 2] = errors[2]

    return all_errors, elapsed


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


if __name__ == '__main__':
    # Python 3 should be used:
    assert sys.version_info.major == 3, 'Python 3 should be used'

    # Load pointclouds from lidar (Xl), camera (Xc), and radar Xr
    csv_file_lidar = ['data/new_detectors/lidar_wo_corners.csv']
    csv_file_camera = ['data/new_detectors/camera_wo_corners.csv']
    csv_file_radar = ['data/new_detectors/radar_wo_corners.csv']
    csv_file_rcs = None

    # Retrieve sensors setup:
    nr_boards = 28
    sensors_test, nr_calib_boards = get_sensor_setup(csv_file_lidar, csv_file_camera, csv_file_radar, csv_file_rcs)
    sensors, nr_calib_boards = get_sensor_setup(csv_file_lidar, csv_file_camera, csv_file_radar, csv_file_rcs)
    sensors = update_sensor_data_with_indices(sensors, np.arange(nr_boards))

    # Other parameters
    correpondences = 'known'
    reference_sensor = 'lidar1'

    all_combinations = []
    stopping_tolerance = [1E-4, 1E-5, 1E-6, 1E-7]
    max_iter = [10, 20, 50, 100, 1000, 2000]
    for xs in itertools.product(stopping_tolerance, max_iter):
        all_combinations.append(xs)

    labels = {}
    labels[0] = 'PSE'
    labels[2] = 'MCPE'
    labels[3] = 'FCPE'

    modes = [0, 2, 3]
    fig = plt.figure()

    i = 1
    for mode in modes:
        print(mode)

        # Find errors for all combinations
        all_errors, elapsed = find_best_optimization_parameters(sensors, sensors_test, all_combinations, mode)

        if True:
            # Plot in 1 figure
            ax = fig.add_subplot(2, 3, i)
            plot_heat_map(stopping_tolerance, max_iter, np.reshape(np.sum(all_errors, axis=1), (len(stopping_tolerance), len(max_iter))).T)
            ax.set_title([labels[mode], nr_boards, "Summed RSME error"])
            ax = fig.add_subplot(2, 3, i + 3)
            plot_heat_map(stopping_tolerance, max_iter, np.reshape(elapsed, (len(stopping_tolerance), len(max_iter))).T)
            ax.set_title([labels[mode], nr_boards, " Duration [seconds]"])

            i = i + 1
        else:
            # Plot seperate figures
            fig, ax = plt.subplots()
            plot_heat_map(stopping_tolerance, max_iter, np.reshape(np.sum(all_errors, axis=1), (len(stopping_tolerance), len(max_iter))).T)
            fig.tight_layout()
            ax.set_title([labels[mode], "Summed RSME error"])

            fig, ax = plt.subplots()
            plot_heat_map(stopping_tolerance, max_iter, np.reshape(elapsed, (len(stopping_tolerance), len(max_iter))).T)
            fig.tight_layout()
            ax.set_title([labels[mode], nr_boards, "Elapsed time"])

    # Show plots
    plt.show()
