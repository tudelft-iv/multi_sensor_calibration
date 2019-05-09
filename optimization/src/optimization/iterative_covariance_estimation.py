# multi_sensor_calibration
# Copyright (C) 2019  Intelligent Vehicles Delft University of Technology

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

import copy
from .PSE import *


def iterative_covariance_estimation(sensors0, reference_sensor, use_initial, assignment, xmap=None, max_ratio=0.01, max_iterations=20):
    sensors = copy.deepcopy(sensors0)

    # Save initial and current stopping tolerance
    initial_stopping_tolerance = optimizer_parameters.stopping_tolerance
    stopping_tolerance_covariance_estimation = 1E-4

    # In case of radar we use inital xmap
    if xmap is None:
        # Non radar mode
        calibration = PSE(sensors, reference_sensor, use_initial, assignment)
    else:
        # radar mode
        calibration = PSE(sensors, reference_sensor, use_initial, assignment, xmap)

    iter = 0
    debug = False
    while True:
        # Store the previous sensor results to check for convergence
        sensors_prev = copy.deepcopy(sensors)
        # Use this as initial vector X
        prev_X = calibration.getX()

        # Setup JointOptimizaion class
        if xmap is None:
            calibration = PSE(sensors, reference_sensor, False)
        else:
            xmap_prev = calibration.convertX2Xmap(prev_X)
            calibration = PSE(sensors, reference_sensor, False, assignment, xmap_prev)

        # Use previous X for faster convergence
        calibration.X = prev_X
        # Use smaller stopping tolerence for covariance estimation
        calibration.set_stopping_tolerance(stopping_tolerance_covariance_estimation)
        calibration.optimize('constrained')

        # Get Transformation matrices and structure
        Tms1 = calibration.convertXtoTms(calibration.getX())
        Xmap1 = calibration.convertX2Poses(calibration.getX())

        Y = []
        for i in range(len(sensors)):
            # Map points to each sensor
            Y.append({
                'lidar': calibration.project2lidar,
                'stereo': calibration.project2stereo,
                'mono': calibration.project2mono,
                'radar': calibration.project2radar
            }.get(calibration.sensors[i].type, 'lidar')(Tms1[i], Xmap1, calibration.sensors[i].parameters))

        # Initialise errors vector
        e = []
        # Compute errors with respect to previously estimated covariances
        for i in range(len(sensors)):
            delta = Y[i] - sensors[i].sensor_data
            var_sensors = np.var(delta, axis=1)
            sensors[i].W[np.diag_indices_from(sensors[i].W)] = 1 / var_sensors

            if debug:
                print(sensors[i].W[np.diag_indices_from(sensors[i].W)])

            for j in range(len(var_sensors)):
                # Compute difference with respect to previous estimate of the covariance
                ratio = var_sensors[j] * sensors_prev[i].W[np.diag_indices_from(sensors_prev[i].W)][j]
                # Compute difference wrt perfect case (cov_prev == cov_now --> ratio =1)
                e.append(np.abs(1 - ratio))

        # Check if it has converged
        iter = iter + 1
        if np.max(e) < max_ratio or iter == max_iterations:
            if debug:
                print('Maximum error, ', np.max(e), ' ', iter)
            break

    # Final calibration
    if xmap is None:
        final_calibration = PSE(sensors, reference_sensor, False)
    else:
        xmap_prev = calibration.convertX2Xmap(prev_X)
        final_calibration = PSE(sensors, reference_sensor, False, assignment, xmap_prev)

    # Calibrate with correct stopping tolerance
    final_calibration.X = prev_X
    final_calibration.set_stopping_tolerance(initial_stopping_tolerance)
    final_calibration.optimize('constrained')

    return final_calibration
