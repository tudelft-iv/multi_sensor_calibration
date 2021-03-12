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
import itertools


class geometry_calibration_board:
    #circle_radius: not needed by optimizer
    width = 0.24  # width: distance between circle centers
    height = width  # Assumption is that centers location is a square --> mainly for is_valid_board() function
    offset_radar = 0.105  # Plate = 0.06 m + trihedral corner reflector depth = 0.0451 [m]


def get_board():
    # Get circle centers coordinates in local reference frame using width and height
    coordinates_board = 1 / 2 * np.array([[-geometry_calibration_board.width, geometry_calibration_board.height, 0], [geometry_calibration_board.width, geometry_calibration_board.height, 0], [-geometry_calibration_board.width, -geometry_calibration_board.height, 0], [geometry_calibration_board.width, -geometry_calibration_board.height, 0]]).T
    return coordinates_board


def board2map(T):
    # Return circle center locations given a pose T
    cb = get_board()
    return np.dot(T, np.vstack([cb, np.ones((1, cb.shape[1]))]))[:3, :]


def get_nr_detection(type):
    if type == 'radar':
        nr_detections = 1
    elif type == 'radar3D':
        nr_detections = 1
    else:
        nr_detections = 4

    return nr_detections


def target2lidar(X_target):
    return X_target


def target2stereo(X_target):
    return X_target


def target2monocular(X_target):
    return X_target


def target2radar3D(X_target, offset=geometry_calibration_board.offset_radar):
    nr_detections_board = 4

    if 1:
        Xm = np.reshape(np.mean(np.reshape(X_target, (-1, nr_detections_board)), axis=1), (3, -1))
        if True:
            # Radar target is behind plate
            Xm = Xm + correct_for_radar_offset(X_target, nr_detections_board, offset)
        else:
            # Do not correct for offsets
            pass
    else:
        Xm = np.zeros([3, int(X_target.shape[1] / nr_detections_board)])
        # for loop can be removed using reshape functions
        for i in range(int(X_target.shape[1] / nr_detections_board)):
            Xm[:, i] = np.mean(X_target[:, nr_detections_board * i:nr_detections_board * (i + 1)], axis=1)

    return Xm


def compute_normal_vector(Xc):
    if np.any(np.isnan(Xc)):
        return np.nan
    # TODO: add least squares refinement to also use the 4th point
    if 1:
        # Substract mean
        X = Xc - np.tile(np.mean(Xc,1), (Xc.shape[1],1)).T

        # Compute eigenvalues and eigenvectors
        _,eigenvalues,eigen_vectors = np.linalg.svd(X.T)
        
        # Get index smallest
        index = np.argmin(eigenvalues)

        # Get eigenvector corresponding to smallest eigenvalue
        normal_vector = eigen_vectors[index, :]
    else:
        # Select 3 points
        p = Xc[:, 0]
        q = Xc[:, 1]
        r = Xc[:, 2]
        # Compute normal vector and normalise
        normal_vector = np.cross(q - p, r - p)
        normal_vector = normal_vector / np.linalg.norm(normal_vector)
        # Definition: normal vector should be pointing away from sensors
        # So if not: mirror it
    
    if np.dot(np.mean(Xc, axis=1), normal_vector) < 0:
        normal_vector = -normal_vector

    return normal_vector


def correct_for_radar_offset(X, nr_detections_board, offset):
    nr_boards = int(X.shape[1] / nr_detections_board)
    X_offset = np.zeros((3, nr_boards))
    for i in range(nr_boards):
        normal_vector = compute_normal_vector(X[:, nr_detections_board * i:nr_detections_board * (i + 1)])
        offset_vector = offset * normal_vector
        X_offset[:, i] = offset_vector
    return X_offset


def compute_eucledian_distance(a, b):
    return np.linalg.norm((a - b))


def is_approximately_equal(a, b, epsilon):
    return compute_eucledian_distance(a, b) < epsilon


def is_valid_board(Xc, identify_individual_outliers):
    # Check to find if there are less than 4 points
    if Xc.shape[1] != 4:
        return np.full((4), False, dtype=bool)
    # If contains nans any nans, then is false
    if np.any(np.isnan(Xc)):
        return np.full((4), False, dtype=bool)

    # Check if there is an outlier
    all_combinations = list(itertools.combinations([x for x in range(4)], 2))
    distances = np.zeros(len(all_combinations), dtype='float')
    for i in range(len(all_combinations)):
        distances[i] = compute_eucledian_distance(Xc[:, all_combinations[i][0]], Xc[:, all_combinations[i][1]])

    # ratio between side of square and diagonal of square should equal sqrt(2), and no distance should be 0
    is_square = np.all(distances>0) and is_approximately_equal(np.max(distances) / np.min(distances), np.sqrt(2), 0.3)
    is_not_outlier = np.full((4), True, dtype=bool) if is_square else np.full((4), False, dtype=bool)

    # If there is an outlier try to indentity it
    if ~is_square and identify_individual_outliers:
        # Find centroid of this calibration board:
        centroid = find_centroid(Xc, 'triangle')

        # Find if points is close to expected location
        for i in range(4):
            d = compute_eucledian_distance(Xc[:, i], centroid)
            is_not_outlier[i] = is_approximately_equal(d, geometry_calibration_board.width / np.sqrt(2), 0.1)

    return is_not_outlier


def find_centroid(Xc, mode):
    # Find centroid of the calibration board in robust way
    # Assumption: only one outlier, there is a still a valid 'good' diagonal
    if mode == 'mean':
        # Mean is affected by an outlier
        centroid = np.mean(Xc, axis=1)
    elif mode == 'median':
        # Median is less affected by an outlier
        centroid = np.median(Xc, axis=1)
    elif 'triangle':
        # Find best triangle gives the indices of inliers
        # Best triangle is defined as one with closest to 90 degree angle
        all_combinations = list(itertools.combinations([x for x in range(3)], 2))
        print(all_combinations)
        best_solution = 1E5
        sol = [0, 0, 0]
        # loop over all points to find outlier
        for i in range(4):
            lo = np.setdiff1d([i for i in range(4)], i)
            for j in range(len(all_combinations)):
                this_c = all_combinations[j]
                p = Xc[:, i]
                q = Xc[:, lo[this_c[0]]]
                r = Xc[:, lo[this_c[1]]]
                current_dot_product = np.abs(np.dot((q - p), (r - p)))

                if current_dot_product < best_solution:
                    best_solution = current_dot_product
                    sol = [i, lo[this_c[0]], lo[this_c[1]]]

        # Centroid is at middle of long side of triangle
        centroid = 0.5 * (Xc[:, sol[1]] + Xc[:, sol[2]])

    return centroid


def remove_outlier_detections(sensors, nr_calib_boards, mode):
    # outlier removal mode:
    # -  'remove_board_locations' if outlier is detected for a single sensor all detection of this calibration board location are discarded
    # -  'remove_detections' if outlier is detected for a single sensor only this detection for this sensor is discarded

    # Loop over all calibration board locations
    for i in range(nr_calib_boards):
        # Find all valid calibration board detections
        valid_sensor_detections = np.full(len(sensors), False, dtype=bool)
        for iS in range(len(sensors)):
            nr_detections_board = get_nr_detection(sensors[iS].type)
            if nr_detections_board > 1:
                valid_sensor_detections[iS] = np.all(is_valid_board(sensors[iS].sensor_data[:, nr_detections_board * i:nr_detections_board * (i + 1)], False))
            else:
                valid_sensor_detections[iS] = sensors[iS].mu[i]

        # Keep all calibration board locations with valid detections
        if mode == 'remove_board_locations':
            # Remove this calibration board location
            if np.all(valid_sensor_detections):
                is_valid = np.full(len(sensors), True, dtype=bool)
            else:
                is_valid = np.full(len(sensors), False, dtype=bool)
        elif mode == 'remove_detections':
            # Remove only invalid detections
            is_valid = valid_sensor_detections
        else:
            raise Exception('Unknown mode for outlier removal')

        # Mark all calibration board locations with invalid detections (aka outliers)
        for iS in range(len(sensors)):
            nr_detections_board = get_nr_detection(sensors[iS].type)
            if nr_detections_board > 1:
                sensors[iS].mu[nr_detections_board * i:nr_detections_board * (i + 1)] = is_valid[iS]
            else:
                sensors[iS].mu[i] = is_valid[iS]

    # Remove the unvalid calibration board locations
    for iS in range(len(sensors)):
        if mode == 'remove_board_locations':
            # Remove this calibration board location
            sensors[iS].sensor_data = sensors[iS].sensor_data[:, sensors[iS].mu]
            sensors[iS].mu = np.full((sensors[iS].sensor_data.shape[1]), True, dtype=bool)
        elif mode == 'remove_detections':
            # Set invalid detections to nan
            sensors[iS].sensor_data[:, ~sensors[iS].mu] = np.nan
        else:
            raise Exception('Unknown mode for outlier removal')

    # Determine valid number of calibration boards:
    nr_calib_boards = int(sensors[0].sensor_data.shape[1] / get_nr_detection(sensors[0].type))

    return sensors, nr_calib_boards


def get_detection_centroid(X):
    nr_detections_board = 4
    X_centroid = np.reshape(np.mean(np.reshape(X, (-1, nr_detections_board)), axis=1), (3, -1))

    return X_centroid
