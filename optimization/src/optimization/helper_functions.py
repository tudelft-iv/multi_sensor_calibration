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
import math
import rmsd
from icp import *
from .calibration_board import *
from scipy.optimize import linear_sum_assignment
import warnings

method_angles = 'rodrigues'
folder = ''


class optimizer_parameters:
    stopping_tolerance = 1E-6
    maximum_iterations = 1000
    verbosity = False


class ransac_parameters:
    nr_selected_points = None
    probability_success = None
    probability_inlier = None
    threshold = None
    min_nr_boards = None


class fov_radar:
    max_range = None  # NOT used right now
    max_elevation = None
    max_azimuth = None  # NOT used right now


class Constraint:
    # Define class constraint which can be used for constrained optimization
    #parent = ''  # parent reference frame
    #lb = None  # upper bound wrt to parent
    #ub = None  # lower bound wrt to parent

    def __init__(self, parent, lb, ub):
        self.parent = parent
        self.lb = lb
        self.ub = ub


class Sensor:
    # Define class sensor, which contains all sensor data
    # Initial values, sensor observations, visibility of target
    # name = ''   # name of this sensor
    # type = ''   # type of sensor: lidar, stereo, mono or radar
    # T = np.identity(4)     # transformation matrix
    # constraints = None  # constraints
    # sensor_data = None  # sensor data containing the observations
    # mu = None  # mu is an array that defines if calibration board locations is visible (1) or not (0)
    # fov = None  # Field of View of sensor
    # parameters = None
    # optional = None
    # W = None  # Inverse of measurement covariance matrix
    # link = None  # [Optional]Link name of sensor, as define in URDF (ROS)

    def __init__(self, name, type, constraints, sensor_data, mu, W, link, T=None, fov=None, parameters=None,
                 optional=None):
        self.name = name
        self.type = type
        self.T = T if T is not None else np.identity(4)
        self.constraints = constraints
        self.sensor_data = sensor_data
        self.fov = fov
        self.mu = mu
        self.parameters = parameters
        self.optional = optional
        self.W = W
        self.link = link

    @property
    def sensor_data_in_3d(self):
        #Fix self.mu in case it's broken
        self.mu = ~np.any(np.isnan(self.sensor_data), axis=0)
        if self.type == 'radar':
            return np.vstack([self.sensor_data, np.zeros([1, self.sensor_data.shape[1]])])
        else:
            return self.sensor_data


def get_aligned_sensor_data(sensor1, sensor2, valid_measurements_only=True):
    # If a sensor is of type radar, make it 3D
    Y2 = sensor2.sensor_data_in_3d
    Y1 = sensor1.sensor_data_in_3d
    mu1 = sensor1.mu
    mu2 = sensor2.mu

    # if mu1.shape is larger, sensor1 must be downsized to sensor2 size
    if mu1.shape[0] > mu2.shape[0]:
        Y1, mu1 = downsample(Y1, mu1, sensor1.type)
    # if mu2.shape is larger, sensor2 must be downsized to sensor1 size
    elif mu2.shape[0] > mu1.shape[0]:
        Y2, mu2 = downsample(Y2, mu2, sensor2.type)

    if valid_measurements_only:
        # determine which measurements are valid in both sensors
        mu = np.bitwise_and(mu1, mu2)
        return Y1[:, mu], Y2[:, mu]
    else:
        return Y1, Y2


def downsample(Y, mu, sensor_type):
    downsample_count = get_nr_detection(sensor_type)
    mu_res = mu.reshape(-1, downsample_count).all(axis=1)
    Y_res = np.mean(np.reshape(Y, (3, -1, downsample_count)), axis=2)
    Y_res = Y_res + correct_for_radar_offset(Y, downsample_count, geometry_calibration_board.offset_radar)
    return Y_res, mu_res


def eul2rotm(euler_angles):
    # Calculates Rotation Matrix (rotm) given euler angles (eul).
    # The result is the same as MATLAB function eul2rotm

    R_x = np.array([[1, 0, 0],
                    [0, math.cos(euler_angles[2]), -math.sin(euler_angles[2])],
                    [0, math.sin(euler_angles[2]), math.cos(euler_angles[2])]
                    ])

    R_y = np.array([[math.cos(euler_angles[1]), 0, math.sin(euler_angles[1])],
                    [0, 1, 0],
                    [-math.sin(euler_angles[1]), 0, math.cos(euler_angles[1])]
                    ])

    R_z = np.array([[math.cos(euler_angles[0]), -math.sin(euler_angles[0]), 0],
                    [math.sin(euler_angles[0]), math.cos(euler_angles[0]), 0],
                    [0, 0, 1]
                    ])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R


def isRotationMatrix(R):
    # Checks if a matrix is a valid rotation matrix.
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    Id = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(Id - shouldBeIdentity)
    return n < 1e-5


def rotm2eul(R):
    # Calculates rotation matrix (rotm) to euler angles (eul)
    # The result is the same as MATLAB function rotm2eul
    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([z, y, x])


def p(Xb):
    """ This function maps 3D coordinates to 2D eucledian coordinates
    by converting eucledian to spherical coordiantes and
    setting the elevation angle to zero
    """
    r, a, e = eucledian2polar_multiple(Xb)
    # Set e = 0
    x, y, z = polar2eucledian_multiple(r, a, np.zeros(len(e)))
    Xm = np.zeros(Xb.shape)
    Xm[0, :] = x
    Xm[1, :] = y
    Xm[2, :] = z
    return Xm


# TODO: rename function to eucledian2spherical_multiple()
def eucledian2polar_multiple(x):
    """ Transform eucledian coordinates to spherical coordinates

    elevation angle: angle with respect to xy plane - positive when z is positive
    azimuth angle: angle with respect to xz plane - positive when z is positive
    range: distance to x,y,z
    """
    if (x.ndim != 2):
        raise Exception("eucledian2polar_multiple: #dimensions should be equal to 2.")
    if (x.shape[0] != 3):
        raise Exception("eucledian2polar_multiple: dimension 0 should be equal to 3.")

    r = np.sqrt(np.sum(x[:3,:]**2, axis=0))
    a = np.arctan2(x[1, :], x[0, :])
    e = np.arcsin(x[2, :] / r)
    return r, a, e


# TODO: rename function to spherical2eucledian_multiple()
def polar2eucledian_multiple(r, a, e):
    """ Transform spherical coordinates to eucledian coordinates

    elevation angle: angle with respect to xy plane - positive when z is positive
    azimuth angle: angle with respect to xz plane - positive when z is positive
    range: distance to x,y,z
    """
    x = r * np.cos(a) * np.cos(e)
    y = r * np.sin(a) * np.cos(e)
    z = r * np.sin(e)
    return x, y, z


def get_transformation_matrix_kabsch(q, p):
    q = q.T
    p = p.T
    # compute centroids
    Pc = rmsd.centroid(p)
    Qc = rmsd.centroid(q)
    # Kabsch algorithm for estimating R and t
    T = np.identity(4)
    T[:3, :3] = rmsd.kabsch(p - Pc, q - Qc)
    T[:3, 3] = Pc - np.dot(T[:3, :3], Qc)

    return T


def get_transformation_matrix_ICP(q, p, T0=None):
    q = q.T
    p = p.T
    if T0 is None:
        T0 = np.identity(4)
        T0[:3, 3] = p.mean(axis=0) - q.mean(axis=0)

    T, distances, i = icp(q, p, T0, 1000, 1E-3)
    return T


def compute_N_best_matches(q, p, T0, percentage):
    # Computes RMSE between q and p
    # Example usage: print(compute_N_best_matches(sensors[0].sensor_data,sensors[1].sensor_data, Tms[1], 1))
    _, distances, i = icp(q.T, p.T, T0, 1, 1E-3)

    number_matches = round(percentage * len(distances))
    sorted_distances = np.sort(distances, axis=None)

    return np.sqrt(np.mean(sorted_distances[:number_matches + 1]**2))


def rmse(X, Y):
    # Check if shape of X and Y is equal
    assert(X.shape == Y.shape)
    # Check if X and Y are 2D
    assert(X.ndim > 1)

    return np.sqrt(np.nanmean(np.sum((X - Y)**2, axis=0)))


def compute_rmse(X, Y, T):
    Yhat = np.dot(T, np.vstack([X, np.ones((1, X.shape[1]))]))[:3, :]
    return np.sqrt(np.mean(np.sum((Yhat - Y)**2, axis=0)))


def compute_rmse_pcl2pcl(X, Y, T):
    return compute_rmse(X, Y, T)


def transform_with_T(T, xmap):
    return np.dot(T[:3, :], np.vstack([xmap, np.ones([1, xmap.shape[1]])]))  # xmap should contains ones


def square_dist_pcl2pcl(X, Y, T):
    return np.sum((transform_with_T(T, X) - Y) ** 2, axis=0)

def square_dist_pcl2radar(X, Y, T):
    return np.sum((p(transform_with_T(T, X)) - Y) ** 2, axis=0)

def square_dist_unknown_correspondences(X, Y, T):
    Yhat = transform_with_T(T, X)
    # Returns sum squared error
    aSumSquare = np.sum(Yhat ** 2, axis=0)
    bSumSquare = np.sum(Y ** 2, axis=0)
    mul = np.dot(Yhat.T, Y)
    sq_errors = aSumSquare[:, np.newaxis] + bSumSquare - 2 * mul
    sq_errors[np.isnan(sq_errors)] = 1e20 #set nan values to something excessively big, so they only get assigned to each other
    row_ind, col_ind = linear_sum_assignment(sq_errors)
    return np.sum((Yhat[:, row_ind] - Y[:, col_ind]) ** 2, axis=0)  # sq_errors[row_ind, col_ind].sum()

def compute_rmse_pcl2radar2(X, Y, T, return_per_item_error=False):
    Yhat0 = transform_with_T(T, X)
    Yhat = p(Yhat0)

    if return_per_item_error:
        return rmse(Yhat, Y), np.sqrt(np.sum((Yhat - Y)**2, axis=0))
    else:
        return rmse(Yhat, Y)


def compute_rmse_pcl2radar(X, Y, T): # TODO make this function obsolete, rename
    Yhat0 = transform_with_T(T, target2radar3D(X))
    Yhat = p(Yhat0)

    return rmse(Yhat[:2, :], Y)


def compute_rmse_pcl2pcl_unknown_assignment(X, Y, T):
    error = square_dist_unknown_correspondences(X, Y, T)
    return np.sqrt(np.nanmean(error))




def rotm2rodrigues(R):
    # https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation
    theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))  # arcos argument should be between -1 and 1

    if theta == 0:
        return np.zeros(3)
    else:
        temp = np.zeros(3)
        temp[0] = R[2, 1] - R[1, 2]
        temp[1] = R[0, 2] - R[2, 0]
        temp[2] = R[1, 0] - R[0, 1]
        uv = 1 / (2 * np.sin(theta)) * temp

        return theta * uv


def rodrigues2rotm(v):
    #https://en.wikipedia.org/wiki/Rotation_matrix
    
    theta = np.linalg.norm(v)
    if theta == 0:
        R = np.identity(3)
    else:
        v = v / theta
        ux = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.cos(theta) * np.identity(3) + np.sin(theta) * ux + (1 - np.cos(theta)) * np.outer(v, v)

    return R


def rotm2vector(R):
    # Generic function that converts rotation matrix to a vector
    # This fucntion is meant as a wrapper for 2 options: euler angles or rodrigues matrix formula
    if method_angles == 'rodrigues':
        return rotm2rodrigues(R)
    elif method_angles == 'euler':
        return rotm2eul(R)
    else:
        print('Error unknown method for rotm2vector conversion')


def vector2rotm(v):
    # Generic function that converts vector to rotation matrix
    # This fucntion is meant as a wrapper for 2 options: euler angles or rodrigues matrix formula
    if method_angles == 'rodrigues':
        return rodrigues2rotm(v)
    elif method_angles == 'euler':
        return eul2rotm(v)
    else:
        print('Error unknown method for vector2rotm conversion')


def compute_calibration_board_poses(xmap, nr_detections, point_correspondences_board):
    # Pose is defined by a 6D vector
    nr_elements_pose = 6
    # Are correspondences between get_board points and sensor data point known?
    # point_correspondences_board = 'known'  # Can be set to known in case of CB width == CB height (calibration_board.py). If not set to 'unknown'
    # Retrieve of calibration board locations
    nr_cb = int(xmap.shape[1] / nr_detections)
    # Get local coordinates of circle centers of calibration board
    cb = get_board()
    # Initialise vector for poses calibration boards
    xb = np.zeros(nr_elements_pose * nr_cb)
    # Loop over all CB
    for i in range(nr_cb):
        # Compute initial poses by using kabsch or ICP
        if point_correspondences_board == 'unknown':
            # The correspondeces are unknown
            # Assumption:
            # - the calibration target is a planar surface with at least 3 detections
            # - first 3 detection can be used to estimate the normal vector of the planar surface

            # Compute good initial T using alingnment of normal vectors
            # We are looking at planar surfaces so normal vectors are easy to compute
            normal1 = np.cross(cb[:, 1] - cb[:, 0], cb[:, 2] - cb[:, 0])
            normal2 = np.cross(xmap[:, 1] - xmap[:, 0], xmap[:, 2] - xmap[:, 0])

            # Find rotation axis and rotation angle for aligning the two normal vectors
            v = np.cross(normal1 / np.linalg.norm(normal1), normal2 / np.linalg.norm(normal2))
            angle = np.cos(np.dot(normal1 / np.linalg.norm(normal1), normal2 / np.linalg.norm(normal2)))

            # Convert rotation to transformation matrix
            T = np.identity(4)
            T[:3, :3] = rodrigues2rotm(angle * v)
            T[:3, 3] = np.mean(xmap[:, i * nr_detections:i * nr_detections + nr_detections] - np.dot(T, np.vstack([cb, np.ones((1, nr_detections))]))[:3, :], axis=1)

            # Refine previous estimate by using ICP to find a beter estimate of T calibration board
            T = get_transformation_matrix_ICP(cb, xmap[:, i * nr_detections:i * nr_detections + nr_detections], T)
        else:
            # The correspondeces are known
            T = get_transformation_matrix_kabsch(cb, xmap[:, i * nr_detections:i * nr_detections + nr_detections])

        xb[i * nr_elements_pose:i * nr_elements_pose + nr_elements_pose] = np.concatenate([rotm2vector(T[:3, :3]), T[:3, 3]])

    return xb


def compute_transformation_matrix(p, q, T0, correspondences, assignment_mode=0):
    if correspondences == 'unknown':
        # Unknown correpondences for detections of the calibration board.
        if assignment_mode == 0:
            # [Default] In this case the centroids of the calibraiton board are used to obtain initial estimate of T
            p0 = get_detection_centroid(p)
            q0 = get_detection_centroid(q)
            # Compute T1 using centroids:
            T1 = get_transformation_matrix_kabsch(p0, q0)
            # Compute final T using all points:
            T = get_transformation_matrix_ICP(p, q, T1)
        elif assignment_mode == 1:
            # Use Kabsch - efficient for radar mode because correspondence are known since there is only one detection for every calibration board location
            T = get_transformation_matrix_kabsch(p, q)
        elif assignment_mode == 2:
            # Use ICP and inital estimate of T. Note that ICP needs a good initial estimate!
            T = get_transformation_matrix_ICP(p, q, T0)
        else:
            raise Exception('Unknown mode in compute_transformation_matrix')
    else:
        # For every calibration board locations, the corresponces between for instance lidar to stereo detections is known.
        T = get_transformation_matrix_kabsch(p, q)

    return T

def posesTo3D(poses):
    # This function provides the position of the keypoints (circle centers) given the pose of the calibration board
    nr_elements_pose = 6
    nr_detections = board2map(np.identity(4)).shape[1]

    Xcb = poses
    # Determine number of calibration board locations
    nr_cb = int(len(Xcb) / nr_elements_pose)
    # Initialise calibration matrix that contains all circle centers
    cb = np.zeros((3, nr_cb * nr_detections))
    for i in range(nr_cb):
        Tm = np.identity(4)
        Tm[:3, :3] = vector2rotm(Xcb[i * nr_elements_pose:i * nr_elements_pose + int(nr_elements_pose / 2):1])
        Tm[:3, 3] = Xcb[i * nr_elements_pose + int(nr_elements_pose / 2):i * nr_elements_pose + nr_elements_pose:1]
        cb[:, i * nr_detections:i * nr_detections + nr_detections] = board2map(Tm)

    return cb


def poses2lidar(poses):
    X_target = target2lidar(posesTo3D(poses))
    return X_target


def poses2stereo(poses):
    X_target = target2stereo(posesTo3D(poses))
    return X_target


def poses2monocular(poses):
    X_target = target2monocular(posesTo3D(poses))
    return X_target


def poses2radar3D(poses, offset=geometry_calibration_board.offset_radar):
    X_target = target2radar3D(posesTo3D(poses), offset)
    return X_target

def reorder_pcl(X, nr_detections, type):
    Y = np.zeros(X.shape)
    for i in range(int(X.shape[1] / nr_detections)):
        Xcb = X[:, i * nr_detections:i * nr_detections + nr_detections]
        if type == 'lidar':
            Xcb = reorder_array(Xcb, -2, 0)          # Load lidar data
        elif type == 'stereo':
            Xcb = reorder_array(Xcb, 1, 0)     # Load camera data
        elif type == 'mono':
            Xcb = reorder_array(Xcb, 1, 0)     # Load camera data
        elif type == 'radar':
            pass
        else:
            raise Exception('Unknown sensor type')
            
        Y[:, i * nr_detections:i * nr_detections + nr_detections] = Xcb

    return Y

def get_centroids(data, n):
    # Returns mean of n consecutive points in data array
    return np.reshape(np.mean(np.reshape(data, (-1, n)), axis=1), (data.shape[0], -1))

def get_indices(a,b):
    # Returns sum squared error
    aSumSquare = np.sum(a**2, axis=0)
    bSumSquare = np.sum(b**2, axis=0)
    mul = np.dot(a.T, b)
    sq_errors = aSumSquare[:, np.newaxis] + bSumSquare - 2 * mul
    row_ind, col_ind = linear_sum_assignment(sq_errors)

    return row_ind, col_ind

def reorder_detections(sensor_other, sensor_reference):
    # Check if reference sensors contains mu = False
    if np.any(sensor_reference.mu==False):
        raise ValueError("The reference sensor data contains calibration board locations which are not visible (mu = False). This results in that reordering detection based on the reference sensor is not possible.")

    # Find common detections to compute transformation matrix
    data_other, data_reference = get_aligned_sensor_data(sensor_other, sensor_reference)

    # Compute transformation matrix using Kabsch
    T = get_transformation_matrix_kabsch(get_centroids(data_other,4), get_centroids(data_reference, 4))

    # Get optimal indices
    _, new_indices = get_indices(data_reference, transform_with_T(T, data_other))

    # Reorder data using indices
    new_data_sensor = data_other[:, new_indices]

    return new_data_sensor,new_indices

def reorder_detections_sensors(sensors, reordering_mode, reference_sensor):
    # reordering_mode: 'based_on_reference_sensor' or 'based_on_definition'
    #                   based_on_reference_sensor: reorder based on reference sensor detections
    #                   based_on_definition: based on labeling of all detections (top-left, top-right, bottom-left and bottom-right, circle centers)

    # Find index of reference sensors
    index_reference_sensor = np.nan
    for i in range(len(sensors)):
        if sensors[i].name == reference_sensor:
            index_reference_sensor = i    

    # Check if reference sensor exists in sensors
    if np.isnan(index_reference_sensor):
        raise ValueError('Reference sensor is not defined correctly')

    if reordering_mode == 'based_on_reference_sensor':
        # Get geometry of calibration board:
        cb = get_board() # Circle center locations
        # Get number of calibration boards
        nr_calibration_boards = int(len(sensors[index_reference_sensor].mu) / get_nr_detection(sensors[index_reference_sensor].type))
        # Get number of detection per board:
        nr_detections = get_nr_detection(sensors[index_reference_sensor].type)

        # Loop over all calibration boards
        j = 0
        for i in range(nr_calibration_boards):
            # This board mus
            this_board_mus = sensors[index_reference_sensor].mu[i * nr_detections:i * nr_detections + nr_detections]
            if np.all(this_board_mus == True):
                # Get this board detections
                this_board_detections = sensors[index_reference_sensor].sensor_data[:, j * nr_detections:j * nr_detections + nr_detections]
                # Get transformation matrix
                T = get_transformation_matrix_ICP(cb, this_board_detections, np.identity(4))
                # Get correct indices:
                _,new_indices = get_indices(cb,transform_with_T(T, this_board_detections))
                # Reorder all detection and mus:
                sensors[index_reference_sensor].sensor_data[:, j * nr_detections:j * nr_detections + nr_detections] = this_board_detections[:, new_indices]
                sensors[index_reference_sensor].mu[i * nr_detections:i * nr_detections + nr_detections] = this_board_mus[new_indices]
                # Next board
                j = j + 1

    # Loop over all sensors and reindex detections such that top-left, top-rigth, bottom-left, bottom-right detection of alll senesors (lidar & camera) match
    for i in range(len(sensors)):
        if get_nr_detection(sensors[i].type) == 1:
            # In this case sensor is radar and we only have a single detection so we do not have to reorder it.
            pass
        else:
            # In this case sensor is either lidar or camera
            if reordering_mode == 'based_on_definition':
                # Reindex based on definition of top/bottom and left/right for every sensors
                sensors[i].sensor_data = reorder_pcl(sensors[i].sensor_data, get_nr_detection(sensors[i].type), sensors[i].type)
            elif reordering_mode == 'based_on_reference_sensor':
                # Reindex based on reference sensor

                sensors[i].sensor_data[:, sensors[i].mu],_ = reorder_detections(sensors[i], sensors[index_reference_sensor])

            else:
                pass

    return sensors

def reorder_array(A, dim1, dim2):
    # TODO: clean up this function
    # dimension = 0 --> 'x' axis direction
    # dimension = 1 --> 'y' axis direction
    # dimension = 2 --> 'z' axis direction
    # dim1: dimension top down
    # dim2: dimension left right
    # dim1: postive means that ground floor is in positive axis direction.
    # dim1: negative means that ground floor is in negative axis direction. 

    if dim1 < 0:
        dim1 = abs(dim1)
        descend1 = True
    else:
        descend1 = False

    if dim2 < 0:
        dim2 = abs(dim2)
        descend2 = True
    else:
        descend2 = False

    order_dim1 = sorted(A[dim1, :], reverse=descend1)  # descend
    order_dim2 = sorted(A[dim2, :], reverse=descend2)  # ascent

    top = order_dim1[:2]
    left = order_dim2[:2]

    index = np.zeros(4, 'int')  # 4 is specific for our calibration board
    for i in range(A.shape[1]):
        is_top = np.in1d(np.array([A[dim1, i]]), top)
        is_left = np.in1d(np.array([A[dim2, i]]), left)
        if is_top:
            if is_left:
                index[i] = 0
            else:
                index[i] = 1
        else:
            if is_left:
                index[i] = 2
            else:
                index[i] = 3

    C = np.zeros((3, 4))
    for i in range(4):
        C[:, index[i]] = A[:, i]

    return C


def print_optimizer_slsqp_feedback(result):
    # Minimize a function using Sequential Least SQuares Programming (SLSQP)
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.fmin_slsqp.html#scipy.optimize.fmin_slsqp
    # -1 : Gradient evaluation required (g & a)
    # 0 : Optimization terminated successfully.
    # 1 : Function evaluation required (f & c)
    # 2 : More equality constraints than independent variables
    # 3 : More than 3*n iterations in LSQ subproblem
    # 4 : Inequality constraints incompatible
    # 5 : Singular matrix E in LSQ subproblem
    # 6 : Singular matrix C in LSQ subproblem
    # 7 : Rank-deficient equality constraint subproblem HFTI
    # 8 : Positive directional derivative for linesearch
    # 9 : Iteration limit exceeded

    # Set boolean to true
    succesful_optimization = False

    # Get print statement from scipy:
    if result.status == 0:
        # Only succesful if status is equal to 0:
        succesful_optimization = True
    elif result.status == 1:
        print('Function evaluation required (f & c)')
    elif result.status == 2:
        print('More equality constraints than independent variables')
    elif result.status == 3:
        print(' More than 3*n iterations in LSQ subproblem')
    elif result.status == 4:
        print('Inequality constraints incompatible')
    elif result.status == 5:
        print('Singular matrix E in LSQ subproblem')
    elif result.status == 6:
        print('Singular matrix C in LSQ subproblem.')
    elif result.status == 7:
        print('Rank-deficient equality constraint subproblem HFTI')
    elif result.status == 8:
        print('Positive directional derivative for linesearch')
    elif result.status == 9:
        print('Iteration limit exceeded')
    elif result.status == -1:
        print('Gradient evaluation required (g & a)')
    else:
        warnings.warn("Unknown warning message!")
    
    # Print succesfull/unsuccesful:
    if succesful_optimization:
        print("-------------------------------------")
        print("Optimization terminated successfully!")
        print("-------------------------------------")
    else:
        warnings.warn("Optimization failed!")
