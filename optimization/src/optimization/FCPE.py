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

import itertools
from scipy import optimize
from scipy.optimize import linear_sum_assignment
from .helper_functions import *
from .calibration_board import *

def check_valid_inputs(sensors):
    # Count number of radar sensors:
    nr_radar_sensors = 0
    for i in range(len(sensors)):
        if sensors[i].type == 'radar':
            nr_radar_sensors = nr_radar_sensors + 1
    
    if nr_radar_sensors > 1:
        raise NotImplementedError("Currently this configuration (FCPE) only supports a setup of one radar, because radar to radar errors are not yet implemented.")

    for i in range(len(sensors)):
        if any(~np.isnan(sensors[i].constraints.lb)) or any(~np.isnan(sensors[i].constraints.ub)):
            raise NotImplementedError('Experimental feature: Box constraints are not yet implemented for FCPE.')

class FCPE:
    """ Fully Connected Pose Estimation (FCPE)

    Assumptions:
        - This class assumes that for lidar and camera the same keypoints are used (aka circle centers)
        - Currently this configurtion only supports a setup of a single radar, since radar to radar errors are not included.

    Attributes:
        sensors: struct containing all the sensor data
        reference_sensor: name of sensor that is used as reference_sensor
        X: vector containing 6D vectors that represent transformation matrices between sensors (inverse of relative sensor poses)
        edges: list containing indices of all combinations of sensor pairs

    Reference:
        Joris Domhof, Julian F. P. Kooij and Dariu M. Gavrila, A Multi-Sensor Extrinsic Calibration Tool for Lidar, Camera and Radar, submitted to ICRA 2019 conference.
    """

    def __init__(self, sensors, reference_sensor='lidar1', initialise_poses=False, correspondences='known'):
        """ Inputs to class are defined in here

        Args:
            sensors: struct containing all the sensor data
            reference_sensor: name of sensor that is used as reference_sensor
            initilalise_pose: get initial estimate of poses using Kabsch and ICP
            correspondences: is 'known' if correspondences of sensor observations are known (are in same order for every calibration board detection), otherwise 'unknown'
        """
        check_valid_inputs(sensors)

        self.sensors = sensors
        self.reference_sensor = reference_sensor # TODO can be removed
        self.nr_elements_pose = int(6)
        self.radar_projection_method = 'eucledian'  # TODO: can be removed
        self.mono_projection_method = 'eucledian'
        self.assignment = correspondences
        self.parameters_optimizer = optimizer_parameters
        self.edges = list(itertools.combinations([x for x in range(len(sensors))], 2))
        self.X = np.zeros(self.nr_elements_pose * len(self.edges))
        if initialise_poses:  # TODO: rename to initialise inverse of poses since we are using transformation matrices
            self.initiate_tms()

    def getX(self):
        return self.X

    def initiate_tms(self):
        X = np.zeros(self.nr_elements_pose * len(self.edges))
        for i in range(int(len(self.edges))):
            this_pair = self.edges[i]
            if self.sensors[this_pair[0]].type != 'radar' and self.sensors[this_pair[1]].type != 'radar':
                # Find union of mus
                mu = self.get_mu_union_pcl_and_pcl(self.sensors[this_pair[0]].mu, self.sensors[this_pair[1]].mu)
                # Get common detections
                Y1 = self.sensors[this_pair[0]].sensor_data[:, mu[self.sensors[this_pair[0]].mu]]
                Y2 = self.sensors[this_pair[1]].sensor_data[:, mu[self.sensors[this_pair[1]].mu]]
                # Estimate transformation matrix
                T = compute_transformation_matrix(Y1.T, Y2.T, None, self.assignment)
            else:
                # Find index of radar and index of non radar
                radar_index = this_pair[0] if self.sensors[this_pair[0]].type == 'radar' else this_pair[1]
                not_radar_index = this_pair[1] if self.sensors[this_pair[0]].type == 'radar' else this_pair[0]

                # Find union of mus
                mu_radar, mu_pcl = self.get_mu_union_radar_and_pcl(self.sensors[radar_index].mu, self.sensors[not_radar_index].mu, get_nr_detection(self.sensors[not_radar_index].type))

                # Get common detections
                Y_radar = self.sensors[radar_index].sensor_data[:, mu_radar[self.sensors[radar_index].mu]]
                Y_pcl = self.sensors[not_radar_index].sensor_data[:, mu_pcl[self.sensors[not_radar_index].mu]]

                # Estimate transformation matrix
                pcl_radar = np.vstack([Y_radar, np.zeros([1, Y_radar.shape[1]])])
                T = compute_transformation_matrix(target2radar3D(Y_pcl).T, pcl_radar.T, None, self.assignment, 1)

                # We map to radar, edges should be defined correctly
                self.edges[i] = (not_radar_index, radar_index)

            angles = rotm2vector(T[: 3, : 3])
            translation = T[: 3, 3]
            nr_rot_vars = int(self.nr_elements_pose / 2)
            X[i * self.nr_elements_pose: i * self.nr_elements_pose + nr_rot_vars] = angles
            X[i * self.nr_elements_pose + nr_rot_vars: i * self.nr_elements_pose + self.nr_elements_pose] = translation

        self.X = X

    def get_mu_union_pcl_and_pcl(self, mu1, mu2):
        # Assumption: both lidar and camera have same number of detections
        mu = np.logical_and(mu1, mu2)
        return mu

    def get_mu_union_radar_and_pcl(self, mu_radar, mu_pcl, nr_detections_lidar_camera):
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

    def function_combinatorial(self, X):
        Tms = self.convertXtoTms(X)

        e = np.zeros(len(self.edges))
        for i in range(len(self.edges)):
            this_pair = self.edges[i]
            if self.sensors[this_pair[0]].type != 'radar' and self.sensors[this_pair[1]].type != 'radar':
                # Find union of mus
                mu = self.get_mu_union_pcl_and_pcl(self.sensors[this_pair[0]].mu, self.sensors[this_pair[1]].mu)
                # Get common detections
                Y1 = self.sensors[this_pair[0]].sensor_data[:, mu[self.sensors[this_pair[0]].mu]]
                Y2 = self.sensors[this_pair[1]].sensor_data[:, mu[self.sensors[this_pair[1]].mu]]
                # Compute errors
                e[i] = self.compute_error_pcl2pcl(Y1, Y2, Tms[i])
            else:
                # Find index of radar and index of non radar
                radar_index = this_pair[0] if self.sensors[this_pair[0]].type == 'radar' else this_pair[1]
                not_radar_index = this_pair[1] if self.sensors[this_pair[0]].type == 'radar' else this_pair[0]
                # Find union of mus
                mu_radar, mu_pcl = self.get_mu_union_radar_and_pcl(self.sensors[radar_index].mu, self.sensors[not_radar_index].mu, get_nr_detection(self.sensors[not_radar_index].type))
                # Get common detections
                Y_radar = self.sensors[radar_index].sensor_data[:, mu_radar[self.sensors[radar_index].mu]]
                Y_pcl = self.sensors[not_radar_index].sensor_data[:, mu_pcl[self.sensors[not_radar_index].mu]]
                # Compute errors
                e[i] = self.compute_error_pcl2radar(Y_pcl, Y_radar, Tms[i])
                # We map to radar, edges should be defined correctly
                self.edges[i] = (not_radar_index, radar_index)

        # Only sum valid edges
        # --> invalid are the ones without enough common detections
        return np.nansum(e)

    def tm_inverse(self, Tms):
        Tinv = np.identity(4)
        Tinv[: 3, : 3] = Tms[: 3, : 3].T
        Tinv[: 3, 3] = -np.dot(Tms[: 3, : 3].T, Tms[: 3, 3])
        return Tinv

    def compute_loop_matrix(self, Tms, loop):
        # Define loop_matrix
        loop_matrix = np.identity(4)

        # Current loop consists of the following nodes
        circle = list(loop)
        circle.append(circle[0])
        for k in range(len(circle) - 1):
            from_index = circle[k]
            to_index = circle[k + 1]
            # find index of edge
            try:
                index = self.edges.index((from_index, to_index))
                T = Tms[index]
            except:
                index = self.edges.index((to_index, from_index))
                T = self.tm_inverse(Tms[index])

            loop_matrix = T.dot(loop_matrix)

        return loop_matrix

    def get_loops(self):
        # Find all loops for N sensors (exhaustive search of all combinations of 3 sensors)
        number_sensors_in_loop = 3
        loops = list(itertools.combinations([x for x in range(len(self.sensors))], number_sensors_in_loop))

        return loops

    def equality_identity_constraint_translation(self, x):
        Tms = self.convertXtoTms(x)
        constraint_vector = np.empty(0)

        # Find all loops for N sensors:
        loops = self.get_loops()

        for j in range(len(loops)):
            loop_matrix = self.compute_loop_matrix(Tms, loops[j])
            constraint = np.reshape(loop_matrix[:3, 3], -1)  # Translation part should be zero
            constraint_vector = np.concatenate([constraint_vector, constraint])

        return constraint_vector

    def equality_identity_constraint_rotation(self, x):
        Tms = self.convertXtoTms(x)
        constraint_vector = np.empty(0)

        # Find all loop for N sensors:
        loops = self.get_loops()

        # Add constraint into constraint_vector
        for j in range(len(loops)):
            loop_matrix = self.compute_loop_matrix(Tms, loops[j])
            constraint = np.diag(np.identity(4) - loop_matrix)[:3]  # R part should be identity
            constraint_vector = np.concatenate([constraint_vector, constraint])

        return constraint_vector

    def equality_identity_constraint_matrix(self, x):
        Tms = self.convertXtoTms(x)
        constraint_vector = np.empty(0)

        # Find all loop for N sensors:
        loops = self.get_loops()

        for j in range(len(loops)):
            loop_matrix = self.compute_loop_matrix(Tms, loops[j])
            constraint = np.reshape(np.identity(4)[:3, :] - loop_matrix[:3, :], -1)  # loop_matrix should be identity
            constraint_vector = np.concatenate([constraint_vector, constraint])

        return constraint_vector

    def convertXtoTms(self, X):
        Tms = []
        for i in range(len(self.edges)):
            start_index = i * self.nr_elements_pose
            end_index = start_index + self.nr_elements_pose
            Tm = np.identity(4)
            Tm[: 3, : 3] = vector2rotm(X[start_index: start_index + int(self.nr_elements_pose / 2)])
            Tm[: 3, 3] = X[end_index - int(self.nr_elements_pose / 2): end_index]
            Tms.append(Tm)

        return Tms

    def optimize(self):
        fov_constraint = {'type': 'ineq', 'fun': lambda x: self.radar_fov_constraint(x)}
        eq_identity_constraint_r = {'type': 'eq', 'fun': lambda x: self.equality_identity_constraint_rotation(x)}
        eq_identity_constraint_t = {'type': 'eq', 'fun': lambda x: self.equality_identity_constraint_translation(x)}

        # Normal constrained optimization
        result = optimize.minimize(self.function_combinatorial, self.X, method='SLSQP', jac=None, constraints=[eq_identity_constraint_r, eq_identity_constraint_t, fov_constraint], options={'maxiter': self.parameters_optimizer.maximum_iterations, 'disp': self.parameters_optimizer.verbosity}, tol=self.parameters_optimizer.stopping_tolerance)
        self.X = result.x

        # Warning if optimizer did not succeed
        print_optimizer_slsqp_feedback(result)

    def transform_with_T(self, T, xmap):
        return np.dot(T[:3, :], np.vstack([xmap, np.ones([1, xmap.shape[1]])]))  # xmap should contains ones

    def compute_error_pcl2radar(self, xmap, radar, T):
        Y0 = transform_with_T(T, target2radar3D(xmap))
        X = p(Y0)

        return np.sum(np.sum((X[: 2, :] - radar)**2, axis=0))

    def a2b_assignment(self, a, b, W):
        # Returns sum squared error
        aSumSquare = np.sum(a**2, axis=0)
        bSumSquare = np.sum(b**2, axis=0)
        mul = np.dot(a.T, b)
        sq_errors = aSumSquare[:, np.newaxis] + bSumSquare - 2 * mul
        row_ind, col_ind = linear_sum_assignment(sq_errors)

        return np.sum(np.dot(W, (a[:, row_ind] - b[:, col_ind])**2))  # sq_errors[row_ind, col_ind].sum()

    def compute_error_pcl2pcl(self, X, Y, T):
        Yhat = np.dot(T, np.vstack([X, np.ones((1, X.shape[1]))]))[:3, :]
        # return np.sum(np.sum((Yhat - Y)**2, axis=0))
        if self.assignment == 'known':
            return np.sum(np.sum((Yhat - Y)**2, axis=0))
        else:
            return self.a2b_assignment(Yhat, Y, np.identity(3))

    def radar_fov_constraint(self, X):
        Tms = self.convertXtoTms(X)

        all_constraint = np.array([])
        for i in range(len(self.edges)):
            this_pair = self.edges[i]
            if self.sensors[this_pair[0]].type != 'radar' and self.sensors[this_pair[1]].type != 'radar':
                pass
            else:
                # Find index of radar and index of non radar
                radar_index = this_pair[0] if self.sensors[this_pair[0]].type == 'radar' else this_pair[1]
                not_radar_index = this_pair[1] if self.sensors[this_pair[0]].type == 'radar' else this_pair[0]
                # Find common mus
                mu_radar, mu_pcl = self.get_mu_union_radar_and_pcl(self.sensors[radar_index].mu, self.sensors[not_radar_index].mu, get_nr_detection(self.sensors[not_radar_index].type))
                # Map PCL to radar and determin range, elevation angle and azimuth angle
                Y_pcl = self.sensors[not_radar_index].sensor_data[:, mu_pcl[self.sensors[not_radar_index].mu]]
                Xr = np.dot(Tms[i], np.vstack([target2radar3D(Y_pcl), np.ones((1, target2radar3D(Y_pcl).shape[1]))]))
                r, a, e = eucledian2polar_multiple(Xr[:3,:])

                # Get upper bound and lower bound
                lb = -self.sensors[radar_index].fov.max_elevation
                ub = self.sensors[radar_index].fov.max_elevation
                c_lb = e - lb
                c_ub = ub - e
                # colllect all constraints
                all_constraint = np.append(all_constraint, c_lb)
                all_constraint = np.append(all_constraint, c_ub)

        return all_constraint[~np.isnan(all_constraint)]
