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
        self.edges = self._get_edges(sensors)
        self.X = np.zeros(self.nr_elements_pose * len(self.edges))
        if initialise_poses:  # TODO: rename to initialise inverse of poses since we are using transformation matrices
            self.initiate_tms()

    @staticmethod
    def _get_edges(sensors):
        # if one of the sensors is of type 'radar', put it right.
        res = []
        for left, right in list(itertools.combinations([x for x in range(len(sensors))], 2)):
            if sensors[left].type == 'radar':
                res.append((right, left))
            else:
                res.append((left, right))
        return res

    def getX(self):
        return self.X

    def get_sensors(self, edge):
        return self.sensors[edge[0]], self.sensors[edge[1]]


    def initiate_tms(self):
        X = np.zeros(self.nr_elements_pose * len(self.edges))
        for i, edge in enumerate(self.edges):
            sensor1, sensor2 = self.get_sensors(edge)
            Y1, Y2 = get_aligned_sensor_data(sensor1, sensor2)

            assignment_mode = 1 if sensor2.type == 'radar' else 0
            transform = compute_transformation_matrix(Y1, Y2, None, self.assignment, assignment_mode)

            angles = rotm2vector(transform[: 3, : 3])
            translation = transform[: 3, 3]
            nr_rot_vars = int(self.nr_elements_pose / 2)
            X[i * self.nr_elements_pose: i * self.nr_elements_pose + nr_rot_vars] = angles
            X[i * self.nr_elements_pose + nr_rot_vars: i * self.nr_elements_pose + self.nr_elements_pose] = translation

        self.X = X

    def function_combinatorial(self, X):
        Tms = self.convertXtoTms(X)

        e = np.zeros(len(self.edges))
        for i, edge in enumerate(self.edges):
            sensor1, sensor2 = self.get_sensors(edge)

            Y1, Y2 = get_aligned_sensor_data(sensor1, sensor2)
            if sensor2.type != 'radar':
                # Compute errors
                e[i] = self.compute_error_pcl2pcl(Y1, Y2, Tms[i])
            else:
                # Compute errors
                e[i] = np.sum(square_dist_pcl2radar(Y1, Y2, Tms[i]))

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

        # loop over [(loop[0], loop[1]), (loop[1], loop[2]) ... (loop[-1], loop[0])]
        for from_index, to_index in zip(loop, loop[1:]+loop[:1]):
            # find index of edge
            if (from_index, to_index) in self.edges:
                index = self.edges.index((from_index, to_index))
                T = Tms[index]
            else:
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
        result = optimize.minimize(self.function_combinatorial, self.X, method='SLSQP', jac=None, constraints=[eq_identity_constraint_t, fov_constraint], options={'maxiter': self.parameters_optimizer.maximum_iterations, 'disp': self.parameters_optimizer.verbosity}, tol=self.parameters_optimizer.stopping_tolerance)
        result = optimize.minimize(self.function_combinatorial, result.x, method='SLSQP', jac=None, constraints=[eq_identity_constraint_r, eq_identity_constraint_t, fov_constraint], options={'maxiter': self.parameters_optimizer.maximum_iterations, 'disp': self.parameters_optimizer.verbosity}, tol=self.parameters_optimizer.stopping_tolerance)
        self.X = result.x

        # Warning if optimizer did not succeed
        print_optimizer_slsqp_feedback(result)

    def compute_error_pcl2pcl(self, X, Y, T):
        if self.assignment == 'known':
            return np.sum(square_dist_pcl2pcl(X, Y, T))
        else:
            return np.sum(square_dist_unknown_correspondences(X, Y, T))

    def radar_fov_constraint(self, X):
        Tms = self.convertXtoTms(X)

        all_constraint = np.array([])
        for i, edge in enumerate(self.edges):
            sensor1, sensor2 = self.get_sensors(edge)
            if sensor2.type != 'radar':
                pass
            else:
                Y_pcl, Y_radar = get_aligned_sensor_data(sensor1, sensor2)

                # Map PCL to radar and determine range, elevation angle and azimuth angle
                Xr = np.dot(Tms[i], np.vstack([Y_pcl, np.ones((1, Y_pcl.shape[1]))]))
                _, _, elevation = eucledian2polar_multiple(Xr[:3,:])

                # Get upper bound and lower bound
                lb = -sensor2.fov.max_elevation
                ub = sensor2.fov.max_elevation
                c_lb = elevation - lb
                c_ub = ub - elevation
                # colllect all constraints
                all_constraint = np.append(all_constraint, c_lb)
                all_constraint = np.append(all_constraint, c_ub)

        return all_constraint[~np.isnan(all_constraint)]
