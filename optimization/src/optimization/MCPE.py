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
import copy
import numpy as np
from scipy import optimize
from scipy.optimize import linear_sum_assignment
from .helper_functions import *
from .calibration_board import *

def check_valid_inputs(sensors, reference_sensor):
    # Check if assumptions are valid:
    index_reference_sensor = np.nan
    valid_reference_sensor = False
    for i in range(len(sensors)):
        if sensors[i].name == reference_sensor:
            # This is the reference sensor
            valid_reference_sensor = True
            index_reference_sensor = i
    
    # Check if reference sensor is found
    if not valid_reference_sensor:
        raise ValueError("Reference sensor is not found")

    # All calibration boards should be visible
    if any(sensors[index_reference_sensor].mu == False):
        raise ValueError("All calibration boards should be visible for the reference sensors (all mus == true)")


class MCPE:
    """ Minimally Connected Pose Estimation (FCPE)

    Assumptions:
        - This class assumes that for reference_sensor all calibration boards are visible. This means that all mus should be true.

    Attributes:
        sensors: struct containing all the sensor data (as defined in helper_functions.py)
        reference_sensor: name of sensor that is used as reference_sensor
        X: vector containing 6D vectors that represent transformation matrices between sensors (inverse of relative sensor poses)

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

        check_valid_inputs(sensors, reference_sensor)

        self.sensors = copy.deepcopy(sensors)
        self.reference_sensor = reference_sensor
        self.nr_elements_pose = 6  # TODO: should be in config file
        self.radar_projection_method = 'eucledian'  # TODO: can be removed
        self.mono_projection_method = 'eucledian'
        self.assignment = correspondences
        self.name_radar_refinement = None
        self.offset_radar = geometry_calibration_board.offset_radar
        self.parameters_optimizer = optimizer_parameters

        # Get initial X
        n = 0
        poses = np.zeros(self.nr_elements_pose * (len(sensors) - 1))
        for i in range(self.getNrSensors()):
            if self.sensors[i].name == self.reference_sensor:
                self.Tm_base = self.sensors[i].T
                self.index_reference_sensor = i
            else:
                start_index, end_index, step_size = self.getIndicesForPose(n)
                angles = rotm2vector(self.sensors[i].T[:3, :3])
                translation = self.sensors[i].T[:3, 3]
                poses[start_index:end_index:step_size] = np.concatenate([angles, translation])
                n = n + 1

        # Save initial poses into X
        self.X = poses

        # Get initials for sensor poses
        if initialise_poses:
            self.X = self.compute_pairwise_poses(self.X)

    def getX(self):
        return self.X

    def getNrSensors(self):
        return len(self.sensors)

    def setOffsetRadar(self, offset):
        self.offset_radar = offset

    def getIndicesForPose(self, index):
        start_index = self.nr_elements_pose * index
        end_index = self.nr_elements_pose * (index + 1)
        step_size = 1
        return start_index, end_index, step_size

    def convertXtoTms(self, X):
        n = 0
        Tms = []
        for i in range(self.getNrSensors()):
            if self.sensors[i].name == self.reference_sensor:
                Tm = self.sensors[i].T
            else:
                start_index, end_index, step_size = self.getIndicesForPose(n)
                Tm = np.identity(4)
                Tm[:3, :3] = vector2rotm(X[start_index:start_index + int(self.nr_elements_pose / 2):step_size])
                Tm[:3, 3] = X[end_index - int(self.nr_elements_pose / 2):end_index:step_size]
                n = n + 1
            Tms.append(Tm)

        return Tms

    def unconstrained_optimization(self):
        #print('Run Unconstrained Optimization')
        self.X = optimize.fmin_slsqp(self.objective_function, self.X, iter=self.parameters_optimizer.maximum_iterations, acc=self.parameters_optimizer.stopping_tolerance, disp=self.parameters_optimizer.verbosity)

    def radar_fov_constraint(self, X):
        xmap = self.sensors[self.index_reference_sensor].sensor_data
        Tms = self.convertXtoTms(X)

        all_constraint = np.array([])
        for i in range(self.getNrSensors()):
            if self.sensors[i].type == 'radar':
                # Get upper bound and lower bound
                lb = -self.sensors[i].fov.max_elevation
                ub = self.sensors[i].fov.max_elevation
                # map to radar
                Xcenter = target2radar3D(xmap, self.offset_radar)
                Xr = self.extrinsic_mapping(Tms[i], Xcenter[:, self.sensors[i].mu])
                # map to polar
                r, a, e = eucledian2polar_multiple(Xr)
                # compute constraints c > 0
                # lb <= x <= ub
                # lb: --> lb-x <=0 -->         x-lb >= 0
                # ub: x <= ub --> x-ub <=0 --> ub-x >=0
                c_lb = e - lb
                c_ub = ub - e
                # colllect all constraints
                all_constraint = np.append(all_constraint, c_lb)
                all_constraint = np.append(all_constraint, c_ub)

        return all_constraint[~np.isnan(all_constraint)]

    def box_constraints(self, X):
        Tms = self.convertXtoTms(X)  # TODO: remove here because already in objective function.
        all_constraint = np.array([])

        debug = False

        # Incrementallly save all constraints
        for i in range(self.getNrSensors()):
            # Get index parent
            index_parent = None
            valid_parent_link = False
            for j in range(self.getNrSensors()):
                if self.sensors[i].constraints.parent == self.sensors[j].name:
                    valid_parent_link = True 
                    index_parent = j
                    break

            # In case of valid parent link add contraints
            if valid_parent_link:
                # Estimate transform
                Tparent = Tms[index_parent]
                Tcurrent = Tms[i]
                Trelative = np.dot(Tparent, np.linalg.inv(Tcurrent))

                if debug:
                    print('Tparent', self.sensors[index_parent].name, ' ', Tparent)
                    print('Tcurrent', self.sensors[i].name, ' ', Tcurrent)
                    print('Trelative', Trelative)

                # Get x,y,z and angles
                t = Trelative[:3, 3]
                angles = rotm2eul(Trelative[:3, :3])

                if debug:
                    # Check if lb > x > ub
                    print(self.sensors[i].constraints.lb)

                # Note that inequlaity constrainst in scipy are define as: c >= 0
                # lb <= x --> lb - x <= 0 --> x - lb >= 0
                # x <= ub --> x - ub <= 0 --> ub - x >= 0
                c_lb = -self.sensors[i].constraints.lb + np.concatenate([angles, t])
                c_ub = - np.concatenate([angles, t]) + self.sensors[i].constraints.ub

                # Add constraints:
                all_constraint = np.append(all_constraint, c_lb)
                all_constraint = np.append(all_constraint, c_ub)
            else:
                # If parent link is invalid and box constraints are defined throw error
                if any(~np.isnan(self.sensors[i].constraints.lb)) or any(~np.isnan(self.sensors[i].constraints.ub)):
                    raise Exception('Parent link of constraint is not found. Update parent link in config file.')

        return all_constraint[~np.isnan(all_constraint)]

    def constrained_optimization(self):
        # print('Run Constrained Optimization')

        box_constraint = {'type': 'ineq', 'fun': lambda x: self.box_constraints(x)}
        fov_constraint = {'type': 'ineq', 'fun': lambda x: self.radar_fov_constraint(x)}

        # Normal constrained optimization
        result = optimize.minimize(self.objective_function, self.X, method='SLSQP', jac=None, constraints=[box_constraint, fov_constraint], options={'maxiter': self.parameters_optimizer.maximum_iterations, 'disp': self.parameters_optimizer.verbosity}, tol=self.parameters_optimizer.stopping_tolerance)
        self.X = result.x

        # Warning if optimizer did not succeed
        print_optimizer_slsqp_feedback(result)

    def optimize(self):
        if any([self.sensors[i].type == 'radar' for i in range(len(self.sensors))]):
            # Pairwise optimization [baseline for radar]
            self.constrained_optimization()
        else:
            # Pairwise optimization [baseline]
            self.unconstrained_optimization()

    def compute_pairwise_poses(self, X):
        # Note that this one can be used for pairwise optimization
        # This fucntion can deal with known and unknown point 2 point assignemnts
        Tms = self.convertXtoTms(X)
        xmap = self.sensors[self.index_reference_sensor].sensor_data

        # Initialise X
        Xinit = X
        # Estimate all poses with respect to reference sensor
        n = 0
        for current_sensor, Tm in zip(self.sensors, Tms):
            if self.reference_sensor != current_sensor.name:
                start_index, end_index, step_size = self.getIndicesForPose(n)

                if current_sensor.type == 'radar':
                    # Radar sensor
                    if current_sensor.parameters == 'eucledian':
                        pcl_radar = np.vstack([current_sensor.sensor_data, np.zeros([1, current_sensor.sensor_data.shape[1]])])

                        Xcenter = target2radar3D(xmap, self.offset_radar)
                        T = compute_transformation_matrix(Xcenter[:, current_sensor.mu], pcl_radar, Tm, self.assignment, 1)
                    elif current_sensor.parameters == 'polar':
                        pcl_radar = np.zeros(3, current_sensor.sensor_data.shape[1])
                        pcl_radar[0, :], pcl_radar[1, :], pcl_radar[2, :] = polar2eucledian_multiple(current_sensor.sensor_data[0, :], current_sensor.sensor_data[1, :], current_sensor.sensor_data[2, :])

                        Xcenter = target2radar3D(xmap, self.offset_radar)
                        T = compute_transformation_matrix(Xcenter[:, current_sensor.mu], pcl_radar, Tm, self.assignment, 1)
                    else:
                        raise Exception('Unknown radar projection method')
                else:
                    xmap_map = {
                        'radar3D':target2radar3D,
                        'lidar':target2lidar,
                        'stereo':target2stereo,
                        'mono':target2monocular,
                    }
                    if current_sensor.type not in xmap_map.keys():
                        raise Exception('Unknown sensor type')

                    X_hat  = xmap_map[current_sensor.type](xmap)
                    T = compute_transformation_matrix(X_hat[:, current_sensor.mu],
                                                      current_sensor.sensor_data[:, current_sensor.mu],
                                                      Tm, self.assignment)
                # Store sensor pose into  X:
                angles = rotm2vector(T[:3, :3])
                translation = T[:3, 3]
                Xinit[start_index:end_index:step_size] = np.concatenate([angles, translation])

                n = n + 1

        return Xinit

    def extrinsic_mapping(self, T, xmap):
        return np.dot(T[:3, :], np.vstack([xmap, np.ones([1, xmap.shape[1]])]))  # xmap should contains ones

    def project2lidar(self, T, xmap, sensor_parameters):
        return self.extrinsic_mapping(T, target2lidar(xmap))

    def project2stereo(self, T, xmap, sensor_parameters):
        return self.extrinsic_mapping(T, target2stereo(xmap))

    def project2mono(self, T, xmap, sensor_parameters):
        # TODO: implement mapping to monocular in pixels as well
        Ym = self.extrinsic_mapping(T, target2monocular(xmap))
        if self.mono_projection_method == 'pixels':
            if sensor_parameters.P is not None:
                uvw = np.dot(sensor_parameters.P, np.vstack([Ym, np.ones([1, Ym.shape[1]])]))
            else:
                raise Exception('Camera projection matrix should be defined')

            return uvw[:2, :] / uvw[2, :]
        elif self.mono_projection_method == 'eucledian':
            return Ym
        else:
            raise Exception('project2mono unknown method')

    def project2radar3D(self, T, xmap, sensor_parameters):
        return self.project2radar(T, xmap, sensor_parameters, do_3d=True)

    def project2radar(self, T, xmap, sensor_parameters, do_3d=False):
        Xr = self.extrinsic_mapping(T, target2radar3D(xmap, self.offset_radar))

        if sensor_parameters == 'eucledian':
            Yr = Xr if do_3d else p(Xr)[:2, :]
        elif sensor_parameters == 'polar':
            r, a, e = eucledian2polar_multiple(Xr)
            Yr = np.stack((r, a, e)) if do_3d else np.stack((r, a))
        else:
            raise Exception('Radar sensor_parameters is undefined')

        return Yr

    def a2b_assignment(self, a, b, W):
        # Returns sum squared error
        aSumSquare = np.sum(a**2, axis=0)
        bSumSquare = np.sum(b**2, axis=0)
        mul = np.dot(a.T, b)
        sq_errors = aSumSquare[:, np.newaxis] + bSumSquare - 2 * mul
        row_ind, col_ind = linear_sum_assignment(sq_errors)

        return np.sum(np.dot(W, (a[:, row_ind] - b[:, col_ind])**2), axis=0)  # sq_errors[row_ind, col_ind].sum()

    def compute_sensor_errors(self, Y, data, mus, W):
        if self.assignment == 'known':
            e = np.sum(np.dot(W, (Y[:, mus] - data[:, mus])**2), axis=0)
        else:
            e = self.a2b_assignment(Y[:, mus], data[:, mus], W)

        return e

    def compute_total_error(self, errors):
        return np.sum(errors)

    def compute_calibration_errors(self, xmap, Tms):
        errors = []
        type_map = {'lidar': self.project2lidar,
                    'stereo': self.project2stereo,
                    'mono': self.project2mono,
                    'radar': self.project2radar,
                    'radar3D': self.project2radar3D
                    }
        for current_sensor, Tm in zip(self.sensors, Tms):
            # Map points to each sensor
            Y = type_map[current_sensor.type](Tm, xmap, current_sensor.parameters)
            errors.append(self.compute_sensor_errors(Y, current_sensor.sensor_data, current_sensor.mu, current_sensor.W))
        return errors

    def objective_function(self, X):
        # Data points of reference sensor
        xmap = self.sensors[self.index_reference_sensor].sensor_data
        # Get current estimate of transformation matrices
        Tms = self.convertXtoTms(X)
        errors = []

        # Compute individual errors
        sensor_errors = self.compute_calibration_errors(xmap, Tms)

        # Create vector with all errors
        for i in range(len(sensor_errors)):
            errors = np.concatenate([errors, sensor_errors[i]])

        # Return total error:
        return self.compute_total_error(errors)

    def objective_function_xmap(self, X):
        Tms = self.convertXtoTms(self.X)
        xmap = np.reshape(X, (3, -1), order='F')
        errors = []

        # Compute individual errors
        sensor_errors = self.compute_calibration_errors(xmap, Tms)

        # Create vector with all errors
        for i in range(len(sensor_errors)):
            errors = np.concatenate([errors, sensor_errors[i]])

        # In this case a least square will be used so all errors are returned:
        return errors

    def optimize_least_squares_xmap(self):
        xmap_est = optimize.least_squares(self.objective_function_xmap, self.X[(self.getNrSensors() - 1) * self.nr_elements_pose:])
        self.X[(self.getNrSensors() - 1) * self.nr_elements_pose:] = np.ravel(xmap_est.x, order='F')

    def fov_optimization(self, rcs_threshold0, name):
        Tms = self.convertXtoTms(self.X)
        c0 = np.array([0, 0, 0, rcs_threshold0])  # relative to initial estimate

        # Find index of radar to update:
        index_radar = -1
        for i in range(self.getNrSensors()):
            if self.sensors[i].name == self.name_radar_refinement:
                index_radar = i

        c = optimize.minimize(self.compute_rcs_error, c0, args=index_radar, method='Nelder-Mead')

        # Update radar transformation matrix
        Xc = np.zeros(6)
        Xc[1] = c.x[0]
        Xc[2] = c.x[1]
        Xc[5] = c.x[2]

        Tm = np.identity(4)
        Tm[:3, :3] = vector2rotm(Xc[0:3])
        Tm[:3, 3] = Xc[3:]

        Tms[i] = np.dot(Tm, Tms[i])  # NOTE: premultiplied

        return Tms

    def compute_rcs_error(self, c, index_radar):
        if 0 > index_radar > self.getNrSensors():
            print(sys._getframe().f_code.co_name, " undefined radar name")

        # Get Xmap, which we will not optimze for
        xmap = self.sensors[self.index_reference_sensor].sensor_data
        Tms = self.convertXtoTms(self.X)

        error = []
        rcs_th = np.zeros(self.getNrSensors())

        # 1) update X with C
        Xc = np.zeros(6)  # --> remember similar to matlab: a_z, a_y, a_x, p_x, p_y, p_z
        # TODO: update Xc --> p_z, a_x and a_y --> aka: heigth, roll angle and pitch angle
        Xc[1] = c[0]
        Xc[2] = c[1]
        Xc[5] = c[2]
        Tm = np.identity(4)
        Tm[:3, :3] = vector2rotm(Xc)
        Tm[:3, 3] = Xc[3:]
        Tms[index_radar] = np.dot(Tm, Tms[index_radar])  # NOTE: premultiplied
        rcs_th[index_radar] = c[3]

        # 2) compute elevation angle of radar point
        Xr = self.extrinsic_mapping(Tms[index_radar], target2radar3D(xmap, self.offset_radar))
        _, _, elevation_angle = eucledian2polar_multiple(Xr)
        # 3) compute RCS error
        for j in range(Xr.shape[1]):
            error.append(self.get_rcs_error(elevation_angle[j], rcs_th[index_radar], self.sensors[index_radar].mu[index_radar], self.sensors[index_radar].optional[j], self.sensors[index_radar].fov))

        # compute sums squared error
        sse = sum(np.array(error)**2)

        return sse

    def get_rcs_error(self, elevation_angle, rcs_th, mu, rcs, fov):
        plane_up = fov.max_elevation
        plane_down = -fov.max_elevation

        d = min(np.absolute(plane_up - elevation_angle), np.absolute(plane_down - elevation_angle))

        if mu:
            # Inside FOV
            if rcs > rcs_th:
                error = 0
            else:
                error = d
        else:
            # Outside FOV
            if rcs < rcs_th:
                error = 0
            else:
                error = d

        return error
