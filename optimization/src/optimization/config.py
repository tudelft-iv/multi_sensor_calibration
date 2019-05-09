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

from .helper_functions import *
from .calibration_board import *
from .io import *


def get_ransac_parameters():

    this_ransac = ransac_parameters()
    this_ransac.nr_selected_points = 3
    this_ransac.probability_success = 0.99
    this_ransac.probability_inlier = 0.7
    # This threshold is defined as maximum error of mapping from base sensor to sensor defined in dictionary
    this_ransac.threshold = {'lidar': 1E-6, 'stereo': 0.01, 'mono': 0.01, 'radar': 0.01}
    this_ransac.min_nr_boards = {'lidar': 1, 'stereo': 1, 'mono': 1, 'radar': 3}

    return this_ransac


def get_lidar(Xl):
    # Setup lidar sensor

    # Experimental function box constraints for relative poses (only for MCPE & PSE)
    lidar_constraint = Constraint()
    lidar_constraint.parent = 'lidar1'
    lidar_constraint.lb = np.empty(6) * np.nan
    lidar_constraint.ub = np.empty(6) * np.nan

    lidar_sensor = Sensor()
    lidar_sensor.name = 'lidar1'
    lidar_sensor.type = 'lidar'
    lidar_sensor.constraints = lidar_constraint
    lidar_sensor.sensor_data = Xl
    lidar_sensor.mu = np.full((Xl.shape[1]), True, dtype=bool)
    lidar_sensor.mu[np.where(np.isnan(Xl[0, :]))] = False
    lidar_sensor.W = np.identity(3)
    lidar_sensor.link = 'velodyne'

    return lidar_sensor


def get_camera(Xc):
    # Setup camera sensor

    # Experimental function box constraints for relative poses (only for MCPE & PSE)
    camera_constraint = Constraint()
    camera_constraint.parent = 'lidar1'
    camera_constraint.lb = np.empty(6) * np.nan
    camera_constraint.ub = np.empty(6) * np.nan

    camera_sensor = Sensor()
    camera_sensor.name = 'camera1'
    camera_sensor.type = 'stereo'
    camera_sensor.constraints = camera_constraint
    camera_sensor.sensor_data = Xc
    camera_sensor.mu = np.full((Xc.shape[1]), True, dtype=bool)
    camera_sensor.mu[np.where(np.isnan(Xc[0, :]))] = False
    camera_sensor.W = np.identity(3)
    camera_sensor.link = 'left'

    return camera_sensor


def get_radar(Xr, rcs):
    # Setup radar sensor

    # Experimental function box constraints for relative poses (only for MCPE & PSE)
    radar_constraint = Constraint()
    radar_constraint.parent = 'lidar1'
    radar_constraint.lb = np.empty(6) * np.nan
    radar_constraint.ub = np.empty(6) * np.nan

    # Radar maximum elevation angle constraint
    radar_fov = fov_radar()
    radar_fov.max_elevation = 9 * math.pi / 180

    radar_sensor = Sensor()
    radar_sensor.name = 'radar1'
    radar_sensor.type = 'radar'
    radar_sensor.constraints = radar_constraint
    radar_sensor.sensor_data = Xr
    radar_sensor.mu = np.full((Xr.shape[1]), True, dtype=bool)  # mu are defined to be mapping from internal X (optimization) to observations
    radar_sensor.mu[np.where(np.isnan(Xr[0, :]))] = False
    radar_sensor.fov = radar_fov
    radar_sensor.parameters = 'eucledian'  # eucledian or polar
    radar_sensor.optional = rcs
    radar_sensor.W = np.identity(2)
    radar_sensor.link = 'front_center_right_sonar_link'

    return radar_sensor


def get_sensor_setup(lidar_path, camera_path, radar_path, rcs_path, outlier_rejection_mode=True, reorder_detections=False, reorder_method='based_on_reference'):
    # Load pointclouds from lidar (Xl), camera (Xc), and radar Xr
    Xl, Xc, Xr, rcs = load_data(lidar_path, camera_path, radar_path, rcs_path)

    # # Setup lidar
    lidar_sensor = get_lidar(Xl)

    # Setup camera
    camera_sensor = get_camera(Xc)

    # Setup radar
    radar_sensor = get_radar(Xr, rcs)

    # Merge all sensors
    sensors = [lidar_sensor, camera_sensor, radar_sensor]
    nr_calib_boards = int(len(lidar_sensor.mu) / get_nr_detection('lidar'))

    # Outlier removal
    if outlier_rejection_mode:
        sensors, nr_calib_boards = remove_outlier_detections(sensors, nr_calib_boards, 'remove_board_locations')

    # Reorder detections
    if reorder_detections: 
        # Select first sensor that is not radar as reference sensor
        for i in range(len(sensors)):
            if sensors[i].type is not 'radar':
                index_reference_sensor = i
        # index_reference_sensor is only used in based_on_reference_sensor

        # Reindex based on selected reference sensor
        sensors = reorder_detections_sensors(sensors, reorder_method, sensors[index_reference_sensor].name)

    return sensors, nr_calib_boards
