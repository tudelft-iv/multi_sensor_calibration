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

from .helper_functions import *
from .calibration_board import *
from .io import *


def get_ransac_parameters():

    this_ransac = ransac_parameters()
    this_ransac.nr_selected_points = 3
    this_ransac.probability_success = 0.99
    this_ransac.probability_inlier = 0.7
    # This threshold is defined as maximum error of mapping from base sensor to sensor defined in dictionary
    this_ransac.threshold = {'lidar': 1E-6, 'stereo': 0.01, 'mono': 0.01, 'radar': 0.01, 'radar3D': 0.01}
    this_ransac.min_nr_boards = {'lidar': 1, 'stereo': 1, 'mono': 1, 'radar': 3, 'radar3D': 1}

    return this_ransac


def get_lidar(Xl, sensor_name="lidar1"):
    # Setup lidar sensor

    # Experimental function box constraints for relative poses (only for MCPE & PSE)
    lidar_constraint = Constraint(parent='lidar1',
                                  lb=np.empty(6) * np.nan,
                                  ub=np.empty(6) * np.nan)

    lidar_sensor = Sensor(name=sensor_name,
                          type='lidar',
                          constraints=lidar_constraint,
                          sensor_data=Xl,
                          mu=np.any(np.isnan(Xl), axis=0) == False,
                          W=np.identity(3),
                          link='velodyne'
                          )

    return lidar_sensor


def get_camera(Xc, sensor_name="camera1"):
    # Setup camera sensor

    # Experimental function box constraints for relative poses (only for MCPE & PSE)
    camera_constraint = Constraint(parent='lidar1',
                                   lb=np.empty(6) * np.nan,
                                   ub=np.empty(6) * np.nan)

    camera_sensor = Sensor(name=sensor_name,
                           type='stereo',
                           constraints=camera_constraint,
                           sensor_data=Xc,
                           mu=np.any(np.isnan(Xc), axis=0) == False,
                           W=np.identity(3),
                           link='left'
                           )

    return camera_sensor


def get_radar(Xr, rcs, sensor_name="radar1"):
    # Setup radar sensor

    # Experimental function box constraints for relative poses (only for MCPE & PSE)
    radar_constraint = Constraint(parent='lidar1',
                                  lb=np.empty(6) * np.nan,
                                  ub=np.empty(6) * np.nan)

    # Radar maximum elevation angle constraint
    radar_fov = fov_radar()
    radar_fov.max_elevation = 9 * math.pi / 180

    # Check whether the z-axis is all zeros, as this indicates a 2D radar which erroneously has 3D measurements.
    if Xr.shape[0] == 3 and np.all(Xr[2, :] == 0):
        Xr = Xr[:2, :]

    radar_type = 'radar' if Xr.shape[0] != 3 else 'radar3D'
    radar_sensor = Sensor(name=sensor_name,
                          type=radar_type,
                          constraints=radar_constraint,
                          sensor_data=Xr,
                          mu=np.any(np.isnan(Xr), axis=0) == False,
                          fov=radar_fov,
                          parameters='eucledian',
                          optional=rcs,
                          W=np.identity(Xr.shape[0]),
                          link='front_center_right_sonar_link'
                          )

    return radar_sensor


def get_sensor_setup(lidar_paths, camera_paths, radar_paths, rcs_paths, outlier_rejection_mode=True,
                     reorder_detections=False, reorder_method='based_on_reference', ignore_file=None):
    if rcs_paths is None:
        rcs_paths = [None] * len(radar_paths)
    assert len(radar_paths) == len(rcs_paths), "Not enough rcs paths given for the set number of radars"

    # Setup all sensors. Note that the enumeration starts at 1 (i.e. enumerate(..., 1)), to make the names start at 1
    # Setup lidar
    lidar_sensors = [get_lidar(load_lidar(path), sensor_name="lidar%d" % idx)
                     for idx, path in enumerate(lidar_paths, 1)]
    # Setup camera
    camera_sensors = [get_camera(load_camera(path), sensor_name="camera%d" % idx)
                      for idx, path in enumerate(camera_paths, 1)]
    # Setup radar (note asterisk (*) to unpack arguments from and into load_radar)
    radar_sensors = [get_radar(*load_radar(*paths), sensor_name="radar%d" % idx)
                     for idx, paths in enumerate(zip(radar_paths, rcs_paths), 1)]

    # Merge all sensors
    sensors = lidar_sensors + camera_sensors + radar_sensors

    calib_boards_per_sensor = [len(sensor.mu) // get_nr_detection(sensor.type) for sensor in sensors]

    #Check if all sensors have equal number of calibration boards. If yes, just select the first.
    assert len(np.unique(calib_boards_per_sensor)) == 1, "No all sensors have equal number of calibration boards:" \
                                                         " %s" % calib_boards_per_sensor
    nr_calib_boards = calib_boards_per_sensor[0]

    # Outlier removal
    if outlier_rejection_mode:
        sensors, nr_calib_boards = remove_outlier_detections(sensors, nr_calib_boards, 'remove_detections')

    # Reorder detections
    if reorder_detections: 
        # Select a sensor that is not radar as reference sensor
        for i in range(len(sensors)):
            if sensors[i].type != 'radar':
                index_reference_sensor = i
                break
        # index_reference_sensor is only used in based_on_reference_sensor

        # Reindex based on selected reference sensor
        sensors = reorder_detections_sensors(sensors, reorder_method, sensors[index_reference_sensor].name)
    if ignore_file:
        sensor_dict = {sensor.name:sensor for sensor in sensors}
        ignore_dict = read_yaml_ignore_file(ignore_file)
        for sensor_name, to_ignore in ignore_dict.items():
            sensor_dict[sensor_name].mu[to_ignore] = False
            sensor_dict[sensor_name].sensor_data[:, to_ignore] = np.nan

    return sensors, nr_calib_boards
