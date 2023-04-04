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
import yaml
from os import listdir
from os.path import isfile, join
from os import path
import pickle
from .calibration_board import *
from .helper_functions import *
import os


def save_obj(obj, name):
    with open(folder + name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)


def load_obj(name):
    with open(folder + name + '.pkl', 'rb') as f:
        return pickle.load(f)


def readYaml2Tms(name):
    with open(name, 'r') as stream:
        try:
            data = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    return data


def write_tms_to_launch_file(sensors, Tms, reference_sensor, directory="results"):
    # Find target link
    for i in range(len(sensors)):
        if sensors[i].name == reference_sensor:
            target_link = sensors[i].link
            break

    # Export to YAML
    for i in range(len(sensors)):
        filename = os.path.join(directory, sensors[i].name + '.launch')
        print('Saving launch files in: ', filename)
        export2launchfile(filename, sensors[i].name, target_link, sensors[i].link, Tms[i])


def write_tms_to_yaml(sensors, Tms, reference_sensor, directory="results"):
    # Find target link
    for i in range(len(sensors)):
        if sensors[i].name == reference_sensor:
            source_link = sensors[i].link
            break

    # Export to YAML
    for i in range(len(sensors)):
        filename = os.path.join(directory, sensors[i].name + '.yaml')
        print('Saving yaml files in: ', filename)
        with open(filename, 'w', encoding='utf8') as outfile:
            yaml.dump({'target': sensors[i].link}, outfile, default_flow_style=False, allow_unicode=True)
            yaml.dump({'source': source_link}, outfile, default_flow_style=False, allow_unicode=True)
            yaml.dump({'transform': Tms[i].tolist()}, outfile, default_flow_style=False, allow_unicode=True)


def read_data_from_folder(mypath, dim1=None, dim2=None):
    onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
    # Read first file to determine the number of detections per board
    nr_cols, nr_rows = read_yaml_file(mypath + onlyfiles[0]).shape
    # Initalise output array
    detector_data = np.zeros((nr_cols, nr_rows * len(onlyfiles)))
    # Loop over all files
    for i, file in enumerate(onlyfiles):
        data = read_yaml_file(mypath + file)
        if dim1 is not None:
            detector_data[:, i * nr_rows:i * nr_rows + nr_rows] = reorder_array(data, dim1, dim2)
        else:
            detector_data[:, i * nr_rows:i * nr_rows + nr_rows] = data
    return detector_data


def read_yaml_ignore_file(name):
    with open(name, 'r') as stream:
        data = yaml.load(stream, Loader=yaml.SafeLoader)
    return data


def read_yaml_file(name):
    with open(name, 'r') as stream:
        try:
            data = yaml.load(stream)
            data_np = np.zeros([len(data[0].keys()), len(data)])
            for i in range(len(data)):
                n = 0
                for j, val in data[i].items():
                    data_np[n, i] = val
                    n = n + 1
        except yaml.YAMLError as exc:
            print(exc)
    return data_np


def read_file(name):
    base, extension = path.splitext(name)
    if extension == '.csv':
        return read_csv_file(name)
    elif extension == '.yaml' or extension == '.yml':
        return read_yaml_file_detector_input(name)
    raise Exception('Cannot load file ', name, ' extension should be yaml, yml or csv.')


def read_yaml_file_detector_input(name): #TODO: improve readibility
    # Read all data points
    points = yaml.load(open(name))

    # Check if radar or non radar:
    if len(points[0]) == 2:
        is_radar = True
        x = np.zeros([len(points)])
        y = np.zeros([len(points)])
    else:
        is_radar = False
        x = np.zeros([len(points)])
        y = np.zeros([len(points)])
        z = np.zeros([len(points)])

    # Loop over all points and save it in numpy array
    i = 0
    for point in points:
        if is_radar:
            x[i] = point["x"]
            y[i] = point["y"]
        else:
            x[i] = point["x"]
            y[i] = point["y"]
            z[i] = point["z"]
        i = i + 1

    # Convert data to numpy array
    if is_radar:
        data = np.array([x, y])
    else:
        data = np.array([x, y, z])

    return data


def read_csv_file(name):
    with open(name, 'rb') as f:
        data = np.loadtxt(f, delimiter=",")
    return data


def export2launchfile(filename, node_name, source_tf, target_tf, T):

    Trf = np.linalg.inv(T)

    x = Trf[0, 3]
    y = Trf[1, 3]
    z = Trf[2, 3]
    r = rotm2eul(Trf[:3, :3])
    f = open(filename, 'w')
    f.write('<launch>')
    f.write('\n<node pkg="tf" type="static_transform_publisher" name="' + node_name + '" ')
    f.write('args= " ')
    f.write('{: .5f}'.format(x) + " " + '{: .5f}'.format(y) + " " + '{: .5f}'.format(z) + " " + '{: .5f}'.format(r[0]) + " " + '{: .5f}'.format(r[1]) + " " + '{: .5f}'.format(r[2]))
    f.write(" " + source_tf + " " + target_tf + " 10")
    f.write(' " />')
    f.write('</launch>')
    f.close()


def read_radar_from_folder(mypath):
    onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
    # Read first file to determine the number of detections per board
    nr_rows, nr_cols = read_file(mypath + onlyfiles[0]).shape
    # Initalise output array
    detector_data = np.zeros((2, len(onlyfiles)))
    rcs_data = np.zeros((2, len(onlyfiles)))
    # Loop over all files
    for i, file in enumerate(sorted(onlyfiles)):
        data = read_file(mypath + file)

        # Find detection of calibration baord
        range = np.sum(data[:2]**2, axis=0)
        index_tcr = np.argmin(range)
        detector_data[0, i * 1:i * 1 + 1] = data[0, index_tcr]
        detector_data[1, i * 1:i * 1 + 1] = data[1, index_tcr]
        rcs_data[i * 1:i * 1 + 1] = data[2, index_tcr]

    return detector_data, rcs_data


def read_from_folder(mypath):
    onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
    # Read first file to determine the number of detections per board
    nr_cols, nr_rows = read_file(mypath + onlyfiles[0]).shape
    # Initalise output array
    detector_data = np.zeros((nr_cols, nr_rows * len(onlyfiles)))
    # Loop over all files
    for i, file in enumerate(sorted(onlyfiles)):
        detector_data[:, i * nr_rows:i * nr_rows + nr_rows] = read_file(mypath + file)

    return detector_data


def _load_lidar_or_cam(path):
    if os.path.isdir(path):
        data = read_from_folder(path)
    elif os.path.isfile(path):
        data = read_file(path)
    else:
        raise Exception('Cannot load data because is neither a valid folder or a valid file (YAML/CSV): %s' % path)
    return data


def load_lidar(path):
    return _load_lidar_or_cam(path)


def load_camera(path):
    return _load_lidar_or_cam(path)


def load_radar(path, rcs_path = None):
    if os.path.isdir(path):
        Xr, rcs = read_radar_from_folder(path)
    elif os.path.isfile(path):
        Xr = read_file(path)
        if rcs_path is not None:
            rcs = read_file(rcs_path)
        else:
            rcs = np.empty(Xr.shape[1]) * np.nan
    else:
        raise Exception('Cannot load data because is neither a valid folder or a valid file (YAML/CSV): %s' % path)

    return Xr, rcs


def export_sensor_data_to_yaml(sensors, folder =''): 
    # Save as YAML files
    print('Export sensor data to YAML files')
    # Loop over all sensor data
    for i in range(len(sensors)):
        filename = os.path.join(folder, sensors[i].name + '_yaml_dump.yaml')
        data_shape = sensors[i].sensor_data.shape
        file = open(filename,'w') 
        # Loop over all detections
        for j in range(data_shape[1]):
            if data_shape[0] == 2:
                # Write X,Y detection to yaml
                file.write('- {x: ' + str(sensors[i].sensor_data[0,j]) + ', y: ' + str(sensors[i].sensor_data[1,j]) + '}\n')
            elif data_shape[0] == 3:
                # Write X,Y,Z detection to yaml
                file.write('- {x: ' + str(sensors[i].sensor_data[0,j]) + ', y: ' + str(sensors[i].sensor_data[1,j]) + ', z: ' + str(sensors[i].sensor_data[2,j])  + '}\n')
            else:
                raise Exception('Data shape should equals 2 for radar and equals 3 for camera and lidar')

def export_sensor_data_to_csv(sensors, folder=''): 
    # Save as csv files
    print('Export sensor data to csv files')
    # Loop over all sensor data
    for i in range(len(sensors)):
        filename = os.path.join(folder, sensors[i].name + '_csv_dump.csv')
        np.savetxt(filename, sensors[i].sensor_data, delimiter=',')
