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
sys.path.append('lib/icp')
import numpy as np
import itertools
import pickle
import argparse
import timeit
import copy
import scipy.io
from optimization.optimize import joint_optimization
from optimization.config import *

if __name__ == '__main__':
    # Python 3 should be used:
    assert sys.version_info.major == 3, 'Python 3 should be used'

    # Instantiate the parser
    parser = argparse.ArgumentParser(description='Experiments for extrinsic calibration')

    # Required positional argument
    parser.add_argument('--calibration-mode', required=True, type=int, help='0: Pose and Structure Estimation (PSE) with unknown observation covariance matrices, 1: Pose and Structure Estimation (PSE) with known observation covariance matrices, 2: Minimally Connected Pose Estimation (MCPE), 3: Fully Connected Pose Estimation (FCPE)')

    # path to csv files
    parser.add_argument('--lidar', type=str, action='append', default=[], help='Path to lidar CSV/YAML file')
    parser.add_argument('--camera', type=str, action='append', default=[], help='Path to camera CSV/YAML file')
    parser.add_argument('--radar', type=str,action='append', default=[], help='Path to radar CSV/YAML file')
    parser.add_argument('--rcs', type=str, action='append', default=None, required=False, help='Path to RCS CSV file') #TODO: add option to load as YAML file as well.
    parser.add_argument('--ignore_file', type=str, default=None, required=False, help="Path to file stating which measurements to ignore")
    # output directory to save yaml
    parser.add_argument('--output-directory', type=str, default="results", help='Path to save output yaml file')

    # Settings for optimizer
    parser.add_argument('--reference-sensor', type=str, default='lidar1', help='Reference sensor for plotting and for MCPE')
    parser.add_argument('--unknown-correspondences', action='store_true', default=False, help='Point correspondences between circle centers are known (camera/lidar)')
    parser.add_argument('--reorder-detections', action='store_true', default=False, help='Reorder lidar and camera detections to make correspondences true. This avoids that you have to run with optimizer setting: unknown_correspondences.')
    parser.add_argument('--reorder-method', default='based_on_reference_sensor', help='Determines how to reorder to sensor detections of lidar/camera to make sure that point correspondences are known, either "based_on_reference_sensor" or "based_on_definition.')
    parser.add_argument('--keep-outliers', action='store_true', default=False, help='Keep outlier detections.')

    # Visualise results
    parser.add_argument('--visualise', action='store_true', default=False, help='Plot results in 3D plot')
    parser.add_argument('--sensors_to_number', action='append', default=['camera1'], help='Add numbering to sensor points')
    parser.add_argument('--plot_correspondence', nargs='+', default=[], help='Plot alignment of two sensor sets')

    # Parse arguments
    args = parser.parse_args()
    assert len(args.lidar) + len(args.camera) + len(args.radar) >= 2, \
                "Optimizer requires at least two sensors. Received the following:\n" \
                "\tLidars: %s\n" \
                "\tCameras: %s\n" \
                "\tRadars: %s" % (args.lidar, args.camera, args.radar)
    # Retrieve sensors setup:
    sensors, nr_calib_boards = get_sensor_setup(args.lidar, args.camera, args.radar, args.rcs, not args.keep_outliers, args.reorder_detections, args.reorder_method, args.ignore_file)

    if args.unknown_correspondences:
        # In this case the correspondences between the keypoints of lidar and camera are not known.
        # The optimizer will find the right correspondences for you during optimzation.
        # This takes more time, so you would like to avoid it.
        # It is recommended to set --reorder-detections argument instead.
        correpondences = 'unknown'
    else:
        # The correspondences are known. The top-left, top-rigth, bottom-left, bottom-right are matching for lidar & camera
        # This order is similar for lidar and camera, for instance detection N for lidar matches detection N for camera
        # For instance, these are both top-left circle center.
        correpondences = 'known'

    # Joint optimization
    joint_optimization(sensors, args.calibration_mode, correpondences, args.reference_sensor, args.visualise, args.output_directory, args.sensors_to_number, args.plot_correspondence)
