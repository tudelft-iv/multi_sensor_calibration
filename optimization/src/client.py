#!/usr/bin/env python3

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

import rospy
from accumulator.srv import *
from accumulator.msg import AccumulatedPatterns
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from optimization.io import *
from optimization.calibration_board import *


def xyz_array_to_pointcloud2(points, stamp=None, frame_id=None):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''

    msg = PointCloud2()
    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if points.shape[0] == 3:
        msg.height = 1
        msg.width = points.shape[1]
    else:
        msg.height = 1
        msg.width = len(points)
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = np.asarray(points.T, np.float32).tostring()

    return msg


def convert_sensor_data(name, data, nr_detections, header):
    msg = AccumulatedPatterns()
    # std_msgs/String sensor
    # sensor_msgs/PointCloud2[] patterns # different board locations for this sensor
    # msg.sensor = 'test'
    t = []
    for i in range(int(data.shape[1] / nr_detections)):
        t.append(xyz_array_to_pointcloud2(data[:, i * nr_detections:i * nr_detections + nr_detections], None, header))
    msg.sensor = String(name)
    msg.patterns = t

    return msg


def pattern_client(pcl_lidar, pcl_camera, pcl_radar):
    rospy.wait_for_service('/optimizer/optimize')
    try:
        send_patterns = rospy.ServiceProxy('/optimizer/optimize', SendPatterns)

        # Convert PCl to ROS messages
        sensor1_data = convert_sensor_data('/lidar_detector/lidar_pattern', pcl_lidar, 4, 'velodyne')
        sensor2_data = convert_sensor_data('/stereo_detector/stereo_pattern', pcl_camera, 4, 'left')
        sensor3_data = convert_sensor_data('/radar_detector/radar_pattern', pcl_radar, 1, 'front_center_right_sonar_link')

        # Append all sensors
        send_data = []
        send_data.append(sensor1_data)
        send_data.append(sensor2_data)
        send_data.append(sensor3_data)

        # Send Request
        send_patterns.call(SendPatternsRequest(send_data))

    except rospy.ServiceException as e:
        print('Service call failed', e)


if __name__ == "__main__":
    # Data files:
    lidar_path = 'src/clr_calibration/optimization/data/lidar_all.csv'
    camera_path = 'src/clr_calibration/optimization/data/camera_all.csv'
    radar_path = 'src/clr_calibration/optimization/data/radar_all.csv'
    rcs_path = 'src/clr_calibration/optimization/data/rcs_values_all.csv'
    # Load data
    Xl = load_lidar(lidar_path)
    Xc = load_camera(camera_path)
    Xr, rcs = load_radar(radar_path, rcs_path)

    # Convert radar data (detections and RCS) to numpy array
    z = np.zeros((3, Xr.shape[1]))
    z[:2, :] = Xr
    z[2, :] = rcs

    # Send Service message
    pattern_client(Xl, Xc, z)
