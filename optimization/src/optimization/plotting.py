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

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from .calibration_board import *
from .config import *
from .PSE import *
from .ransac import *
from tikzplotlib import save as tikz_save

markers = ['o', 'v', '^', '<', '>', 's', '8', 'p']
colours = ['r', 'g', 'b', 'k', 'm', 'c']
folder = 'results/'


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def extrinsic_mapping(T, xmap):
    if xmap.shape[0] < 3:
        # TODO: fix this but plotting arcs instead
        pcl = np.vstack([np.zeros([1, xmap.shape[1]]), np.ones([1, xmap.shape[1]])])
        return np.dot(T[:3, :], np.vstack([xmap, pcl]))  # xmap should contains ones
    else:
        return np.dot(T[:3, :], np.vstack([xmap, np.ones([1, xmap.shape[1]])]))  # xmap should contains ones


def plot_reference_frame(Tm, name, ax):
    T = np.linalg.inv(Tm)
    pos = T[:, 3]
    v1 = np.dot(T, np.array([1, 0, 0, 1]))
    ax.plot([pos[0], v1[0]], [pos[1], v1[1]], [pos[2], v1[2]], label='xaxis', c='r')
    v2 = np.dot(T, np.array([0, 1, 0, 1]))
    ax.plot([pos[0], v2[0]], [pos[1], v2[1]], [pos[2], v2[2]], label='xaxis', c='g')
    v3 = np.dot(T, np.array([0, 0, 1, 1]))
    ax.plot([pos[0], v3[0]], [pos[1], v3[1]], [pos[2], v3[2]], label='xaxis', c='b')

    ax.text(pos[0], pos[1], pos[2], name, 'x')
    return ax


def plot_alignment_sensors(ax, sensor1, sensor2, Tm1, Tm2):
    X, Y = get_aligned_sensor_data(sensor1, sensor2)
    X = extrinsic_mapping(np.linalg.inv(Tm1), X)
    Y = extrinsic_mapping(np.linalg.inv(Tm2), Y)
    all_data = np.stack([X, Y])
    for line in all_data.transpose(2,1,0):
        ax.plot3D(line[0], line[1], line[2], 'k')


def plot_3D_calibration_result(sensors, Tms, sensors_to_number, sensor_correspondence_to_plot=[]):
    fig = plt.figure('3D visualisation of calibration result')
    ax = fig.add_subplot(111, projection='3d')
    for j in range(len(sensors)):
        if sensors[j].type != 'radar':
            # Map to base reference frame
            data = extrinsic_mapping(np.linalg.inv(Tms[j]), sensors[j].sensor_data)
            # Plot it
            ax.scatter(data[0, :], data[1, :], data[2, :], c=colours[j], marker=markers[j], edgecolors=colours[j], label=sensors[j].name)
            if sensors[j].name in sensors_to_number:
                for k in range(data.shape[1]):
                    if np.all(~np.isnan(data[0, k])):
                        ax.text(data[0, k], data[1, k], data[2, k], str(k), 'x')

        else:
            plot_polar_with_eucledian = True
            if sensors[j].parameters == 'polar' or plot_polar_with_eucledian:
                # Polar plot
                if plot_polar_with_eucledian and sensors[j].parameters == 'eucledian':
                    # Plot arcs by converting from eucledian coordinates
                    r, a, _ = eucledian2polar_multiple(np.vstack([sensors[j].sensor_data, np.zeros((1, sensors[j].sensor_data.shape[1]))]))
                else:
                    # Sensor dat contains range and azimuth
                    r = sensors[j].sensor_data[0, :]
                    a = sensors[j].sensor_data[1, :]

                for i in range(len(r)):
                    arc_e = np.linspace(-sensors[j].fov.max_elevation, sensors[j].fov.max_elevation, 10)
                    arc_r = np.tile(r[i], (1, len(arc_e)))
                    arc_a = np.tile(a[i], (1, len(arc_e)))
                    x, y, z = polar2eucledian_multiple(arc_r, arc_a, arc_e)
                    data = np.zeros((3, len(arc_e)))
                    data[0, :] = x
                    data[1, :] = y
                    data[2, :] = z
                    data = extrinsic_mapping(np.linalg.inv(Tms[j]), data)

                    ax.plot(data[0, :], data[1, :], data[2, :], c=colours[j], label=sensors[j].name if i == 0 else "")
                    if sensors[j].name in sensors_to_number and np.all(~np.isnan(data)):
                        ax.text(data[0, 0], data[1, 0], data[2, 0], str(i), 'x')

            elif sensors[j].parameters == 'eucledian':
                # Map to base reference frame
                data = extrinsic_mapping(np.linalg.inv(Tms[j]), sensors[j].sensor_data)
                # Plot it
                ax.scatter(data[0, :], data[1, :], data[2, :], c=colours[j], marker=markers[j], edgecolors=colours[j], label=sensors[j].name)
            else:
                print(sensors[j].data_type)
                print('Error: unknown plotting mode for radar')

    if len(sensor_correspondence_to_plot) == 2:
        sensor_dict = {sensor.name: (sensor, Tms[index]) for index, sensor in enumerate(sensors)}
        sensor1, Tm1 = sensor_dict[sensor_correspondence_to_plot[0]]
        sensor2, Tm2 = sensor_dict[sensor_correspondence_to_plot[1]]
        plot_alignment_sensors(ax, sensor1, sensor2, Tm1, Tm2)

    plt.legend()

    for i in range(len(Tms)):
        ax = plot_reference_frame(Tms[i], sensors[i].name, ax)

    set_axes_equal(ax)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    plt.title('3D Visualization in base sensor reference frame')

    # NOTE: cannnot convert 3D visualisation to tikz using matplotlib2tikz because of following issue: https://github.com/matplotlib/matplotlib/issues/7243
    #fig.savefig(folder + '3D_visualisation_calibration_result.png')


def plot_3D_errors(sensors, Tms, xmap, calibration, plot_relative_errors=False, min_circle=50, max_circle=300, plot3d=True):
    # NOTE: only deals with fully observable detections, so cannot deal with outlier detections (mu=False)
    # TODO: Make sure that it can deal with mu = False

    # Retrieve all errors:
    all_errors = calibration.compute_calibration_errors(xmap, Tms)
    # Get nr of calibration boards used in extrinsic calibration
    nr_calib_boards = int(xmap.shape[1] / get_nr_detection(calibration.reference_sensor))
    # Initialise errors
    e = np.zeros((nr_calib_boards, len(sensors)))
    for j in range(len(sensors)):
        errors = all_errors[j]
        nr_detections_board = get_nr_detection(sensors[j].type)
        nr_targets = int(len(errors) / nr_detections_board)

        n = 0
        for i in range(nr_targets):
            if sensors[j].mu[i] == 1:  # errors only contains visible boards but we need to collect calibration boards errors
                e[n, j] = np.sqrt(np.mean(errors[n * nr_detections_board:n * nr_detections_board + nr_detections_board]))  # TODO: move the fours into sensor struct
                n = n + 1

    # Compute total error for calibration baord --> sum of all sensor errors
    total_error = np.sum(e, axis=1)
    # Get centers of calibration boards
    xmap_center = np.reshape(np.mean(np.reshape(xmap, (-1, 4)), axis=1), (3, -1))
    # Get circle size based on min and max values
    circle_size = min_circle + (max_circle - min_circle) * (total_error - (total_error).min()) / (total_error.max() - total_error.min())
    # Get indices
    ind = np.lexsort(([x for x in range(nr_targets)], np.argsort(total_error)))
    # Show absolute or relative error
    if plot_relative_errors:
        e = np.divide(e, np.tile(total_error.T, (3, 1)).T)

    # 3D visualisation
    fig = plt.figure('3D error visualisation', figsize=(15, 5))
    if plot3d:
        ax = fig.add_subplot(121, projection='3d')
        ax.scatter(xmap_center[0, :], xmap_center[1, :], xmap_center[2, :], c='b', marker='o', s=circle_size)
        set_axes_equal(ax)

        for i in range(nr_targets):
            ax.text(xmap_center[0, i], xmap_center[1, i], xmap_center[2, i], ind[i])

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        plt.title('3D Visualization in base sensor reference frame')
    else:
        ax = fig.add_subplot(121)
        plt.scatter(xmap_center[0, :], xmap_center[1, :], s=circle_size, c='b', marker='o')

        for i in range(nr_targets):
            plt.text(xmap_center[0, i] + 0.05, xmap_center[1, i], ind[i])

        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('3D Visualization in base sensor reference frame')

    # Stacked bar plot
    ax = fig.add_subplot(122)
    bottom_d = []
    for i in range(len(sensors)):
        if i > 0:
            plt.bar(ind, e[:, i], bottom=bottom_d, color=colours[i], width=0.5, label=sensors[i].name)
            bottom_d += list(e[:, i])
        else:
            plt.bar(ind, e[:, i], color=colours[i], width=0.5, label=sensors[i].name)
            bottom_d = e[:, i]

    ax.legend()
    ax.set_xlabel('Calibration board number [-]')
    if plot_relative_errors:
        ax.set_ylabel('Relative error')
    else:
        ax.set_ylabel('Total RMSE [m]')
    plt.title('Calibration board errors for every board location')

    # NOTE: cannnot convert 3D visualisation to tikz using matplotlib2tikz because of following issue: https://github.com/matplotlib/matplotlib/issues/7243
    fig.savefig(folder + '3D_error_visualisation.png')
    if not plot3d:
        tikz_save(folder + "3D_error_visualisation.tex", figureheight='\\figureheight', figurewidth='\\figurewidth', strict=False, extra_axis_parameters={'xticklabel style={font=\\scriptsize}', 'yticklabel style={font=\\scriptsize}'})

    # Print errors to terminal
    debug = False
    if debug:
        print("Error all calibration boards", total_error.sum())
        print("Error calibration board = ", total_error)
        print("Errors xyz calib board = ", e)


def sequential_removal_calibration_boards(sensors, sensors_test, nr_calibration_boards, reference_sensor, init_poses=True):
    # Setup test set
    test = PSE(sensors_test, reference_sensor, init_poses, 'known', None, '3D')
    xmap_test = test.convertX2Xmap(test.getX())

    # Params
    min_nr_calibration_boards = 3
    e_train_set = np.zeros((len(sensors), nr_calibration_boards - min_nr_calibration_boards + 1))
    e_test_set = np.zeros((len(sensors) - 1, nr_calibration_boards - min_nr_calibration_boards + 1))
    x = range(nr_calibration_boards - min_nr_calibration_boards + 1)
    for ir in range(nr_calibration_boards - min_nr_calibration_boards + 1):
        # Optimize
        proposal = PSE(sensors, reference_sensor, init_poses)
        proposal.optimize('constrained')
        xmap = proposal.convertX2Xmap(proposal.getX())
        Tms = proposal.convertXtoTms(proposal.getX())

        # Step 2: same as in plotting.py
        # Retrieve all errors:
        all_errors = proposal.compute_calibration_errors(xmap, Tms)
        # Get nr of calibration boards used in extrinsic calibration
        nr_calib_boards = int(xmap.shape[1] / get_nr_detection(proposal.reference_sensor))
        # Initialise errors
        e = np.zeros((nr_calib_boards, len(sensors)))
        for j in range(len(sensors)):
            errors = all_errors[j]
            nr_detections_board = get_nr_detection(sensors[j].type)
            nr_targets = int(len(errors) / nr_detections_board)

            for i in range(nr_targets):
                e[i, j] = np.sqrt(np.mean(errors[i * nr_detections_board:i * nr_detections_board + nr_detections_board]))

        all_errors_test = test.compute_calibration_errors(xmap_test, Tms)

        # Get nr of calibration boards used in extrinsic calibration
        nr_calib_boards_test = int(xmap_test.shape[1] / get_nr_detection(test.reference_sensor))
        # Initialise errors
        e_test = np.zeros((nr_calib_boards_test, len(sensors_test) - 1))
        n = 0
        for j in range(len(sensors_test)):
            errors_t = all_errors_test[j]
            nr_detections_board = get_nr_detection(sensors_test[j].type)
            nr_targets = int(len(errors_t) / nr_detections_board)

            if sensors_test[j].name is not test.reference_sensor:
                for i in range(nr_targets):
                    e_test[i, n] = np.sqrt(np.mean(errors_t[i * nr_detections_board:i * nr_detections_board + nr_detections_board]))
                n = n + 1

        # Compute total error for calibration baord
        total_error = np.sum(e, axis=1)
        sorted_indices = np.argsort(total_error)
        keep_indices = sorted_indices[:len(sorted_indices) - 1]

        # Update for next round
        sensors = update_sensor_data_with_indices(sensors, keep_indices)

        # Error
        e_train_set[:, ir] = np.mean(e, axis=0)
        e_test_set[:, ir] = np.mean(e_test, axis=0)

    # Plotting
    legend_labels = {}
    legend_labels[0] = 'RMSE lidar to stereo'
    legend_labels[1] = 'RMSE lidar to radar'
    fig = plt.figure('Sequential Removal of Calibration Boards')
    plt.plot(x, np.sum(e_train_set, axis=0), label='total RMSE: training set')
    plt.plot(x, np.sum(e_test_set, axis=0), label='total RMSE: test set')
    plt.plot(x[np.argmin(np.sum(e_test_set, axis=0))], np.min(np.sum(e_test_set, axis=0)), marker='o', c='k')
    for i in range(e_test_set.shape[0]):
        plt.plot(x, e_test_set[i, :], c=colours[i], label=legend_labels[i])

    plt.xlabel('removed \# of calibration boards')
    plt.ylabel('RMSE [m]')
    plt.ylim((0, 0.1))
    plt.legend(loc=2)

    tikz_save(folder + "sequential_removal_of_calibration_boards.tex", figureheight='\\figureheight', figurewidth='\\figurewidth', strict=True)
    fig.savefig(folder + 'sequential_removal_of_calibration_boards.png')

    return e_train_set, e_test_set, [i for i in range(nr_calibration_boards - min_nr_calibration_boards)]


def plot_observation_errors(sensors, calibration):
    print('--- plot_observation_errors ---')

    Tms = calibration.convertXtoTms(calibration.getX())
    xmap = calibration.convertX2Xmap(calibration.getX())

    Y = []
    for i in range(len(sensors)):
        # Map points to each sensor
        Y.append({
            'lidar': calibration.project2lidar,
            'stereo': calibration.project2stereo,
            'mono': calibration.project2mono,
            'radar': calibration.project2radar
        }.get(calibration.sensors[i].type, 'lidar')(Tms[i], xmap, calibration.sensors[i].parameters))

    # Find min and max values
    min_e = 0
    max_e = 0
    for j in range(len(sensors)):
        visualise_error = Y[j][:, sensors[j].mu] - sensors[j].sensor_data
        min_e = np.min([np.min(visualise_error), min_e])
        max_e = np.max([np.max(visualise_error), max_e])

    # Define labels for legend
    hist_labels = {}
    hist_labels['lidar'] = ['x', 'y', 'z']
    hist_labels['stereo'] = ['x', 'y', 'z']
    hist_labels['radar'] = ['x', 'y']

    # Plot it
    binwidth = 0.005
    x = np.arange(min_e, max_e + binwidth, binwidth)

    colors = ['r', 'g', 'b']
    fig = plt.figure('Histogram observation errors', figsize=(25, 5))
    for j in range(len(sensors)):
        ax = fig.add_subplot(1, len(sensors), j + 1)

        visualise_error = Y[j][:, sensors[j].mu] - sensors[j].sensor_data
        for i in range(visualise_error.shape[0]):
            n, bins, patches = plt.hist(visualise_error[i], bins=x, alpha=0.7, edgecolor='black', weights=np.ones_like(visualise_error[i]) / float(len(visualise_error[i])), stacked=True, color=colors[i], label=hist_labels[sensors[j].type][i])

        plt.xlabel('Error [m]')
        plt.ylabel('Occurance [-]')
        plt.title(sensors[j].name)
        plt.legend()
