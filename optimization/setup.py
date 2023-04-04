#!/usr/bin/python3.5

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

from setuptools import setup

package_name = 'optimization'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='jfmdomhof',
    author_email='j.f.m.domhof@tudelft.nl',
    maintainer='jfmdomhof',
    maintainer_email='j.f.m.domhof@tudelft.nl',
    keywords=['ROS'],
    description='The optimization package',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = optimization.server:main',
        ],
    },
)
