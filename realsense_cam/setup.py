# Copyright 2024 nathan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'realsense_cam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include model files
        (os.path.join('share', package_name, 'meshes'),
         glob(os.path.join('meshes', '*.stl')) +
         glob(os.path.join('meshes', '*.dae'))),
        # Include URDF (.urdf) files
        (os.path.join('share', package_name, 'urdf'),
         glob(os.path.join('urdf', '*.urdf.xacro*')) +
         glob(os.path.join('urdf', '*.gazebo.xacro*'))),
        # Include rviz (.rviz) files
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.rviz'))),
        # Include world (.sdf) files
        (os.path.join('share', package_name, 'worlds'),
         glob(os.path.join('worlds', '*.sdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nathan',
    maintainer_email='nathanshankar465@gmail.com',
    description='ROS 2 package for RealSense camera simulation in Gazebo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
