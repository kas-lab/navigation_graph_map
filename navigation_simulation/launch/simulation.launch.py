# Copyright 2024 Gustavo Rezende Silva
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    headless = LaunchConfiguration('headless')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='headless simulation'
    )

    pkg_navigation_simulation = get_package_share_directory(
        'navigation_simulation')
    navigation_params_path = os.path.join(
        pkg_navigation_simulation,
        'config',
        'navigation_params.yaml')

    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup')
    tb3_sim_launch_path = os.path.join(
        pkg_nav2_bringup,
        'launch',
        'tb3_simulation_launch.py')

    tb3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_sim_launch_path),
        launch_arguments={
            'headless': headless,
            'params_file': navigation_params_path,
        }.items()
    )

    return LaunchDescription([
        headless_arg,
        tb3_launch,
    ])
