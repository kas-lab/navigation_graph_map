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
    db_name = LaunchConfiguration('db_name')
    data_path = LaunchConfiguration('data_path')

    db_name_arg = DeclareLaunchArgument(
        'db_name',
        default_value='navigation_rosa',
        description='ROSA db name'
    )

    pkg_navigation_rosa = get_package_share_directory(
        'navigation_rosa')
    data_path_ = "[{}]".format(
        os.path.join(pkg_navigation_rosa, 'config', 'navigation_rosa.tql'))
    data_path_arg = DeclareLaunchArgument(
        'data_path',
        default_value=data_path_,
        description='typeQL file to use for data'
    )

    pkg_rosa_kb = get_package_share_directory(
        'rosa_kb')

    pkg_rosa_bringup = get_package_share_directory(
        'rosa_bringup')
    rosa_bringup_launch_path = os.path.join(
        pkg_rosa_bringup,
        'launch',
        'rosa_bringup.launch.py')

    schema_path = "[{0}, {1}]".format(
        os.path.join(pkg_rosa_kb, 'config', 'schema.tql'),
        os.path.join(pkg_rosa_kb, 'config', 'ros_schema.tql'))

    rosa_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rosa_bringup_launch_path),
        launch_arguments={
            'schema_path': schema_path,
            'data_path': data_path,
            'database_name': db_name,
            'force_data': 'True',
            'force_database': 'True',
            'infer': 'True',
        }.items()
    )

    return LaunchDescription([
        db_name_arg,
        data_path_arg,
        rosa_bringup,
    ])
