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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    schema_path = LaunchConfiguration('schema_path')
    data_path = LaunchConfiguration('data_path')
    database_name = LaunchConfiguration('database_name')
    address = LaunchConfiguration('address')
    force_data = LaunchConfiguration('force_data')
    force_database = LaunchConfiguration('force_database')
    infer = LaunchConfiguration('infer')

    pkg_ros_typedb = get_package_share_directory('ros_typedb')
    pkg_navigation_kb = get_package_share_directory('navigation_kb')

    default_schema_path = "[{0}]".format(
        os.path.join(pkg_navigation_kb, 'config', 'schema.tql'))

    default_data_path = "[{0}]".format(
        os.path.join(pkg_navigation_kb, 'config', 'data.tql'))

    schema_path_arg = DeclareLaunchArgument(
        'schema_path',
        default_value=default_schema_path,
        description='path for KB schema'
    )

    data_path_arg = DeclareLaunchArgument(
        'data_path',
        default_value=default_data_path,
        description='path for KB data'
    )

    database_name_arg = DeclareLaunchArgument(
        'database_name',
        default_value='navigation_kb',
        description='database name'
    )

    address_arg = DeclareLaunchArgument(
        'address',
        default_value='localhost:1729',
        description='typedb server address'
    )

    force_data_arg = DeclareLaunchArgument(
        'force_data',
        default_value='True',
        description='force data'
    )

    force_database_arg = DeclareLaunchArgument(
        'force_database',
        default_value='True',
        description='force database'
    )

    infer_arg = DeclareLaunchArgument(
        'infer',
        default_value='True',
        description='use inference engine'
    )

    ros_typedb_launch_path = os.path.join(
        pkg_ros_typedb, 'launch', 'ros_typedb.launch.py')
    navigation_kb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_typedb_launch_path),
        launch_arguments={
            'schema_path': schema_path,
            'data_path': data_path,
            'database_name': database_name,
            'address': address,
            'force_data': force_data,
            'force_database': force_database,
            'infer': infer,
        }.items()
    )

    return LaunchDescription([
        schema_path_arg,
        data_path_arg,
        database_name_arg,
        address_arg,
        force_data_arg,
        force_database_arg,
        infer_arg,
        navigation_kb,
    ])
