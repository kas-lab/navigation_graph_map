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
    navigation_task_plan_path = get_package_share_directory('navigation_task_plan')
    plansys_path = get_package_share_directory('plansys2_bringup')
    plansys2_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                plansys_path,
                'launch',
                'plansys2_bringup_launch_distributed.py')),
            launch_arguments={
                'model_file':
                    navigation_task_plan_path + '/pddl/domain.pddl',
                'problem_file':
                    navigation_task_plan_path + '/pddl/problem.pddl',
                # 'params_file':
                #     navigation_task_plan_path + '/config/plansys2_params.yaml',
            }.items()
        )

    navigation_controller_node = Node(
        package='rosa_task_plan_plansys',
        executable='rosa_plansys_controller_node',
        parameters=[{'rosa_actions': ['move']}]
    )

    waypoints_file = navigation_task_plan_path + '/config/waypoints.yaml'
    pddl_move_action_node = Node(
        package='navigation_task_plan',
        executable='action_move',
        parameters=[{'action_name': 'move'}, waypoints_file]
    )

    return LaunchDescription([
        plansys2_bringup,
        navigation_controller_node,
        pddl_move_action_node,
    ])
