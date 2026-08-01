# Copyright (c) 2023 Óscar Pons Fernández
# Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
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

"""Launches a the object with region node."""

import os

from ament_index_python import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg


def generate_launch_description():
    # Getting directories and launch-files
    object_with_region_dir = get_package_share_directory(
        'object_with_region')
    default_params_file = os.path.join(
        object_with_region_dir, 'params', 'params.yaml')

    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file with detection '
                     'configuration'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        name='log-level',
        default_value='info',
        description='Logging level (info, debug, ...)'
    )

    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically configure and activate '
                     'object_with_region on launch'
    )

    # Prepare the ROS2 lifecycle node.
    object_with_region_node = LifecycleNode(
        package='object_with_region',
        namespace='',
        executable='object_with_region_node',
        name='object_with_region',
        parameters=[params_file],
        emulate_tty=True,
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level',
            ['object_with_region:=', LaunchConfiguration('log-level')]]
    )

    # object_with_region_node is a managed lifecycle node: it does nothing
    # until it is configured and activated. Drive that state machine from
    # here, entirely outside the node's own code, so it can be disabled
    # (e.g. when an external lifecycle manager is used instead) by simply
    # setting autostart to False.
    configure_on_start = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(
                object_with_region_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=object_with_region_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            object_with_region_node),
                        transition_id=(
                            lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                        ),
                    ),
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        declare_params_file_arg,
        declare_log_level_arg,
        declare_autostart_arg,
        object_with_region_node,
        activate_on_inactive,
        configure_on_start,
    ])
