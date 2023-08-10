# Copyright 2020 TaiTing Tsai <errrr0501done@gmail.com>
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
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_name = "mobile_dual_arm"
    package_name = robot_name + "_description"
    rviz_config = os.path.join(get_package_share_directory(
        package_name), "launch", robot_name + ".rviz")
    robot_description = os.path.join(get_package_share_directory(
        package_name), "urdf", robot_name + "_sucker"+ ".xacro")
    robot_description_config = Command(['xacro ', robot_description, ' use_fake_hardware:=', use_fake_hardware])
    xacro_params = {'robot_description': ParameterValue(robot_description_config, value_type=str), 'use_fake_hardware': use_fake_hardware}   

    controller_config = os.path.join(
        get_package_share_directory(
            package_name), "controllers", "sucker_controllers.yaml"
    )


    left_controller_config = os.path.join(
        get_package_share_directory(
            package_name), "controllers", "left_controllers.yaml"
    )
    right_controller_config = os.path.join(
        get_package_share_directory(
            package_name), "controllers", "right_controllers.yaml"
    )

    timda_body_controller_config = os.path.join(
        get_package_share_directory(
            'timda_body'), "config", "timda_body_controller.yaml"
    )

    mir_controller_config = os.path.join(
        get_package_share_directory(
            'mir_description'), "config", "caster_controllers.yaml"
    )

    mobile_dual_arm_control_node =  Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[xacro_params, controller_config],
        output="screen",
        )  

    robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[xacro_params],
            output="screen",
        )  

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value="true",
            description='Flag to use fake hardware time'), 

        mobile_dual_arm_control_node,

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller", "-c", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            output="screen",
        ),


        robot_state_publisher_node,

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )

    ])
