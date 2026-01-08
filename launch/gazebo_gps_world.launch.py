# Copyright (c) 2018 Intel Corporation
# Copyright (C) 2024 Stevedan Ogochukwu Omodolor Omodia
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

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _render_xacro(xacro_path, mappings):
    doc = xacro.process_file(xacro_path, mappings=mappings)
    return doc.toprettyxml()


def _launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    tb3_model = LaunchConfiguration("tb3_model").perform(context)
    use_sim_time_bool = use_sim_time == "True"

    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    tb3_desc_dir = get_package_share_directory("turtlebot3_description")

    world_path = os.path.join(tb3_gazebo_dir, "worlds", "turtlebot3_world.world")
    model_path = os.path.join(
        tb3_gazebo_dir, "models", f"turtlebot3_{tb3_model}", "model.sdf"
    )

    tb3_urdf = os.path.join(tb3_desc_dir, "urdf", f"turtlebot3_{tb3_model}.urdf")
    robot_description = _render_xacro(tb3_urdf, {"namespace": "", "prefix": ""})

    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            world_path,
            "-slibgazebo_ros_init.so",
            "-slibgazebo_ros_factory.so",
        ],
        output="screen",
    )

    gazebo_client = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
    )

    spawn_tb3 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            "tb3",
            "-file",
            model_path,
            "-x",
            "1.75",
            "-y",
            "0.0",
            "-z",
            "0.2",
        ],
        parameters=[{"use_sim_time": use_sim_time_bool}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_bool,
                "robot_description": robot_description,
            }
        ],
    )

    return [gazebo_server, gazebo_client, spawn_tb3, robot_state_publisher]


def generate_launch_description():
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_tb3_model_cmd = DeclareLaunchArgument(
        "tb3_model",
        default_value="waffle",
        description="TurtleBot3 model to spawn",
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_tb3_model_cmd)
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
