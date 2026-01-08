# Copyright (c) 2018 Intel Corporation
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _read_file(path):
    with open(path, "r", encoding="utf-8") as file_handle:
        return file_handle.read()


def _launch_gazebo(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    custom_urdf = LaunchConfiguration("custom_urdf").perform(context)

    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    world_path = os.path.join(tb3_gazebo_dir, "worlds", "turtlebot3_world.world")
    robot_description = _read_file(custom_urdf)

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

    spawn_custom = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            "custom_robot",
            "-file",
            custom_urdf,
            "-x",
            "-4.0",
            "-y",
            "0",
            "-z",
            "0.2",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time == "True",
                "robot_description": robot_description,
            }
        ],
    )

    return [gazebo_server, gazebo_client, spawn_custom, robot_state_publisher]


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav2_bringup")
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    launch_dir = os.path.join(gps_wpf_dir, "launch")
    params_dir = os.path.join(gps_wpf_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_mapviz = LaunchConfiguration("use_mapviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    custom_urdf = LaunchConfiguration("custom_urdf")
    use_ackermann_converter = LaunchConfiguration("use_ackermann_converter")
    rviz_config = LaunchConfiguration("rviz_config")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_custom_urdf_cmd = DeclareLaunchArgument(
        "custom_urdf",
        default_value=os.path.join(gps_wpf_dir, "urdf", "modelo.urdf"),
        description="Path to custom URDF to spawn in Gazebo",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Whether to start RVIZ",
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(params_dir, "rviz_nav2_full.rviz"),
        description="Path to the RViz config file",
    )
    declare_use_mapviz_cmd = DeclareLaunchArgument(
        "use_mapviz",
        default_value="False",
        description="Whether to start mapviz",
    )
    declare_use_ackermann_converter_cmd = DeclareLaunchArgument(
        "use_ackermann_converter",
        default_value="False",
        description="Whether to convert /cmd_vel Twist to Ackermann commands",
    )

    ackermann_converter_cmd = Node(
        package="navegacion_gps",
        executable="twist_to_ackermann",
        name="twist_to_ackermann",
        output="screen",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/cmd_vel_steer",
                "output_type": "twist",
                "wheelbase": 0.60,
                "steering_limit": 0.5235987756,
            }
        ],
        condition=IfCondition(use_ackermann_converter),
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "dual_ekf_navsat.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(use_rviz),
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "mapviz.launch.py")
        ),
        condition=IfCondition(use_mapviz),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_custom_urdf_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_ackermann_converter_cmd)
    ld.add_action(OpaqueFunction(function=_launch_gazebo))
    ld.add_action(robot_localization_cmd)
    ld.add_action(navigation2_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(mapviz_cmd)
    ld.add_action(ackermann_converter_cmd)

    return ld
