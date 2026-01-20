# Copyright 2018 Open Source Robotics Foundation, Inc.
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav2_bringup")
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    launch_dir = os.path.join(gps_wpf_dir, "launch")
    params_dir = os.path.join(gps_wpf_dir, "config")
    collision_monitor_params = os.path.join(params_dir, "collision_monitor.yaml")
    lidar_to_scan_params = os.path.join(params_dir, "pointcloud_to_laserscan.yaml")
    bt_xml = os.path.join(
        params_dir, "navigate_to_pose_w_replanning_and_recovery_no_spin.xml"
    )
    bt_through_poses_xml = os.path.join(
        params_dir, "navigate_through_poses_w_replanning_and_recovery_no_spin.xml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_navsat = LaunchConfiguration("use_navsat")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_mapviz = LaunchConfiguration("use_mapviz")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    params_file = LaunchConfiguration("params_file")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation clock if true",
    )
    declare_use_navsat_cmd = DeclareLaunchArgument(
        "use_navsat",
        default_value="True",
        description="Whether to start navsat_transform_node",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
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
    declare_use_collision_monitor_cmd = DeclareLaunchArgument(
        "use_collision_monitor",
        default_value="True",
        description="Whether to start collision monitor",
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(params_dir, "nav2_no_map_params.yaml"),
        description="Path to the Nav2 parameters file",
    )

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "default_nav_to_pose_bt_xml": bt_xml,
            "default_nav_through_poses_bt_xml": bt_through_poses_xml,
        },
        convert_types=True,
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "dual_ekf_navsat.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_navsat": use_navsat,
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
        arguments=[
            "-d",
            rviz_config,
            "--ros-args",
            "-p",
            PythonExpression(["'use_sim_time:=' + str(", use_sim_time, ")"]),
        ],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(use_rviz),
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "mapviz.launch.py")
        ),
        condition=IfCondition(use_mapviz),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    collision_monitor_cmd = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            collision_monitor_params,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(use_collision_monitor),
    )
    collision_monitor_lifecycle_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="collision_monitor_lifecycle_manager",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "autostart": True,
                "node_names": ["collision_monitor"],
            }
        ],
        condition=IfCondition(use_collision_monitor),
    )
    lidar_to_scan_cmd = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            lidar_to_scan_params,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"output_qos": "sensor_data"},
        ],
        remappings=[
            ("cloud_in", "/scan_3d"),
            ("scan", "/scan"),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_navsat_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(declare_use_collision_monitor_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(robot_localization_cmd)
    ld.add_action(navigation2_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(mapviz_cmd)
    ld.add_action(collision_monitor_cmd)
    ld.add_action(lidar_to_scan_cmd)
    ld.add_action(collision_monitor_lifecycle_cmd)

    return ld
