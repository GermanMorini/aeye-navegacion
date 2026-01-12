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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _read_file(path):
    with open(path, "r", encoding="utf-8") as file_handle:
        return file_handle.read()


def _spawn_robot(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    custom_urdf = LaunchConfiguration("custom_urdf").perform(context)
    model_name = LaunchConfiguration("model_name").perform(context)
    use_sim_time_bool = use_sim_time == "True"

    robot_description = _read_file(custom_urdf)

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

    spawn_custom = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            model_name,
            "-file",
            custom_urdf,
            "-x",
            "0.0",
            "-y",
            "0",
            "-z",
            "0.2",
        ],
    )

    return [robot_state_publisher, spawn_custom]


def _build_gz_bridge(context, *, bridge_config):
    bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )
    return [bridge_cmd]


def _build_joint_state_bridge(context):
    use_joint_state_bridge = LaunchConfiguration("use_joint_state_bridge").perform(context)
    if use_joint_state_bridge.lower() != "true":
        return []

    world_name = LaunchConfiguration("world_name").perform(context)
    model_name = LaunchConfiguration("model_name").perform(context)
    joint_state_topic = (
        f"/world/{world_name}/model/{model_name}/joint_state"
        "@sensor_msgs/msg/JointState[gz.msgs.Model"
    )

    joint_state_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[joint_state_topic],
    )
    return [joint_state_bridge_cmd]


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav2_bringup")
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    launch_dir = os.path.join(gps_wpf_dir, "launch")
    params_dir = os.path.join(gps_wpf_dir, "config")
    world_path = os.path.join(gps_wpf_dir, "worlds", "pasillos_obstaculos.world")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    collision_monitor_params = os.path.join(params_dir, "collision_monitor.yaml")
    bridge_config = os.path.join(params_dir, "bridge_config.yaml")
    bt_xml = os.path.join(
        params_dir, "navigate_to_pose_w_replanning_and_recovery_no_spin.xml"
    )
    bt_through_poses_xml = os.path.join(
        params_dir, "navigate_through_poses_w_replanning_and_recovery_no_spin.xml"
    )
    configured_params = RewrittenYaml(
        source_file=nav2_params,
        root_key="",
        param_rewrites={
            "default_nav_to_pose_bt_xml": bt_xml,
            "default_nav_through_poses_bt_xml": bt_through_poses_xml,
        },
        convert_types=True,
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_mapviz = LaunchConfiguration("use_mapviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ackermann_converter = LaunchConfiguration("use_ackermann_converter")
    use_navsat = LaunchConfiguration("use_navsat")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    use_frame_id_stripper = LaunchConfiguration("use_frame_id_stripper")
    rviz_config = LaunchConfiguration("rviz_config")
    use_joint_state_bridge = LaunchConfiguration("use_joint_state_bridge")
    world_name = LaunchConfiguration("world_name")
    model_name = LaunchConfiguration("model_name")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_custom_urdf_cmd = DeclareLaunchArgument(
        "custom_urdf",
        default_value=os.path.join(gps_wpf_dir, "urdf", "cuatri.urdf"),
        description="Path to custom URDF to spawn in Gazebo Sim",
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
    declare_use_ackermann_converter_cmd = DeclareLaunchArgument(
        "use_ackermann_converter",
        default_value="True",
        description="Whether to convert /cmd_vel Twist to Ackermann commands",
    )
    declare_use_navsat_cmd = DeclareLaunchArgument(
        "use_navsat",
        default_value="True",
        description="Whether to start navsat_transform_node",
    )
    declare_use_collision_monitor_cmd = DeclareLaunchArgument(
        "use_collision_monitor",
        default_value="False",
        description="Whether to start collision monitor",
    )
    declare_use_frame_id_stripper_cmd = DeclareLaunchArgument(
        "use_frame_id_stripper",
        default_value="True",
        description="Whether to strip model prefixes from sensor and odom frame_ids",
    )
    declare_use_joint_state_bridge_cmd = DeclareLaunchArgument(
        "use_joint_state_bridge",
        default_value="False",
        description="Whether to bridge Gazebo joint_state into ROS",
    )
    declare_world_name_cmd = DeclareLaunchArgument(
        "world_name",
        default_value="pasillos_obstaculos",
        description="Gazebo world name for joint_state bridge",
    )
    declare_model_name_cmd = DeclareLaunchArgument(
        "model_name",
        default_value="quad_ackermann_viewer_safe",
        description="Gazebo model name for TF/joint_state topics",
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
                "wheelbase": 0.94,
                "steering_limit": 0.5235987756,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
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

    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
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
    frame_id_stripper_cmd = Node(
        package="navegacion_gps",
        executable="frame_id_stripper",
        name="frame_id_stripper",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"imu_in_topic": "/imu/data_raw", "imu_out_topic": "/imu/data"},
            {"gps_in_topic": "/gps/fix_raw", "gps_out_topic": "/gps/fix"},
            {"lidar_in_topic": "/scan_3d_raw", "lidar_out_topic": "/scan_3d"},
            {"odom_in_topic": "/odom_raw", "odom_out_topic": "/odom"},
            {"imu_frame_id": "imu_link"},
            {"gps_frame_id": "gps_link"},
            {"lidar_frame_id": "lidar_link"},
            {"odom_frame_id": "odom"},
            {"base_link_frame_id": "base_footprint"},
        ],
        condition=IfCondition(use_frame_id_stripper),
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

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_custom_urdf_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_ackermann_converter_cmd)
    ld.add_action(declare_use_navsat_cmd)
    ld.add_action(declare_use_collision_monitor_cmd)
    ld.add_action(declare_use_frame_id_stripper_cmd)
    ld.add_action(declare_use_joint_state_bridge_cmd)
    ld.add_action(declare_world_name_cmd)
    ld.add_action(declare_model_name_cmd)
    ld.add_action(gz_sim_cmd)
    ld.add_action(OpaqueFunction(function=_build_gz_bridge, kwargs={"bridge_config": bridge_config}))
    ld.add_action(OpaqueFunction(function=_build_joint_state_bridge))
    ld.add_action(OpaqueFunction(function=_spawn_robot))
    ld.add_action(robot_localization_cmd)
    ld.add_action(navigation2_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(mapviz_cmd)
    ld.add_action(ackermann_converter_cmd)
    ld.add_action(collision_monitor_cmd)
    ld.add_action(frame_id_stripper_cmd)
    ld.add_action(collision_monitor_lifecycle_cmd)

    return ld
