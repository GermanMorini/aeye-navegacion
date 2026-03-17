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
import re
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _read_file(path):
    with open(path, "r", encoding="utf-8") as file_handle:
        return file_handle.read()


def _resolve_config_file_path(package_share_dir: str, filename: str) -> str:
    package_share_path = Path(package_share_dir)
    default_path = package_share_path / "config" / filename
    try:
        workspace_root = package_share_path.parents[3]
        source_path = workspace_root / "src" / "navegacion_gps" / "config" / filename
        if source_path.parent.exists():
            return str(source_path)
    except IndexError:
        pass
    return str(default_path)


def _extract_world_name_from_sdf(world_path: str) -> str:
    try:
        with open(world_path, "r", encoding="utf-8") as file_handle:
            world_contents = file_handle.read()
        match = re.search(r"<world\s+name\s*=\s*['\"]([^'\"]+)['\"]", world_contents)
        if match:
            return match.group(1)
    except OSError:
        pass
    return ""


def _materialize_bridge_config_for_world(bridge_config_path: str, world_name: str) -> str:
    with open(bridge_config_path, "r", encoding="utf-8") as file_handle:
        bridge_config_contents = file_handle.read()
    # Keep one bridge config for all worlds by replacing world-scoped Gazebo topics at runtime.
    patched_contents = re.sub(
        r"/world/[^/\s]+/",
        f"/world/{world_name}/",
        bridge_config_contents,
    )
    tmp_file = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        suffix=".yaml",
        prefix="bridge_config_",
        delete=False,
    )
    with tmp_file:
        tmp_file.write(patched_contents)
    return tmp_file.name


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
    world_path = LaunchConfiguration("world").perform(context)
    world_name_arg = LaunchConfiguration("world_name").perform(context)
    world_name_from_sdf = _extract_world_name_from_sdf(world_path)
    resolved_world_name = world_name_from_sdf or world_name_arg
    resolved_bridge_config = _materialize_bridge_config_for_world(
        bridge_config, resolved_world_name
    )

    bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": resolved_bridge_config}],
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
    map_tools_dir = get_package_share_directory("map_tools")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    params_dir = os.path.join(gps_wpf_dir, "config")
    zones_geojson_path = _resolve_config_file_path(gps_wpf_dir, "no_go_zones.geojson")
    keepout_mask_image_path = _resolve_config_file_path(gps_wpf_dir, "keepout_mask.pgm")
    keepout_mask_yaml_path = _resolve_config_file_path(gps_wpf_dir, "keepout_mask.yaml")
    world_path = os.path.join(gps_wpf_dir, "worlds", "vacio.world")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    rl_params_file = os.path.join(params_dir, "dual_ekf_navsat_params.yaml")
    collision_monitor_params = os.path.join(params_dir, "collision_monitor.yaml")
    lidar_to_scan_params = os.path.join(params_dir, "pointcloud_to_laserscan.yaml")
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
    use_navsat = LaunchConfiguration("use_navsat")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    use_gazebo_utils = LaunchConfiguration("use_gazebo_utils")
    rviz_config = LaunchConfiguration("rviz_config")
    world = LaunchConfiguration("world")
    ws_host = LaunchConfiguration("ws_host")
    ws_port = LaunchConfiguration("ws_port")
    gps_topic = LaunchConfiguration("gps_topic")
    map_frame = LaunchConfiguration("map_frame")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_custom_urdf_cmd = DeclareLaunchArgument(
        "custom_urdf",
        default_value=os.path.join(gps_wpf_dir, "models", "cuatri_real.urdf"),
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
    declare_use_navsat_cmd = DeclareLaunchArgument(
        "use_navsat",
        default_value="True",
        description="Whether to start navsat_transform_node",
    )
    declare_use_collision_monitor_cmd = DeclareLaunchArgument(
        "use_collision_monitor",
        default_value="True",
        description="Whether to start collision monitor",
    )
    declare_use_gazebo_utils_cmd = DeclareLaunchArgument(
        "use_gazebo_utils",
        default_value="True",
        description="Whether to run gazebo_utils (frame normalization + cmd bridge)",
    )
    declare_use_joint_state_bridge_cmd = DeclareLaunchArgument(
        "use_joint_state_bridge",
        default_value="False",
        description="Whether to bridge Gazebo joint_state into ROS",
    )
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=world_path,
        description="Path to the Gazebo world file",
    )
    declare_world_name_cmd = DeclareLaunchArgument(
        "world_name",
        default_value="vacio",
        description="Gazebo world name for joint_state bridge",
    )
    declare_model_name_cmd = DeclareLaunchArgument(
        "model_name",
        default_value="quad_ackermann_viewer_safe",
        description="Gazebo model name for TF/joint_state topics",
    )
    declare_ws_host_cmd = DeclareLaunchArgument(
        "ws_host",
        default_value="0.0.0.0",
        description="WebSocket host for map_tools web gateway",
    )
    declare_ws_port_cmd = DeclareLaunchArgument(
        "ws_port",
        default_value="8766",
        description="WebSocket port for map_tools web gateway",
    )
    declare_gps_topic_cmd = DeclareLaunchArgument(
        "gps_topic",
        default_value="/gps/fix",
        description="GPS topic used by web console backend/gateway",
    )
    declare_map_frame_cmd = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="Global map frame for navigation web backend",
    )

    ekf_odom_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[
            rl_params_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[("odometry/filtered", "odometry/local")],
    )
    ekf_map_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[
            rl_params_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )
    navsat_transform_cmd = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[
            rl_params_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[
            ("imu/data", "imu/data"),
            ("gps/fix", "gps/fix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/local"),
        ],
        condition=IfCondition(use_navsat),
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
    keepout_filter_mask_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        name="keepout_filter_mask_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": keepout_mask_yaml_path,
                "topic_name": "/keepout_filter_mask",
                "frame_id": map_frame,
            },
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )
    keepout_costmap_filter_info_server_cmd = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="keepout_costmap_filter_info_server",
        output="screen",
        parameters=[
            {
                "type": 0,
                "filter_info_topic": "/costmap_filter_info",
                "mask_topic": "/keepout_filter_mask",
                "base": 0.0,
                "multiplier": 1.0,
            },
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )
    keepout_lifecycle_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_keepout_filters",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "bond_timeout": 4.0,
                "node_names": [
                    "keepout_filter_mask_server",
                    "keepout_costmap_filter_info_server",
                ],
            },
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )
    zones_manager_cmd = Node(
        package="navegacion_gps",
        executable="zones_manager",
        name="zones_manager",
        output="screen",
        parameters=[
            {
                "fromll_service": "/fromLL",
                "fromll_service_fallback": "/navsat_transform/fromLL",
                "fromll_wait_timeout_s": 2.0,
                "load_map_service": "/keepout_filter_mask_server/load_map",
                "set_geojson_service": "/zones_manager/set_geojson",
                "get_state_service": "/zones_manager/get_state",
                "reload_from_disk_service": "/zones_manager/reload_from_disk",
                "map_frame": map_frame,
                "geojson_file": zones_geojson_path,
                "mask_image_file": keepout_mask_image_path,
                "mask_yaml_file": keepout_mask_yaml_path,
                "buffer_margin_m": 0.8,
                "degrade_enabled": True,
                "degrade_radius_m": 1.5,
                "degrade_edge_cost": 40,
                "degrade_min_cost": 1,
                "degrade_use_l2": True,
                "mask_origin_mode": "explicit",
                "mask_origin_x": -150.0,
                "mask_origin_y": -150.0,
                "mask_width": 3000,
                "mask_height": 3000,
                "mask_resolution": 0.1,
            }
        ],
    )
    nav_command_server_cmd = Node(
        package="navegacion_gps",
        executable="nav_command_server",
        name="nav_command_server",
        output="screen",
        parameters=[
            {
                "fromll_service": "/fromLL",
                "fromll_service_fallback": "/navsat_transform/fromLL",
                "fromll_wait_timeout_s": 2.0,
                "map_frame": map_frame,
                "gps_topic": gps_topic,
                "cmd_vel_safe_topic": "/cmd_vel_safe",
                "brake_topic": "/cmd_vel_safe",
                "manual_cmd_topic": "/cmd_vel_safe",
                "teleop_cmd_topic": "/cmd_vel_teleop",
                "brake_publish_count": 5,
                "brake_publish_interval_s": 0.1,
                "manual_cmd_timeout_s": 0.4,
                "manual_watchdog_hz": 10.0,
                "nav_telemetry_hz": 5.0,
                "telemetry_topic": "/nav_command_server/telemetry",
                "set_goal_service": "/nav_command_server/set_goal_ll",
                "cancel_goal_service": "/nav_command_server/cancel_goal",
                "brake_service": "/nav_command_server/brake",
                "set_manual_mode_service": "/nav_command_server/set_manual_mode",
                "get_state_service": "/nav_command_server/get_state",
            }
        ],
    )
    nav_snapshot_server_cmd = Node(
        package="navegacion_gps",
        executable="nav_snapshot_server",
        name="nav_snapshot_server",
        output="screen",
        parameters=[
            {
                "get_snapshot_service": "/nav_snapshot_server/get_nav_snapshot",
                "local_costmap_topic": "/local_costmap/costmap",
                "global_costmap_topic": "/global_costmap/costmap",
                "keepout_mask_topic": "/keepout_filter_mask",
                "local_footprint_topic": "/local_costmap/published_footprint",
                "stop_zone_topic": "/stop_zone",
                "collision_polygons_topic": "/collision_monitor/polygons",
                "scan_topic": "/scan",
                "plan_topic": "/plan",
                "base_frame": "base_footprint",
                "snapshot_extent_m": 30.0,
                "snapshot_size_px": 512,
                "snapshot_global_inset_px": 160,
                "snapshot_timeout_ms": 500,
            }
        ],
    )
    no_go_editor_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(map_tools_dir, "launch", "no_go_editor.launch.py")
        ),
        launch_arguments={
            "ws_host": ws_host,
            "ws_port": ws_port,
            "gps_topic": gps_topic,
            "map_frame": map_frame,
            "launch_zones_manager": "false",
            "launch_nav_command_server": "false",
            "launch_nav_snapshot_server": "false",
            "teleop_cmd_topic": "/cmd_vel_teleop",
            "zones_set_geojson_service": "/zones_manager/set_geojson",
            "zones_get_state_service": "/zones_manager/get_state",
            "zones_reload_service": "/zones_manager/reload_from_disk",
        }.items(),
    )

    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [TextSubstitution(text="-r "), world]}.items(),
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

    mapviz_cmd = Node(
        package="mapviz",
        executable="mapviz",
        name="mapviz",
        output="screen",
        condition=IfCondition(use_mapviz),
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
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
    gazebo_utils_cmd = Node(
        package="navegacion_gps",
        executable="gazebo_utils",
        name="gazebo_utils",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"imu_in_topic": "/imu/data_raw", "imu_out_topic": "/imu/data"},
            {"gps_in_topic": "/gps/fix_raw", "gps_out_topic": "/gps/fix"},
            {"lidar_in_topic": "/scan_3d_raw", "lidar_out_topic": "/scan_3d"},
            {
                "ultrasound_rear_center_in_topic": "/ultrasound/rear_center_raw",
                "ultrasound_rear_center_out_topic": "/ultrasound/rear_center",
            },
            {
                "ultrasound_rear_left_in_topic": "/ultrasound/rear_left_raw",
                "ultrasound_rear_left_out_topic": "/ultrasound/rear_left",
            },
            {
                "ultrasound_rear_right_in_topic": "/ultrasound/rear_right_raw",
                "ultrasound_rear_right_out_topic": "/ultrasound/rear_right",
            },
            {
                "ultrasound_front_left_in_topic": "/ultrasound/front_left_raw",
                "ultrasound_front_left_out_topic": "/ultrasound/front_left",
            },
            {
                "ultrasound_front_right_in_topic": "/ultrasound/front_right_raw",
                "ultrasound_front_right_out_topic": "/ultrasound/front_right",
            },
            {"odom_in_topic": "/odom_raw", "odom_out_topic": "/odom"},
            {"imu_frame_id": "imu_link"},
            {"gps_frame_id": "gps_link"},
            {"lidar_frame_id": "lidar_link"},
            {"ultrasound_rear_center_frame_id": "rear_center_ultrasound"},
            {"ultrasound_rear_left_frame_id": "rear_left_ultrasound"},
            {"ultrasound_rear_right_frame_id": "rear_right_ultrasound"},
            {"ultrasound_front_left_frame_id": "front_left_ultrasound"},
            {"ultrasound_front_right_frame_id": "front_right_ultrasound"},
            {"odom_frame_id": "odom"},
            {"base_link_frame_id": "base_footprint"},
            {"enable_cmd_vel_final_bridge": True},
            {"cmd_vel_final_in_topic": "/cmd_vel_final"},
            {"cmd_vel_gazebo_out_topic": "/cmd_vel_gazebo"},
        ],
        condition=IfCondition(use_gazebo_utils),
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
    ld.add_action(declare_custom_urdf_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_navsat_cmd)
    ld.add_action(declare_use_collision_monitor_cmd)
    ld.add_action(declare_use_gazebo_utils_cmd)
    ld.add_action(declare_use_joint_state_bridge_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_world_name_cmd)
    ld.add_action(declare_model_name_cmd)
    ld.add_action(declare_ws_host_cmd)
    ld.add_action(declare_ws_port_cmd)
    ld.add_action(declare_gps_topic_cmd)
    ld.add_action(declare_map_frame_cmd)
    ld.add_action(gz_sim_cmd)
    ld.add_action(
        OpaqueFunction(
            function=_build_gz_bridge,
            kwargs={"bridge_config": bridge_config},
        )
    )
    ld.add_action(OpaqueFunction(function=_build_joint_state_bridge))
    ld.add_action(OpaqueFunction(function=_spawn_robot))
    ld.add_action(ekf_odom_cmd)
    ld.add_action(ekf_map_cmd)
    ld.add_action(navsat_transform_cmd)
    ld.add_action(navigation2_cmd)
    ld.add_action(keepout_filter_mask_server_cmd)
    ld.add_action(keepout_costmap_filter_info_server_cmd)
    ld.add_action(keepout_lifecycle_cmd)
    ld.add_action(zones_manager_cmd)
    ld.add_action(nav_command_server_cmd)
    ld.add_action(nav_snapshot_server_cmd)
    ld.add_action(no_go_editor_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(mapviz_cmd)
    ld.add_action(collision_monitor_cmd)
    ld.add_action(gazebo_utils_cmd)
    ld.add_action(lidar_to_scan_cmd)
    ld.add_action(collision_monitor_lifecycle_cmd)

    return ld
