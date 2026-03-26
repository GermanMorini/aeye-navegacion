import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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


def _build_robot_state_publisher(context):
    use_rsp = LaunchConfiguration("use_robot_state_publisher").perform(context)
    if use_rsp.lower() != "true":
        return []

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    custom_urdf = LaunchConfiguration("custom_urdf").perform(context)
    use_sim_time_bool = use_sim_time == "True"

    robot_description = _read_file(custom_urdf)

    return [
        Node(
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
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav2_bringup")
    gps_wpf_dir = get_package_share_directory("navegacion_gps")

    nav2_params = _resolve_config_file_path(gps_wpf_dir, "nav2_no_map_params.yaml")
    collision_monitor_params = _resolve_config_file_path(
        gps_wpf_dir, "collision_monitor.yaml"
    )
    keepout_mask_yaml_path = _resolve_config_file_path(gps_wpf_dir, "keepout_mask.yaml")
    rviz_default = _resolve_config_file_path(gps_wpf_dir, "rviz_nav2_full.rviz")

    bt_xml = _resolve_config_file_path(
        gps_wpf_dir, "navigate_to_pose_w_replanning_and_recovery_no_spin.xml"
    )
    bt_through_poses_xml = _resolve_config_file_path(
        gps_wpf_dir, "navigate_through_poses_w_replanning_and_recovery_no_spin.xml"
    )
    use_keepout = LaunchConfiguration("use_keepout")
    configured_params = RewrittenYaml(
        source_file=nav2_params,
        root_key="",
        param_rewrites={
            "default_nav_to_pose_bt_xml": bt_xml,
            "default_nav_through_poses_bt_xml": bt_through_poses_xml,
            "local_costmap.local_costmap.ros__parameters.keepout_filter.enabled": use_keepout,
            "global_costmap.global_costmap.ros__parameters.keepout_filter.enabled": use_keepout,
        },
        convert_types=True,
    )

    # Force real-time for Nav2 stack regardless of launch argument.
    nav2_use_sim_time = False
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_frame = LaunchConfiguration("map_frame")
    resolved_map_frame = PythonExpression(
        [
            "('",
            map_frame,
            "'.strip().lower() if '",
            map_frame,
            "'.strip().lower() not in ('', 'auto') else 'odom')",
        ]
    )
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock if true",
    )
    declare_map_frame_cmd = DeclareLaunchArgument(
        "map_frame",
        default_value="odom",
        description="Frame used by keepout filter mask (odom/map; auto resolves to odom)",
    )
    declare_use_keepout_cmd = DeclareLaunchArgument(
        "use_keepout",
        default_value="True",
        description="Enable keepout filter servers and keepout layer in Nav2 costmaps",
    )
    declare_use_collision_monitor_cmd = DeclareLaunchArgument(
        "use_collision_monitor",
        default_value="True",
        description="Whether to start collision monitor",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Whether to start RVIZ",
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_default,
        description="Path to the RViz config file",
    )
    declare_use_robot_state_publisher_cmd = DeclareLaunchArgument(
        "use_robot_state_publisher",
        default_value="False",
        description="Publish TF using robot_state_publisher",
    )
    declare_custom_urdf_cmd = DeclareLaunchArgument(
        "custom_urdf",
        default_value=os.path.join(gps_wpf_dir, "models", "cuatri_real.urdf"),
        description="Path to custom URDF for TF tree",
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "False",
            "params_file": configured_params,
            "autostart": "True",
            "use_velocity_smoother": "False",
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
                "frame_id": resolved_map_frame,
            },
            {"use_sim_time": nav2_use_sim_time},
        ],
        condition=IfCondition(use_keepout),
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
            {"use_sim_time": nav2_use_sim_time},
        ],
        condition=IfCondition(use_keepout),
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
            {"use_sim_time": nav2_use_sim_time},
        ],
        condition=IfCondition(use_keepout),
    )

    collision_monitor_cmd = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            collision_monitor_params,
            {"use_sim_time": nav2_use_sim_time},
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
                "use_sim_time": nav2_use_sim_time,
                "autostart": True,
                "node_names": ["collision_monitor"],
            }
        ],
        condition=IfCondition(use_collision_monitor),
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

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_frame_cmd)
    ld.add_action(declare_use_keepout_cmd)
    ld.add_action(declare_use_collision_monitor_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_robot_state_publisher_cmd)
    ld.add_action(declare_custom_urdf_cmd)

    ld.add_action(OpaqueFunction(function=_build_robot_state_publisher))
    ld.add_action(navigation2_cmd)
    ld.add_action(keepout_filter_mask_server_cmd)
    ld.add_action(keepout_costmap_filter_info_server_cmd)
    ld.add_action(keepout_lifecycle_cmd)
    ld.add_action(collision_monitor_cmd)
    ld.add_action(collision_monitor_lifecycle_cmd)
    ld.add_action(rviz_cmd)

    return ld
