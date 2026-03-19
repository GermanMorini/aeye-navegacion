from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    default_nav2_params = _resolve_config_file_path(gps_wpf_dir, "nav2_local_v2_params.yaml")
    default_collision_monitor_params = _resolve_config_file_path(
        gps_wpf_dir, "collision_monitor_v2.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    collision_monitor_params_file = LaunchConfiguration("collision_monitor_params_file")
    goal_pose_use_goal_orientation = LaunchConfiguration("goal_pose_use_goal_orientation")

    lifecycle_node_names = [
        "controller_server",
        "velocity_smoother",
        "collision_monitor",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument(
                "collision_monitor_params_file",
                default_value=default_collision_monitor_params,
            ),
            DeclareLaunchArgument("goal_pose_use_goal_orientation", default_value="False"),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[
                    nav2_params_file,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=[("/cmd_vel", "/cmd_vel_nav")],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[
                    nav2_params_file,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=[
                    ("/cmd_vel", "/cmd_vel_nav"),
                    ("/cmd_vel_smoothed", "/cmd_vel_smoothed"),
                ],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[
                    collision_monitor_params_file,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_local_navigation_v2",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "autostart": True,
                        "node_names": lifecycle_node_names,
                    }
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="goal_pose_to_follow_path_v2",
                name="goal_pose_to_follow_path_v2",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {
                        "use_goal_orientation": ParameterValue(
                            goal_pose_use_goal_orientation,
                            value_type=bool,
                        )
                    },
                ],
            ),
        ]
    )
