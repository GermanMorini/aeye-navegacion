import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    default_rviz = os.path.join(gps_wpf_dir, "config", "rviz_global_v2.rviz")
    default_world = os.path.join(gps_wpf_dir, "worlds", "vacio.world")
    nav2_params_file = os.path.join(gps_wpf_dir, "config", "nav2_global_v2_params.yaml")
    default_global_localization_params_file = os.path.join(
        gps_wpf_dir, "config", "global_localization_v2.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    gps_profile = LaunchConfiguration("gps_profile")
    world = LaunchConfiguration("world")
    nav_start_delay_s = LaunchConfiguration("nav_start_delay_s")
    global_localization_params_file = LaunchConfiguration(
        "global_localization_params_file"
    )
    gps_reference_mode = LaunchConfiguration("gps_reference_mode")
    gps_covariance_horizontal_stddev_override_m = LaunchConfiguration(
        "gps_covariance_horizontal_stddev_override_m"
    )
    gps_covariance_vertical_stddev_override_m = LaunchConfiguration(
        "gps_covariance_vertical_stddev_override_m"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("use_rviz", default_value="True"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("gps_profile", default_value="ideal"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("nav_start_delay_s", default_value="4.0"),
            DeclareLaunchArgument(
                "global_localization_params_file",
                default_value=default_global_localization_params_file,
            ),
            DeclareLaunchArgument("gps_reference_mode", default_value="ideal_from_local_odom"),
            DeclareLaunchArgument("gps_covariance_horizontal_stddev_override_m", default_value="0.0"),
            DeclareLaunchArgument("gps_covariance_vertical_stddev_override_m", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gps_wpf_dir, "launch", "sim_nav_v2_base.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_rviz": use_rviz,
                    "rviz_config": rviz_config,
                    "gps_profile": gps_profile,
                    "world": world,
                    "nav_start_delay_s": nav_start_delay_s,
                    "gps_reference_mode": gps_reference_mode,
                    "gps_covariance_horizontal_stddev_override_m": (
                        gps_covariance_horizontal_stddev_override_m
                    ),
                    "gps_covariance_vertical_stddev_override_m": (
                        gps_covariance_vertical_stddev_override_m
                    ),
                    "use_global_localization": "True",
                    "map_frame": "map",
                    "fromll_frame": "map",
                    "use_keepout": "False",
                    "nav2_params_file": nav2_params_file,
                    "global_localization_params_file": global_localization_params_file,
                    # The synthetic GPS represents the robot base pose, not the URDF sensor offset.
                    "gps_frame_id": "base_footprint",
                }.items(),
            ),
        ]
    )
