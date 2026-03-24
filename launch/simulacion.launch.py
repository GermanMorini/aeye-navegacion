import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    map_tools_dir = get_package_share_directory("map_tools")
    rviz_full = os.path.join(gps_wpf_dir, "config", "rviz_nav2_full.rviz")
    rviz_local = os.path.join(gps_wpf_dir, "config", "rviz_ekf_local_tuning.rviz")
    rviz_global = os.path.join(gps_wpf_dir, "config", "rviz_ekf_global_tuning.rviz")
    keepout_mask_yaml = os.path.join(gps_wpf_dir, "config", "keepout_mask.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_conf = LaunchConfiguration("rviz_conf")
    use_keepout = LaunchConfiguration("use_keepout")
    nav_start_delay_s = LaunchConfiguration("nav_start_delay_s")
    wheelbase_m = LaunchConfiguration("wheelbase_m")
    invert_measured_steer_sign = LaunchConfiguration("invert_measured_steer_sign")
    vx_deadband_mps = LaunchConfiguration("vx_deadband_mps")
    vx_min_effective_mps = LaunchConfiguration("vx_min_effective_mps")
    invert_steer_from_cmd_vel = LaunchConfiguration("invert_steer_from_cmd_vel")
    use_cmd_vel_ackermann_bridge = LaunchConfiguration("use_cmd_vel_ackermann_bridge")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    collision_monitor_params_file = LaunchConfiguration("collision_monitor_params_file")
    keepout_mask_yaml_arg = LaunchConfiguration("keepout_mask_yaml")
    custom_urdf = LaunchConfiguration("custom_urdf")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    model_name = LaunchConfiguration("model_name")
    pose_covariance_xy = LaunchConfiguration("pose_covariance_xy")
    pose_covariance_yaw = LaunchConfiguration("pose_covariance_yaw")
    twist_covariance_vx = LaunchConfiguration("twist_covariance_vx")
    twist_covariance_vy = LaunchConfiguration("twist_covariance_vy")
    twist_covariance_yaw_rate = LaunchConfiguration("twist_covariance_yaw_rate")
    ekf_local = LaunchConfiguration("ekf_local")
    ekf_global = LaunchConfiguration("ekf_global")
    launch_web_zone_server = LaunchConfiguration("launch_web_zone_server")
    web_ws_host = LaunchConfiguration("web_ws_host")
    web_ws_port = LaunchConfiguration("web_ws_port")
    selected_rviz_config = PythonExpression(
        [
            "'",
            rviz_global,
            "' if '",
            rviz_conf,
            "' == 'global' else '",
            rviz_local,
            "' if '",
            rviz_conf,
            "' == 'local' else '",
            rviz_full,
            "'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("use_rviz", default_value="True"),
            DeclareLaunchArgument(
                "rviz_conf",
                default_value="full",
                choices=["full", "local", "global"],
            ),
            # Kept for backward compatibility; ignored in simulacion wrapper in favor of rviz_conf.
            DeclareLaunchArgument("rviz_config", default_value=rviz_full),
            DeclareLaunchArgument("use_keepout", default_value="True"),
            DeclareLaunchArgument("nav_start_delay_s", default_value="4.0"),
            DeclareLaunchArgument("wheelbase_m", default_value="0.94"),
            DeclareLaunchArgument("invert_measured_steer_sign", default_value="True"),
            DeclareLaunchArgument("vx_deadband_mps", default_value="0.01"),
            DeclareLaunchArgument("vx_min_effective_mps", default_value="0.5"),
            DeclareLaunchArgument("invert_steer_from_cmd_vel", default_value="True"),
            DeclareLaunchArgument("use_cmd_vel_ackermann_bridge", default_value="False"),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(gps_wpf_dir, "config", "nav2_no_map_params.yaml"),
            ),
            DeclareLaunchArgument(
                "collision_monitor_params_file",
                default_value=os.path.join(gps_wpf_dir, "config", "collision_monitor.yaml"),
            ),
            DeclareLaunchArgument("keepout_mask_yaml", default_value=keepout_mask_yaml),
            DeclareLaunchArgument(
                "custom_urdf",
                default_value=os.path.join(gps_wpf_dir, "models", "cuatri_real.urdf"),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(gps_wpf_dir, "worlds", "vacio.world"),
            ),
            DeclareLaunchArgument("world_name", default_value="vacio"),
            DeclareLaunchArgument("model_name", default_value="quad_ackermann_viewer_safe"),
            DeclareLaunchArgument("pose_covariance_xy", default_value="0.05"),
            DeclareLaunchArgument("pose_covariance_yaw", default_value="0.1"),
            DeclareLaunchArgument("twist_covariance_vx", default_value="0.05"),
            DeclareLaunchArgument("twist_covariance_vy", default_value="0.01"),
            DeclareLaunchArgument("twist_covariance_yaw_rate", default_value="0.1"),
            DeclareLaunchArgument("ekf_local", default_value="True"),
            DeclareLaunchArgument("ekf_global", default_value="False"),
            DeclareLaunchArgument("launch_web_zone_server", default_value="True"),
            DeclareLaunchArgument("web_ws_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_ws_port", default_value="8766"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gps_wpf_dir, "launch", "sim_local_v2.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_rviz": use_rviz,
                    "rviz_config": selected_rviz_config,
                    "use_keepout": use_keepout,
                    "nav_start_delay_s": nav_start_delay_s,
                    "wheelbase_m": wheelbase_m,
                    "invert_measured_steer_sign": invert_measured_steer_sign,
                    "vx_deadband_mps": vx_deadband_mps,
                    "vx_min_effective_mps": vx_min_effective_mps,
                    "invert_steer_from_cmd_vel": invert_steer_from_cmd_vel,
                    "use_cmd_vel_ackermann_bridge": use_cmd_vel_ackermann_bridge,
                    "nav2_params_file": nav2_params_file,
                    "collision_monitor_params_file": collision_monitor_params_file,
                    "keepout_mask_yaml": keepout_mask_yaml_arg,
                    "custom_urdf": custom_urdf,
                    "world": world,
                    "world_name": world_name,
                    "model_name": model_name,
                    "pose_covariance_xy": pose_covariance_xy,
                    "pose_covariance_yaw": pose_covariance_yaw,
                    "twist_covariance_vx": twist_covariance_vx,
                    "twist_covariance_vy": twist_covariance_vy,
                    "twist_covariance_yaw_rate": twist_covariance_yaw_rate,
                    "ekf_local": ekf_local,
                    "ekf_global": ekf_global,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(map_tools_dir, "launch", "no_go_editor.launch.py")
                ),
                launch_arguments={
                    "ws_host": web_ws_host,
                    "ws_port": web_ws_port,
                    "gps_topic": "/gps/fix",
                    "map_frame": "odom",
                    "launch_nav_command_server": "false",
                    "launch_zones_manager": launch_web_zone_server,
                    "launch_nav_snapshot_server": launch_web_zone_server,
                }.items(),
            ),
        ]
    )
