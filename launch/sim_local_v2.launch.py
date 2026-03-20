import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    default_rviz = os.path.join(gps_wpf_dir, "config", "rviz_local_v2.rviz")
    keepout_mask_yaml = os.path.join(gps_wpf_dir, "config", "keepout_mask.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheelbase_m = LaunchConfiguration("wheelbase_m")
    nav_start_delay_s = LaunchConfiguration("nav_start_delay_s")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    vx_deadband_mps = LaunchConfiguration("vx_deadband_mps")
    vx_min_effective_mps = LaunchConfiguration("vx_min_effective_mps")
    localization_params_file = LaunchConfiguration("localization_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    collision_monitor_params_file = LaunchConfiguration("collision_monitor_params_file")
    keepout_mask_yaml_arg = LaunchConfiguration("keepout_mask_yaml")
    pose_covariance_xy = LaunchConfiguration("pose_covariance_xy")
    pose_covariance_yaw = LaunchConfiguration("pose_covariance_yaw")
    twist_covariance_vx = LaunchConfiguration("twist_covariance_vx")
    twist_covariance_vy = LaunchConfiguration("twist_covariance_vy")
    twist_covariance_yaw_rate = LaunchConfiguration("twist_covariance_yaw_rate")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("wheelbase_m", default_value="0.94"),
            DeclareLaunchArgument("nav_start_delay_s", default_value="4.0"),
            DeclareLaunchArgument("use_rviz", default_value="True"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("vx_deadband_mps", default_value="0.01"),
            DeclareLaunchArgument("vx_min_effective_mps", default_value="0.5"),
            DeclareLaunchArgument(
                "localization_params_file",
                default_value=os.path.join(gps_wpf_dir, "config", "localization_v2.yaml"),
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(gps_wpf_dir, "config", "nav2_local_v2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "collision_monitor_params_file",
                default_value=os.path.join(gps_wpf_dir, "config", "collision_monitor_v2.yaml"),
            ),
            DeclareLaunchArgument("keepout_mask_yaml", default_value=keepout_mask_yaml),
            DeclareLaunchArgument("pose_covariance_xy", default_value="0.05"),
            DeclareLaunchArgument("pose_covariance_yaw", default_value="0.1"),
            DeclareLaunchArgument("twist_covariance_vx", default_value="0.05"),
            DeclareLaunchArgument("twist_covariance_vy", default_value="0.01"),
            DeclareLaunchArgument("twist_covariance_yaw_rate", default_value="0.1"),
            Node(
                package="navegacion_gps",
                executable="sim_drive_telemetry",
                name="sim_drive_telemetry",
                output="screen",
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
            ),
            Node(
                package="navegacion_gps",
                executable="sim_sensor_normalizer_v2",
                name="sim_sensor_normalizer_v2",
                output="screen",
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
            ),
            Node(
                package="navegacion_gps",
                executable="cmd_vel_ackermann_bridge_v2",
                name="cmd_vel_ackermann_bridge_v2",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {"vx_deadband_mps": ParameterValue(vx_deadband_mps, value_type=float)},
                    {
                        "vx_min_effective_mps": ParameterValue(
                            vx_min_effective_mps, value_type=float
                        )
                    },
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gps_wpf_dir, "launch", "sim_v2_base.launch.py")
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gps_wpf_dir, "launch", "localization_v2.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "wheelbase_m": wheelbase_m,
                    "localization_params_file": localization_params_file,
                    "pose_covariance_xy": pose_covariance_xy,
                    "pose_covariance_yaw": pose_covariance_yaw,
                    "twist_covariance_vx": twist_covariance_vx,
                    "twist_covariance_vy": twist_covariance_vy,
                    "twist_covariance_yaw_rate": twist_covariance_yaw_rate,
                }.items(),
            ),
            TimerAction(
                period=nav_start_delay_s,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(gps_wpf_dir, "launch", "nav_local_v2.launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "nav2_params_file": nav2_params_file,
                            "collision_monitor_params_file": collision_monitor_params_file,
                            "keepout_mask_yaml": keepout_mask_yaml_arg,
                        }.items(),
                    )
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
