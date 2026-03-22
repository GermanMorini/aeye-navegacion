from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

GLOBAL_LOCALIZATION_START_DELAY_S = 2.0


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
    default_local_params_file = _resolve_config_file_path(
        gps_wpf_dir, "localization_v2.yaml"
    )
    default_global_params_file = _resolve_config_file_path(
        gps_wpf_dir, "global_localization_v2.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheelbase_m = LaunchConfiguration("wheelbase_m")
    invert_measured_steer_sign = LaunchConfiguration("invert_measured_steer_sign")
    localization_params_file = LaunchConfiguration("localization_params_file")
    global_localization_params_file = LaunchConfiguration(
        "global_localization_params_file"
    )
    pose_covariance_xy = LaunchConfiguration("pose_covariance_xy")
    pose_covariance_yaw = LaunchConfiguration("pose_covariance_yaw")
    twist_covariance_vx = LaunchConfiguration("twist_covariance_vx")
    twist_covariance_vy = LaunchConfiguration("twist_covariance_vy")
    twist_covariance_yaw_rate = LaunchConfiguration("twist_covariance_yaw_rate")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument("wheelbase_m", default_value="0.94"),
            DeclareLaunchArgument(
                "invert_measured_steer_sign",
                default_value="False",
            ),
            DeclareLaunchArgument(
                "localization_params_file",
                default_value=default_local_params_file,
            ),
            DeclareLaunchArgument(
                "global_localization_params_file",
                default_value=default_global_params_file,
            ),
            DeclareLaunchArgument("pose_covariance_xy", default_value="0.05"),
            DeclareLaunchArgument("pose_covariance_yaw", default_value="0.1"),
            DeclareLaunchArgument("twist_covariance_vx", default_value="0.05"),
            DeclareLaunchArgument("twist_covariance_vy", default_value="0.01"),
            DeclareLaunchArgument("twist_covariance_yaw_rate", default_value="0.1"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(gps_wpf_dir) / "launch" / "localization_v2.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "wheelbase_m": wheelbase_m,
                    "invert_measured_steer_sign": invert_measured_steer_sign,
                    "localization_params_file": localization_params_file,
                    "pose_covariance_xy": pose_covariance_xy,
                    "pose_covariance_yaw": pose_covariance_yaw,
                    "twist_covariance_vx": twist_covariance_vx,
                    "twist_covariance_vy": twist_covariance_vy,
                    "twist_covariance_yaw_rate": twist_covariance_yaw_rate,
                }.items(),
            ),
            TimerAction(
                period=GLOBAL_LOCALIZATION_START_DELAY_S,
                actions=[
                    Node(
                        package="robot_localization",
                        executable="ekf_node",
                        name="ekf_filter_node_map",
                        output="screen",
                        parameters=[
                            global_localization_params_file,
                            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                        ],
                    ),
                    Node(
                        package="robot_localization",
                        executable="navsat_transform_node",
                        name="navsat_transform",
                        output="screen",
                        parameters=[
                            global_localization_params_file,
                            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                        ],
                        remappings=[
                            ("odometry/gps", "/odometry/gps"),
                            ("odometry/filtered", "/odometry/local"),
                        ],
                    ),
                ],
            ),
        ]
    )
