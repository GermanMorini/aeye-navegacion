import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_DATUM_LAT = -31.4858037
DEFAULT_DATUM_LON = -64.2410570
DEFAULT_DATUM_YAW_DEG = 135.0


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


def _build_navsat_transform(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    gps_topic = LaunchConfiguration("gps_topic").perform(context)
    global_localization_params_file = LaunchConfiguration(
        "global_localization_params_file"
    ).perform(context)
    datum_lat = float(LaunchConfiguration("datum_lat").perform(context))
    datum_lon = float(LaunchConfiguration("datum_lon").perform(context))
    datum_yaw_deg = float(LaunchConfiguration("datum_yaw_deg").perform(context))
    datum_yaw_rad = math.radians(datum_yaw_deg)

    return [
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[
                global_localization_params_file,
                {
                    "use_sim_time": use_sim_time,
                    "wait_for_datum": False,
                    "datum": [datum_lat, datum_lon, datum_yaw_rad],
                },
            ],
            remappings=[
                ("imu/data", imu_topic),
                ("gps/fix", gps_topic),
                ("odometry/filtered", "/odometry/local"),
                ("odometry/gps", "/odometry/gps"),
            ],
        )
    ]


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    default_global_params_file = _resolve_config_file_path(
        gps_wpf_dir, "localization_global_v2.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    drive_telemetry_topic = LaunchConfiguration("drive_telemetry_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    gps_topic = LaunchConfiguration("gps_topic")
    wheelbase_m = LaunchConfiguration("wheelbase_m")
    invert_measured_steer_sign = LaunchConfiguration("invert_measured_steer_sign")
    pose_covariance_xy = LaunchConfiguration("pose_covariance_xy")
    pose_covariance_yaw = LaunchConfiguration("pose_covariance_yaw")
    twist_covariance_vx = LaunchConfiguration("twist_covariance_vx")
    twist_covariance_vy = LaunchConfiguration("twist_covariance_vy")
    twist_covariance_yaw_rate = LaunchConfiguration("twist_covariance_yaw_rate")
    global_localization_params_file = LaunchConfiguration("global_localization_params_file")
    datum_setter = LaunchConfiguration("datum_setter")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument(
                "drive_telemetry_topic",
                default_value="/controller/drive_telemetry",
            ),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument("gps_topic", default_value="/gps/fix"),
            DeclareLaunchArgument("wheelbase_m", default_value="0.94"),
            DeclareLaunchArgument(
                "invert_measured_steer_sign",
                default_value="False",
            ),
            DeclareLaunchArgument("pose_covariance_xy", default_value="0.05"),
            DeclareLaunchArgument("pose_covariance_yaw", default_value="0.1"),
            DeclareLaunchArgument("twist_covariance_vx", default_value="0.05"),
            DeclareLaunchArgument("twist_covariance_vy", default_value="0.01"),
            DeclareLaunchArgument("twist_covariance_yaw_rate", default_value="0.1"),
            DeclareLaunchArgument(
                "global_localization_params_file",
                default_value=default_global_params_file,
            ),
            DeclareLaunchArgument("datum_setter", default_value="false"),
            DeclareLaunchArgument("datum_lat", default_value=str(DEFAULT_DATUM_LAT)),
            DeclareLaunchArgument("datum_lon", default_value=str(DEFAULT_DATUM_LON)),
            DeclareLaunchArgument("datum_yaw_deg", default_value=str(DEFAULT_DATUM_YAW_DEG)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(gps_wpf_dir) / "launch" / "localization_v2.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "drive_telemetry_topic": drive_telemetry_topic,
                    "imu_topic": imu_topic,
                    "wheelbase_m": wheelbase_m,
                    "invert_measured_steer_sign": invert_measured_steer_sign,
                    "pose_covariance_xy": pose_covariance_xy,
                    "pose_covariance_yaw": pose_covariance_yaw,
                    "twist_covariance_vx": twist_covariance_vx,
                    "twist_covariance_vy": twist_covariance_vy,
                    "twist_covariance_yaw_rate": twist_covariance_yaw_rate,
                }.items(),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[
                    global_localization_params_file,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=[
                    ("imu/data", imu_topic),
                    ("odometry/filtered", "/odometry/global"),
                ],
            ),
            OpaqueFunction(function=_build_navsat_transform),
            Node(
                package="navegacion_gps",
                executable="datum_setter",
                name="datum_setter",
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", datum_setter, "'.lower() == 'true'"])
                ),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "gps_topic": gps_topic,
                        "imu_topic": imu_topic,
                        "rtk_status_topic": "/gps/rtk_status",
                        "set_datum_service": "/datum_setter/set_datum",
                        "get_datum_service": "/datum_setter/get_datum",
                        "datum_service": "/datum",
                        "datum_service_fallback": "/navsat_transform/datum",
                        "imu_yaw_max_age_s": 1.0,
                        "datum_wait_timeout_s": 2.0,
                        "datum_call_timeout_s": 2.5,
                        "datum_call_retries": 3,
                        "datum_retry_delay_s": 0.15,
                    }
                ],
            ),
        ]
    )
