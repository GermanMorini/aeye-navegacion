from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    dual_ekf_params_file = _resolve_config_file_path(gps_wpf_dir, "dual_ekf_navsat_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    drive_telemetry_topic = LaunchConfiguration("drive_telemetry_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    pixhawk_gps_topic = LaunchConfiguration("pixhawk_gps_topic")
    pixhawk_output_odom_topic = LaunchConfiguration("pixhawk_output_odom_topic")
    wheelbase_m = LaunchConfiguration("wheelbase_m")
    invert_measured_steer_sign = LaunchConfiguration("invert_measured_steer_sign")
    pose_covariance_xy = LaunchConfiguration("pose_covariance_xy")
    pose_covariance_yaw = LaunchConfiguration("pose_covariance_yaw")
    twist_covariance_vx = LaunchConfiguration("twist_covariance_vx")
    twist_covariance_vy = LaunchConfiguration("twist_covariance_vy")
    twist_covariance_yaw_rate = LaunchConfiguration("twist_covariance_yaw_rate")
    ekf_local = LaunchConfiguration("ekf_local")
    ekf_global = LaunchConfiguration("ekf_global")
    publish_odom_tf = ParameterValue(
        PythonExpression(["'", ekf_local, "' == 'False'"]),
        value_type=bool,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument(
                "drive_telemetry_topic",
                default_value="/controller/drive_telemetry",
            ),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument("pixhawk_gps_topic", default_value="/gps/fix"),
            DeclareLaunchArgument(
                "pixhawk_output_odom_topic",
                default_value="/odometry/pixhawk",
            ),
            DeclareLaunchArgument("wheelbase_m", default_value="0.94"),
            DeclareLaunchArgument(
                "invert_measured_steer_sign",
                default_value="False",
            ),
            DeclareLaunchArgument("pose_covariance_xy", default_value="0.01"),
            DeclareLaunchArgument("pose_covariance_yaw", default_value="0.05"),
            DeclareLaunchArgument("twist_covariance_vx", default_value="0.02"),
            DeclareLaunchArgument("twist_covariance_vy", default_value="0.02"),
            DeclareLaunchArgument("twist_covariance_yaw_rate", default_value="0.05"),
            DeclareLaunchArgument("ekf_local", default_value="True"),
            DeclareLaunchArgument("ekf_global", default_value="False"),
            Node(
                package="navegacion_gps",
                executable="ackermann_odometry",
                name="ackermann_odometry",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {"telemetry_topic": drive_telemetry_topic},
                    {"wheelbase_m": ParameterValue(wheelbase_m, value_type=float)},
                    {
                        "invert_measured_steer_sign": ParameterValue(
                            invert_measured_steer_sign,
                            value_type=bool,
                        )
                    },
                    {
                        "pose_covariance_xy": ParameterValue(
                            pose_covariance_xy,
                            value_type=float,
                        )
                    },
                    {
                        "pose_covariance_yaw": ParameterValue(
                            pose_covariance_yaw,
                            value_type=float,
                        )
                    },
                    {
                        "twist_covariance_vx": ParameterValue(
                            twist_covariance_vx,
                            value_type=float,
                        )
                    },
                    {
                        "twist_covariance_vy": ParameterValue(
                            twist_covariance_vy,
                            value_type=float,
                        )
                    },
                    {
                        "twist_covariance_yaw_rate": ParameterValue(
                            twist_covariance_yaw_rate,
                            value_type=float,
                        )
                    },
                    {"publish_odom_tf": publish_odom_tf},
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="pixhawk_odometry",
                name="pixhawk_odometry",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {"imu_topic": imu_topic},
                    {"gps_topic": pixhawk_gps_topic},
                    {"output_odom_topic": pixhawk_output_odom_topic},
                ],
            ),
            Node(
                package="robot_localization",
                executable="ukf_node",
                name="ekf_filter_node_odom",
                output="screen",
                condition=IfCondition(ekf_local),
                parameters=[
                    dual_ekf_params_file,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=[
                    ("imu/data", imu_topic),
                    ("/odom", "/wheel/odometry"),
                    ("odometry/filtered", "/odometry/local"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="ukf_node",
                name="ekf_filter_node_map",
                output="screen",
                condition=IfCondition(ekf_global),
                parameters=[
                    dual_ekf_params_file,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=[
                    ("imu/data", imu_topic),
                    ("odometry/filtered", "/odometry/global"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                condition=IfCondition(ekf_global),
                parameters=[
                    dual_ekf_params_file,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=[
                    ("odometry/filtered", "/odometry/global"),
                ],
            ),
        ]
    )
