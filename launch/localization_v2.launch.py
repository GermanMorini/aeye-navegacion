from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
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


def _build_localization_nodes(context):
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    base_params = _resolve_config_file_path(gps_wpf_dir, "dual_ekf_navsat_params.yaml")
    overlay_params = _resolve_config_file_path(
        gps_wpf_dir, "dual_gps_heading_ekf_overlay.yaml"
    )
    sim_global_overlay_params = _resolve_config_file_path(
        gps_wpf_dir, "dual_ekf_navsat_sim_global_overlay.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "True"
    ekf_local = LaunchConfiguration("ekf_local").perform(context).lower() == "true"
    ekf_global = LaunchConfiguration("ekf_global").perform(context).lower() == "true"
    enable_navsat_transform = (
        LaunchConfiguration("enable_navsat_transform").perform(context).lower()
        == "true"
    )
    use_sim_global_overlay = (
        LaunchConfiguration("use_sim_global_overlay").perform(context).lower()
        == "true"
    )
    use_ukf = LaunchConfiguration("ukf").perform(context).lower() == "true"
    use_dual = (
        LaunchConfiguration("use_dual_gps_heading").perform(context).lower() == "true"
    )
    ackermann_odom_topic = "/wheel/odometry" if ekf_local else "/odometry/local"

    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    drive_telemetry_topic = LaunchConfiguration("drive_telemetry_topic").perform(context)
    wheelbase_m = float(LaunchConfiguration("wheelbase_m").perform(context))
    invert_measured_steer_sign = (
        LaunchConfiguration("invert_measured_steer_sign").perform(context).lower()
        == "true"
    )
    pose_covariance_xy = float(
        LaunchConfiguration("pose_covariance_xy").perform(context)
    )
    pose_covariance_yaw = float(
        LaunchConfiguration("pose_covariance_yaw").perform(context)
    )
    twist_covariance_vx = float(
        LaunchConfiguration("twist_covariance_vx").perform(context)
    )
    twist_covariance_vy = float(
        LaunchConfiguration("twist_covariance_vy").perform(context)
    )
    twist_covariance_yaw_rate = float(
        LaunchConfiguration("twist_covariance_yaw_rate").perform(context)
    )
    global_start_delay_s = max(
        0.0, float(LaunchConfiguration("global_start_delay_s").perform(context))
    )

    executable = "ukf_node" if use_ukf else "ekf_node"

    # Base params; when dual GPS heading is active, also load the overlay
    # (overlay params override conflicting keys in the base file).
    ekf_param_files = [base_params]
    if use_dual:
        ekf_param_files.append(overlay_params)
    if use_sim_time and use_sim_global_overlay:
        ekf_param_files.append(sim_global_overlay_params)

    publish_odom_tf = not ekf_local  # ackermann_odometry publishes TF when EKF is off

    nodes = [
        Node(
            package="navegacion_gps",
            executable="ackermann_odometry",
            name="ackermann_odometry",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"telemetry_topic": drive_telemetry_topic},
                {"odom_topic": ackermann_odom_topic},
                {"wheelbase_m": wheelbase_m},
                {"invert_measured_steer_sign": invert_measured_steer_sign},
                {"pose_covariance_xy": pose_covariance_xy},
                {"pose_covariance_yaw": pose_covariance_yaw},
                {"twist_covariance_vx": twist_covariance_vx},
                {"twist_covariance_vy": twist_covariance_vy},
                {"twist_covariance_yaw_rate": twist_covariance_yaw_rate},
                {"periodic_log_enabled": False},
                {"publish_odom_tf": publish_odom_tf},
            ],
        ),
    ]

    if ekf_local:
        nodes.append(
            Node(
                package="robot_localization",
                executable=executable,
                name="ekf_filter_node_odom",
                output="screen",
                parameters=ekf_param_files + [{"use_sim_time": use_sim_time}],
                remappings=[
                    ("imu/data", imu_topic),
                    ("/odom", "/wheel/odometry"),
                    ("odometry/filtered", "/odometry/local"),
                ],
            )
        )

    if ekf_global:
        global_nodes = [
            Node(
                package="robot_localization",
                executable=executable,
                name="ekf_filter_node_map",
                output="screen",
                parameters=ekf_param_files + [{"use_sim_time": use_sim_time}],
                remappings=[
                    ("imu/data", imu_topic),
                    ("odometry/filtered", "/odometry/global"),
                ],
            )
        ,
        ]
        nodes.append(TimerAction(period=global_start_delay_s, actions=global_nodes))

    if ekf_global or enable_navsat_transform:
        nodes.append(
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=ekf_param_files + [{"use_sim_time": use_sim_time}],
                remappings=[
                    ("odometry/filtered", "/odometry/local"),
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument(
                "drive_telemetry_topic",
                default_value="/controller/drive_telemetry",
            ),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument("wheelbase_m", default_value="0.94"),
            DeclareLaunchArgument("invert_measured_steer_sign", default_value="False"),
            DeclareLaunchArgument("pose_covariance_xy", default_value="0.01"),
            DeclareLaunchArgument("pose_covariance_yaw", default_value="0.05"),
            DeclareLaunchArgument("twist_covariance_vx", default_value="0.02"),
            DeclareLaunchArgument("twist_covariance_vy", default_value="0.02"),
            DeclareLaunchArgument("twist_covariance_yaw_rate", default_value="0.05"),
            DeclareLaunchArgument("ekf_local", default_value="True"),
            DeclareLaunchArgument("ekf_global", default_value="False"),
            DeclareLaunchArgument("enable_navsat_transform", default_value="False"),
            DeclareLaunchArgument("use_sim_global_overlay", default_value="False"),
            DeclareLaunchArgument("ukf", default_value="False"),
            DeclareLaunchArgument("use_dual_gps_heading", default_value="false"),
            DeclareLaunchArgument("global_start_delay_s", default_value="0.0"),
            OpaqueFunction(function=_build_localization_nodes),
        ]
    )
