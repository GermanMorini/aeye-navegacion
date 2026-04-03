import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    default_ublox_params = os.path.join(gps_wpf_dir, "config", "ublox_dual_gps.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    ublox_device = LaunchConfiguration("ublox_device")
    ublox_params_file = LaunchConfiguration("ublox_params_file")
    yaw_offset_rad = LaunchConfiguration("yaw_offset_rad")
    output_frame = LaunchConfiguration("output_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument(
                "ublox_device",
                default_value="/dev/ttyUSB1",
                description="Serial device for the rover ZED-F9P",
            ),
            DeclareLaunchArgument(
                "ublox_params_file",
                default_value=default_ublox_params,
                description="Path to ublox_gps YAML config file",
            ),
            DeclareLaunchArgument(
                "yaw_offset_rad",
                default_value=str(math.pi / 2.0),
                description="Yaw offset applied to ublox heading (+π/2; equivalent to adding π over -π/2)",
            ),
            DeclareLaunchArgument(
                "output_frame",
                default_value="base_link",
                description="Frame id for the output /dual_gps/heading Imu",
            ),
            # ublox_gps rover node — publishes navheading on /ublox_rover/navheading
            Node(
                package="ublox_gps",
                executable="ublox_gps_node",
                name="ublox_rover",
                output="screen",
                parameters=[
                    ublox_params_file,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "device": ublox_device,
                    },
                ],
            ),
            # heading_offset_relay — corrects for lateral baseline geometry
            Node(
                package="navegacion_gps",
                executable="dual_gps_heading_real",
                name="dual_gps_heading_real",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "input_topic": "/ublox_rover/navheading",
                        "output_topic": "/dual_gps/heading",
                        "yaw_offset_rad": ParameterValue(yaw_offset_rad, value_type=float),
                        "output_frame": output_frame,
                    }
                ],
            ),
        ]
    )
