import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    default_rviz = os.path.join(gps_wpf_dir, "config", "rviz_local_v2.rviz")
    keepout_mask_yaml = os.path.join(gps_wpf_dir, "config", "keepout_mask.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheelbase_m = LaunchConfiguration("wheelbase_m")
    invert_measured_steer_sign = LaunchConfiguration("invert_measured_steer_sign")
    nav_start_delay_s = LaunchConfiguration("nav_start_delay_s")
    use_keepout = LaunchConfiguration("use_keepout")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
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
    datum_setter = LaunchConfiguration("datum_setter")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("wheelbase_m", default_value="0.94"),
            DeclareLaunchArgument("invert_measured_steer_sign", default_value="True"),
            DeclareLaunchArgument("nav_start_delay_s", default_value="4.0"),
            DeclareLaunchArgument("use_keepout", default_value="True"),
            DeclareLaunchArgument("use_rviz", default_value="True"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
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
                default_value=os.path.join(gps_wpf_dir, "config", "collision_monitor_v2.yaml"),
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
            DeclareLaunchArgument(
                "model_name", default_value="quad_ackermann_viewer_safe"
            ),
            DeclareLaunchArgument("pose_covariance_xy", default_value="0.01"),
            DeclareLaunchArgument("pose_covariance_yaw", default_value="0.05"),
            DeclareLaunchArgument("twist_covariance_vx", default_value="0.02"),
            DeclareLaunchArgument("twist_covariance_vy", default_value="0.02"),
            DeclareLaunchArgument("twist_covariance_yaw_rate", default_value="0.05"),
            DeclareLaunchArgument("ekf_local", default_value="True"),
            DeclareLaunchArgument("ekf_global", default_value="False"),
            DeclareLaunchArgument("datum_setter", default_value="false"),
            Node(
                package="navegacion_gps",
                executable="sim_sensor_normalizer_v2",
                name="sim_sensor_normalizer_v2",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {"imu_auto_calibrate_yaw_from_odom": True},
                    {"imu_yaw_offset_rad": 0.0},
                    {"imu_yaw_calib_odom_topic": "/odom_raw"},
                    {"imu_yaw_calib_speed_threshold_mps": 0.05},
                    {"imu_yaw_calib_timeout_s": 3.0},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gps_wpf_dir, "launch", "sim_v2_base.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "custom_urdf": custom_urdf,
                    "world": world,
                    "world_name": world_name,
                    "model_name": model_name,
                }.items(),
            ),
            Node(
                package="controller_server",
                executable="controller_server_node",
                name="vehicle_controller_server",
                output="screen",
                condition=UnlessCondition(use_cmd_vel_ackermann_bridge),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "transport_backend": "sim_gazebo",
                        "serial_port": "/dev/null",
                        "serial_baud": 115200,
                        "serial_tx_hz": 50.0,
                        "max_reverse_mps": 1.30,
                        "max_abs_angular_z": 0.4,
                        "vx_deadband_mps": ParameterValue(
                            vx_deadband_mps, value_type=float
                        ),
                        "vx_min_effective_mps": ParameterValue(
                            vx_min_effective_mps, value_type=float
                        ),
                        "invert_steer_from_cmd_vel": ParameterValue(
                            invert_steer_from_cmd_vel, value_type=bool
                        ),
                        "sim_cmd_vel_topic": "/cmd_vel_gazebo",
                        "sim_odom_topic": "/odom_raw",
                        "sim_joint_states_topic": "/joint_states",
                        "sim_front_left_steer_joint": "front_left_steer_joint",
                        "sim_front_right_steer_joint": "front_right_steer_joint",
                        "sim_max_steering_angle_rad": 0.5235987756,
                        "sim_telemetry_timeout_s": 0.5,
                        "sim_invert_actuation_steer_sign": True,
                        "sim_invert_measured_steer_sign": True,
                    }
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="cmd_vel_ackermann_bridge_v2",
                name="cmd_vel_ackermann_bridge_v2",
                output="screen",
                condition=IfCondition(use_cmd_vel_ackermann_bridge),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "input_topic": "/cmd_vel_safe",
                        "output_topic": "/cmd_vel_gazebo",
                        "max_speed_mps": 4.0,
                        "max_reverse_mps": 1.30,
                        "vx_deadband_mps": ParameterValue(
                            vx_deadband_mps, value_type=float
                        ),
                        "vx_min_effective_mps": ParameterValue(
                            vx_min_effective_mps, value_type=float
                        ),
                        "max_abs_angular_z": 0.4,
                        "invert_steer_from_cmd_vel": ParameterValue(
                            invert_steer_from_cmd_vel, value_type=bool
                        ),
                        "auto_drive_enabled": True,
                        "reverse_brake_pct": 20,
                        "sim_max_forward_mps": 4.0,
                        "sim_max_reverse_mps": 1.30,
                        "sim_max_steering_angle_rad": 0.5235987756,
                        "input_timeout_s": 0.5,
                        "watchdog_hz": 20.0,
                        "stop_hold_topic": "/local_nav_v2/stop_hold",
                        "stop_hold_duration_s": 1.5,
                    }
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="sim_drive_telemetry",
                name="sim_drive_telemetry",
                output="screen",
                condition=IfCondition(use_cmd_vel_ackermann_bridge),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "odom_topic": "/odom_raw",
                        "joint_states_topic": "/joint_states",
                        "drive_telemetry_topic": "/controller/drive_telemetry",
                        "front_left_steer_joint": "front_left_steer_joint",
                        "front_right_steer_joint": "front_right_steer_joint",
                        "wheelbase_m": ParameterValue(wheelbase_m, value_type=float),
                        "steer_from_odom_min_speed_mps": 0.05,
                    }
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="nav_command_server",
                name="nav_command_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "fromll_service": "/fromLL",
                        "fromll_service_fallback": "/navsat_transform/fromLL",
                        "fromll_wait_timeout_s": 2.0,
                        "map_frame": "odom",
                        "gps_topic": "/gps/fix",
                        "cmd_vel_safe_topic": "/cmd_vel_safe",
                        "cmd_vel_final_topic": "/cmd_vel_final",
                        "forward_cmd_vel_safe_without_goal": True,
                        "brake_topic": "/cmd_vel_safe",
                        "manual_cmd_topic": "/cmd_vel_safe",
                        "teleop_cmd_topic": "/cmd_vel_teleop",
                        "brake_publish_count": 5,
                        "brake_publish_interval_s": 0.1,
                        "manual_cmd_timeout_s": 0.4,
                        "manual_watchdog_hz": 10.0,
                        "nav_telemetry_hz": 5.0,
                        "telemetry_topic": "/nav_command_server/telemetry",
                        "set_goal_service": "/nav_command_server/set_goal_ll",
                        "cancel_goal_service": "/nav_command_server/cancel_goal",
                        "brake_service": "/nav_command_server/brake",
                        "set_manual_mode_service": "/nav_command_server/set_manual_mode",
                        "get_state_service": "/nav_command_server/get_state",
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gps_wpf_dir, "launch", "localization_v2.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "wheelbase_m": wheelbase_m,
                    "invert_measured_steer_sign": invert_measured_steer_sign,
                    "pose_covariance_xy": pose_covariance_xy,
                    "pose_covariance_yaw": pose_covariance_yaw,
                    "twist_covariance_vx": twist_covariance_vx,
                    "twist_covariance_vy": twist_covariance_vy,
                    "twist_covariance_yaw_rate": twist_covariance_yaw_rate,
                    "ekf_local": ekf_local,
                    "ekf_global": ekf_global,
                }.items(),
            ),
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
                        "gps_topic": "/gps/fix",
                        "imu_topic": "/imu/data",
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
            TimerAction(
                period=nav_start_delay_s,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(gps_wpf_dir, "launch", "nav_local_v2.launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "use_keepout": use_keepout,
                            "nav2_params_file": nav2_params_file,
                            "collision_monitor_params_file": collision_monitor_params_file,
                            "keepout_mask_yaml": keepout_mask_yaml_arg,
                            "keepout_mask_frame": "map",
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
