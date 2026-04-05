from pathlib import Path


def test_sim_local_v2_launch_uses_realistic_command_chain() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'executable="nav_command_server"' in launch_contents
    assert 'package="controller_server"' in launch_contents
    assert '"transport_backend": "sim_gazebo"' in launch_contents
    assert '"cmd_vel_final_topic": "/cmd_vel_final"' in launch_contents
    assert '"forward_cmd_vel_safe_without_goal": True' in launch_contents
    assert '"serial_tx_hz": 20.0' in launch_contents
    assert '"control_hz": 20.0' in launch_contents
    assert '"wheelbase_m": ParameterValue(wheelbase_m, value_type=float)' in launch_contents
    assert '"max_steering_angle_rad": 0.5235987756' in launch_contents
    assert '"cmd_log_enabled": False' in launch_contents
    assert 'DeclareLaunchArgument("gz_headless", default_value="true")' in launch_contents
    assert '"gz_headless": gz_headless' in launch_contents


def test_sim_local_v2_launch_exposes_optional_bridge_mode() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("use_cmd_vel_ackermann_bridge", default_value="False")' in launch_contents
    assert 'executable="cmd_vel_ackermann_bridge_v2"' in launch_contents
    assert 'executable="sim_drive_telemetry"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "use_ackermann_geometry_steering", default_value="False"' in launch_contents
    assert 'DeclareLaunchArgument("invert_measured_steer_sign", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("invert_steer_from_cmd_vel", default_value="True")' in launch_contents
    assert '"use_ackermann_geometry_steering": ParameterValue(' in launch_contents
    assert '"wheelbase_m": ParameterValue(wheelbase_m, value_type=float)' in launch_contents


def test_sim_local_v2_launch_forwards_ekf_global_toggle() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("ekf_local", default_value="True")' in launch_contents
    assert '"ekf_local": ekf_local' in launch_contents
    assert 'DeclareLaunchArgument("ekf_global", default_value="False")' in launch_contents
    assert '"ekf_global": ekf_global' in launch_contents
    assert 'DeclareLaunchArgument("enable_navsat_transform", default_value="False")' in launch_contents
    assert '"enable_navsat_transform": enable_navsat_transform' in launch_contents
    assert 'DeclareLaunchArgument("use_sim_global_overlay", default_value="False")' in launch_contents
    assert '"use_sim_global_overlay": use_sim_global_overlay' in launch_contents
    assert '"global_start_delay_s": "5.0"' in launch_contents


def test_sim_local_v2_launch_forwards_spawn_yaw_override() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("spawn_yaw_rad", default_value="auto")' in launch_contents
    assert '"spawn_yaw_rad": spawn_yaw_rad' in launch_contents


def test_sim_local_v2_launch_resolves_map_frame_from_ekf_global_toggle() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "resolved_map_frame = PythonExpression(" in launch_contents
    assert "resolved_multi_waypoint_spacing_m = PythonExpression(" in launch_contents
    assert "resolved_goal_arrival_radius_m = PythonExpression(" in launch_contents
    assert "resolved_multi_waypoint_action_mode = PythonExpression(" in launch_contents
    assert '"map_frame": ParameterValue(' in launch_contents
    assert "resolved_map_frame" in launch_contents
    assert "'odom' if '" in launch_contents
    assert "else ('map' if '" in launch_contents
    assert "'4.0' if '" in launch_contents
    assert '"multi_waypoint_spacing_m": ParameterValue(' in launch_contents
    assert "'0.5' if '" in launch_contents
    assert '"goal_arrival_radius_m": ParameterValue(' in launch_contents
    assert "'follow_waypoints'" in launch_contents
    assert '"multi_waypoint_action_mode": ParameterValue(' in launch_contents


def test_sim_local_v2_launch_forwards_ukf_toggle() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("ukf", default_value="False")' in launch_contents
    assert '"ukf": ukf' in launch_contents


def test_sim_local_v2_launch_exposes_datum_setter_toggle() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("datum_setter", default_value="false")' in launch_contents
    assert 'executable="datum_setter"' in launch_contents
    assert "PythonExpression([\"'\", datum_setter, \"'.lower() == 'true'\"])" in launch_contents
    assert '"auto_set_fixed_coords": [-31.4858037, -64.2410570]' in launch_contents


def test_sim_local_v2_launch_forces_keepout_mask_frame_to_resolved_nav_frame() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert '"map_frame": resolved_map_frame' in launch_contents
    assert '"keepout_mask_frame": resolved_map_frame' in launch_contents


def test_nav_local_v2_launch_defaults_keepout_mask_frame_to_odom() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "nav_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("map_frame", default_value="odom")' in launch_contents
    assert 'DeclareLaunchArgument("keepout_mask_frame", default_value="odom")' in launch_contents
    assert '"bt_navigator.ros__parameters.global_frame": map_frame' in launch_contents
    assert '"behavior_server.ros__parameters.global_frame": map_frame' in launch_contents
    assert '"global_costmap.global_costmap.ros__parameters.global_frame": map_frame' in launch_contents
    assert '"keepout_mask_frame": keepout_mask_frame' in launch_contents


def test_sim_local_v2_launch_sets_imu_yaw_auto_calibration_defaults() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert '"imu_auto_calibrate_yaw_from_odom": True' in launch_contents
    assert '"imu_yaw_offset_rad": 0.0' in launch_contents
    assert '"imu_yaw_calib_odom_topic": "/odom_raw"' in launch_contents
    assert '"imu_yaw_calib_speed_threshold_mps": 0.05' in launch_contents
    assert '"imu_yaw_calib_timeout_s": 3.0' in launch_contents
    assert 'DeclareLaunchArgument("gps_horizontal_variance", default_value="25.0")' in launch_contents
    assert 'DeclareLaunchArgument("gps_vertical_variance", default_value="36.0")' in launch_contents
    assert '"gps_horizontal_variance": ParameterValue(' in launch_contents
    assert '"gps_vertical_variance": ParameterValue(' in launch_contents


def test_sim_local_v2_launch_uses_real_dual_gps_heading_contract() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("use_dual_gps_heading", default_value="true")' in launch_contents
    assert 'executable="dual_gps_heading_sim"' in launch_contents
    assert 'executable="dual_gps_heading_real"' in launch_contents
    assert '"raw_heading_imu_topic": "/ublox_rover/navheading"' in launch_contents
    assert '"odom_heading_topic": "/odom_raw"' in launch_contents
    assert '"input_topic": "/ublox_rover/navheading"' in launch_contents
    assert '"output_topic": "/dual_gps/heading"' in launch_contents
    assert launch_contents.count('executable="imu_pose_republisher"') == 2
    assert '"output_topic": "/ublox_rover/navheading_pose"' in launch_contents
    assert '"output_topic": "/dual_gps/heading_pose"' in launch_contents
    assert launch_contents.count('{"odom_topic": "/odometry/local"}') == 2
    assert launch_contents.count('{"output_frame": "odom"}') >= 2
